/*
 * Copyright 2026 Manifold Tech Ltd.
 * Author: MENG Guotao <mengguotao@manifoldtech.cn>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Drone physics v3 — quaternion-based orientation.
 *
 * All rotations are applied in the drone's BODY frame via quaternion multiplication.
 * This eliminates Euler-angle cross-coupling: roll is always around the drone's
 * nose-to-tail axis regardless of heading.
 *
 * Geometry (top view = square):
 *   - droneSize: width = depth (configurable)
 *   - CG at center
 *   - Camera at front edge (CG + local forward * droneSize/2)
 *   - Thrust along local +Y through CG
 *   - Forward = local -Z at identity orientation
 *
 * FPV:   sticks → body-frame angular rates,  throttle → thrust,  no self-leveling
 * Drone: sticks → jerk-limited velocity setpoint → position/velocity hold
 */

import { reportUserError } from './error-report.js';
import {
    YopoTrajectoryTracker,
    validateYopoTrajectory,
} from './yopo-trajectory.js';
import {
    ControlCommandType,
    allocateEasyForce,
    capDirectForce,
    desiredAttitudeFromForce,
    firstOrderRateServo,
    integrateBodyRates,
    limitVector,
    piecewiseHoverThrottle,
    reducedQuaternionBodyRateSetpoint,
    rotateVectorByQuaternion,
    shapeAssistedAxis,
    vectorNorm,
} from './flight-control.js';

export { validateYopoTrajectory } from './yopo-trajectory.js';

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;
const G = 9.81;              // gravitational acceleration (m/s²)
const AIR_DENSITY = 1.225;   // kg/m³ at sea level
const DRONE_BOOST_MULTIPLIER = 2.0;
const FPV_BOOST_MULTIPLIER = 1.7;
const DRONE_MAX_SUPPORTED_SPEED = 300 / 3.6; // 300 km/h in m/s
const DRONE_MAX_SUPPORTED_VSPEED = 25;
const FAILSAFE_HOLD_REASONS = new Set([
    'unknown-flight-mode',
    'non-finite-command',
    'control-overrun',
    'trajectory-rejected',
    'trajectory-expired',
    'collision',
]);
// Navigation completion is a product-level tolerance, not the YOPO lattice
// radio_range. The external YOPO reference uses 4 m; using radio_range (9 m)
// here caused a vehicle travelling at 5 m/s to be declared arrived at 8.94 m.
export const ARRIVAL_DISTANCE_M = 4.0;

// Reusable PlayCanvas math objects (avoid per-frame allocation)
const _quat  = new pc.Quat();
const _quat2 = new pc.Quat();
const _mat4  = new pc.Mat4();
const _v3    = new pc.Vec3();

/**
 * 推力矢量倾角限幅 —— 移植自 NetworkControl.cpp: get_Q_from_ACC()
 * 与 SO3Control.cpp: calculateControl() 中的同一段解析解。
 *
 * 把期望力拆成 重力补偿 m*g*e_up 与 加速度项 f = F_d - m*g*e_up，
 * 当 F_d 相对世界竖直方向的倾角超过 maxTiltDeg 时，**只对 f 缩放**：
 *     F_d' = s * f + m*g*e_up
 * 求 s 使 F_d' 的倾角恰好等于上限，解 A s² + B s + C = 0：
 *     A = c²|f|² - f_up²,  B = 2(c²-1) f_up m g,  C = (c²-1)(m g)²,  c = cos(θ)
 *
 * 绝不能改成对整个 F_d 等比缩放：那会把重力补偿一起缩掉，垂直分量
 * 低于 m*g 就必然掉高（历史 bug，见 tests/test_so3_tilt_limit.js）。
 *
 * 本项目坐标系 y 轴向上，故参考实现中的 e3/f(2) 在这里对应 e_y/fY。
 *
 * @param {number} FdX   期望力 x 分量 (N)
 * @param {number} FdY   期望力 y 分量 (N)，y 为世界竖直向上
 * @param {number} FdZ   期望力 z 分量 (N)
 * @param {number} weightN 机体重力 m*g (N)
 * @param {number} maxTiltDeg 最大倾角 (deg)
 * @returns {{x: number, y: number, z: number}} 限幅后的期望力
 */
export function limitTiltPreservingGravity(FdX, FdY, FdZ, weightN, maxTiltDeg) {
    const hover = { x: 0, y: weightN, z: 0 };
    const Fnorm = Math.sqrt(FdX * FdX + FdY * FdY + FdZ * FdZ);
    if (!Number.isFinite(Fnorm) || Fnorm <= 1e-6) return hover;

    const c = Math.cos(Math.max(5, Math.min(80, maxTiltDeg)) * DEG2RAD);
    if (FdY / Fnorm >= c) return { x: FdX, y: FdY, z: FdZ };  // 未超限，原样返回

    const fX = FdX, fY = FdY - weightN, fZ = FdZ;
    const nf2 = fX * fX + fY * fY + fZ * fZ;
    const A = c * c * nf2 - fY * fY;
    const B = 2 * (c * c - 1) * fY * weightN;
    const C = (c * c - 1) * weightN * weightN;
    const disc = B * B - 4 * A * C;
    if (Math.abs(A) <= 1e-9 || disc < 0) return hover;

    const s = (-B + Math.sqrt(disc)) / (2 * A);
    if (!Number.isFinite(s) || s < 0) return hover;
    return { x: s * fX, y: s * fY + weightN, z: s * fZ };
}

/**
 * Convert the horizontal projection of body +Z (backward) to this simulator's
 * yaw convention: yaw=0 faces local -Z and positive yaw turns toward -X.
 */
export function leveledYawFromBackward(bx, bz) {
    const hLen = Math.hypot(bx, bz);
    const fwdX = hLen > 1e-6 ? -bx / hLen : 0;
    const fwdZ = hLen > 1e-6 ? -bz / hLen : -1;
    return {
        fwdX,
        fwdZ,
        yawRad: Math.atan2(-fwdX, -fwdZ),
    };
}

export class Drone {
    constructor() {
        // ---- Geometry ----
        this.droneSize = 0.3;

        // ---- State ----
        this.x = 0; this.y = 2; this.z = 0;
        this.vx = 0; this.vy = 0; this.vz = 0;

        // Quaternion orientation (single source of truth)
        this.orientation = new pc.Quat();

        // Angular velocity in body frame (deg/s)
        this.pitchRate = 0;
        this.rollRate  = 0;
        this.yawRate   = 0;

        // Euler angles (derived from orientation each frame, for HUD/readout)
        this.pitch = 0;
        this.roll  = 0;
        this.yaw   = 0;

        // ---- Tunable parameters ----
        this.flightMode  = 'drone';
        // Previous-frame flight mode: used by update() to detect mode
        // transitions and re-anchor position / integrator state so the new
        // mode starts cleanly from the drone's current pose.
        this._prevFlightMode = this.flightMode;
        this.mass        = 980;    // grams (YOPO Hummingbird: 0.98 kg)
        this.maxThrust   = 2600;   // grams-force (YOPO: mass*g/hover_thrust = 0.98*9.81/0.38 ≈ 25.3N ≈ 2580gf)
        this.dragCd      = 1.0;    // drag coefficient (dimensionless)
        this.dragArea     = 0.0015; // frontal area (m²), tuned for high-speed quad flight

        this.maxPitchRate = 220;
        this.maxRollRate  = 220;
        this.maxYawRate   = 120;
        this.droneMaxYawRate = 80;  // Drone mode yaw rate limit (deg/s)

        this.droneMaxVSpeed  = 8.0;
        this.droneMaxSpeed   = DRONE_MAX_SUPPORTED_SPEED;

        // PX4-v1.17-style position/velocity defaults. Position is P-only;
        // velocity uses measurement-side D and actuator-aware anti-windup.
        this.dronePosKp  = 0.95;
        this.dronePosKi  = 0;
        this.dronePosKd  = 0;
        this.droneVelKp  = 1.8;
        this.droneVelKi  = 0.4;
        this.droneVelKd  = 0.2;
        this.droneAltKp  = 4.0;
        this.droneAltKi  = 2.0;
        this.droneAltKd  = 0;

        // Position-hold setpoints (horizontal XY + altitude Y). Drone mode
        // yaw is pure rate control and does not use a target heading.
        this._targetX = 0; this._targetY = 2; this._targetZ = 0;

        // Integral accumulators (velocity loop)
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;

        this.angularDrag = 8.0;

        this.collisionRadius = 0.6;   // YOPO vehicle_radius_m = 0.60
        this.bounceDamping   = 0.3;

        // ---- Output state ----
        this.isColliding      = false;
        this.collisionIntensity = 0;
        this.speed            = 0;
        this.groundSpeed      = 0;
        this.airSpeed         = 0;
        this.verticalSpeed    = 0;
        this.thrustOutput     = 0;
        this.throttlePercent  = 0;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = this.droneMaxSpeed;
        this.boostActive      = false;
        this.boostMultiplier  = 1.0;

        // Camera mount angle (degrees, positive = tilted up)
        // FPV mode: fixed during flight, set via settings (0..60)
        // Drone mode: live tilt via input (-90..0)
        this.cameraMountAngle = 30; // FPV default
        this.cameraTiltAngle  = 0;  // Drone mode live tilt

        // Spawn
        this._spawnX = 0; this._spawnY = 2; this._spawnZ = 0;

        // ---- Ideal controller state ----
        this._idealGoal = null;      // {x,y,z,yaw} or null
        this._arrivalDistanceM = ARRIVAL_DISTANCE_M;

        // ---- YOPO trajectory tracking ----
        this._trajectory = new YopoTrajectoryTracker();
        // Compatibility read-only aliases retained for logging and existing
        // integration tests while lifecycle ownership lives in the tracker.
        Object.defineProperties(this, {
            _yopoPolyX: { get: () => this._trajectory.polynomials?.x ?? null },
            _yopoPolyY: { get: () => this._trajectory.polynomials?.y ?? null },
            _yopoPolyZ: { get: () => this._trajectory.polynomials?.z ?? null },
            _yopoTrajTime: { get: () => this._trajectory.duration },
            _yopoTrackerTime: { get: () => this._trajectory.time },
        });
        // ---- Yaw lock (SO3 mode: fix initial yaw like YOPO lock_yaw=True) ----
        this._so3FixedYaw = null;    // null = unlocked, number = fixed yaw degrees
        this._yopoDecayRef = null;   // compatibility: decay was removed in v4 controller
        this._yopoPlanTriggered = false; // YOPO 轨迹已下发后才允许到达判定
        this._navigationState = 'idle';
        this._navigationTransitionReason = null;
        this._replanRequested = false;

        // ---- Shared ideal-controller state ----
        this._simTimeS = 0;
        this._easyYawSetpointDeg = 0;
        this._levelYawSetpointDeg = 0;
        this._so3YawSetpointDeg = 0;
        this._so3YawRateSetpointDeg = 0;
        this._so3Hold = { x: this.x, y: this.y, z: this.z, yawDeg: this.yaw };
        this._so3HoldCommandType = ControlCommandType.POSITION_VELOCITY_HOLD;
        this._controlOverrunHoldRemaining = 0;
        this._easyHorizontalState = 'hold';
        this._easyVerticalState = 'hold';
        this._easyHorizontalBrakeRamp = false;
        this._easyVelocitySetpoint = { x: 0, y: 0, z: 0 };
        this._easyAccelerationSetpoint = { x: 0, y: 0, z: 0 };
        this._easyLimitedAcceleration = { x: 0, y: 0, z: 0 };
        this._measuredAcceleration = { x: 0, y: 0, z: 0 };
        this._controlDiagnostics = this._makeControlDiagnostics('initializing');
        this._lastControlCommand = Object.freeze({
            type: ControlCommandType.FAILSAFE_HOLD,
            source: 'controller',
            frame: 'sim-world-y-up',
            generation: null,
            createdSimTime: 0,
            expirySimTime: null,
        });

        // ---- YOPO hold/direct-acceleration envelope ----
        this.so3Kx = 5.7;            // compatibility scalar: horizontal hold gain
        this.so3Kv = 3.4;            // compatibility scalar: horizontal hold gain
        this.so3KxVertical = 6.2;
        this.so3KvVertical = 4.0;
        this.so3CruiseMps = 15;      // cruise speed (YOPO cruise_target_mps = 15)
    }

    // ---- Public API ----

    _makeControlDiagnostics(reason = null) {
        return {
            simTimeS: Number(this._simTimeS || 0),
            commandType: ControlCommandType.FAILSAFE_HOLD,
            source: 'controller',
            frame: 'sim-world-y-up',
            generation: null,
            createdSimTime: Number(this._simTimeS || 0),
            expirySimTime: null,
            referenceState: null,
            actualState: {
                position: { x: Number(this.x || 0), y: Number(this.y || 0), z: Number(this.z || 0) },
                velocity: { x: Number(this.vx || 0), y: Number(this.vy || 0), z: Number(this.vz || 0) },
            },
            rawAcceleration: { x: 0, y: 0, z: 0 },
            limitedAcceleration: { x: 0, y: 0, z: 0 },
            requestedForce: { x: 0, y: 0, z: 0 },
            allocatedForce: { x: 0, y: 0, z: 0 },
            tiltDeg: 0,
            thrustGf: Number(this.thrustOutput || 0),
            saturation: { horizontal: false, vertical: false, direct: false },
            antiWindup: { horizontal: false, vertical: false },
            trajectoryAgeS: null,
            fallbackReason: reason,
            overrunCount: Number(this._controlDiagnostics?.overrunCount || 0),
            overrunDroppedSeconds: Number(this._controlDiagnostics?.overrunDroppedSeconds || 0),
        };
    }

    getControlDiagnostics() {
        const diagnostics = this._controlDiagnostics || this._makeControlDiagnostics('unavailable');
        return Object.freeze({
            ...diagnostics,
            rawAcceleration: Object.freeze({ ...diagnostics.rawAcceleration }),
            limitedAcceleration: Object.freeze({ ...diagnostics.limitedAcceleration }),
            requestedForce: Object.freeze({ ...diagnostics.requestedForce }),
            allocatedForce: Object.freeze({ ...diagnostics.allocatedForce }),
            saturation: Object.freeze({ ...diagnostics.saturation }),
            antiWindup: Object.freeze({ ...diagnostics.antiWindup }),
            actualState: diagnostics.actualState ? Object.freeze({
                ...diagnostics.actualState,
                position: Object.freeze({ ...diagnostics.actualState.position }),
                velocity: Object.freeze({ ...diagnostics.actualState.velocity }),
            }) : null,
            referenceState: diagnostics.referenceState ? Object.freeze({ ...diagnostics.referenceState }) : null,
            ...(diagnostics.attitudeRateSetpoint ? {
                attitudeRateSetpoint: Object.freeze({ ...diagnostics.attitudeRateSetpoint }),
            } : {}),
        });
    }

    handleControlOverrun({ droppedSeconds = 0, acceptedSeconds = 0.1 } = {}) {
        const previous = this._controlDiagnostics || this._makeControlDiagnostics();
        this._controlDiagnostics = {
            ...previous,
            overrunCount: previous.overrunCount + 1,
            overrunDroppedSeconds: previous.overrunDroppedSeconds + Math.max(0, Number(droppedSeconds) || 0),
            fallbackReason: 'control-overrun',
        };
        this._controlOverrunHoldRemaining = Math.max(
            this._controlOverrunHoldRemaining,
            Math.max(0.1, Number(acceptedSeconds) || 0),
        );
        this._latchSo3Hold('control-overrun', ControlCommandType.FAILSAFE_HOLD);
        if (this.flightMode === 'so3' && this._trajectory.active) {
            this._trajectory.clear('control-overrun');
            this._navigationTransitionReason = 'control-overrun';
            this._replanRequested = true;
        }
    }

    _latchSo3Hold(reason = 'hold', commandType = null) {
        this._so3Hold = { x: this.x, y: this.y, z: this.z, yawDeg: this.yaw };
        this._so3YawSetpointDeg = this.yaw;
        this._so3HoldCommandType = commandType || (FAILSAFE_HOLD_REASONS.has(reason)
            ? ControlCommandType.FAILSAFE_HOLD
            : ControlCommandType.POSITION_VELOCITY_HOLD);
        if (this._controlDiagnostics) this._controlDiagnostics.fallbackReason = reason;
    }

    /** Set ideal goal (point-to-point, no YOPO trajectory). */
    setIdealGoal(goal) {
        // A goal change starts a new navigation generation.  Never keep
        // tracking a polynomial generated for the prior
        // goal while the next perception response is still in flight.
        this._clearYopoMotionState();
        this._idealGoal = goal ? { x: goal.x, y: goal.y, z: goal.z, yaw: goal.yaw } : null;
        this._so3FixedYaw = goal ? this.yaw : null;
        this._so3YawSetpointDeg = this.yaw;
        this._latchSo3Hold(goal ? 'awaiting-trajectory' : 'goal-cleared');
        this._navigationState = goal ? 'active' : 'idle';
        this._navigationTransitionReason = goal ? 'goal-set' : 'goal-cleared';
    }

    clearIdealGoal() {
        // Public compatibility API: clearing a goal has the same safety
        // boundary as setIdealGoal(null). An active polynomial must never
        // outlive the navigation target that authorized it.
        this.setIdealGoal(null);
    }

    _clearYopoMotionState() {
        this._yopoPlanTriggered = false;
        this._trajectory.clear('motion-cleared');
        this._replanRequested = false;
    }

    /** Cancel current waypoint without teleporting physical velocity. */
    cancelWaypoint() {
        this._idealGoal = null;
        this._clearYopoMotionState();
        this._replanRequested = false;
        this._latchSo3Hold('cancelled');
        this._navigationState = 'cancelled';
        this._navigationTransitionReason = 'cancelled';
    }

    /** 载入 YOPO 轨迹末端状态 → 拟合五次多项式。 */
    setYopoTrajectory(endpoint, trajTime, context = null) {
        // endpoint 采用**轴主序** [px,vx,ax, py,vy,ay, pz,vz,az]（sim 世界系）。
        // 这与 yopo_bridge.py 的输出、以及参考实现 test_yopo_ros.py 的
        // endstate_w[id, axis, order] 一致 —— 每个轴连续排布 位置/速度/加速度。
        // 切勿改成量主序 [px,py,pz, vx,vy,vz, ...]：那会把高度值填进 X 轴的
        // 终端速度、把加速度填进 Z 轴的终点位置，产生发散的参考轨迹。
        // 契约由 tests/test_yopo_endstate_layout.js 锁定。
        const previousReference = this._trajectory.active
            ? this._trajectory.referenceAt(this._trajectory.time)
            : this._trajectory.lastReference;
        const duration = trajTime == null ? 1.125 : Number(trajTime);
        const providedContext = context && typeof context === 'object' ? context : {};
        const commandContext = Object.freeze({
            ...providedContext,
            source: typeof providedContext.source === 'string' ? providedContext.source : 'yopo',
            frame: 'sim-world-y-up',
            generation: providedContext.generation ?? null,
            createdSimTime: this._simTimeS,
            expirySimTime: this._simTimeS + (Number.isFinite(duration) ? duration : 0),
        });
        const checked = this._trajectory.install(endpoint, trajTime, {
            x: this.x, y: this.y, z: this.z,
            vx: this.vx, vy: this.vy, vz: this.vz,
            ax: previousReference?.ax ?? 0,
            ay: previousReference?.ay ?? 0,
            az: previousReference?.az ?? 0,
        }, commandContext);
        if (!checked.valid) {
            const now = globalThis.performance?.now?.() ?? Date.now();
            if (this._lastYopoRejectReason !== checked.reason || now - (this._lastYopoRejectAt || 0) > 1000) {
                console.warn(`[YOPO] rejected trajectory: ${checked.reason}`);
                this._lastYopoRejectReason = checked.reason;
                this._lastYopoRejectAt = now;
            }
            this._trajectory.clear('trajectory-rejected');
            this._latchSo3Hold('trajectory-rejected');
            this._replanRequested = true;
            return false;
        }
        // 轨迹交接使用旧轨迹当前期望加速度作为新 Poly5 的初值，
        // 避免重规划边界上的加速度跳变。
        this._yopoPlanTriggered = true;  // 标记已有轨迹到达，允许到达判定
        this._replanRequested = false;
        this._so3HoldCommandType = ControlCommandType.POSITION_VELOCITY_HOLD;
        this._navigationState = 'active';
        // 不清除 _idealGoal —— 目标是持久导航参考，轨迹是对它的连续逼近。
        // 到达判断依据实际位置，不在轨迹开始时丢弃目标。
        return true;
    }

    setSpawnPoint(x, y, z) {
        this._spawnX = x; this._spawnY = y; this._spawnZ = z;
        this.reset();
    }

    reset() {
        // 清除所有导航/轨迹状态 —— reset 可能发生在重选出生点后，
        // 旧坐标系下的多项式、目标、高度参考在新坐标系里毫无意义。
        this._idealGoal = null;
        this._trajectory.clear('reset');
        this._yopoPlanTriggered = false;
        this._navigationState = 'idle';
        this._navigationTransitionReason = 'reset';
        this._replanRequested = false;
        this._so3FixedYaw = null;

        this.x = this._spawnX; this.y = this._spawnY; this.z = this._spawnZ;
        this.vx = 0; this.vy = 0; this.vz = 0;
        this.orientation.set(0, 0, 0, 1); // identity
        this.pitchRate = 0; this.rollRate = 0; this.yawRate = 0;
        this.pitch = 0; this.roll = 0; this.yaw = 0;
        this.isColliding = false;
        this.collisionIntensity = 0;
        this.thrustOutput = 0;
        this.throttlePercent = 0;
        this.speed = 0;
        this.groundSpeed = 0;
        this.airSpeed = 0;
        this.verticalSpeed = 0;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = this.droneMaxSpeed;
        this.boostActive = false;
        this.boostMultiplier = 1.0;
        this._targetX = this._spawnX; this._targetY = this._spawnY; this._targetZ = this._spawnZ;
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        this._simTimeS = 0;
        this._easyYawSetpointDeg = 0;
        this._levelYawSetpointDeg = 0;
        this._so3YawSetpointDeg = 0;
        this._so3YawRateSetpointDeg = 0;
        this._so3Hold = { x: this.x, y: this.y, z: this.z, yawDeg: this.yaw };
        this._so3HoldCommandType = ControlCommandType.POSITION_VELOCITY_HOLD;
        this._controlOverrunHoldRemaining = 0;
        this._easyHorizontalState = 'hold';
        this._easyVerticalState = 'hold';
        this._easyHorizontalBrakeRamp = false;
        this._easyVelocitySetpoint = { x: 0, y: 0, z: 0 };
        this._easyAccelerationSetpoint = { x: 0, y: 0, z: 0 };
        this._easyLimitedAcceleration = { x: 0, y: 0, z: 0 };
        this._measuredAcceleration = { x: 0, y: 0, z: 0 };
        this._controlDiagnostics = this._makeControlDiagnostics('reset');
        this._lastControlCommand = Object.freeze({
            type: ControlCommandType.FAILSAFE_HOLD,
            source: 'controller',
            frame: 'sim-world-y-up',
            generation: null,
            createdSimTime: 0,
            expirySimTime: null,
        });
    }

    readSettings() {
        const el = (id) => document.getElementById(id);
        const v = (id) => {
            const element = el(id);
            if (!element) return null;
            const value = Number(element.value);
            return Number.isFinite(value) ? value : null;
        };
        const massVal   = v('phys-mass');
        const thrustVal = v('phys-thrust');
        const cdVal     = v('phys-drag-cd');
        const areaVal   = v('phys-drag-area');
        const radiusVal = v('phys-collision-radius');
        const sizeVal   = v('phys-drone-size');
        const droneMaxSpeedVal  = v('drone-max-speed');
        const droneMaxVSpeedVal = v('drone-max-vspeed');
        const modeEl    = el('flight-mode-select');
        const posKp = v('ctrl-pos-kp');
        const posKi = v('ctrl-pos-ki');
        const velKp = v('ctrl-vel-kp');
        const velKi = v('ctrl-vel-ki');
        const altKp = v('ctrl-alt-kp');
        const altKi = v('ctrl-alt-ki');
        if (massVal !== null)   this.mass = massVal;
        if (thrustVal !== null) this.maxThrust = thrustVal;
        if (cdVal !== null)     this.dragCd = cdVal;
        if (areaVal !== null)   this.dragArea = areaVal;
        if (radiusVal !== null) this.collisionRadius = radiusVal;
        if (sizeVal !== null)   this.droneSize = sizeVal;
        if (droneMaxSpeedVal !== null) {
            this.droneMaxSpeed = Math.max(1, Math.min(DRONE_MAX_SUPPORTED_SPEED, droneMaxSpeedVal));
        }
        if (droneMaxVSpeedVal !== null) {
            this.droneMaxVSpeed = Math.max(1, Math.min(DRONE_MAX_SUPPORTED_VSPEED, droneMaxVSpeedVal));
        }
        if (modeEl) this.flightMode = modeEl.value;
        const mountAngle = v('cam-mount-angle');
        if (mountAngle !== null) this.cameraMountAngle = mountAngle;
        const posKd = v('ctrl-pos-kd');
        const velKd = v('ctrl-vel-kd');
        const altKd = v('ctrl-alt-kd');
        if (posKp !== null) this.dronePosKp = posKp;
        if (posKi !== null) this.dronePosKi = posKi;
        if (posKd !== null) this.dronePosKd = posKd;
        if (velKp !== null) this.droneVelKp = velKp;
        if (velKi !== null) this.droneVelKi = velKi;
        if (velKd !== null) this.droneVelKd = velKd;
        if (altKp !== null) this.droneAltKp = altKp;
        if (altKi !== null) this.droneAltKi = altKi;
        if (altKd !== null) this.droneAltKd = altKd;

    }

    /** 到达目标后刹车：对齐参考 test_yopo_ros.py 的 TRAJECTORY_STATUS_EMPTY + 位置保持 */
    _onArrival() {
        this._trajectory.clear('arrival');
        this._latchSo3Hold('arrival');
        this._replanRequested = false;
        this._idealGoal = null;
        this._navigationState = 'arrived';
        this._navigationTransitionReason = 'arrival-distance';
    }

    update(dt, input, collisionProvider) {
        dt = Math.min(Math.max(Number(dt) || 0, 0), 0.05);
        if (dt <= 0) return;
        const suppliedInput = input || { armed: false };
        const suppliedRates = suppliedInput.rates || {};
        const invalidControlInput = ['pitch', 'roll', 'yaw', 'throttle'].some(
            axis => suppliedInput[axis] != null && !Number.isFinite(Number(suppliedInput[axis])),
        ) || ['pitch', 'roll', 'yaw'].some(
            axis => suppliedRates[axis] != null && !Number.isFinite(Number(suppliedRates[axis])),
        );
        input = {
            ...suppliedInput,
            pitch: Number.isFinite(Number(suppliedInput.pitch)) ? Number(suppliedInput.pitch) : 0,
            roll: Number.isFinite(Number(suppliedInput.roll)) ? Number(suppliedInput.roll) : 0,
            yaw: Number.isFinite(Number(suppliedInput.yaw)) ? Number(suppliedInput.yaw) : 0,
            throttle: Number.isFinite(Number(suppliedInput.throttle)) ? Number(suppliedInput.throttle) : 0,
            rates: {
                pitch: Number.isFinite(Number(suppliedRates.pitch)) ? Number(suppliedRates.pitch) : 1,
                roll: Number.isFinite(Number(suppliedRates.roll)) ? Number(suppliedRates.roll) : 1,
                yaw: Number.isFinite(Number(suppliedRates.yaw)) ? Number(suppliedRates.yaw) : 1,
            },
        };
        this._simTimeS += dt;
        const velocityBeforeStep = { x: this.vx, y: this.vy, z: this.vz };

        // 0a. 到达判定。radio_range=9m 是网络格点范围，不是到达容差。
        // 到达半径与外部 YOPO 参考实现统一为 4m。
        if (this._idealGoal) {
            const g = this._idealGoal;
            const d = Math.sqrt(
                (g.x - this.x) ** 2 +
                ((g.y != null ? g.y : this.y) - this.y) ** 2 +
                (g.z - this.z) ** 2
            );
            if (d < this._arrivalDistanceM) {
                this._onArrival();
            }
        }

        // 0b. Handle flight-mode transitions (M key, RC channel, or dropdown).
        // readSettings() has already copied the latest dropdown value into
        // this.flightMode for this frame, so comparing against the cached
        // previous value detects a change on the first frame it becomes
        // effective.
        if (this.flightMode !== this._prevFlightMode) {
            this._onFlightModeChanged(this._prevFlightMode, this.flightMode);
            this._prevFlightMode = this.flightMode;
        }

        // 1. Control law → updates orientation quaternion and thrustOutput
        if (!input.armed) {
            this._updateDisarmed(dt);
        } else if (invalidControlInput) {
            this._controlFailsafeHold(dt, 'non-finite-command');
        } else if (this._controlOverrunHoldRemaining > 0) {
            this._controlOverrunHoldRemaining = Math.max(0, this._controlOverrunHoldRemaining - dt);
            this._controlFailsafeHold(dt, 'control-overrun');
        } else if (this.flightMode === 'stabilized') {
            this._controlStabilized(dt, input, collisionProvider);
        } else if (this.flightMode === 'so3') {
            this._controlSO3(dt, input, collisionProvider);
        } else if (this.flightMode === 'drone') {
            this._controlDrone(dt, input);
        } else if (this.flightMode === 'fpv') {
            this._controlFPV(dt, input);
        } else {
            this._controlFailsafeHold(dt, 'unknown-flight-mode');
        }

        // 2. Extract rotation matrix from orientation
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);

        // Local up = Y column of rotation matrix
        _mat4.getY(_v3);
        const upX = _v3.x, upY = _v3.y, upZ = _v3.z;

        // 3. Forces: thrust along local up + gravity + quadratic drag
        const massG = Math.max(this.mass, 1); // guard against zero mass
        const massKg = massG / 1000;
        // thrustOutput is in grams-force; convert to acceleration: (gf / g_mass) * G
        const thrustAccel = (this.thrustOutput / massG) * G;
        let ax = upX * thrustAccel;
        let ay = upY * thrustAccel - G;
        let az = upZ * thrustAccel;

        // Quadratic drag: F = 0.5 * Cd * A * rho * v^2, a = F / m
        const spd = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
        if (spd > 0.001) {
            const dragForce = 0.5 * this.dragCd * this.dragArea * AIR_DENSITY * spd * spd;
            const dragAccel = dragForce / massKg;
            ax -= (this.vx / spd) * dragAccel;
            ay -= (this.vy / spd) * dragAccel;
            az -= (this.vz / spd) * dragAccel;
        }

        const previousPosition = { x: this.x, y: this.y, z: this.z };

        // 4. Integrate velocity & position
        this.vx += ax * dt;
        this.vy += ay * dt;
        this.vz += az * dt;
        this.x += this.vx * dt;
        this.y += this.vy * dt;
        this.z += this.vz * dt;

        // NaN guard — reset if physics blew up
        if (!Number.isFinite(this.x) || !Number.isFinite(this.y) || !Number.isFinite(this.z) ||
            !Number.isFinite(this.vx) || !Number.isFinite(this.vy) || !Number.isFinite(this.vz)) {
            reportUserError(
                'Drone physics produced invalid state; resetting',
                new Error(`pos=${this.x},${this.y},${this.z}, vel=${this.vx},${this.vy},${this.vz}, mass=${this.mass}, thrust=${this.thrustOutput}, dragCd=${this.dragCd}, dragArea=${this.dragArea}`),
                { key: 'drone-physics-nan', intervalMs: 10000 }
            );
            this.reset();
            return;
        }

        // 5. Collisions
        this._handleCollisions(collisionProvider, previousPosition, dt);

        this._measuredAcceleration = {
            x: (this.vx - velocityBeforeStep.x) / dt,
            y: (this.vy - velocityBeforeStep.y) / dt,
            z: (this.vz - velocityBeforeStep.z) / dt,
        };

        // 6. Derive euler angles for HUD
        this._updateEulerFromQuat();
        this.groundSpeed = Math.sqrt(this.vx * this.vx + this.vz * this.vz);
        this.airSpeed = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
        this.speed = this.groundSpeed;
        this.verticalSpeed = this.vy;
        if (this._controlDiagnostics) {
            this._controlDiagnostics.simTimeS = this._simTimeS;
            this._controlDiagnostics.thrustGf = this.thrustOutput;
            // Publish the state after the common rigid-body/collision step so
            // diagnostics never mix a post-step timestamp with pre-step state.
            this._controlDiagnostics.actualState = {
                position: { x: this.x, y: this.y, z: this.z },
                velocity: { x: this.vx, y: this.vy, z: this.vz },
            };
        }
    }

    getCameraTransform() {
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);

        // Local forward = -Z column
        _mat4.getZ(_v3);
        _v3.mulScalar(-1);
        const halfSize = this.droneSize * 0.5;

        // Camera mount pitch offset (body-frame X rotation)
        const mountDeg = this.flightMode === 'fpv' ? this.cameraMountAngle : this.cameraTiltAngle;
        const mountRad = mountDeg * DEG2RAD * 0.5;
        _quat.set(Math.sin(mountRad), 0, 0, Math.cos(mountRad));
        _quat2.copy(this.orientation).mul(_quat);

        // Extract euler angles from camera orientation (with mount offset)
        const euler = this._quatToEuler(_quat2);

        return {
            position: {
                x: this.x + _v3.x * halfSize,
                y: this.y + _v3.y * halfSize,
                z: this.z + _v3.z * halfSize
            },
            rotation: { x: euler.x, y: euler.y, z: euler.z },
            orientation: { x: _quat2.x, y: _quat2.y, z: _quat2.z, w: _quat2.w }
        };
    }

    getPanoramaTransform() {
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        _v3.mulScalar(-1);
        const noseOffset = this.droneSize * 0.5;

        return {
            position: {
                x: this.x + _v3.x * noseOffset,
                y: this.y + _v3.y * noseOffset,
                z: this.z + _v3.z * noseOffset
            },
            rotation: { x: this.pitch, y: this.yaw, z: this.roll },
            orientation: {
                x: this.orientation.x,
                y: this.orientation.y,
                z: this.orientation.z,
                w: this.orientation.w
            }
        };
    }

    /** 水平校正的全景相机位姿。
     *  从完整四元数中提取 body forward 在水平面(XZ)上的投影方向，
     *  据此重建 yaw-only 四元数。不依赖 Euler 角度分解，对任意
     *  pitch/roll 组合都能给出正确的水平朝向。
     *  位置也基于 yaw-only 朝向计算，roll 时相机位置不左右摆动。 */
    getLeveledPanoramaTransform() {
        // 从完整四元数提取 body forward 在水平面上的投影
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);  // body Z (backward) 在世界系中的方向
        const bx = _v3.x, bz = _v3.z;  // backward 的水平分量
        // leveled forward = -backward 的水平归一化方向
        const { fwdX, fwdZ, yawRad } = leveledYawFromBackward(bx, bz);
        const halfYaw = yawRad * 0.5;
        const noseOffset = this.droneSize * 0.5;

        return {
            position: {
                x: this.x + fwdX * noseOffset,
                y: this.y,
                z: this.z + fwdZ * noseOffset
            },
            rotation: { x: 0, y: yawRad * RAD2DEG, z: 0 },
            orientation: { x: 0, y: Math.sin(halfYaw), z: 0, w: Math.cos(halfYaw) }
        };
    }

    /** Return fixed yaw if SO3 yaw-lock is active, else current yaw. */
    getFixedYaw() {
        const yawLockEl = globalThis.document?.getElementById?.('yaw-lock-toggle');
        const lockEnabled = yawLockEl ? yawLockEl.checked : true;
        return (lockEnabled && this._so3FixedYaw != null) ? this._so3FixedYaw : this.yaw;
    }

    getBodyTransform() {
        return {
            position: { x: this.x, y: this.y, z: this.z },
            rotation: { x: this.pitch, y: this.yaw, z: this.roll },
            orientation: {
                x: this.orientation.x,
                y: this.orientation.y,
                z: this.orientation.z,
                w: this.orientation.w
            }
        };
    }

    /**
     * Immutable state snapshot for one YOPO observation. The observation
     * origin follows the active polynomial reference while Poly5 construction
     * continues to start from the actual vehicle position and velocity.
     */
    getYopoPlanningState() {
        const actualState = {
            position: Object.freeze({ x: this.x, y: this.y, z: this.z }),
            velocity: Object.freeze({ x: this.vx, y: this.vy, z: this.vz }),
        };
        const activeRef = this._yopoPolyX
            ? this._getYopoReference(Math.min(this._yopoTrackerTime, this._yopoTrajTime))
            : null;
        const referenceState = {
            position: Object.freeze(activeRef
                ? { x: activeRef.x, y: activeRef.y, z: activeRef.z }
                : { x: this.x, y: this.y, z: this.z }),
            velocity: Object.freeze(activeRef
                ? { x: activeRef.vx, y: activeRef.vy, z: activeRef.vz }
                : { x: this.vx, y: this.vy, z: this.vz }),
            acceleration: Object.freeze(activeRef
                ? { x: activeRef.ax, y: activeRef.ay, z: activeRef.az }
                : { x: 0, y: 0, z: 0 }),
        };
        return Object.freeze({
            actualState: Object.freeze(actualState),
            referenceState: Object.freeze(referenceState),
            yaw: this.getFixedYaw(),
        });
    }

    consumeNavigationTransition() {
        const transition = this._navigationTransitionReason
            ? { state: this._navigationState, reason: this._navigationTransitionReason }
            : null;
        this._navigationTransitionReason = null;
        return transition;
    }

    adjustCameraTilt(delta) {
        this.cameraTiltAngle = Math.max(-90, Math.min(0, this.cameraTiltAngle + delta));
    }

    // ---- Orientation helpers ----

    /**
     * Apply an incremental body-frame rotation.
     * bodyAxis: 'x' (pitch), 'y' (yaw), or 'z' (roll)
     * angleDeg: rotation in degrees
     *
     * Body-frame: orientation = orientation * deltaQuat
     * World-frame (yaw): orientation = deltaQuat * orientation
     */
    _applyBodyRotation(axisX, axisY, axisZ, angleDeg) {
        if (Math.abs(angleDeg) < 1e-8) return;
        const halfRad = (angleDeg * DEG2RAD) * 0.5;
        const s = Math.sin(halfRad);
        _quat.set(axisX * s, axisY * s, axisZ * s, Math.cos(halfRad));
        // Body frame: q_new = q_current * q_delta
        _quat2.copy(this.orientation).mul(_quat);
        this.orientation.copy(_quat2).normalize();
    }


    /**
     * Decompose orientation into yaw (world Y rotation) and body tilt.
     * Returns { yawDeg, bodyPitchDeg, bodyRollDeg }
     */
    _decomposeOrientation() {
        // Extract yaw from the local +Z column projected onto the XZ plane.
        // R_Y(yaw) maps (0,0,1) → (sinYaw, 0, cosYaw), so:
        //   sinYaw = localZ.x,  cosYaw = localZ.z
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3); // local +Z direction in world
        const yawRad = Math.atan2(_v3.x, _v3.z);
        const yawDeg = yawRad * RAD2DEG;

        // Build yaw-only quaternion
        const halfYaw = yawRad * 0.5;
        _quat.set(0, Math.sin(halfYaw), 0, Math.cos(halfYaw));

        // Body tilt = inverse(yawQuat) * orientation
        _quat2.copy(_quat).invert().mul(this.orientation);

        // Extract pitch and roll from the tilt quaternion
        // tiltQuat represents R_X(pitch) * R_Z(roll) approximately
        const tiltEuler = new pc.Vec3();
        _quat2.getEulerAngles(tiltEuler);

        return {
            yawDeg: yawDeg,
            bodyPitchDeg: tiltEuler.x,
            bodyRollDeg: tiltEuler.z
        };
    }

    _updateEulerFromQuat() {
        const e = new pc.Vec3();
        this.orientation.getEulerAngles(e);
        this.pitch = e.x;
        this.yaw   = e.y;
        this.roll  = e.z;

        // Yaw-independent body tilt for OSD artificial horizon
        const dec = this._decomposeOrientation();
        this.bodyPitch = dec.bodyPitchDeg;
        this.bodyRoll  = dec.bodyRollDeg;
    }

    _quatToEuler(q) {
        const e = new pc.Vec3();
        q.getEulerAngles(e);
        return { x: e.x, y: e.y, z: e.z };
    }

    // ---- Control laws ----

    /**
     * Called once on the frame a flight-mode transition is detected.
     * Re-anchors position-hold + altitude-hold setpoints to the drone's
     * current state and clears controller integrator / filter memory so the
     * new mode does not fly toward stale targets or apply leftover control
     * effort accumulated during the previous mode.
     *
     * Note on orientation: we deliberately do NOT reset pitch/roll here.
     * Drone mode's tilt controller will naturally level the craft over a
     * few hundred ms from whatever attitude FPV left behind, which matches
     * the user-visible "roll and pitch switch to level" expectation. Yaw
     * is pure rate control and needs no reset.
     */
    _onFlightModeChanged(oldMode, newMode) {
        this._targetX = this.x;
        this._targetY = this.y;
        this._targetZ = this.z;
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        this._easyVelocitySetpoint = { x: this.vx, y: this.vy, z: this.vz };
        this._easyAccelerationSetpoint = { x: 0, y: 0, z: 0 };
        this._easyLimitedAcceleration = { x: 0, y: 0, z: 0 };
        this._easyHorizontalState = Math.hypot(this.vx, this.vz) < 0.5 ? 'hold' : 'brake';
        this._easyVerticalState = Math.abs(this.vy) < 0.2 ? 'hold' : 'brake';
        this._easyHorizontalBrakeRamp = false;
        this._easyYawSetpointDeg = this.yaw;
        this._levelYawSetpointDeg = this.yaw;
        // 离开 SO3 模式即结束本次导航会话。旧目标、轨迹与高度覆盖不能
        // 在之后切回 SO3 时悄悄恢复；main.js 会消费 mode-exit 并同步清 UI。
        if (oldMode === 'so3' && newMode !== 'so3') {
            this.cancelWaypoint();
            this._navigationTransitionReason = 'mode-exit';
        }
        // Lock yaw on entering SO3 mode (like YOPO lock_yaw=True)
        if (newMode === 'so3') {
            this._so3FixedYaw = this.yaw;
            this._so3YawSetpointDeg = this.yaw;
            this._so3YawRateSetpointDeg = 0;
            this._latchSo3Hold('mode-enter');
        } else {
            this._so3FixedYaw = null;
        }
    }

    _updateDisarmed(dt) {
        this.thrustOutput = 0;
        this.throttlePercent = 0;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = this.flightMode === 'drone' ? this.droneMaxSpeed : null;
        this.boostActive = false;
        this.boostMultiplier = 1.0;
        // Damp angular rates
        const damp = Math.exp(-this.angularDrag * dt);
        this.pitchRate *= damp;
        this.rollRate  *= damp;
        this.yawRate   *= damp;

        // Auto-level toward identity tilt (keep current yaw)
        const dec = this._decomposeOrientation();
        const levelSpeed = 60; // deg/s
        const pitchStep = Math.min(levelSpeed * dt, Math.abs(dec.bodyPitchDeg));
        const rollStep  = Math.min(levelSpeed * dt, Math.abs(dec.bodyRollDeg));

        if (pitchStep > 0.01) {
            this._applyBodyRotation(1, 0, 0, -Math.sign(dec.bodyPitchDeg) * pitchStep);
        }
        if (rollStep > 0.01) {
            this._applyBodyRotation(0, 0, 1, -Math.sign(dec.bodyRollDeg) * rollStep);
        }
    }

    _assistedAxis(input, axis) {
        const raw = Number(input?.rawAxes?.[axis]);
        if (!Number.isFinite(raw)) return Number(input?.[axis]) || 0;
        const config = input?.axisConfig?.[axis] || {};
        return shapeAssistedAxis(raw, Number(config.deadzone) || 0, Number(config.expo) || 0);
    }

    _bodyBasis() {
        const quaternion = {
            x: this.orientation.x,
            y: this.orientation.y,
            z: this.orientation.z,
            w: this.orientation.w,
        };
        const right = rotateVectorByQuaternion(quaternion, { x: 1, y: 0, z: 0 });
        const up = rotateVectorByQuaternion(quaternion, { x: 0, y: 1, z: 0 });
        const backward = rotateVectorByQuaternion(quaternion, { x: 0, y: 0, z: 1 });
        let forward = { x: -backward.x, y: 0, z: -backward.z };
        const forwardNorm = Math.hypot(forward.x, forward.z);
        forward = forwardNorm > 1e-6
            ? { x: forward.x / forwardNorm, y: 0, z: forward.z / forwardNorm }
            : { x: 0, y: 0, z: -1 };
        // Use one yaw-only heading to construct both horizontal axes. Projecting
        // and normalizing tilted body X/Z independently makes them non-orthogonal
        // and can amplify a diagonal Easy command beyond its speed limit.
        // The simulator's geographic frame is x=east, y=up, z=north.  Its
        // identity heading points south (-Z), so body-right is west (-X).
        // Keep this yaw-only vector consistent with the keyboard contract
        // (negative roll = left, positive roll = right) and with the depth
        // projection's body-left convention.
        const horizontalRight = { x: forward.z, y: 0, z: -forward.x };
        return { right, up, backward, forward, horizontalRight };
    }

    _trackDesiredAttitude(dt, desiredQuaternion, rateLimitsDeg = null) {
        const currentQuaternion = {
            x: this.orientation.x,
            y: this.orientation.y,
            z: this.orientation.z,
            w: this.orientation.w,
        };
        const targetRate = reducedQuaternionBodyRateSetpoint(
            currentQuaternion,
            desiredQuaternion,
            rateLimitsDeg ? { rateLimitsDeg } : {},
        );
        const currentRate = {
            x: this.pitchRate * DEG2RAD,
            y: this.yawRate * DEG2RAD,
            z: this.rollRate * DEG2RAD,
        };
        const actualRate = firstOrderRateServo(currentRate, targetRate, dt, 15);
        const nextQuaternion = integrateBodyRates(currentQuaternion, actualRate, dt);
        this.orientation.set(
            nextQuaternion.x,
            nextQuaternion.y,
            nextQuaternion.z,
            nextQuaternion.w,
        );
        this.pitchRate = actualRate.x * RAD2DEG;
        this.yawRate = actualRate.y * RAD2DEG;
        this.rollRate = actualRate.z * RAD2DEG;
        return { targetRate, actualRate };
    }

    _applyForceAttitude(dt, force, yawRad, maxThrustN, rateLimitsDeg = null) {
        const desiredQuaternion = desiredAttitudeFromForce(force, yawRad);
        const rates = this._trackDesiredAttitude(dt, desiredQuaternion, rateLimitsDeg);
        // Projection uses the exact same post-servo attitude that the common
        // translational plant uses below; no cached pre-update body axis.
        const bodyUp = this._bodyBasis().up;
        const thrustN = Math.max(0, Math.min(
            maxThrustN,
            force.x * bodyUp.x + force.y * bodyUp.y + force.z * bodyUp.z,
        ));
        this.thrustOutput = thrustN / G * 1000;
        this.throttlePercent = maxThrustN > 0 ? thrustN / maxThrustN : 0;
        return { desiredQuaternion, bodyUp, thrustN, ...rates };
    }

    _setControlDiagnostics(values) {
        const previous = this._controlDiagnostics || this._makeControlDiagnostics();
        this._controlDiagnostics = {
            ...this._makeControlDiagnostics(null),
            overrunCount: previous.overrunCount,
            overrunDroppedSeconds: previous.overrunDroppedSeconds,
            ...values,
            simTimeS: this._simTimeS,
            thrustGf: this.thrustOutput,
            actualState: {
                position: { x: this.x, y: this.y, z: this.z },
                velocity: { x: this.vx, y: this.vy, z: this.vz },
            },
        };
        this._lastControlCommand = Object.freeze({
            type: this._controlDiagnostics.commandType,
            source: this._controlDiagnostics.source,
            frame: this._controlDiagnostics.frame,
            generation: this._controlDiagnostics.generation,
            createdSimTime: this._controlDiagnostics.createdSimTime,
            expirySimTime: this._controlDiagnostics.expirySimTime,
        });
    }

    _controlFailsafeHold(dt, reason) {
        this._so3HoldCommandType = ControlCommandType.FAILSAFE_HOLD;
        const invalidatesActiveAutoTrajectory = this.flightMode === 'so3' && this._trajectory.active;
        if (!this._so3Hold || this._controlDiagnostics?.fallbackReason !== reason) {
            if (this._trajectory.active) this._trajectory.clear(reason);
            this._latchSo3Hold(reason, ControlCommandType.FAILSAFE_HOLD);
        }
        if (invalidatesActiveAutoTrajectory) {
            this._navigationTransitionReason = reason;
            this._replanRequested = true;
        }
        this._controlPositionHold(dt, ControlCommandType.FAILSAFE_HOLD, reason);
    }

    _isYawLockEnabled() {
        const element = globalThis.document?.getElementById?.('yaw-lock-toggle');
        return element ? !!element.checked : true;
    }

    _wrapDegrees(value) {
        let wrapped = (value + 180) % 360;
        if (wrapped < 0) wrapped += 360;
        return wrapped - 180;
    }

    _controlFPV(dt, input) {
        const boost = input.boost ? FPV_BOOST_MULTIPLIER : 1.0;
        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };
        this.boostActive = !!input.boost;
        this.boostMultiplier = boost;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = null;

        // Sticks → target angular rates (body frame), scaled by rate
        const tPR = input.pitch * this.maxPitchRate * rates.pitch * boost;
        const tRR = input.roll * this.maxRollRate * rates.roll * boost;
        const tYR = input.yaw  * this.maxYawRate  * rates.yaw  * boost;

        // Smooth rate tracking
        const s = 1 - Math.exp(-15 * dt);
        this.pitchRate += (tPR - this.pitchRate) * s;
        this.rollRate  += (tRR - this.rollRate)  * s;
        this.yawRate   += (tYR - this.yawRate)   * s;

        // Damp when centered
        const ad = Math.exp(-this.angularDrag * dt);
        if (Math.abs(input.pitch) < 0.05) this.pitchRate *= ad;
        if (Math.abs(input.roll)  < 0.05) this.rollRate  *= ad;
        if (Math.abs(input.yaw)   < 0.05) this.yawRate   *= ad;

        // Preserve the legacy 60 Hz right-endpoint attitude trajectory while
        // the common plant now runs at 200 Hz. Stored rates keep the original
        // exact 15/s response; only the integration sample gets the small
        // deterministic phase lead that the coarser legacy step introduced.
        const legacyStep = 1 / 60;
        const legacyLag = legacyStep / Math.expm1(15 * legacyStep);
        const currentLag = dt / Math.expm1(15 * dt);
        const integrationLead = dt < legacyStep
            ? Math.max(0, Math.min(1, 1 - legacyLag / currentLag))
            : 0;
        const integratedPitchRate = this.pitchRate + (tPR - this.pitchRate) * integrationLead;
        const integratedRollRate = this.rollRate + (tRR - this.rollRate) * integrationLead;
        const integratedYawRate = this.yawRate + (tYR - this.yawRate) * integrationLead;

        if (dt < legacyStep) {
            // Fractional power of the legacy one-frame Rx·Rz·Ry delta. This
            // retains its non-commutative rotation order at 200 Hz instead of
            // accumulating a different path from three smaller Euler steps.
            const hp = integratedPitchRate * legacyStep * DEG2RAD * 0.5;
            const hr = integratedRollRate * legacyStep * DEG2RAD * 0.5;
            const hy = integratedYawRate * legacyStep * DEG2RAD * 0.5;
            const sp = Math.sin(hp), cp = Math.cos(hp);
            const sr = Math.sin(hr), cr = Math.cos(hr);
            const sy = Math.sin(hy), cy = Math.cos(hy);
            const fullX = sp * cr * cy - cp * sr * sy;
            const fullY = cp * cr * sy - sp * sr * cy;
            const fullZ = sp * cr * sy + cp * sr * cy;
            const fullW = cp * cr * cy + sp * sr * sy;
            const vectorLength = Math.hypot(fullX, fullY, fullZ);
            if (vectorLength > 1e-12) {
                const halfAngle = Math.atan2(vectorLength, fullW);
                const fraction = dt / legacyStep;
                const scale = Math.sin(halfAngle * fraction) / vectorLength;
                _quat.set(
                    fullX * scale,
                    fullY * scale,
                    fullZ * scale,
                    Math.cos(halfAngle * fraction),
                );
                _quat2.copy(this.orientation).mul(_quat);
                this.orientation.copy(_quat2).normalize();
            }
        } else {
            this._applyBodyRotation(1, 0, 0, integratedPitchRate * dt); // pitch around body X
            this._applyBodyRotation(0, 0, 1, integratedRollRate * dt);  // roll around body Z
            this._applyBodyRotation(0, 1, 0, integratedYawRate * dt);   // yaw around body Y
        }

        // Throttle → thrust (in grams-force)
        this.thrustOutput = ((input.throttle + 1) * 0.5) * this.maxThrust * boost;
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / (this.maxThrust * boost)))
            : 0;
        this._setControlDiagnostics({
            commandType: ControlCommandType.BODY_RATE_THRUST,
            source: 'fpv-stick',
            createdSimTime: this._simTimeS,
            expirySimTime: this._simTimeS + dt,
            fallbackReason: null,
        });
    }

    // ---- Unified ideal-controller implementation ----

    _updateEasyMotionSetpoint(targetVelocity, dt, horizontalAcceleration, verticalUp, verticalDown) {
        const currentVelocity = this._easyVelocitySetpoint;
        const currentAcceleration = this._easyAccelerationSetpoint;
        const responseTime = 0.35;
        const desiredHorizontal = limitVector({
            x: (targetVelocity.x - currentVelocity.x) / responseTime,
            y: 0,
            z: (targetVelocity.z - currentVelocity.z) / responseTime,
        }, horizontalAcceleration);
        const desiredVertical = Math.max(-verticalDown, Math.min(
            verticalUp,
            (targetVelocity.y - currentVelocity.y) / responseTime,
        ));

        const horizontalJerk = Math.max(0, horizontalAcceleration * 3);
        const accelerationDelta = limitVector({
            x: desiredHorizontal.x - currentAcceleration.x,
            y: 0,
            z: desiredHorizontal.z - currentAcceleration.z,
        }, horizontalJerk * dt);
        const verticalJerk = 3 * Math.max(verticalUp, verticalDown);
        const nextAcceleration = {
            x: currentAcceleration.x + accelerationDelta.x,
            y: currentAcceleration.y + Math.max(-verticalJerk * dt, Math.min(
                verticalJerk * dt,
                desiredVertical - currentAcceleration.y,
            )),
            z: currentAcceleration.z + accelerationDelta.z,
        };
        const nextVelocity = {
            x: currentVelocity.x + nextAcceleration.x * dt,
            y: currentVelocity.y + nextAcceleration.y * dt,
            z: currentVelocity.z + nextAcceleration.z * dt,
        };

        for (const axis of ['x', 'y', 'z']) {
            const before = targetVelocity[axis] - currentVelocity[axis];
            const after = targetVelocity[axis] - nextVelocity[axis];
            if (before === 0 || before * after <= 0) {
                nextVelocity[axis] = targetVelocity[axis];
                nextAcceleration[axis] = 0;
            }
        }
        this._easyVelocitySetpoint = nextVelocity;
        this._easyAccelerationSetpoint = nextAcceleration;
    }

    _controlDrone(dt, input) {
        const clamp = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, value));
        const finiteScale = (value) => Number.isFinite(Number(value)) ? Math.max(0, Number(value)) : 1;
        const rates = input.rates || {};
        const pitch = this._assistedAxis(input, 'pitch') * finiteScale(rates.pitch);
        const roll = this._assistedAxis(input, 'roll') * finiteScale(rates.roll);
        const throttle = this._assistedAxis(input, 'throttle');
        const yaw = this._assistedAxis(input, 'yaw') * finiteScale(rates.yaw);
        const horizontalStick = limitVector({ x: roll, y: 0, z: -pitch }, 1);
        const horizontalActive = Math.hypot(horizontalStick.x, horizontalStick.z) > 1e-4;
        const verticalActive = Math.abs(throttle) > 1e-4;
        const basis = this._bodyBasis();
        const boostScale = input.boost ? DRONE_BOOST_MULTIPLIER : 1;
        const maxSpeed = Math.min(
            DRONE_MAX_SUPPORTED_SPEED,
            Math.max(1, this.droneMaxSpeed) * boostScale,
        );
        const maxVerticalSpeed = Math.min(
            DRONE_MAX_SUPPORTED_VSPEED,
            Math.max(1, this.droneMaxVSpeed) * boostScale,
        );
        const horizontalPilotVelocity = limitVector({
            x: (basis.forward.x * horizontalStick.z + basis.horizontalRight.x * horizontalStick.x) * maxSpeed,
            y: 0,
            z: (basis.forward.z * horizontalStick.z + basis.horizontalRight.z * horizontalStick.x) * maxSpeed,
        }, maxSpeed);
        const pilotVelocity = {
            x: horizontalPilotVelocity.x,
            y: throttle * maxVerticalSpeed,
            z: horizontalPilotVelocity.z,
        };

        let targetVelocityX = 0;
        let targetVelocityZ = 0;
        if (horizontalActive) {
            this._easyHorizontalState = 'velocity';
            this._easyHorizontalBrakeRamp = false;
            targetVelocityX = pilotVelocity.x;
            targetVelocityZ = pilotVelocity.z;
            this._targetX = this.x;
            this._targetZ = this.z;
        } else {
            if (this._easyHorizontalState === 'velocity') this._easyHorizontalState = 'brake';
            if (this._easyHorizontalState === 'brake'
                && Math.hypot(this.vx, this.vz) < 0.5
                && Math.hypot(this._easyVelocitySetpoint.x, this._easyVelocitySetpoint.z) < 0.05
                && Math.hypot(this._easyLimitedAcceleration.x, this._easyLimitedAcceleration.z) < 0.1) {
                this._easyHorizontalState = 'hold';
                this._easyHorizontalBrakeRamp = false;
                this._targetX = this.x;
                this._targetZ = this.z;
            }
            if (this._easyHorizontalState === 'hold') {
                const holdVelocity = limitVector({
                    x: this.dronePosKp * (this._targetX - this.x),
                    y: 0,
                    z: this.dronePosKp * (this._targetZ - this.z),
                }, maxSpeed);
                targetVelocityX = holdVelocity.x;
                targetVelocityZ = holdVelocity.z;
            }
        }

        let targetVelocityY = 0;
        if (verticalActive) {
            this._easyVerticalState = 'velocity';
            targetVelocityY = pilotVelocity.y;
            this._targetY = this.y;
        } else {
            if (this._easyVerticalState === 'velocity') this._easyVerticalState = 'brake';
            if (this._easyVerticalState === 'brake'
                && Math.abs(this.vy) < 0.2
                && Math.abs(this._easyVelocitySetpoint.y) < 0.2) {
                this._easyVerticalState = 'hold';
                this._targetY = this.y;
            }
            if (this._easyVerticalState === 'hold') {
                targetVelocityY = clamp(this._targetY - this.y, -maxVerticalSpeed, maxVerticalSpeed);
            }
        }

        const massKg = Math.max(0.001, this.mass / 1000);
        const maxThrustN = Math.max(0, this.maxThrust * G / 1000);
        const maximumTotalAcceleration = maxThrustN / massKg;
        const thrustHorizontalAcceleration = Math.sqrt(Math.max(
            0,
            maximumTotalAcceleration ** 2 - G ** 2,
        ));
        const horizontalAcceleration = Math.max(0.5, Math.min(
            8,
            0.8 * G * Math.tan(45 * DEG2RAD),
            0.8 * thrustHorizontalAcceleration,
        ));
        const targetVelocity = { x: targetVelocityX, y: targetVelocityY, z: targetVelocityZ };
        this._updateEasyMotionSetpoint(targetVelocity, dt, horizontalAcceleration, 4, 3);

        const velocityError = {
            x: this._easyVelocitySetpoint.x - this.vx,
            y: this._easyVelocitySetpoint.y - this.vy,
            z: this._easyVelocitySetpoint.z - this.vz,
        };
        const requestedAcceleration = {
            x: this._easyAccelerationSetpoint.x
                + this.droneVelKp * velocityError.x
                + this.droneVelKi * this._velIntX
                - this.droneVelKd * this._measuredAcceleration.x,
            y: this._easyAccelerationSetpoint.y
                + this.droneAltKp * velocityError.y
                + this.droneAltKi * this._velIntY
                - this.droneAltKd * this._measuredAcceleration.y,
            z: this._easyAccelerationSetpoint.z
                + this.droneVelKp * velocityError.z
                + this.droneVelKi * this._velIntZ
                - this.droneVelKd * this._measuredAcceleration.z,
        };
        let targetHorizontalAcceleration = limitVector({
            x: requestedAcceleration.x,
            y: 0,
            z: requestedAcceleration.z,
        }, horizontalAcceleration);
        const priorLimitedAcceleration = this._easyLimitedAcceleration;
        const horizontalJerkLimit = horizontalAcceleration * 3;
        if (this._easyHorizontalState === 'brake') {
            const actualSpeed = Math.hypot(this.vx, this.vz);
            const priorAccelerationNorm = Math.hypot(
                priorLimitedAcceleration.x,
                priorLimitedAcceleration.z,
            );
            const accelerationOpposesVelocity = this.vx * priorLimitedAcceleration.x
                + this.vz * priorLimitedAcceleration.z < 0;
            const rampDownVelocity = priorAccelerationNorm ** 2
                / Math.max(1e-9, 2 * horizontalJerkLimit);
            if (accelerationOpposesVelocity
                // The ideal plant still needs finite attitude-servo time to
                // rotate the thrust axis back to level. Begin the jerk ramp
                // before the pure translational stopping boundary so that
                // residual tilt cannot create a second speed hump.
                && actualSpeed <= rampDownVelocity + 1.0) {
                if (!this._easyHorizontalBrakeRamp) {
                    // Tracking ARW handles actuator saturation, while this is
                    // an intentional jerk-ramp state transition. Discard the
                    // velocity-loop memory that would otherwise re-accelerate
                    // the craft as soon as HOLD is latched.
                    this._velIntX = 0;
                    this._velIntZ = 0;
                }
                this._easyHorizontalBrakeRamp = true;
            }
            if (this._easyHorizontalBrakeRamp) {
                targetHorizontalAcceleration = { x: 0, y: 0, z: 0 };
            }
        }
        const horizontalCommandStep = limitVector({
            x: targetHorizontalAcceleration.x - priorLimitedAcceleration.x,
            y: 0,
            z: targetHorizontalAcceleration.z - priorLimitedAcceleration.z,
        }, horizontalJerkLimit * dt);
        const targetVerticalAcceleration = clamp(requestedAcceleration.y, -3, 4);
        const verticalCommandStep = clamp(
            targetVerticalAcceleration - priorLimitedAcceleration.y,
            -12 * dt,
            12 * dt,
        );
        const limitedAcceleration = {
            x: priorLimitedAcceleration.x + horizontalCommandStep.x,
            y: priorLimitedAcceleration.y + verticalCommandStep,
            z: priorLimitedAcceleration.z + horizontalCommandStep.z,
        };
        this._easyLimitedAcceleration = limitedAcceleration;
        const requestedForce = {
            x: massKg * limitedAcceleration.x,
            y: massKg * (G + limitedAcceleration.y),
            z: massKg * limitedAcceleration.z,
        };
        const tiltLimited = limitTiltPreservingGravity(
            requestedForce.x,
            requestedForce.y,
            requestedForce.z,
            massKg * G,
            45,
        );
        const allocation = allocateEasyForce(tiltLimited, maxThrustN, {
            horizontalMarginFraction: 0.3,
            fallbackForce: { x: 0, y: Math.min(massKg * G, maxThrustN), z: 0 },
        });

        const producedAcceleration = {
            x: allocation.force.x / massKg,
            y: allocation.force.y / massKg - G,
            z: allocation.force.z / massKg,
        };
        const horizontalAntiWindup = allocation.saturatedHorizontal
            || Math.hypot(
                producedAcceleration.x - limitedAcceleration.x,
                producedAcceleration.z - limitedAcceleration.z,
            ) > 1e-4;
        const arwGain = 2 / Math.max(0.1, this.droneVelKp);
        const integralErrorX = horizontalAntiWindup
            ? velocityError.x - arwGain * (limitedAcceleration.x - producedAcceleration.x)
            : velocityError.x;
        const integralErrorZ = horizontalAntiWindup
            ? velocityError.z - arwGain * (limitedAcceleration.z - producedAcceleration.z)
            : velocityError.z;
        this._velIntX = clamp(this._velIntX + integralErrorX * dt, -20, 20);
        this._velIntZ = clamp(this._velIntZ + integralErrorZ * dt, -20, 20);
        const verticalCommandSaturated = Math.abs(
            requestedAcceleration.y - limitedAcceleration.y,
        ) > 1e-4;
        const verticalActuatorSaturated = Math.abs(
            limitedAcceleration.y - producedAcceleration.y,
        ) > 1e-4;
        const verticalSaturated = verticalCommandSaturated || verticalActuatorSaturated;
        const verticalSameDirection = (verticalCommandSaturated
            && ((requestedAcceleration.y > limitedAcceleration.y && velocityError.y > 0)
                || (requestedAcceleration.y < limitedAcceleration.y && velocityError.y < 0)))
            || (verticalActuatorSaturated
                && ((limitedAcceleration.y > producedAcceleration.y && velocityError.y > 0)
                    || (limitedAcceleration.y < producedAcceleration.y && velocityError.y < 0)));
        if (!verticalSameDirection) {
            this._velIntY = clamp(this._velIntY + velocityError.y * dt, -10, 10);
        }

        this._easyYawSetpointDeg += clamp(yaw, -1, 1) * this.droneMaxYawRate * dt;
        const attitude = this._applyForceAttitude(
            dt,
            allocation.force,
            this._easyYawSetpointDeg * DEG2RAD,
            maxThrustN,
            { roll: 220, pitch: 220, yaw: 120 },
        );
        const tiltDeg = Math.atan2(
            Math.hypot(allocation.force.x, allocation.force.z),
            Math.max(1e-9, allocation.force.y),
        ) * RAD2DEG;

        this.boostActive = !!input.boost;
        this.boostMultiplier = 1;
        this.effectiveMaxSpeed = maxSpeed;
        this.commandedGroundSpeed = Math.hypot(
            this._easyVelocitySetpoint.x,
            this._easyVelocitySetpoint.z,
        );
        this.targetGroundSpeed = Math.hypot(targetVelocityX, targetVelocityZ);
        this.pilotGroundSpeedCommand = Math.hypot(pilotVelocity.x, pilotVelocity.z);
        this._setControlDiagnostics({
            commandType: ControlCommandType.POSITION_VELOCITY_HOLD,
            source: 'easy-stick-hold',
            createdSimTime: this._simTimeS,
            expirySimTime: this._simTimeS + dt,
            referenceState: {
                x: this._targetX, y: this._targetY, z: this._targetZ,
                vx: this._easyVelocitySetpoint.x,
                vy: this._easyVelocitySetpoint.y,
                vz: this._easyVelocitySetpoint.z,
            },
            rawAcceleration: requestedAcceleration,
            limitedAcceleration,
            requestedForce,
            allocatedForce: allocation.force,
            tiltDeg,
            saturation: {
                horizontal: horizontalAntiWindup,
                vertical: verticalSaturated,
                direct: false,
            },
            antiWindup: { horizontal: horizontalAntiWindup, vertical: verticalSameDirection },
            fallbackReason: null,
            attitudeRateSetpoint: attitude.targetRate,
        });
    }

    _controlStabilized(dt, input, _collisionProvider) {
        const finiteScale = (value) => Number.isFinite(Number(value)) ? Math.max(0, Number(value)) : 1;
        const rates = input.rates || {};
        const pitch = this._assistedAxis(input, 'pitch') * finiteScale(rates.pitch);
        const roll = this._assistedAxis(input, 'roll') * finiteScale(rates.roll);
        const yaw = this._assistedAxis(input, 'yaw') * finiteScale(rates.yaw);
        const tiltStick = limitVector({ x: roll, y: 0, z: -pitch }, 1);
        const tiltMagnitude = Math.min(1, Math.hypot(tiltStick.x, tiltStick.z));
        const tiltRad = tiltMagnitude * 45 * DEG2RAD;
        const yawRad = this._levelYawSetpointDeg * DEG2RAD;
        const forward = { x: -Math.sin(yawRad), z: -Math.cos(yawRad) };
        // Identity yaw faces south (-Z), hence body-right is west (-X).
        // The previous vector was body-left and inverted every Level roll
        // command relative to the FPV/keyboard convention.
        const right = { x: -Math.cos(yawRad), z: Math.sin(yawRad) };
        let horizontalX = 0;
        let horizontalZ = 0;
        if (tiltMagnitude > 1e-9) {
            horizontalX = (right.x * tiltStick.x + forward.x * tiltStick.z) / tiltMagnitude;
            horizontalZ = (right.z * tiltStick.x + forward.z * tiltStick.z) / tiltMagnitude;
        }
        const desiredThrustAxis = {
            x: horizontalX * Math.sin(tiltRad),
            y: Math.cos(tiltRad),
            z: horizontalZ * Math.sin(tiltRad),
        };
        this._levelYawSetpointDeg += Math.max(-1, Math.min(1, yaw)) * 60 * dt;
        const desiredQuaternion = desiredAttitudeFromForce(
            desiredThrustAxis,
            this._levelYawSetpointDeg * DEG2RAD,
        );
        const attitude = this._trackDesiredAttitude(
            dt,
            desiredQuaternion,
            { roll: 220, pitch: 220, yaw: 120 },
        );

        this.boostActive = !!input.boost;
        this.boostMultiplier = 1;
        this.thrustOutput = piecewiseHoverThrottle(
            this._assistedAxis(input, 'throttle'),
            this.mass,
            this.maxThrust,
        );
        this.throttlePercent = this.maxThrust > 0 ? this.thrustOutput / this.maxThrust : 0;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = null;
        this._setControlDiagnostics({
            commandType: ControlCommandType.ATTITUDE_THRUST,
            source: 'level-stick',
            createdSimTime: this._simTimeS,
            expirySimTime: this._simTimeS + dt,
            tiltDeg: tiltRad * RAD2DEG,
            fallbackReason: null,
            attitudeRateSetpoint: attitude.targetRate,
        });
    }

    _getYopoReference(t) {
        return this._trajectory.referenceAt(t);
    }

    _updateSo3Yaw(dt, reference) {
        if (this._isYawLockEnabled()) {
            this._so3YawSetpointDeg = this._so3FixedYaw ?? this._so3Hold?.yawDeg ?? this.yaw;
            this._so3YawRateSetpointDeg = 0;
            return this._so3YawSetpointDeg;
        }
        const horizontalSpeed = Math.hypot(reference?.vx || 0, reference?.vz || 0);
        if (horizontalSpeed <= 0.2) return this._so3YawSetpointDeg;

        const desiredYaw = Math.atan2(-reference.vx, -reference.vz) * RAD2DEG;
        const yawError = this._wrapDegrees(desiredYaw - this._so3YawSetpointDeg);
        const desiredRate = Math.max(-60, Math.min(60, yawError * 3));
        const rateStep = 20 * dt;
        this._so3YawRateSetpointDeg += Math.max(-rateStep, Math.min(
            rateStep,
            desiredRate - this._so3YawRateSetpointDeg,
        ));
        const yawStep = this._so3YawRateSetpointDeg * dt;
        if (Math.abs(yawStep) >= Math.abs(yawError)) {
            this._so3YawSetpointDeg = desiredYaw;
            this._so3YawRateSetpointDeg = 0;
        } else {
            this._so3YawSetpointDeg = this._wrapDegrees(this._so3YawSetpointDeg + yawStep);
        }
        return this._so3YawSetpointDeg;
    }

    _controlPositionHold(dt, commandType = ControlCommandType.POSITION_VELOCITY_HOLD, reason = null) {
        if (!this._so3Hold) this._latchSo3Hold(reason || 'hold');
        const massKg = Math.max(0.001, this.mass / 1000);
        const weightN = massKg * G;
        const maxThrustN = Math.max(0, this.maxThrust * G / 1000);
        const hold = this._so3Hold;
        const requestedForce = {
            x: -this.so3Kx * (this.x - hold.x) - this.so3Kv * this.vx,
            y: weightN
                - this.so3KxVertical * (this.y - hold.y)
                - this.so3KvVertical * this.vy,
            z: -this.so3Kx * (this.z - hold.z) - this.so3Kv * this.vz,
        };
        const tiltLimited = limitTiltPreservingGravity(
            requestedForce.x,
            requestedForce.y,
            requestedForce.z,
            weightN,
            45,
        );
        const allocation = allocateEasyForce(tiltLimited, maxThrustN, {
            horizontalMarginFraction: 0.3,
            fallbackForce: { x: 0, y: Math.min(weightN, maxThrustN), z: 0 },
        });
        const acceleration = {
            x: requestedForce.x / massKg,
            y: requestedForce.y / massKg - G,
            z: requestedForce.z / massKg,
        };
        const limitedAcceleration = {
            x: allocation.force.x / massKg,
            y: allocation.force.y / massKg - G,
            z: allocation.force.z / massKg,
        };
        const horizontalSaturated = Math.hypot(
            limitedAcceleration.x - acceleration.x,
            limitedAcceleration.z - acceleration.z,
        ) > 1e-4;
        const verticalSaturated = Math.abs(limitedAcceleration.y - acceleration.y) > 1e-4;
        const yawDeg = this._isYawLockEnabled()
            ? (this._so3FixedYaw ?? hold.yawDeg)
            : this._so3YawSetpointDeg;
        this._applyForceAttitude(
            dt,
            allocation.force,
            yawDeg * DEG2RAD,
            maxThrustN,
            { roll: 220, pitch: 220, yaw: 120 },
        );
        const tiltDeg = Math.atan2(
            Math.hypot(allocation.force.x, allocation.force.z),
            Math.max(1e-9, allocation.force.y),
        ) * RAD2DEG;
        this.boostActive = false;
        this.boostMultiplier = 1;
        this.effectiveMaxSpeed = this.so3CruiseMps;
        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this._setControlDiagnostics({
            commandType,
            source: commandType === ControlCommandType.FAILSAFE_HOLD ? 'failsafe' : 'yopo-hold',
            createdSimTime: this._simTimeS,
            expirySimTime: null,
            referenceState: { x: hold.x, y: hold.y, z: hold.z, vx: 0, vy: 0, vz: 0 },
            rawAcceleration: acceleration,
            limitedAcceleration,
            requestedForce,
            allocatedForce: allocation.force,
            tiltDeg,
            saturation: {
                horizontal: horizontalSaturated,
                vertical: verticalSaturated,
                direct: false,
            },
            antiWindup: { horizontal: false, vertical: false },
            trajectoryAgeS: null,
            fallbackReason: reason,
        });
    }

    _controlSO3(dt, _input, _collisionProvider) {
        this.boostActive = false;
        this.boostMultiplier = 1;
        const context = this._trajectory.context;
        // Tracker time normally advances one-for-one with simulation time.
        // The absolute envelope also prevents an old command from resuming if
        // tracking was paused while disarmed.
        if (this._trajectory.active
            && Number.isFinite(context?.expirySimTime)
            && this._simTimeS >= context.expirySimTime - 1e-12) {
            this._trajectory.clear('trajectory-expired');
            this._latchSo3Hold('trajectory-expired');
            this._navigationTransitionReason = 'trajectory-expired';
            this._replanRequested = true;
            this._controlPositionHold(
                dt,
                ControlCommandType.FAILSAFE_HOLD,
                'trajectory-expired',
            );
            return;
        }
        const elapsedEnvelopeTime = Number.isFinite(context?.createdSimTime)
            ? Math.max(0, this._simTimeS - context.createdSimTime)
            : this._trajectory.time + dt;
        // Catch up tracker sampling to the authoritative simulation clock if
        // control execution was briefly paused (for example while disarmed).
        const trackerAdvance = Math.max(dt, elapsedEnvelopeTime - this._trajectory.time);
        const step = this._trajectory.advance(trackerAdvance);
        if (!step.active) {
            if (step.expired) {
                this._latchSo3Hold('trajectory-expired');
                this._navigationTransitionReason = 'trajectory-expired';
                this._replanRequested = true;
            }
            this._controlPositionHold(
                dt,
                step.expired ? ControlCommandType.FAILSAFE_HOLD : this._so3HoldCommandType,
                step.expired ? 'trajectory-expired' : (this._controlDiagnostics?.fallbackReason || 'awaiting-trajectory'),
            );
            return;
        }

        const reference = step.reference;
        const rawAcceleration = { x: reference.ax, y: reference.ay, z: reference.az };
        if (![rawAcceleration.x, rawAcceleration.y, rawAcceleration.z].every(Number.isFinite)) {
            this._trajectory.clear('non-finite-acceleration');
            this._latchSo3Hold('non-finite-command');
            this._replanRequested = true;
            this._controlPositionHold(dt, ControlCommandType.FAILSAFE_HOLD, 'non-finite-command');
            return;
        }

        const limitedAcceleration = limitVector(rawAcceleration, 25);
        const massKg = Math.max(0.001, this.mass / 1000);
        const weightN = massKg * G;
        const maxThrustN = Math.max(0, this.maxThrust * G / 1000);
        const requestedForce = {
            x: massKg * limitedAcceleration.x,
            y: massKg * (G + limitedAcceleration.y),
            z: massKg * limitedAcceleration.z,
        };
        const tiltLimited = limitTiltPreservingGravity(
            requestedForce.x,
            requestedForce.y,
            requestedForce.z,
            weightN,
            60,
        );
        const allocation = capDirectForce(tiltLimited, maxThrustN);
        const producedAcceleration = {
            x: allocation.force.x / massKg,
            y: allocation.force.y / massKg - G,
            z: allocation.force.z / massKg,
        };
        const horizontalSaturated = Math.hypot(
            rawAcceleration.x - limitedAcceleration.x,
            rawAcceleration.z - limitedAcceleration.z,
        ) > 1e-4 || Math.hypot(
            producedAcceleration.x - limitedAcceleration.x,
            producedAcceleration.z - limitedAcceleration.z,
        ) > 1e-4;
        const verticalSaturated = Math.abs(rawAcceleration.y - limitedAcceleration.y) > 1e-4
            || Math.abs(producedAcceleration.y - limitedAcceleration.y) > 1e-4;
        const yawDeg = this._updateSo3Yaw(dt, reference);
        this._applyForceAttitude(
            dt,
            allocation.force,
            yawDeg * DEG2RAD,
            maxThrustN,
            { roll: 220, pitch: 220, yaw: 120 },
        );
        const tiltDeg = Math.atan2(
            Math.hypot(allocation.force.x, allocation.force.z),
            Math.max(1e-9, allocation.force.y),
        ) * RAD2DEG;
        this.effectiveMaxSpeed = this.so3CruiseMps;
        this.commandedGroundSpeed = Math.hypot(reference.vx, reference.vz);
        this.targetGroundSpeed = this.so3CruiseMps;
        this.pilotGroundSpeedCommand = 0;
        this._setControlDiagnostics({
            commandType: ControlCommandType.DIRECT_ACCELERATION,
            source: context?.source || 'yopo',
            frame: context?.frame || 'sim-world-y-up',
            generation: context?.generation ?? null,
            createdSimTime: context?.createdSimTime ?? this._simTimeS - this._trajectory.time,
            expirySimTime: context?.expirySimTime ?? this._simTimeS + (this._trajectory.duration - this._trajectory.time),
            referenceState: reference,
            rawAcceleration,
            limitedAcceleration,
            requestedForce,
            allocatedForce: allocation.force,
            tiltDeg,
            saturation: {
                horizontal: horizontalSaturated,
                vertical: verticalSaturated,
                direct: allocation.saturated || horizontalSaturated || verticalSaturated,
            },
            antiWindup: { horizontal: false, vertical: false },
            trajectoryAgeS: this._trajectory.time,
            fallbackReason: null,
        });
    }

    consumeReplanRequest() {
        const requested = !!this._replanRequested;
        this._replanRequested = false;
        return requested;
    }

    // ---- Collision ----

    _handleCollisions(collisionProvider, previousPosition = null, dt = 0.016) {
        const wasColliding = this.isColliding;
        this.isColliding = false;
        this.collisionIntensity = 0;

        if (collisionProvider && typeof collisionProvider.queryCollisionResponse === 'function') {
            let anyCollision = false;
            let strongest = 0;

            for (let i = 0; i < 3; i++) {
                const collision = collisionProvider.queryCollisionResponse(this.x, this.y, this.z, this.collisionRadius, {
                    previous: i === 0 ? previousPosition : null,
                    velocity: { x: this.vx, y: this.vy, z: this.vz },
                    dt,
                });

                if (!collision || collision.penetration <= 0) break;

                anyCollision = true;
                strongest = Math.max(strongest, collision.penetration);

                const pushDist = collision.penetration + 0.04;
                this.x += collision.normal.x * pushDist;
                this.y += collision.normal.y * pushDist;
                this.z += collision.normal.z * pushDist;

                const vDotN = this.vx * collision.normal.x +
                              this.vy * collision.normal.y +
                              this.vz * collision.normal.z;
                if (vDotN < 0) {
                    const bounce = collision.source === 'swept' || collision.source === 'ray'
                        ? Math.max(this.bounceDamping, 0.55)
                        : this.bounceDamping;
                    this.vx -= collision.normal.x * vDotN * (1 + bounce);
                    this.vy -= collision.normal.y * vDotN * (1 + bounce);
                    this.vz -= collision.normal.z * vDotN * (1 + bounce);
                }

                const separationSpeed = Math.min(8, collision.penetration * 24);
                this.vx += collision.normal.x * separationSpeed;
                this.vy += collision.normal.y * separationSpeed;
                this.vz += collision.normal.z * separationSpeed;

                this.vx *= 0.65;
                this.vy *= 0.65;
                this.vz *= 0.65;
            }

            if (anyCollision) {
                this.isColliding = true;
                this.collisionIntensity = Math.min(1, strongest / Math.max(this.collisionRadius, 0.05));
                if (this.flightMode === 'drone') {
                    this._targetX = this.x;
                    this._targetY = this.y;
                    this._targetZ = this.z;
                    this._easyHorizontalState = 'brake';
                    this._easyVerticalState = 'brake';
                } else if (this.flightMode === 'so3'
                    && (!wasColliding || this._trajectory.active)) {
                    this._trajectory.clear('collision');
                    this._latchSo3Hold('collision');
                    this._navigationTransitionReason = 'collision';
                    this._replanRequested = true;
                }
            }
        }

    }
}
