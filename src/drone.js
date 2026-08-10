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
 * Drone: sticks → velocity command → position setpoint,  cascaded PI position/velocity/tilt hold
 */

import { reportUserError } from './error-report.js';

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;
const G = 9.81;              // gravitational acceleration (m/s²)
const AIR_DENSITY = 1.225;   // kg/m³ at sea level
const DRONE_BOOST_MULTIPLIER = 2.0;
const FPV_BOOST_MULTIPLIER = 1.7;
const DRONE_MAX_SUPPORTED_SPEED = 300 / 3.6; // 300 km/h in m/s
const DRONE_MAX_SUPPORTED_VSPEED = 25;
// Navigation completion is a product-level tolerance, not the YOPO lattice
// radio_range. The external YOPO reference uses 4 m; using radio_range (9 m)
// here caused a vehicle travelling at 5 m/s to be declared arrived at 8.94 m.
export const ARRIVAL_DISTANCE_M = 4.0;
const YOPO_DEFAULT_TRAJ_TIME_S = 1.125;
const YOPO_MIN_TRAJ_TIME_S = 0.05;
const YOPO_MAX_TRAJ_TIME_S = 5.0;
const YOPO_MAX_ENDPOINT_DISTANCE_M = 120.0;
const YOPO_MAX_ENDPOINT_SPEED_MPS = 50.0;
const YOPO_MAX_ENDPOINT_ACCEL_MPS2 = 80.0;

// Reusable PlayCanvas math objects (avoid per-frame allocation)
const _quat  = new pc.Quat();
const _quat2 = new pc.Quat();
const _mat4  = new pc.Mat4();
const _v3    = new pc.Vec3();

// ── Poly5Solver — YOPO 5th-order polynomial trajectory ──────────────────
// Ported from YOPO_360_v15/YOPO/policy/poly_solver.py
class Poly5Solver {
    constructor(pos0, vel0, acc0, pos1, vel1, acc1, Tf) {
        const t = Tf;
        const Coef_inv = [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0.5, 0, 0, 0],
            [-10/t**3, -6/t**2, -1.5/t, 10/t**3, -4/t**2, 0.5/t],
            [15/t**4, 8/t**3, 1.5/t**2, -15/t**4, 7/t**3, -1/t**2],
            [-6/t**5, -3/t**4, -0.5/t**3, 6/t**5, -3/t**4, 0.5/t**3],
        ];
        const s = [pos0, vel0, acc0, pos1, vel1, acc1];
        this.A = Array.from({length: 6}, (_, i) =>
            Coef_inv[i].reduce((sum, c, j) => sum + c * s[j], 0)
        );
    }
    position(t) { return this.A[0]+this.A[1]*t+this.A[2]*t*t+this.A[3]*t**3+this.A[4]*t**4+this.A[5]*t**5; }
    velocity(t) { return this.A[1]+2*this.A[2]*t+3*this.A[3]*t*t+4*this.A[4]*t**3+5*this.A[5]*t**4; }
    acceleration(t) { return 2*this.A[2]+6*this.A[3]*t+12*this.A[4]*t*t+20*this.A[5]*t**3; }
}

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

/** Validate the public axis-major YOPO trajectory contract before mutation. */
export function validateYopoTrajectory(endpoint, trajTime, currentPosition = { x: 0, y: 0, z: 0 }) {
    if (!Array.isArray(endpoint) && !ArrayBuffer.isView(endpoint)) {
        return { valid: false, reason: 'endstate must be an array' };
    }
    if (endpoint.length !== 9) {
        return { valid: false, reason: `endstate must contain 9 values, got ${endpoint.length}` };
    }
    const values = Array.from(endpoint, Number);
    if (!values.every(Number.isFinite)) {
        return { valid: false, reason: 'endstate contains a non-finite value' };
    }

    const duration = trajTime == null ? YOPO_DEFAULT_TRAJ_TIME_S : Number(trajTime);
    if (!Number.isFinite(duration) || duration < YOPO_MIN_TRAJ_TIME_S || duration > YOPO_MAX_TRAJ_TIME_S) {
        return { valid: false, reason: `trajTime ${trajTime} is outside ${YOPO_MIN_TRAJ_TIME_S}-${YOPO_MAX_TRAJ_TIME_S}s` };
    }

    const positions = [values[0], values[3], values[6]];
    const velocities = [values[1], values[4], values[7]];
    const accelerations = [values[2], values[5], values[8]];
    const displacement = Math.hypot(
        positions[0] - Number(currentPosition.x || 0),
        positions[1] - Number(currentPosition.y || 0),
        positions[2] - Number(currentPosition.z || 0),
    );
    if (displacement > YOPO_MAX_ENDPOINT_DISTANCE_M) {
        return { valid: false, reason: `endpoint displacement ${displacement.toFixed(2)}m exceeds ${YOPO_MAX_ENDPOINT_DISTANCE_M}m` };
    }
    if (Math.hypot(...velocities) > YOPO_MAX_ENDPOINT_SPEED_MPS) {
        return { valid: false, reason: `endpoint speed exceeds ${YOPO_MAX_ENDPOINT_SPEED_MPS}m/s` };
    }
    if (Math.hypot(...accelerations) > YOPO_MAX_ENDPOINT_ACCEL_MPS2) {
        return { valid: false, reason: `endpoint acceleration exceeds ${YOPO_MAX_ENDPOINT_ACCEL_MPS2}m/s²` };
    }
    return { valid: true, values, trajTime: duration };
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

        this.droneMaxAngle   = 45;   // max tilt angle
        this.droneAngleRate  = 280;
        this.droneMaxVSpeed  = 8.0;
        this.droneMaxSpeed   = DRONE_MAX_SUPPORTED_SPEED;

        // Cascaded PID gains
        this.dronePosKp  = 2.0;
        this.dronePosKi  = 0.3;
        this.dronePosKd  = 0.1;
        this.droneVelKp  = 3.0;
        this.droneVelKi  = 1.0;
        this.droneVelKd  = 0.05;
        this.droneAltKp  = 4.0;
        this.droneAltKi  = 2.0;
        this.droneAltKd  = 0.1;

        // Position-hold setpoints (horizontal XY + altitude Y). Drone mode
        // yaw is pure rate control and does not use a target heading.
        this._targetX = 0; this._targetY = 2; this._targetZ = 0;

        // Smoothed attitude targets (prevent limit-cycle at angle clamp)
        this._smoothTargetPitch = 0;
        this._smoothTargetRoll  = 0;

        // Integral accumulators (position loop)
        this._posIntX = 0; this._posIntY = 0; this._posIntZ = 0;
        // Integral accumulators (velocity loop)
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        // Previous errors for derivative term
        this._prevPosErrX = 0; this._prevPosErrY = 0; this._prevPosErrZ = 0;
        this._prevVelErrX = 0; this._prevVelErrY = 0; this._prevVelErrZ = 0;
        // Filtered derivative values (low-pass to suppress jitter)
        this._filtPosDerrX = 0; this._filtPosDerrY = 0; this._filtPosDerrZ = 0;
        this._filtVelDerrX = 0; this._filtVelDerrY = 0; this._filtVelDerrZ = 0;
        // Anti-windup limits
        this._posIntMax = 5.0;
        this._velIntMax = 15.0;

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
        this._idealCruiseMps = 15;   // cruise speed (m/s)
        this._idealYawRate = 60;     // max yaw rate (deg/s)
        this._arrivalDistanceM = ARRIVAL_DISTANCE_M;

        // ---- YOPO trajectory tracking ----
        this._yopoPolyX = null;      // Poly5Solver | null
        this._yopoPolyY = null;
        this._yopoPolyZ = null;
        this._yopoTrajTime = 0;      // trajectory duration (s)
        this._yopoTrackerTime = 0;   // current time along trajectory
        this._yopoGoalYaw = 0;       // locked yaw for this trajectory

        // ---- Yaw lock (SO3 mode: fix initial yaw like YOPO lock_yaw=True) ----
        this._so3FixedYaw = null;    // null = unlocked, number = fixed yaw degrees
        this._yopoDecayRef = null;   // smooth decel after YOPO trajectory ends
        this._yopoDecayTimer = 0;
        this._so3StickYaw = null;    // accumulated yaw from A/D keys in SO3 stick mode
        this._so3AltitudeRef = null; // fixed altitude reference for goal mode
        this._yopoPlanTriggered = false; // YOPO 轨迹已下发后才允许到达判定
        this._navigationState = 'idle';
        this._navigationTransitionReason = null;

        // ---- SO3 geometric controller gains (verified in YOPO_360_v15) ----
        // YOPO uses: kx=(5.7,5.7,6.2), kv=(3.4,3.4,4.0), kR=(1.5,1.5,1.0), kOm=(0.13,0.13,0.1)
        // Our rate-control scheme: omega_des = (KR/KOmega)*eR, match ratio kR/kOm≈11.5
        this.so3Kx = 6.0;            // position error gain (YOPO: 5.7~6.2)
        this.so3Kv = 3.5;            // velocity error gain (YOPO: 3.4~4.0)
        this.so3KR = 70.0;           // attitude error gain (KR/KOmega ≈ 11.5 matches YOPO kR/kOm)
        this.so3KOmega = 6.0;        // angular velocity damping
        this.so3MaxBodyRate = 220;   // max body rate (deg/s)
        this.so3YawRate = 120;       // max yaw rate (deg/s, YOPO allows faster yaw)
        this.so3CruiseMps = 15;      // cruise speed (YOPO cruise_target_mps = 15)
        // 推力矢量相对世界竖直方向的最大倾角。对应 NetworkControl.cpp 的
        // max_tilt_deg_（默认 60°，且该文件拒绝 > 60 的取值）。
        // 限倾角而非限力的模，才能保证重力补偿不被削弱。
        this.so3MaxTiltDeg = 60;
    }

    // ---- Public API ----

    /** Set ideal goal (point-to-point, no YOPO trajectory). */
    setIdealGoal(goal) {
        // A goal change starts a new navigation generation.  Never keep
        // tracking a polynomial or decay reference generated for the prior
        // goal while the next perception response is still in flight.
        this._clearYopoMotionState();
        this._idealGoal = goal ? { x: goal.x, y: goal.y, z: goal.z, yaw: goal.yaw } : null;
        this._so3AltitudeRef = goal ? goal.y : null;  // lock altitude
        this._navigationState = goal ? 'active' : 'idle';
        this._navigationTransitionReason = goal ? 'goal-set' : 'goal-cleared';
    }

    clearIdealGoal() { this._idealGoal = null; }

    _clearYopoMotionState() {
        this._yopoPlanTriggered = false;
        this._yopoPolyX = this._yopoPolyY = this._yopoPolyZ = null;
        this._yopoTrajTime = 0;
        this._yopoTrackerTime = 0;
        this._yopoDecayRef = null;
        this._yopoDecayTimer = 0;
    }

    /** Cancel current waypoint — clear goal + YOPO trajectory + decay + stop */
    cancelWaypoint() {
        this._idealGoal = null;
        this._so3AltitudeRef = null;
        this._clearYopoMotionState();
        this._navigationState = 'cancelled';
        this._navigationTransitionReason = 'cancelled';
        // Set current position as hold reference so drone stops immediately
        this.vx = 0; this.vy = 0; this.vz = 0;
    }

    /** 载入 YOPO 轨迹末端状态 → 拟合五次多项式。 */
    setYopoTrajectory(endpoint, trajTime) {
        // endpoint 采用**轴主序** [px,vx,ax, py,vy,ay, pz,vz,az]（sim 世界系）。
        // 这与 yopo_bridge.py 的输出、以及参考实现 test_yopo_ros.py 的
        // endstate_w[id, axis, order] 一致 —— 每个轴连续排布 位置/速度/加速度。
        // 切勿改成量主序 [px,py,pz, vx,vy,vz, ...]：那会把高度值填进 X 轴的
        // 终端速度、把加速度填进 Z 轴的终点位置，产生发散的参考轨迹。
        // 契约由 tests/test_yopo_endstate_layout.js 锁定。
        const checked = validateYopoTrajectory(endpoint, trajTime, {
            x: this.x,
            y: this.y,
            z: this.z,
        });
        if (!checked.valid) {
            const now = globalThis.performance?.now?.() ?? Date.now();
            if (this._lastYopoRejectReason !== checked.reason || now - (this._lastYopoRejectAt || 0) > 1000) {
                console.warn(`[YOPO] rejected trajectory: ${checked.reason}`);
                this._lastYopoRejectReason = checked.reason;
                this._lastYopoRejectAt = now;
            }
            return false;
        }
        endpoint = checked.values;
        const trajT = checked.trajTime;
        // 轨迹交接：若旧轨迹仍活跃，取当前加速度做新轨迹初值，避免
        // refAcc 跳变导致 _controlSO3 的姿态咯噔（"断断续续"的直接来源）。
        const ax0 = this._yopoPolyX ? this._yopoPolyX.acceleration(Math.min(this._yopoTrackerTime, this._yopoTrajTime)) : 0;
        const ay0 = this._yopoPolyY ? this._yopoPolyY.acceleration(Math.min(this._yopoTrackerTime, this._yopoTrajTime)) : 0;
        const az0 = this._yopoPolyZ ? this._yopoPolyZ.acceleration(Math.min(this._yopoTrackerTime, this._yopoTrajTime)) : 0;

        this._yopoPolyX = new Poly5Solver(this.x, this.vx, ax0, endpoint[0], endpoint[1], endpoint[2], trajT);
        this._yopoPolyY = new Poly5Solver(this.y, this.vy, ay0, endpoint[3], endpoint[4], endpoint[5], trajT);
        this._yopoPolyZ = new Poly5Solver(this.z, this.vz, az0, endpoint[6], endpoint[7], endpoint[8], trajT);
        this._yopoTrajTime = trajT;
        this._yopoTrackerTime = 0;
        this._yopoGoalYaw = this.yaw;
        this._yopoDecayRef = null;  // clear decay from previous trajectory
        this._yopoPlanTriggered = true;  // 标记已有轨迹到达，允许到达判定
        this._navigationState = 'active';
        // 不清除 _idealGoal —— 目标是持久导航参考，轨迹是对它的连续逼近。
        // 到达判断在 decay 结束时根据实际距离决定，不是在轨迹开始时就丢弃目标。
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
        this._so3AltitudeRef = null;
        this._yopoPolyX = this._yopoPolyY = this._yopoPolyZ = null;
        this._yopoTrackerTime = 0;
        this._yopoDecayRef = null;
        this._yopoDecayTimer = 0;
        this._yopoPlanTriggered = false;
        this._navigationState = 'idle';
        this._navigationTransitionReason = 'reset';
        this._so3FixedYaw = null;
        this._so3StickYaw = null;

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
        this._posIntX = 0; this._posIntY = 0; this._posIntZ = 0;
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        this._prevPosErrX = 0; this._prevPosErrY = 0; this._prevPosErrZ = 0;
        this._prevVelErrX = 0; this._prevVelErrY = 0; this._prevVelErrZ = 0;
        this._filtPosDerrX = 0; this._filtPosDerrY = 0; this._filtPosDerrZ = 0;
        this._filtVelDerrX = 0; this._filtVelDerrY = 0; this._filtVelDerrZ = 0;
        this._smoothTargetPitch = 0;
        this._smoothTargetRoll  = 0;
    }

    readSettings() {
        const el = (id) => document.getElementById(id);
        const v  = (id) => { const e = el(id); return e ? parseFloat(e.value) : null; };
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

        // SO3 geometric controller gains
        const so3KxVal = v('so3-kx');
        const so3KvVal = v('so3-kv');
        const so3KRVal  = v('so3-kr');
        const so3KOmegaVal = v('so3-komega');
        const so3MaxBodyRateVal = v('so3-max-body-rate');
        const so3YawRateVal = v('so3-yaw-rate');
        if (so3KxVal !== null) this.so3Kx = so3KxVal;
        if (so3KvVal !== null) this.so3Kv = so3KvVal;
        if (so3KRVal !== null) this.so3KR = so3KRVal;
        if (so3KOmegaVal !== null) this.so3KOmega = so3KOmegaVal;
        if (so3MaxBodyRateVal !== null) this.so3MaxBodyRate = so3MaxBodyRateVal;
        if (so3YawRateVal !== null) this.so3YawRate = so3YawRateVal;
    }

    /** 到达目标后刹车：对齐参考 test_yopo_ros.py 的 TRAJECTORY_STATUS_EMPTY + 位置保持 */
    _onArrival() {
        this._yopoPolyX = this._yopoPolyY = this._yopoPolyZ = null;  // 停轨迹跟踪
        this._yopoDecayRef = null; this._yopoDecayTimer = 0;
        this._idealGoal = null;
        this._so3AltitudeRef = null;
        this._navigationState = 'arrived';
        this._navigationTransitionReason = 'arrival-distance';
    }

    update(dt, input, collisionProvider) {
        dt = Math.min(dt, 0.05);

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
        } else if (this.flightMode === 'stabilized') {
            this._controlStabilized(dt, input, collisionProvider);
        } else if (this.flightMode === 'so3') {
            this._controlSO3(dt, input, collisionProvider);
        } else if (this.flightMode === 'drone') {
            this._controlDrone(dt, input);
        } else {
            this._controlFPV(dt, input);
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

        // 6. Derive euler angles for HUD
        this._updateEulerFromQuat();
        this.groundSpeed = Math.sqrt(this.vx * this.vx + this.vz * this.vz);
        this.airSpeed = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
        this.speed = this.groundSpeed;
        this.verticalSpeed = this.vy;
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
        const yawLockEl = document.getElementById('yaw-lock-toggle');
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
     * current state and clears PID integrator / derivative memory so the
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
        this._posIntX = 0; this._posIntY = 0; this._posIntZ = 0;
        this._velIntX = 0; this._velIntY = 0; this._velIntZ = 0;
        this._prevPosErrX = 0; this._prevPosErrY = 0; this._prevPosErrZ = 0;
        this._prevVelErrX = 0; this._prevVelErrY = 0; this._prevVelErrZ = 0;
        this._filtPosDerrX = 0; this._filtPosDerrY = 0; this._filtPosDerrZ = 0;
        this._filtVelDerrX = 0; this._filtVelDerrY = 0; this._filtVelDerrZ = 0;
        this._smoothTargetPitch = 0;
        this._smoothTargetRoll  = 0;

        // Clear angular rates when entering so3 or stabilized (prevent FPV rate carryover)
        if (newMode === 'so3' || newMode === 'stabilized') {
            this.pitchRate = 0;
            this.rollRate = 0;
            this.yawRate = 0;
        }
        // 离开 SO3 模式即结束本次导航会话。旧目标、轨迹与高度覆盖不能
        // 在之后切回 SO3 时悄悄恢复；main.js 会消费 mode-exit 并同步清 UI。
        if (oldMode === 'so3' && newMode !== 'so3') {
            this.cancelWaypoint();
            this._navigationTransitionReason = 'mode-exit';
        }
        // Lock yaw on entering SO3 mode (like YOPO lock_yaw=True)
        if (newMode === 'so3') {
            this._so3FixedYaw = this.yaw;
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

        // Apply body-frame rotations
        this._applyBodyRotation(1, 0, 0, this.pitchRate * dt); // pitch around body X
        this._applyBodyRotation(0, 0, 1, this.rollRate * dt);  // roll around body Z
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);      // yaw around body Y

        // Throttle → thrust (in grams-force)
        this.thrustOutput = ((input.throttle + 1) * 0.5) * this.maxThrust * boost;
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / (this.maxThrust * boost)))
            : 0;
    }

    _controlDrone(dt, input) {
        const boost = input.boost ? DRONE_BOOST_MULTIPLIER : 1.0;
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        this.boostActive = !!input.boost;
        this.boostMultiplier = boost;

        // ---- 1. Determine stick state and compute desired velocity ----
        // Get body-frame forward (-Z) and right (+X) in world XZ plane
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        let fwdX = -_v3.x, fwdZ = -_v3.z;
        _mat4.getX(_v3);
        let rightX = _v3.x, rightZ = _v3.z;

        const fwdLen = Math.sqrt(fwdX * fwdX + fwdZ * fwdZ);
        if (fwdLen > 1e-4) {
            fwdX /= fwdLen; fwdZ /= fwdLen;
        }
        const rightLen = Math.sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLen > 1e-4) {
            rightX /= rightLen; rightZ /= rightLen;
        }

        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };
        const maxSpd = Math.min(DRONE_MAX_SUPPORTED_SPEED, this.droneMaxSpeed * boost);
        this.effectiveMaxSpeed = maxSpd;

        const horizActive = Math.abs(input.pitch) > 0.05 || Math.abs(input.roll) > 0.05;
        const vertActive  = Math.abs(input.throttle) > 0.05;

        const yawActive = Math.abs(input.yaw) > 0.05;

        let vDesX, vDesY, vDesZ;
        let pilotCmdX = 0;
        let pilotCmdZ = 0;

        // ---- Horizontal: stick = target velocity, centered = position hold ----
        if (horizActive) {
            // Stick directly commands target velocity (body-frame → world-frame)
            const cmdFwd   = -input.pitch * maxSpd * rates.pitch;
            const cmdRight = -input.roll  * maxSpd * rates.roll;
            pilotCmdX = cmdFwd * fwdX + cmdRight * rightX;
            pilotCmdZ = cmdFwd * fwdZ + cmdRight * rightZ;
            const pilotCmdH = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
            if (pilotCmdH > maxSpd) {
                const s = maxSpd / pilotCmdH;
                pilotCmdX *= s; pilotCmdZ *= s;
            }
            vDesX = pilotCmdX;
            vDesZ = pilotCmdZ;

            // Latch current position as hold target for when stick is released
            this._targetX = this.x;
            this._targetZ = this.z;
            // Clear position-loop state (not needed while stick is active)
            this._posIntX = 0; this._posIntZ = 0;
            this._filtPosDerrX = 0; this._filtPosDerrZ = 0;
            this._prevPosErrX = 0; this._prevPosErrZ = 0;
        } else {
            // Sticks centered → position hold via PID
            const posErrX = this._targetX - this.x;
            const posErrZ = this._targetZ - this.z;

            const piMax = this._posIntMax;
            this._posIntX = clamp(this._posIntX + posErrX * dt, -piMax, piMax);
            this._posIntZ = clamp(this._posIntZ + posErrZ * dt, -piMax, piMax);

            const dAlpha = 1 - Math.exp(-20 * dt);
            const rawPosDerrX = dt > 0 ? (posErrX - this._prevPosErrX) / dt : 0;
            const rawPosDerrZ = dt > 0 ? (posErrZ - this._prevPosErrZ) / dt : 0;
            this._filtPosDerrX += (rawPosDerrX - this._filtPosDerrX) * dAlpha;
            this._filtPosDerrZ += (rawPosDerrZ - this._filtPosDerrZ) * dAlpha;
            this._prevPosErrX = posErrX;
            this._prevPosErrZ = posErrZ;

            vDesX = this.dronePosKp * posErrX + this.dronePosKi * this._posIntX + this.dronePosKd * this._filtPosDerrX;
            vDesZ = this.dronePosKp * posErrZ + this.dronePosKi * this._posIntZ + this.dronePosKd * this._filtPosDerrZ;
        }

        // ---- Vertical: stick = target vertical speed, centered = altitude hold ----
        if (vertActive) {
            vDesY = input.throttle * this.droneMaxVSpeed * boost;

            // Latch current altitude as hold target
            this._targetY = this.y;
            this._posIntY = 0;
            this._filtPosDerrY = 0;
            this._prevPosErrY = 0;
        } else {
            const posErrY = this._targetY - this.y;

            const piMax = this._posIntMax;
            this._posIntY = clamp(this._posIntY + posErrY * dt, -piMax, piMax);

            const dAlpha = 1 - Math.exp(-20 * dt);
            const rawPosDerrY = dt > 0 ? (posErrY - this._prevPosErrY) / dt : 0;
            this._filtPosDerrY += (rawPosDerrY - this._filtPosDerrY) * dAlpha;
            this._prevPosErrY = posErrY;

            vDesY = this.droneAltKp * posErrY + this.droneAltKi * this._posIntY + this.droneAltKd * this._filtPosDerrY;
        }

        // Clamp desired velocity
        const vDesH = Math.sqrt(vDesX * vDesX + vDesZ * vDesZ);
        if (vDesH > maxSpd) {
            const s = maxSpd / vDesH;
            vDesX *= s; vDesZ *= s;
        }
        vDesY = clamp(vDesY, -this.droneMaxVSpeed * boost, this.droneMaxVSpeed * boost);
        this.targetGroundSpeed = Math.sqrt(vDesX * vDesX + vDesZ * vDesZ);
        this.pilotGroundSpeedCommand = Math.sqrt(pilotCmdX * pilotCmdX + pilotCmdZ * pilotCmdZ);
        this.commandedGroundSpeed = this.targetGroundSpeed;

        // ---- 2. Inner loop: Velocity PID → desired tilt angles ----
        const maxAngle = this.droneMaxAngle;
        let velErrX = vDesX - this.vx;
        const velErrY = vDesY - this.vy;
        let velErrZ = vDesZ - this.vz;

        // Clamp velocity error so acceleration demand stays within angle limit
        const aMaxHoriz = G * Math.tan(maxAngle * DEG2RAD);
        const velErrClamp = aMaxHoriz / this.droneVelKp;
        velErrX = clamp(velErrX, -velErrClamp, velErrClamp);
        velErrZ = clamp(velErrZ, -velErrClamp, velErrClamp);

        // Accumulate velocity integral (with anti-windup)
        const viMax = this._velIntMax;
        this._velIntX = clamp(this._velIntX + velErrX * dt, -viMax, viMax);
        this._velIntY = clamp(this._velIntY + velErrY * dt, -viMax, viMax);
        this._velIntZ = clamp(this._velIntZ + velErrZ * dt, -viMax, viMax);

        // Derivative of velocity error (low-pass filtered to suppress jitter)
        const vdAlpha = 1 - Math.exp(-15 * dt);
        const rawVelDerrX = dt > 0 ? (velErrX - this._prevVelErrX) / dt : 0;
        const rawVelDerrY = dt > 0 ? (velErrY - this._prevVelErrY) / dt : 0;
        const rawVelDerrZ = dt > 0 ? (velErrZ - this._prevVelErrZ) / dt : 0;
        this._filtVelDerrX += (rawVelDerrX - this._filtVelDerrX) * vdAlpha;
        this._filtVelDerrY += (rawVelDerrY - this._filtVelDerrY) * vdAlpha;
        this._filtVelDerrZ += (rawVelDerrZ - this._filtVelDerrZ) * vdAlpha;
        this._prevVelErrX = velErrX;
        this._prevVelErrY = velErrY;
        this._prevVelErrZ = velErrZ;

        // Desired world-frame horizontal acceleration
        const aDesX = this.droneVelKp * velErrX + this.droneVelKi * this._velIntX + this.droneVelKd * this._filtVelDerrX;
        const aDesZ = this.droneVelKp * velErrZ + this.droneVelKi * this._velIntZ + this.droneVelKd * this._filtVelDerrZ;

        // Project desired acceleration onto body forward/right to get tilt angles
        const aFwd   = aDesX * fwdX + aDesZ * fwdZ;
        const aRight = aDesX * rightX + aDesZ * rightZ;

        // Forward accel → negative pitch (nose down), right accel → positive roll
        const targetPitch = clamp(-aFwd / G * RAD2DEG, -maxAngle, maxAngle);
        const targetRoll  = clamp(-aRight / G * RAD2DEG, -maxAngle, maxAngle);

        // Smooth target angles to prevent residual oscillation at saturation boundary
        const smoothFactor = 1 - Math.exp(-10 * dt);
        this._smoothTargetPitch += (targetPitch - this._smoothTargetPitch) * smoothFactor;
        this._smoothTargetRoll  += (targetRoll  - this._smoothTargetRoll)  * smoothFactor;

        // ---- 3. Attitude P-controller: tilt error → body rotation ----
        const dec = this._decomposeOrientation();
        const pitchErr = this._smoothTargetPitch - dec.bodyPitchDeg;
        const rollErr  = this._smoothTargetRoll  - dec.bodyRollDeg;

        const maxStep = this.droneAngleRate * dt;
        const dpitch = clamp(pitchErr, -maxStep, maxStep);
        const droll  = clamp(rollErr,  -maxStep, maxStep);

        this._applyBodyRotation(1, 0, 0, dpitch);
        this._applyBodyRotation(0, 0, 1, droll);

        this.pitchRate = pitchErr * 5;
        this.rollRate  = rollErr  * 5;

        // ---- 4. Yaw: pure rate control, no target heading ----
        // Stick commands yaw rate directly; a centered stick damps the rate
        // toward zero (same pattern as FPV). This preserves whatever heading
        // the drone has at that moment — in particular, a FPV→drone switch
        // keeps the current heading instead of snapping to a stale setpoint.
        const droneYawMax = this.droneMaxYawRate * rates.yaw * boost;
        const tYR = input.yaw * droneYawMax;
        const ys = 1 - Math.exp(-15 * dt);
        this.yawRate += (tYR - this.yawRate) * ys;
        if (!yawActive) {
            // Stick centered → angular drag damps residual yaw rate to zero.
            this.yawRate *= Math.exp(-this.angularDrag * dt);
        }
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);

        // ---- 5. Altitude PID → thrust (in grams-force) ----
        const aDesY = this.droneVelKp * velErrY + this.droneVelKi * this._velIntY + this.droneVelKd * this._filtVelDerrY;
        let cmdGf = this.mass * (G + aDesY) / G;

        // Tilt compensation
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getY(_v3);
        const cosT = Math.max(0.1, _v3.y);
        cmdGf /= cosT;

        this.thrustOutput = clamp(cmdGf, 0, this.maxThrust * boost);
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / (this.maxThrust * boost)))
            : 0;
    }

    // ---- Stabilized controller (PX4 self-level: stick → angle, auto-level) ----
    // Angle-command mode: roll/pitch sticks set target tilt angle, yaw = rate,
    // throttle = thrust.  Goes through normal physics path (no early return).
    // YOPO trajectory and goal clicking use SO3 mode instead.

    _controlStabilized(dt, input, _collisionProvider) {
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        const boost = input.boost ? DRONE_BOOST_MULTIPLIER : 1.0;
        this.boostActive = !!input.boost;
        this.boostMultiplier = boost;

        const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };
        const maxTilt = 45; // max tilt angle in degrees (stick at edge)

        // Decompose orientation to get current body tilt
        const dec = this._decomposeOrientation();
        const currentPitch = dec.bodyPitchDeg;
        const currentRoll  = dec.bodyRollDeg;

        // Stick → target tilt angle (centered → 0° = auto-level)
        const targetPitch =  input.pitch * maxTilt * rates.pitch;  // ArrowUp=-1 → neg pitch = nose-down = forward
        const targetRoll  =  input.roll  * maxTilt * rates.roll;   // ArrowLeft=-1 → neg roll = left tilt

        // Angle error P-control → target body rate
        const angleP = 8.0;  // P gain: deg/s per degree of angle error
        const pitchErr = targetPitch - currentPitch;
        const rollErr  = targetRoll  - currentRoll;

        const maxRate = 220; // deg/s
        const pitchRateDes = clamp(pitchErr * angleP, -maxRate, maxRate) * boost;
        const rollRateDes  = clamp(rollErr  * angleP, -maxRate, maxRate) * boost;

        // Yaw: rate control (stick = rate, centered = stop)
        const yawActive = Math.abs(input.yaw) > 0.05;
        const yawRateDes = input.yaw * this._idealYawRate * rates.yaw * boost;

        // Smooth rate tracking
        const rateSmooth = 1 - Math.exp(-15 * dt);
        this.pitchRate += (pitchRateDes - this.pitchRate) * rateSmooth;
        this.rollRate  += (rollRateDes  - this.rollRate)  * rateSmooth;
        this.yawRate   += (yawRateDes   - this.yawRate)   * rateSmooth;

        // Damp when sticks centered and error small
        if (Math.abs(input.pitch) < 0.05 && Math.abs(pitchErr) < 1.0) this.pitchRate *= Math.exp(-this.angularDrag * dt);
        if (Math.abs(input.roll)  < 0.05 && Math.abs(rollErr)  < 1.0) this.rollRate  *= Math.exp(-this.angularDrag * dt);
        if (!yawActive) this.yawRate *= Math.exp(-this.angularDrag * dt);

        // Apply body-frame rotations
        this._applyBodyRotation(1, 0, 0, this.pitchRate * dt);  // pitch around body X
        this._applyBodyRotation(0, 0, 1, this.rollRate * dt);   // roll  around body Z
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);    // yaw   around body Y

        // 油门映射：摇杆中位=悬停推力，上推=爬升，下拉=下降。
        // 旧公式 ((throttle+1)*0.5)*maxThrust 在 throttle=0 时给出 50% 最大推力
        // (1300gf)，远超 980g 自重 → 无输入时自动上升，被用户误判为"油门自动增大"。
        const hoverGf = this.mass;   // 悬停推力 = 自重（理想推重比 1:1）
        const thrustRange = Math.max(this.maxThrust * boost - hoverGf, 0);
        this.thrustOutput = Math.max(0, hoverGf + input.throttle * thrustRange);
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / (this.maxThrust * boost)))
            : 0;

        this.commandedGroundSpeed = 0;
        this.targetGroundSpeed = 0;
        this.pilotGroundSpeedCommand = 0;
        this.effectiveMaxSpeed = null;
    }

    // ---- SO3 geometric controller ----
    // Based on Lee et al. (2010) "Geometric Tracking Control of a Quadrotor UAV on SE(3)"
    // Outputs thrust + body angular rates; physics integrator handles the rest.

    /**
     * Get YOPO reference state at time t along the current trajectory.
     * Returns null if no YOPO trajectory is active.
     */
    /** 从 YOPO 多项式返回当前时刻的参考状态（位置/速度/加速度，sim 世界系）。 */
    _getYopoReference(t) {
        if (!this._yopoPolyX) return null;
        // Y 通道直接取多项式值，不做硬覆盖。参考实现 test_yopo_ros.py
        // 跟踪的是完整三轴多项式，没有对任一轴做硬覆盖。
        // 此前的实现把 y/vy/ay 钉在 _so3AltitudeRef/0/0，会导致异面目标
        // 的高度过渡靠位置误差（限幅 ±3m）驱动，爬升缓慢（~0.07 m/s）
        // 且到达后因缺速度参考而过冲。
        // _so3AltitudeRef 的语义现为"多项式不可用时的 fallback 高度"，
        // 仅在 _controlSO3 的 _idealGoal/stickInput 分支中用于 refY。
        return {
            x:  this._yopoPolyX.position(t),
            y:  this._yopoPolyY.position(t),
            z:  this._yopoPolyZ.position(t),
            vx: this._yopoPolyX.velocity(t),
            vy: this._yopoPolyY.velocity(t),
            vz: this._yopoPolyZ.velocity(t),
            ax: this._yopoPolyX.acceleration(t),
            ay: this._yopoPolyY.acceleration(t),
            az: this._yopoPolyZ.acceleration(t),
        };
    }

    /**
     * Build a desired-attitude quaternion from a world-frame force direction and desired yaw.
     *
     * Body-frame convention (matching the existing physics in update()):
     *   body X = right,  body Y = up / thrust axis,  body Z = backward (-forward)
     *   At identity: b1=(1,0,0)=east, b2=(0,1,0)=up, b3=(0,0,1)=north
     *   Forward = -body Z = south at identity (yaw=0 faces -Z)
     *
     * R_d columns: b1d=right, b2d=thrust=forceDir, b3d=backward
     * Forward direction in world: c1 = (-sin yaw, 0, -cos yaw)
     */
    _so3DesiredAttitude(forceX, forceY, forceZ, yawDesRad) {
        const lenF = Math.sqrt(forceX * forceX + forceY * forceY + forceZ * forceZ);
        if (lenF < 1e-6) {
            // force too small — return level attitude at current yaw
            const hy = yawDesRad * 0.5;
            _quat.set(0, Math.sin(hy), 0, Math.cos(hy));
            return _quat.clone();
        }

        // b2d = F_d / ||F_d||  (thrust axis = body Y)
        // Tilt already clamped in _controlSO3 before calling this function
        const b2x = forceX / lenF;
        const b2y = forceY / lenF;
        const b2z = forceZ / lenF;

        // Forward direction in world (sim convention: yaw=0 → forward = (0,0,-1) = south)
        const c1x = -Math.sin(yawDesRad);
        const c1y = 0;   // forward stays in XZ plane
        const c1z = -Math.cos(yawDesRad);

        // b3d = body Z = backward = -forward_projected_to_b2d_orthogonal_plane
        // Project c1 onto b2d: c1_parallel = (c1·b2d) * b2d
        // c1_perp = c1 - c1_parallel → backward = -normalize(c1_perp)
        const dotC = c1x * b2x + c1y * b2y + c1z * b2z;
        let b3x = -(c1x - dotC * b2x);
        let b3y = -(c1y - dotC * b2y);
        let b3z = -(c1z - dotC * b2z);
        const lenB3 = Math.sqrt(b3x * b3x + b3y * b3y + b3z * b3z);
        if (lenB3 < 1e-6) {
            // c1 is parallel to b2d (pure vertical thrust) — fall back to a perpendicular
            const absY = Math.abs(b2y);
            if (absY < 0.99) {
                // cross(b2d, [0,1,0]) gives horizontal perpendicular, use as body Z
                b3x = b2z; b3y = 0; b3z = -b2x;
            } else {
                // b2d is mostly vertical — cross with world forward gives body X, then body Z
                b3x = 0; b3y = b2x; b3z = 0;
            }
            const altLen = Math.sqrt(b3x * b3x + b3y * b3y + b3z * b3z);
            if (altLen < 1e-6) { b3x = 0; b3y = 0; b3z = 1; }
            else { b3x /= altLen; b3y /= altLen; b3z /= altLen; }
        } else {
            b3x /= lenB3; b3y /= lenB3; b3z /= lenB3;
        }

        // b1d = b2d × b3d (right-hand rule: right = up × backward)
        const b1x = b2y * b3z - b2z * b3y;
        const b1y = b2z * b3x - b2x * b3z;
        const b1z = b2x * b3y - b2y * b3x;

        // Rotation matrix R_d = [b1d | b2d | b3d] → quaternion
        // PlayCanvas Mat4 is COLUMN-MAJOR: data[0..3]=col0, data[4..7]=col1, data[8..11]=col2
        // Columns: col 0 = b1d (right), col 1 = b2d (up), col 2 = b3d (backward)
        _mat4.data.set([
            b1x, b1y, b1z, 0,    // column 0: body X (right)
            b2x, b2y, b2z, 0,    // column 1: body Y (up / thrust)
            b3x, b3y, b3z, 0,    // column 2: body Z (backward)
            0,   0,   0,   1,    // column 3: translation
        ]);
        _quat.setFromMat4(_mat4);
        return _quat.clone();
    }

    /**
     * Compute SO(3) attitude error: e_R = vee(R_d^T R - R^T R_d) / 2
     * In quaternion form: e_R = 2 * sgn(q_d·q) * (q_d.w * q.xyz - q.w * q_d.xyz - q_d.xyz × q.xyz)
     * Returns {x, y, z} in the body frame.
     */
    _so3AttitudeError(qDesired) {
        const qd = qDesired;
        const q = this.orientation;
        // Dot product for shortest-path sign
        const dot = qd.x * q.x + qd.y * q.y + qd.z * q.z + qd.w * q.w;
        const sign = dot >= 0 ? 1 : -1;

        const ex = sign * (qd.w * q.x - q.w * qd.x - (qd.y * q.z - qd.z * q.y));
        const ey = sign * (qd.w * q.y - q.w * qd.y - (qd.z * q.x - qd.x * q.z));
        const ez = sign * (qd.w * q.z - q.w * qd.z - (qd.x * q.y - qd.y * q.x));

        return { x: ex, y: ey, z: ez };
    }

    /**
     * SO3 geometric control law.
     * Tracks YOPO trajectory with feedback, or holds position / follows stick input.
     */
    _controlSO3(dt, input, _collisionProvider) {
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        const maxSpd = this.so3CruiseMps * (input.boost ? 2.0 : 1.0);
        this.boostActive = !!input.boost;
        this.boostMultiplier = input.boost ? 2.0 : 1.0;
        this.effectiveMaxSpeed = maxSpd;

        // ---- Body-frame basis for stick-to-world conversion and thrust projection ----
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getY(_v3);
        const bodyUpX = _v3.x, bodyUpY = _v3.y, bodyUpZ = _v3.z;
        _mat4.getZ(_v3);
        let fwdX = -_v3.x, fwdZ = -_v3.z;
        _mat4.getX(_v3);
        let rightX = _v3.x, rightZ = _v3.z;
        const fwdLen = Math.sqrt(fwdX * fwdX + fwdZ * fwdZ);
        if (fwdLen > 1e-4) { fwdX /= fwdLen; fwdZ /= fwdLen; }
        const rightLen = Math.sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLen > 1e-4) { rightX /= rightLen; rightZ /= rightLen; }

        // ---- Determine reference trajectory ----
        let refX, refY, refZ, refVX, refVY, refVZ, refAX, refAY, refAZ;
        // Yaw lock: use fixed initial yaw if enabled (like YOPO lock_yaw=True)
        const fixedYaw = this.getFixedYaw();
        let yawDes = fixedYaw * DEG2RAD;       // default: fixed or current yaw
        let refActive = false;

        if (this._yopoPolyX) {
            // YOPO trajectory mode
            this._yopoTrackerTime += dt;
            const t = Math.min(this._yopoTrackerTime, this._yopoTrajTime);
            if (t >= this._yopoTrajTime - 0.001) {
                // End of trajectory — save final state for smooth deceleration
                const ref = this._getYopoReference(this._yopoTrajTime);
                this._yopoDecayRef = { x: ref.x, z: ref.z, vx: ref.vx, vz: ref.vz };
                this._yopoDecayTimer = 0;
                this._yopoPolyX = this._yopoPolyY = this._yopoPolyZ = null;
            } else {
                const ref = this._getYopoReference(t);
                refX = ref.x; refY = ref.y; refZ = ref.z;
                refVX = ref.vx; refVY = ref.vy; refVZ = ref.vz;
                refAX = ref.ax; refAY = ref.ay; refAZ = ref.az;
                refActive = true;
                // Yaw toward velocity direction (only when not locked)
                if (this._so3FixedYaw == null && (Math.abs(refVX) > 0.2 || Math.abs(refVZ) > 0.2)) {
                    yawDes = Math.atan2(-refVX, -refVZ);  // sim convention: 0=south
                }
            }
        }

        // Smooth deceleration after YOPO trajectory ends (P3 fix)
        if (!refActive && this._yopoDecayRef) {
            this._yopoDecayTimer += dt;
            const decayDuration = 0.5;  // seconds — 缩短以缩小轨迹段间减速间隙
            const t = Math.min(this._yopoDecayTimer / decayDuration, 1.0);
            const r = this._yopoDecayRef;
            refX = r.x; refZ = r.z;
            refY = (this._so3AltitudeRef != null ? this._so3AltitudeRef : this.y);
            refVX = r.vx * (1 - t);
            refVZ = r.vz * (1 - t);
            refVY = 0;
            refAX = 0; refAY = 0; refAZ = 0;
            refActive = true;
            if (t >= 1.0) {
                this._yopoDecayRef = null;
                this._yopoDecayTimer = 0;
                // YOPO 轨迹+衰减完成，判断是否真正到达目标
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
            }
        }

        if (!refActive && this._idealGoal) {
            // 有目标但无 YOPO 轨迹 → 原地悬停，等待首条轨迹到达。
            // 参考实现 NetworkControl.cpp 无轨迹时走 hover 保持。
            // 目标只在 YOPO 轨迹+衰减完成后由 decay 分支的到达检查清除；
            // 本分支不主动清除目标，也不盲追目标位置。
            const goal = this._idealGoal;

            // 悬停：参考位置 = 当前位置。有任何手动输入时绕过高度锁定
            const manualInput = Math.abs(input.throttle) > 0.05 ||
                                Math.abs(input.pitch) > 0.05 ||
                                Math.abs(input.roll) > 0.05;
            refX = this.x;
            refY = (manualInput ? this.y :
                    (this._so3AltitudeRef != null ? this._so3AltitudeRef : this.y));
            refZ = this.z;
            refVX = 0; refVY = 0; refVZ = 0;
            refAX = 0; refAY = 0; refAZ = 0;
            refActive = true;
            if (this._so3FixedYaw == null) {
                const dx = goal.x - this.x;
                const dz = goal.z - this.z;
                if (Math.hypot(dx, dz) > 0.1) {
                    yawDes = Math.atan2(-dx, -dz);
                }
            }
        }

        if (!refActive) {
            // Stick input mode — body-frame velocity command
            const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };
            const cmdFwd   = -input.pitch * maxSpd * rates.pitch;
            const cmdRight = -input.roll  * maxSpd * rates.roll;
            const horizActive = Math.abs(input.pitch) > 0.05 || Math.abs(input.roll) > 0.05;
            const vertActive  = Math.abs(input.throttle) > 0.05;

            if (horizActive) {
                refVX = cmdFwd * fwdX + cmdRight * rightX;
                refVZ = cmdFwd * fwdZ + cmdRight * rightZ;
            } else {
                refVX = 0; refVZ = 0;
            }
            refVY = vertActive ? input.throttle * this.droneMaxVSpeed : 0;
            // Position hold at current location, locked altitude
            refX = this.x; refY = (this._so3AltitudeRef != null ? this._so3AltitudeRef : this.y); refZ = this.z;
            refAX = 0; refAY = 0; refAZ = 0;
            refActive = true;
            // Yaw rate from A/D keys (P6 fix)
            const yawActive = Math.abs(input.yaw) > 0.05;
            if (yawActive && this._so3FixedYaw == null) {
                if (this._so3StickYaw == null) this._so3StickYaw = this.yaw;
                this._so3StickYaw += input.yaw * this.so3YawRate * rates.yaw * dt;
                yawDes = this._so3StickYaw * DEG2RAD;
            } else {
                this._so3StickYaw = null;
            }
        }

        // ---- Compute desired force F_d (world frame, Newtons) ----
        const massKg = Math.max(this.mass, 1) / 1000;
        // Cap position error: avoid massive F_d from far-away goals destroying control
        const posErrMax = 3.0;  // meters — above 3m the drone tilts too much to hover
        const ePosX = clamp(this.x - refX, -posErrMax, posErrMax);
        const ePosY = clamp(this.y - refY, -posErrMax, posErrMax);
        const ePosZ = clamp(this.z - refZ, -posErrMax, posErrMax);
        const eVelX = this.vx - refVX;
        const eVelY = this.vy - refVY;
        const eVelZ = this.vz - refVZ;

        // F_d = -K_x e_x - K_v e_v + m*g*e_3 + m*a_d
        // Our coordinate: Y = up, so gravity compensation is +y
        let FdX = -this.so3Kx * ePosX - this.so3Kv * eVelX + refAX * massKg;
        let FdY = -this.so3Kx * ePosY - this.so3Kv * eVelY + (G + refAY) * massKg;
        let FdZ = -this.so3Kx * ePosZ - this.so3Kv * eVelZ + refAZ * massKg;

        // --- 倾角限幅，取代此前的力模等比缩放 ---
        // 可行倾角上限：满推力下垂直分量仍须撑得住自重，否则倾到该角度就是掉高。
        // 参考实现固定 60°（NetworkControl）/45°（SO3Control）；这里额外用推重比兜底，
        // 因为设置面板允许用户任意修改 mass / maxThrust。
        const weightN = massKg * G;
        const Fmax = this.maxThrust * this.boostMultiplier * G / 1000 * 0.95;  // N
        let tiltDeg = this.so3MaxTiltDeg;
        if (Fmax > weightN) {
            const feasibleDeg = Math.acos(weightN / Fmax) * RAD2DEG * 0.9;  // 留 10% 裕度
            tiltDeg = Math.min(tiltDeg, feasibleDeg);
        }
        const Flim = limitTiltPreservingGravity(FdX, FdY, FdZ, weightN, tiltDeg);
        FdX = Flim.x; FdY = Flim.y; FdZ = Flim.z;

        // ---- Attitude control ----
        // b2d = F_d/|F_d| naturally. Let the attitude controller handle tilt;
        // the Fmax cap and position error cap already bound the control authority.
        const qDesired = this._so3DesiredAttitude(FdX, FdY, FdZ, yawDes);
        const eR = this._so3AttitudeError(qDesired);

        // Control torque in body frame: M = -K_R e_R - K_Ω Ω
        // Body angular velocity: Ω = (pitchRate, yawRate, rollRate) in deg/s
        // e_R x→body X→pitch, e_R y→body Y→yaw, e_R z→body Z→roll
        const omegaX = this.pitchRate * DEG2RAD;  // rad/s around body X
        const omegaY = this.yawRate * DEG2RAD;    // rad/s around body Y
        const omegaZ = this.rollRate * DEG2RAD;   // rad/s around body Z
        const torqueX = -this.so3KR * eR.x - this.so3KOmega * omegaX;
        const torqueY = -this.so3KR * eR.y - this.so3KOmega * omegaY;
        const torqueZ = -this.so3KR * eR.z - this.so3KOmega * omegaZ;

        // Simplified rate-control: torque → desired angular rate
        // body X torque → pitchRate, body Y torque → yawRate, body Z torque → rollRate
        const kOmegaInv = 1.0 / Math.max(this.so3KOmega, 0.01);
        const pitchDes  = clamp(torqueX * kOmegaInv * RAD2DEG, -this.so3MaxBodyRate, this.so3MaxBodyRate);
        const yawRateDes = clamp(torqueY * kOmegaInv * RAD2DEG, -this.so3YawRate, this.so3YawRate);
        const rollDes   = clamp(torqueZ * kOmegaInv * RAD2DEG, -this.so3MaxBodyRate, this.so3MaxBodyRate);

        // Smooth rate tracking (same pattern as _controlFPV)
        const rateSmooth = 1 - Math.exp(-15 * dt);
        this.rollRate  += (rollDes   - this.rollRate)  * rateSmooth;
        this.pitchRate += (pitchDes  - this.pitchRate) * rateSmooth;
        this.yawRate   += (yawRateDes - this.yawRate)  * rateSmooth;

        // Apply body-frame rotations
        this._applyBodyRotation(1, 0, 0, this.pitchRate * dt);  // pitch around body X
        this._applyBodyRotation(0, 0, 1, this.rollRate * dt);   // roll  around body Z
        this._applyBodyRotation(0, 1, 0, this.yawRate * dt);    // yaw   around body Y

        // Damp angular rates when errors are small
        const eRmag = Math.sqrt(eR.x * eR.x + eR.y * eR.y + eR.z * eR.z);
        if (eRmag < 0.02) {
            const ad = Math.exp(-this.angularDrag * dt * 0.5);
            this.rollRate *= ad;
            this.pitchRate *= ad;
            this.yawRate *= ad;
        }

        // ---- Thrust: 将 F_d 投影到实际 body Y 方向（非取模） ----
        // 取模法 `thrust = |F_d|` 在稳态时正确（body Y ≈ F_d 方向），
        // 但在瞬态（按下/松开方向键，姿态未跟踪到位）时：
        // body Y 比 F_d 更垂直 → |F_d| 通过不够倾斜的 body Y 产生
        // 过量垂直分量 → 无人机在倾斜/回正瞬间升高。
        // 投影法：推力 = F_d · bodyY，自动补偿姿态瞬态，输出恒等于
        // F_d 通过当前 body Y 产生的净力。
        const thrustN = FdX * bodyUpX + FdY * bodyUpY + FdZ * bodyUpZ;
        const cmdGf = Math.max(0, thrustN / G * 1000);
        this.thrustOutput = clamp(cmdGf, 0, this.maxThrust * this.boostMultiplier);
        this.throttlePercent = this.maxThrust > 0
            ? Math.max(0, Math.min(1, this.thrustOutput / (this.maxThrust * this.boostMultiplier)))
            : 0;

        // ---- Display outputs ----
        this.commandedGroundSpeed = Math.sqrt(refVX * refVX + refVZ * refVZ);
        this.targetGroundSpeed = maxSpd;
        this.pilotGroundSpeedCommand = Math.sqrt(
            (input.pitch || 0) * (input.pitch || 0) + (input.roll || 0) * (input.roll || 0)
        ) * maxSpd;
    }

    // ---- Collision ----

    _handleCollisions(collisionProvider, previousPosition = null, dt = 0.016) {
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
                }
            }
        }

    }
}
