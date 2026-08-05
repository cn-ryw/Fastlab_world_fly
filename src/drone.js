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
        this.mass        = 500;    // grams
        this.maxThrust   = 1000;   // grams-force
        this.dragCd      = 1.0;    // drag coefficient (dimensionless)
        this.dragArea     = 0.0015; // frontal area (m²), tuned for high-speed quad flight

        this.maxPitchRate = 220;
        this.maxRollRate  = 220;
        this.maxYawRate   = 120;
        this.droneMaxYawRate = 80;  // Drone mode yaw rate limit (deg/s)

        this.droneMaxAngle   = 58;
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

        this.collisionRadius = 0.3;
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

        // ---- YOPO trajectory tracking ----
        this._yopoPolyX = null;      // Poly5Solver | null
        this._yopoPolyY = null;
        this._yopoPolyZ = null;
        this._yopoTrajTime = 0;      // trajectory duration (s)
        this._yopoTrackerTime = 0;   // current time along trajectory
        this._yopoGoalYaw = 0;       // locked yaw for this trajectory
    }

    // ---- Public API ----

    /** Set ideal goal (point-to-point, no YOPO trajectory). */
    setIdealGoal(goal) {
        this._idealGoal = goal ? { x: goal.x, y: goal.y, z: goal.z, yaw: goal.yaw } : null;
    }

    clearIdealGoal() { this._idealGoal = null; }

    /** Load a YOPO trajectory endpoint → fit 5th-order polynomials. */
    setYopoTrajectory(endpoint, trajTime) {
        // endpoint: [px,py,pz, vx,vy,vz, ax,ay,az] world-frame
        const trajT = trajTime || 1.125;
        this._yopoPolyX = new Poly5Solver(this.x, this.vx, 0, endpoint[0], endpoint[3], endpoint[6], trajT);
        this._yopoPolyY = new Poly5Solver(this.y, this.vy, 0, endpoint[1], endpoint[4], endpoint[7], trajT);
        this._yopoPolyZ = new Poly5Solver(this.z, this.vz, 0, endpoint[2], endpoint[5], endpoint[8], trajT);
        this._yopoTrajTime = trajT;
        this._yopoTrackerTime = 0;
        this._yopoGoalYaw = this.yaw;
        this._idealGoal = null;  // disable point-to-point goal mode
    }

    setSpawnPoint(x, y, z) {
        this._spawnX = x; this._spawnY = y; this._spawnZ = z;
        this.reset();
    }

    reset() {
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
    }

    update(dt, input, collisionProvider) {
        dt = Math.min(dt, 0.05);

        // 0. Handle flight-mode transitions (M key, RC channel, or dropdown).
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
        } else if (this.flightMode === 'ideal') {
            this._controlIdeal(dt, input, collisionProvider);
        } else if (this.flightMode === 'drone') {
            this._controlDrone(dt, input);
        } else {
            this._controlFPV(dt, input);
        }

        // Ideal mode: position & orientation directly controlled, skip physics
        if (this.flightMode === 'ideal') {
            this._updateEulerFromQuat();
            this.groundSpeed = Math.sqrt(this.vx * this.vx + this.vz * this.vz);
            this.airSpeed = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
            this.speed = this.groundSpeed;
            this.verticalSpeed = this.vy;
            return;
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
            const cmdRight =  input.roll  * maxSpd * rates.roll;
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

    // ---- Ideal controller (direct position/orientation, no physics) ----

    _controlIdeal(dt, input, _collisionProvider) {
        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        const maxSpd = this._idealCruiseMps * (input.boost ? 2.0 : 1.0);
        this.boostActive = !!input.boost;
        this.effectiveMaxSpeed = maxSpd;
        this.thrustOutput = this.maxThrust;
        this.throttlePercent = 1.0;

        // --- Body-frame basis (world velocity from body commands) ---
        _mat4.setTRS(pc.Vec3.ZERO, this.orientation, pc.Vec3.ONE);
        _mat4.getZ(_v3);
        let fwdX = -_v3.x, fwdZ = -_v3.z;  // forward = -body_Z
        _mat4.getX(_v3);
        let rightX = _v3.x, rightZ = _v3.z; // right = +body_X
        const fwdLen = Math.sqrt(fwdX * fwdX + fwdZ * fwdZ);
        if (fwdLen > 1e-4) { fwdX /= fwdLen; fwdZ /= fwdLen; }
        const rightLen = Math.sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLen > 1e-4) { rightX /= rightLen; rightZ /= rightLen; }


        // --- YOPO trajectory mode ---
        if (this._yopoPolyX) {
            this._yopoTrackerTime += dt;
            const t = Math.min(this._yopoTrackerTime, this._yopoTrajTime);
            if (t >= this._yopoTrajTime - 0.001) {
                this.x = this._yopoPolyX.position(this._yopoTrajTime);
                this.y = this._yopoPolyY.position(this._yopoTrajTime);
                this.z = this._yopoPolyZ.position(this._yopoTrajTime);
                this.vx = 0; this.vy = 0; this.vz = 0;
                this._yopoPolyX = this._yopoPolyY = this._yopoPolyZ = null;
            } else {
                this.x = this._yopoPolyX.position(t);
                this.y = this._yopoPolyY.position(t);
                this.z = this._yopoPolyZ.position(t);
                this.vx = this._yopoPolyX.velocity(t);
                this.vy = this._yopoPolyY.velocity(t);
                this.vz = this._yopoPolyZ.velocity(t);
                if (Math.abs(this.vx) > 0.2 || Math.abs(this.vz) > 0.2) {
                    const velYaw = Math.atan2(-this.vx, -this.vz);
                    const yawStep = clamp((velYaw - this.yaw * DEG2RAD) * RAD2DEG * 5 * dt, -this._idealYawRate * dt, this._idealYawRate * dt);
                    this._applyBodyRotation(0, 1, 0, yawStep);
                }
            }
            this.commandedGroundSpeed = Math.sqrt(this.vx * this.vx + this.vz * this.vz);
            this.targetGroundSpeed = this._idealCruiseMps;
            this.pilotGroundSpeedCommand = 0;
            return;
        }

        // --- Handle goal or direct input ---
        const goal = this._idealGoal;
        let desVxW, desVyW, desVzW, desYawRate = 0;  // world-frame desired velocity

        if (goal) {
            const dx = goal.x - this.x;
            const dy = goal.y - this.y;
            const dz = goal.z - this.z;
            const distH = Math.sqrt(dx * dx + dz * dz);
            const dist3D = Math.sqrt(dx * dx + dy * dy + dz * dz);

            if (dist3D < 1.0) {
                // Arrived — stop and hold
                this._idealGoal = null;
                this.vx = 0; this.vy = 0; this.vz = 0;
            } else {
                // Softer speed: gain 0.5, cap at maxSpd, decelerate near goal
                const decelDist = 20; // start decelerating within 20m
                const speedCap = Math.min(maxSpd, distH * (distH < decelDist ? 0.5 + 0.5 * distH / decelDist : 1.0));
                const speedH = clamp(distH * 0.5, 0, speedCap);
                const nx = distH > 0.1 ? dx / distH : 0;
                const nz = distH > 0.1 ? dz / distH : 0;
                desVxW = nx * speedH;
                desVzW = nz * speedH;
                desVyW = clamp(dy * 1.5, -this.droneMaxVSpeed, this.droneMaxVSpeed);
                // Prevent going below ground
                const minY = 2; // minimum altitude above local origin
                if (this.y + desVyW * dt < minY) desVyW = (minY - this.y) / Math.max(dt, 0.001);
            }
        } else {
            // Direct stick → body-frame velocity → convert to world
            const rates = input.rates || { roll: 1, pitch: 1, yaw: 1 };
            const cmdFwd   = -input.pitch * maxSpd * rates.pitch;
            const cmdRight =  input.roll  * maxSpd * rates.roll;
            desVxW = cmdFwd * fwdX + cmdRight * rightX;
            desVzW = cmdFwd * fwdZ + cmdRight * rightZ;
            desVyW = input.throttle * this.droneMaxVSpeed;
            desYawRate = input.yaw * this._idealYawRate * rates.yaw;
        }

        // --- Smooth velocity toward desired ---
        const alpha = 1 - Math.exp(-8 * dt);
        this.vx += (desVxW - this.vx) * alpha;
        this.vy += (desVyW - this.vy) * alpha;
        this.vz += (desVzW - this.vz) * alpha;

        // --- Position integration ---
        this.x += this.vx * dt;
        this.y += this.vy * dt;
        this.z += this.vz * dt;

        // Floor: don't go below 2m above local ground origin
        if (this.y < 2) { this.y = 2; if (this.vy < 0) this.vy = 0; }

        // --- Orientation: yaw from velocity direction + stick yaw ---
        if (Math.abs(this.vx) > 0.1 || Math.abs(this.vz) > 0.1) {
            const velYaw = Math.atan2(-this.vx, -this.vz);
            const targetYaw = velYaw + desYawRate * dt * DEG2RAD;
            const yawErr = targetYaw - this.yaw * DEG2RAD;
            const yawStep = clamp(yawErr * RAD2DEG * 5.0 * dt, -this._idealYawRate * dt, this._idealYawRate * dt);
            this._applyBodyRotation(0, 1, 0, yawStep);
        }

        // Derive speed
        this.commandedGroundSpeed = Math.sqrt(desVxW * desVxW + desVzW * desVzW);
        this.targetGroundSpeed = maxSpd;
        this.pilotGroundSpeedCommand = Math.sqrt(input.pitch * input.pitch + input.roll * input.roll) * maxSpd;
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
