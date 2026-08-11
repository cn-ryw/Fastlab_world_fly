/**
 * Headless acceptance tests for Level, Easy and SO3 controller integration.
 *
 * This exercises the real Drone controller and rigid-body loop with the
 * vendored PlayCanvas quaternion implementation. Safety bounds intentionally
 * mirror the controller-unification acceptance plan and must not be widened to
 * hide regressions.
 *
 * Run: node tests/test_flight_modes_integration.js
 */

import { createRequire } from 'node:module';

const pc = createRequire(import.meta.url)('../asset/vendor/playcanvas.min.js');
globalThis.pc = pc;
let yawLockEnabled = true;
globalThis.document = {
    getElementById: id => id === 'yaw-lock-toggle' ? { checked: yawLockEnabled } : null,
};
globalThis.window ||= { addEventListener() {} };
globalThis.localStorage ||= { getItem: () => null, setItem() {} };

const { Drone } = await import('../src/drone.js');
const { Controller } = await import('../src/controller.js');
const { ControlCommandType } = await import('../src/flight-control.js');

const DT = 1 / 200;
const DEG2RAD = Math.PI / 180;
const MODES = ['drone', 'fpv', 'stabilized', 'so3'];

let passed = 0;
let failed = 0;

function assert(condition, message) {
    if (condition) {
        passed++;
        return;
    }
    failed++;
    console.error(`FAIL: ${message}`);
}

function near(actual, expected, tolerance, message) {
    assert(Math.abs(actual - expected) <= tolerance,
        `${message}: expected ${expected}, got ${actual}`);
}

function input(overrides = {}) {
    return {
        armed: true,
        boost: false,
        pitch: 0,
        roll: 0,
        yaw: 0,
        throttle: 0,
        rates: { roll: 1, pitch: 1, yaw: 1 },
        cameraTiltKeyboard: 0,
        cameraTiltAxisChanged: false,
        ...overrides,
        rates: {
            roll: 1,
            pitch: 1,
            yaw: 1,
            ...(overrides.rates || {}),
        },
    };
}

function bodyTiltDeg(drone) {
    const matrix = new pc.Mat4();
    matrix.setTRS(pc.Vec3.ZERO, drone.orientation, pc.Vec3.ONE);
    const up = new pc.Vec3();
    matrix.getY(up);
    return Math.acos(Math.max(-1, Math.min(1, up.y))) / DEG2RAD;
}

function finiteDroneState(drone) {
    return [
        drone.x, drone.y, drone.z,
        drone.vx, drone.vy, drone.vz,
        drone.orientation.x, drone.orientation.y, drone.orientation.z, drone.orientation.w,
        drone.pitchRate, drone.rollRate, drone.yawRate,
        drone.thrustOutput,
    ].every(Number.isFinite);
}

function finiteVector(vector) {
    return vector
        && Number.isFinite(vector.x)
        && Number.isFinite(vector.y)
        && Number.isFinite(vector.z);
}

function runSteps(drone, seconds, command, collisionProvider = null, observer = null) {
    const count = Math.round(seconds / DT);
    for (let step = 0; step < count; step++) {
        drone.update(DT, command, collisionProvider);
        observer?.(drone, step);
    }
}

function keyboardInput(code) {
    const controller = new Controller();
    controller._getHIDAxes = () => null;
    controller._getGamepad = () => null;
    controller._updateGamepadDisplay = () => {};
    controller.armed = true;
    controller._keysDown.add(code);
    return controller.update();
}

function keyboardRollResponse(mode, code) {
    const drone = new Drone();
    drone.flightMode = mode;
    runSteps(drone, 0.2, keyboardInput(code));
    return {
        bodyRollDeg: drone._decomposeOrientation().bodyRollDeg,
        vx: drone.vx,
        vz: drone.vz,
    };
}

// -------------------------------------------------------------------------
// Level: PX4 Stabilized throttle, combined-tilt bound, high rate multiplier,
// boost independence, and one-second self-level response.
// -------------------------------------------------------------------------

// Exercise the real Controller keyboard map, not a hand-built roll value.
// ArrowLeft/Right must produce the same bank sign in every manual mode.
for (const mode of ['fpv', 'drone', 'stabilized']) {
    const left = keyboardRollResponse(mode, 'ArrowLeft');
    const right = keyboardRollResponse(mode, 'ArrowRight');
    assert(left.bodyRollDeg < -1,
        `${mode} ArrowLeft must produce negative body roll, got ${left.bodyRollDeg}°`);
    assert(right.bodyRollDeg > 1,
        `${mode} ArrowRight must produce positive body roll, got ${right.bodyRollDeg}°`);
}

function levelAxisVelocity(value, yawDeg = 0) {
    const drone = new Drone();
    drone.orientation.setFromAxisAngle(new pc.Vec3(0, 1, 0), yawDeg);
    drone.yaw = yawDeg;
    drone.flightMode = 'stabilized';
    runSteps(drone, 1, input({ roll: value }));
    return { x: drone.vx, z: drone.vz };
}

{
    const yaw0Left = levelAxisVelocity(-1);
    const yaw0Right = levelAxisVelocity(1);
    assert(yaw0Left.x > 1 && Math.abs(yaw0Left.x) > 10 * Math.abs(yaw0Left.z),
        `Level left must accelerate world +X at yaw 0, got vx=${yaw0Left.x}, vz=${yaw0Left.z}`);
    assert(yaw0Right.x < -1 && Math.abs(yaw0Right.x) > 10 * Math.abs(yaw0Right.z),
        `Level right must accelerate world -X at yaw 0, got vx=${yaw0Right.x}, vz=${yaw0Right.z}`);

    const yaw90Left = levelAxisVelocity(-1, 90);
    const yaw90Right = levelAxisVelocity(1, 90);
    assert(yaw90Left.z < -1 && Math.abs(yaw90Left.z) > 10 * Math.abs(yaw90Left.x),
        `Level left must rotate to world -Z at yaw +90°, got vx=${yaw90Left.x}, vz=${yaw90Left.z}`);
    assert(yaw90Right.z > 1 && Math.abs(yaw90Right.z) > 10 * Math.abs(yaw90Right.x),
        `Level right must rotate to world +Z at yaw +90°, got vx=${yaw90Right.x}, vz=${yaw90Right.z}`);
}

for (const [stick, expected] of [
    [-1, 0],
    [-0.5, 490],
    [0, 980],
    [0.5, 1790],
    [1, 2600],
]) {
    const drone = new Drone();
    drone.flightMode = 'stabilized';
    drone.update(DT, input({ throttle: stick }), null);
    near(drone.thrustOutput, expected, 1e-9,
        `Level throttle ${stick} uses piecewise hover mapping`);
    assert(drone.getControlDiagnostics().commandType === ControlCommandType.ATTITUDE_THRUST,
        `Level throttle ${stick} emits ATTITUDE_THRUST`);
}

{
    const drone = new Drone();
    drone.flightMode = 'stabilized';
    drone.update(DT, input({ pitch: 0.5 }), null);
    const diagnostics = drone.getControlDiagnostics();
    assert(Object.isFrozen(diagnostics), 'control diagnostics envelope is read-only');
    assert(Object.isFrozen(diagnostics.actualState)
        && Object.isFrozen(diagnostics.actualState.position)
        && Object.isFrozen(diagnostics.attitudeRateSetpoint),
    'control diagnostics nested controller/state values are read-only');
}

{
    const drone = new Drone();
    drone.flightMode = 'stabilized';
    let maximumActualTilt = 0;
    let maximumCommandTilt = 0;
    let maximumPitchRate = 0;
    let maximumRollRate = 0;
    let allFinite = true;
    const diagonal = input({
        pitch: 1,
        roll: 1,
        boost: true,
        rates: { pitch: 10, roll: 10, yaw: 10 },
    });

    runSteps(drone, 2, diagonal, null, (state) => {
        const diagnostics = state.getControlDiagnostics();
        maximumActualTilt = Math.max(maximumActualTilt, bodyTiltDeg(state));
        maximumCommandTilt = Math.max(maximumCommandTilt, diagnostics.tiltDeg);
        maximumPitchRate = Math.max(maximumPitchRate, Math.abs(state.pitchRate));
        maximumRollRate = Math.max(maximumRollRate, Math.abs(state.rollRate));
        allFinite = allFinite && finiteDroneState(state);
    });

    assert(allFinite, 'Level full diagonal with rate=10 and boost remains finite');
    assert(maximumCommandTilt <= 45.5,
        `Level commanded combined tilt ${maximumCommandTilt.toFixed(6)}° must be <=45.5°`);
    assert(maximumActualTilt <= 45.5,
        `Level actual combined tilt ${maximumActualTilt.toFixed(6)}° must be <=45.5°`);
    assert(maximumPitchRate <= 220 + 1e-6,
        `Level pitch rate ${maximumPitchRate.toFixed(6)}°/s must be <=220°/s`);
    assert(maximumRollRate <= 220 + 1e-6,
        `Level roll rate ${maximumRollRate.toFixed(6)}°/s must be <=220°/s`);

    runSteps(drone, 1, input({ rates: { pitch: 10, roll: 10, yaw: 10 } }));
    const centeredTilt = bodyTiltDeg(drone);
    assert(centeredTilt < 2,
        `Level must return within 2° after one centered second, got ${centeredTilt.toFixed(6)}°`);
}

// -------------------------------------------------------------------------
// Easy: body-relative signs at yaw 0/90, jerk-limited brake then hold, and
// force/tilt/finite-state safety throughout a high-speed command.
// -------------------------------------------------------------------------

function easyAxisVelocity(axis, value, yawDeg = 0) {
    const drone = new Drone();
    drone.orientation.setFromAxisAngle(new pc.Vec3(0, 1, 0), yawDeg);
    drone.yaw = yawDeg;
    drone._easyYawSetpointDeg = yawDeg;
    runSteps(drone, 1, input({ [axis]: value }));
    return { x: drone.vx, z: drone.vz };
}

{
    const forward = easyAxisVelocity('pitch', -1);
    const backward = easyAxisVelocity('pitch', 1);
    const left = easyAxisVelocity('roll', -1);
    const right = easyAxisVelocity('roll', 1);
    assert(forward.z < -1 && Math.abs(forward.z) > 10 * Math.abs(forward.x),
        `Easy forward must accelerate world -Z at yaw 0, got vx=${forward.x}, vz=${forward.z}`);
    assert(backward.z > 1 && Math.abs(backward.z) > 10 * Math.abs(backward.x),
        `Easy backward must accelerate world +Z at yaw 0, got vx=${backward.x}, vz=${backward.z}`);
    assert(left.x > 1 && Math.abs(left.x) > 10 * Math.abs(left.z),
        `Easy left must accelerate world +X at yaw 0, got vx=${left.x}, vz=${left.z}`);
    assert(right.x < -1 && Math.abs(right.x) > 10 * Math.abs(right.z),
        `Easy right must accelerate world -X at yaw 0, got vx=${right.x}, vz=${right.z}`);

    const yaw90Forward = easyAxisVelocity('pitch', -1, 90);
    const yaw90Left = easyAxisVelocity('roll', -1, 90);
    const yaw90Right = easyAxisVelocity('roll', 1, 90);
    assert(yaw90Forward.x < -1 && Math.abs(yaw90Forward.x) > 10 * Math.abs(yaw90Forward.z),
        `Easy forward must rotate to world -X at yaw +90°, got vx=${yaw90Forward.x}, vz=${yaw90Forward.z}`);
    assert(yaw90Left.z < -1 && Math.abs(yaw90Left.z) > 10 * Math.abs(yaw90Left.x),
        `Easy left must rotate to world -Z at yaw +90°, got vx=${yaw90Left.x}, vz=${yaw90Left.z}`);
    assert(yaw90Right.z > 1 && Math.abs(yaw90Right.z) > 10 * Math.abs(yaw90Right.x),
        `Easy right must rotate to world +Z at yaw +90°, got vx=${yaw90Right.x}, vz=${yaw90Right.z}`);
}

{
    const drone = new Drone();
    const diagonalTiltAxis = new pc.Vec3(1, 0, 1).normalize();
    drone.orientation.setFromAxisAngle(diagonalTiltAxis, 44.99);
    assert(bodyTiltDeg(drone) < 45,
        'Easy speed-cap test starts from a legal sub-45° combined tilt');
    const basis = drone._bodyBasis();
    near(
        basis.forward.x * basis.horizontalRight.x
            + basis.forward.z * basis.horizontalRight.z,
        0,
        1e-12,
        'Easy yaw-only horizontal basis remains orthogonal while the body is tilted',
    );
    drone.update(DT, input({ pitch: 1, roll: 1 }), null);
    assert(drone.targetGroundSpeed <= drone.droneMaxSpeed + 1e-9,
        `Easy tilted diagonal target ${drone.targetGroundSpeed}m/s must respect ${drone.droneMaxSpeed}m/s`);
    near(drone.targetGroundSpeed, drone.droneMaxSpeed, 1e-9,
        'Easy full diagonal still reaches, but never exceeds, its configured limit');
}

{
    const drone = new Drone();
    let maximumActualTilt = 0;
    let maximumCommandTilt = 0;
    let maximumThrust = 0;
    let maximumHorizontalJerk = 0;
    let previousLimitedAcceleration = { x: 0, z: 0 };
    let allFinite = true;
    const observe = (state) => {
        const diagnostics = state.getControlDiagnostics();
        maximumActualTilt = Math.max(maximumActualTilt, bodyTiltDeg(state));
        maximumCommandTilt = Math.max(maximumCommandTilt, diagnostics.tiltDeg);
        maximumThrust = Math.max(maximumThrust, state.thrustOutput);
        maximumHorizontalJerk = Math.max(maximumHorizontalJerk, Math.hypot(
            diagnostics.limitedAcceleration.x - previousLimitedAcceleration.x,
            diagnostics.limitedAcceleration.z - previousLimitedAcceleration.z,
        ) / DT);
        previousLimitedAcceleration = {
            x: diagnostics.limitedAcceleration.x,
            z: diagnostics.limitedAcceleration.z,
        };
        allFinite = allFinite
            && finiteDroneState(state)
            && finiteVector(diagnostics.rawAcceleration)
            && finiteVector(diagnostics.limitedAcceleration)
            && finiteVector(diagnostics.allocatedForce);
    };

    runSteps(drone, 3, input({ pitch: -1 }), null, observe);
    const speedAtRelease = Math.hypot(drone.vx, drone.vz);
    drone.update(DT, input(), null);
    observe(drone);
    assert(drone._easyHorizontalState === 'brake',
        `Easy must enter BRAKE on stick release, got ${drone._easyHorizontalState}`);

    let crossedStopThreshold = false;
    let maximumAfterStopThreshold = 0;
    runSteps(drone, 8 - DT, input(), null, (state) => {
        observe(state);
        const speed = Math.hypot(state.vx, state.vz);
        if (speed < 0.5) crossedStopThreshold = true;
        if (crossedStopThreshold) maximumAfterStopThreshold = Math.max(maximumAfterStopThreshold, speed);
    });
    const finalSpeed = Math.hypot(drone.vx, drone.vz);

    assert(speedAtRelease > 5,
        `Easy brake scenario must exercise meaningful speed, got ${speedAtRelease.toFixed(6)}m/s`);
    assert(crossedStopThreshold,
        `Easy must brake below 0.5m/s within 8s, final=${finalSpeed.toFixed(6)}m/s`);
    assert(finalSpeed < 0.5,
        `Easy final speed after 8s release must be <0.5m/s, got ${finalSpeed.toFixed(6)}m/s`);
    assert(maximumAfterStopThreshold < 0.5,
        `Easy must not re-amplify after stopping; post-threshold max=${maximumAfterStopThreshold.toFixed(6)}m/s`);
    assert(drone._easyHorizontalState === 'hold',
        `Easy must finish in HOLD, got ${drone._easyHorizontalState}`);
    assert(maximumCommandTilt <= 45.5,
        `Easy commanded tilt ${maximumCommandTilt.toFixed(6)}° must be <=45.5°`);
    assert(maximumActualTilt <= 45.5,
        `Easy actual tilt ${maximumActualTilt.toFixed(6)}° must be <=45.5°`);
    assert(maximumThrust <= drone.maxThrust + 1e-6,
        `Easy thrust ${maximumThrust.toFixed(6)}gf must not exceed ${drone.maxThrust}gf`);
    assert(maximumHorizontalJerk <= 3 * 0.8 * 9.81 + 1e-9,
        `Easy horizontal jerk ${maximumHorizontalJerk.toFixed(9)}m/s³ must stay within 3× acceleration`);
    assert(allFinite, 'Easy command/brake/hold loop must not produce NaN or Infinity');
}

{
    const drone = new Drone();
    const invalid = input({
        pitch: NaN,
        roll: Infinity,
        yaw: -Infinity,
        throttle: NaN,
        rates: { roll: Infinity, pitch: NaN, yaw: -Infinity },
    });
    runSteps(drone, 1, invalid);
    assert(finiteDroneState(drone), 'Easy non-finite pilot input has a deterministic finite state');
    const diagnostics = drone.getControlDiagnostics();
    assert(finiteVector(diagnostics.allocatedForce), 'Easy non-finite pilot input has finite allocated force');
}

{
    const normal = new Drone();
    normal.droneMaxSpeed = 10;
    normal.update(DT, input({ pitch: -1 }), null);
    const boosted = new Drone();
    boosted.droneMaxSpeed = 10;
    boosted.update(DT, input({ pitch: -1, boost: true }), null);
    near(normal.targetGroundSpeed, 10, 1e-9, 'Easy normal target uses configured speed');
    near(boosted.targetGroundSpeed, 20, 1e-9, 'Easy boost doubles target speed below the 83.333m/s cap');
    assert(boosted.thrustOutput <= boosted.maxThrust + 1e-9,
        'Easy boost never raises the physical thrust ceiling');
}

{
    const drone = new Drone();
    drone.update(DT, input(), null);
    drone._easyVerticalState = 'velocity';
    drone._easyVelocitySetpoint.y = 2;
    drone._easyAccelerationSetpoint.y = 4;
    drone._easyLimitedAcceleration.y = 4;
    drone._velIntY = 1;
    const integralBefore = drone._velIntY;

    drone.update(DT, input({ throttle: 1 }), null);
    const diagnostics = drone.getControlDiagnostics();
    assert(diagnostics.rawAcceleration.y > diagnostics.limitedAcceleration.y,
        'Easy vertical anti-windup test exercises the +4m/s² command limit');
    assert(diagnostics.saturation.vertical,
        'Easy diagnostics include acceleration/jerk limiting as vertical saturation');
    assert(diagnostics.antiWindup.vertical,
        'Easy freezes same-direction integration at the vertical acceleration limit');
    near(drone._velIntY, integralBefore, 1e-12,
        'Easy vertical integrator does not accumulate through the acceleration limit');
}

{
    const drone = new Drone();
    drone.flightMode = 'not-a-flight-mode';
    drone.update(DT, input(), null);
    const diagnostics = drone.getControlDiagnostics();
    assert(diagnostics.commandType === ControlCommandType.FAILSAFE_HOLD,
        'unknown runtime flight mode enters FAILSAFE_HOLD instead of FPV');
    assert(diagnostics.fallbackReason === 'unknown-flight-mode',
        `unknown mode fallback reason is explicit, got ${diagnostics.fallbackReason}`);
    assert(finiteDroneState(drone), 'unknown runtime flight mode remains finite');
}

// -------------------------------------------------------------------------
// SO3: READY/direct acceleration ignores tracking error, enforces 25m/s² and
// 60°, expires at the polynomial boundary, and invalidates on collision.
// -------------------------------------------------------------------------

const SO3_ENDPOINT = [15, 0, 0, 2, 0, 0, 0, 0, 0];

function readySo3Drone(endpoint = SO3_ENDPOINT, duration = 1.125) {
    const drone = new Drone();
    drone.flightMode = 'so3';
    drone.update(DT, input(), null); // apply the mode transition and latch hold
    const accepted = drone.setYopoTrajectory(endpoint, duration, { generation: 7 });
    assert(accepted, 'SO3 test trajectory is accepted by the public gate');
    return drone;
}

{
    const nominal = readySo3Drone();
    const displaced = readySo3Drone();
    displaced.x = 30;
    displaced.y = -10;
    displaced.z = 40;
    displaced.vx = 12;
    displaced.vy = -4;
    displaced.vz = 7;

    nominal.update(DT, input(), null);
    displaced.update(DT, input(), null);
    const first = nominal.getControlDiagnostics();
    const second = displaced.getControlDiagnostics();
    assert(first.commandType === ControlCommandType.DIRECT_ACCELERATION
        && second.commandType === ControlCommandType.DIRECT_ACCELERATION,
    'SO3 valid Poly5 uses DIRECT_ACCELERATION');
    assert(first.source === 'yopo' && first.frame === 'sim-world-y-up' && first.generation === 7,
        'SO3 command diagnostics preserve the frozen source/frame/generation envelope');
    assert(Number.isFinite(first.createdSimTime) && Number.isFinite(first.expirySimTime)
        && first.expirySimTime > first.createdSimTime,
    'SO3 command diagnostics expose finite creation and expiry simulation times');
    for (const axis of ['x', 'y', 'z']) {
        near(second.limitedAcceleration[axis], first.limitedAcceleration[axis], 1e-12,
            `SO3 READY acceleration ${axis} is independent of position/velocity error`);
        near(second.allocatedForce[axis], first.allocatedForce[axis], 1e-12,
            `SO3 READY allocated force ${axis} is independent of position/velocity error`);
    }
}

{
    const drone = readySo3Drone();
    let maximumRawAcceleration = 0;
    let maximumLimitedAcceleration = 0;
    let maximumCommandTilt = 0;
    let maximumActualTilt = 0;
    let maximumThrust = 0;
    let sawDirectSaturation = false;
    let allFinite = true;
    runSteps(drone, 1.1, input(), null, (state) => {
        const diagnostics = state.getControlDiagnostics();
        maximumRawAcceleration = Math.max(maximumRawAcceleration,
            Math.hypot(
                diagnostics.rawAcceleration.x,
                diagnostics.rawAcceleration.y,
                diagnostics.rawAcceleration.z,
            ));
        maximumLimitedAcceleration = Math.max(maximumLimitedAcceleration,
            Math.hypot(
                diagnostics.limitedAcceleration.x,
                diagnostics.limitedAcceleration.y,
                diagnostics.limitedAcceleration.z,
            ));
        maximumCommandTilt = Math.max(maximumCommandTilt, diagnostics.tiltDeg);
        maximumActualTilt = Math.max(maximumActualTilt, bodyTiltDeg(state));
        maximumThrust = Math.max(maximumThrust, state.thrustOutput);
        sawDirectSaturation = sawDirectSaturation || diagnostics.saturation.direct;
        allFinite = allFinite && finiteDroneState(state)
            && finiteVector(diagnostics.allocatedForce);
    });

    assert(maximumRawAcceleration > 25,
        `SO3 cap test must exercise raw acceleration above 25m/s², got ${maximumRawAcceleration.toFixed(6)}`);
    assert(maximumLimitedAcceleration <= 25 + 1e-9,
        `SO3 limited acceleration ${maximumLimitedAcceleration.toFixed(12)}m/s² must be <=25`);
    assert(maximumCommandTilt <= 60 + 1e-9,
        `SO3 commanded tilt ${maximumCommandTilt.toFixed(12)}° must be <=60°`);
    assert(maximumCommandTilt > 59,
        `SO3 tilt test must reach the limiter, got ${maximumCommandTilt.toFixed(6)}°`);
    assert(maximumActualTilt <= 60.5,
        `SO3 actual tilt ${maximumActualTilt.toFixed(12)}° must stay near the 60° limit`);
    assert(maximumThrust <= drone.maxThrust + 1e-6,
        `SO3 thrust ${maximumThrust.toFixed(6)}gf must not exceed ${drone.maxThrust}gf`);
    assert(sawDirectSaturation, 'SO3 diagnostics report active acceleration/tilt/thrust saturation');
    assert(allFinite, 'SO3 capped direct-acceleration loop remains finite');
}

{
    const duration = 0.05;
    const drone = readySo3Drone([0, 0, 0, 2, 0, 0, 0, 0, 0], duration);
    runSteps(drone, duration, input());
    const diagnostics = drone.getControlDiagnostics();
    assert(!drone._trajectory.active,
        `SO3 trajectory must expire at ${duration}s, tracker time=${drone._trajectory.time}`);
    assert(diagnostics.commandType === ControlCommandType.FAILSAFE_HOLD,
        `expired SO3 trajectory must enter FAILSAFE_HOLD, got ${diagnostics.commandType}`);
    assert(diagnostics.fallbackReason === 'trajectory-expired',
        `expired SO3 fallback reason must be trajectory-expired, got ${diagnostics.fallbackReason}`);
    assert(drone.consumeReplanRequest(), 'expired SO3 trajectory requests immediate replanning');
    assert(finiteDroneState(drone), 'SO3 expiration hold remains finite');
}

{
    const drone = readySo3Drone();
    runSteps(drone, 0.05, input());
    const holdPosition = { x: drone.x, y: drone.y, z: drone.z };
    drone.clearIdealGoal();
    assert(!drone._trajectory.active,
        'clearIdealGoal invalidates an active Poly5 immediately');
    near(drone._so3Hold.x, holdPosition.x, 1e-12,
        'clearIdealGoal latches the current X hold position');
    near(drone._so3Hold.y, holdPosition.y, 1e-12,
        'clearIdealGoal latches the current Y hold position');
    near(drone._so3Hold.z, holdPosition.z, 1e-12,
        'clearIdealGoal latches the current Z hold position');
    drone.update(DT, input(), null);
    assert(drone.getControlDiagnostics().commandType === ControlCommandType.POSITION_VELOCITY_HOLD,
        'SO3 with a cleared goal holds instead of continuing direct acceleration');
}

{
    const drone = readySo3Drone([0, 0, 0, 2, 0, 0, 0, 0, 0], 0.5);
    runSteps(drone, 0.05, input({ armed: false }));
    drone.update(DT, input(), null);
    const diagnostics = drone.getControlDiagnostics();
    assert(diagnostics.commandType === ControlCommandType.DIRECT_ACCELERATION,
        'a still-fresh SO3 envelope may resume after a short disarmed pause');
    assert(diagnostics.trajectoryAgeS >= 0.05,
        `SO3 tracker catches up to simulation time after a pause, age=${diagnostics.trajectoryAgeS}`);
}

{
    const drone = readySo3Drone([0, 0, 0, 2, 0, 0, 0, 0, 0], 0.05);
    runSteps(drone, 0.06, input({ armed: false }));
    assert(drone._trajectory.active, 'disarmed controller pauses tracker sampling only');
    drone.update(DT, input(), null);
    const diagnostics = drone.getControlDiagnostics();
    assert(!drone._trajectory.active,
        'expired SO3 command envelope cannot resume after a disarmed pause');
    assert(diagnostics.commandType === ControlCommandType.FAILSAFE_HOLD
        && diagnostics.fallbackReason === 'trajectory-expired',
    'rearming an expired SO3 envelope enters trajectory-expired FAILSAFE_HOLD');
    assert(drone.consumeReplanRequest(), 'expired paused SO3 envelope requests replanning');
}

{
    const drone = readySo3Drone();
    let queries = 0;
    const collisionProvider = {
        queryCollisionResponse() {
            if (queries++ > 0) return null;
            return {
                penetration: 0.1,
                normal: { x: -1, y: 0, z: 0 },
                source: 'overlap',
            };
        },
    };
    drone.update(DT, input(), collisionProvider);
    assert(drone.isColliding, 'SO3 collision is reported in the same physics step');
    assert(!drone._trajectory.active, 'SO3 collision invalidates the old trajectory immediately');
    assert(drone.consumeReplanRequest(), 'SO3 collision requests immediate replanning');
    assert(drone._navigationTransitionReason === 'collision',
        `SO3 collision transition reason is collision, got ${drone._navigationTransitionReason}`);

    drone.update(DT, input(), null);
    const diagnostics = drone.getControlDiagnostics();
    assert(diagnostics.commandType === ControlCommandType.FAILSAFE_HOLD,
        `post-collision SO3 must use FAILSAFE_HOLD, got ${diagnostics.commandType}`);
    assert(diagnostics.fallbackReason === 'collision',
        `post-collision SO3 fallback reason must persist, got ${diagnostics.fallbackReason}`);
    assert(finiteDroneState(drone), 'SO3 collision fallback remains finite');
}

{
    const drone = readySo3Drone();
    drone.handleControlOverrun({ droppedSeconds: 0.4, acceptedSeconds: 0.1 });
    assert(!drone._trajectory.active, 'control overrun invalidates an active SO3 trajectory');
    drone.update(DT, input(), null);
    const diagnostics = drone.getControlDiagnostics();
    assert(diagnostics.commandType === ControlCommandType.FAILSAFE_HOLD,
        `control overrun must execute FAILSAFE_HOLD, got ${diagnostics.commandType}`);
    assert(diagnostics.fallbackReason === 'control-overrun',
        `control overrun reason must persist, got ${diagnostics.fallbackReason}`);
    assert(diagnostics.overrunCount === 1,
        `control overrun diagnostics count must be 1, got ${diagnostics.overrunCount}`);
    assert(drone.consumeReplanRequest(), 'SO3 control overrun requests immediate replanning');
}

{
    const drone = readySo3Drone();
    drone.update(DT, input({ pitch: NaN }), null);
    const diagnostics = drone.getControlDiagnostics();
    assert(!drone._trajectory.active, 'non-finite command invalidates an active SO3 trajectory');
    assert(diagnostics.commandType === ControlCommandType.FAILSAFE_HOLD,
        `non-finite SO3 command must execute FAILSAFE_HOLD, got ${diagnostics.commandType}`);
    assert(diagnostics.fallbackReason === 'non-finite-command',
        `non-finite SO3 fallback reason must persist, got ${diagnostics.fallbackReason}`);
    assert(drone.consumeReplanRequest(), 'non-finite SO3 command requests immediate replanning');
}

{
    yawLockEnabled = true;
    const drone = new Drone();
    drone.orientation.setFromAxisAngle(new pc.Vec3(0, 1, 0), 35);
    drone.yaw = 35;
    drone.flightMode = 'so3';
    drone.update(DT, input(), null);
    drone.setIdealGoal({ x: 100, y: drone.y, z: 0 });
    const lockedYaw = drone._so3FixedYaw;
    assert(drone.setYopoTrajectory(SO3_ENDPOINT, 1.125, { generation: 10 }),
        'yaw-lock trajectory is accepted');
    runSteps(drone, 0.2, input({ yaw: 1, pitch: 1, roll: -1, throttle: 1 }));
    near(drone._so3YawSetpointDeg, lockedYaw, 1e-9,
        'SO3 yaw lock ignores all manual sticks during active automation');
    assert(drone.setYopoTrajectory(
        [drone.x + 5, 5, 0, drone.y, 0, 0, drone.z, 0, 0],
        1.125,
        { generation: 11 },
    ), 'replacement trajectory is accepted under yaw lock');
    near(drone._so3FixedYaw, lockedYaw, 1e-9,
        'SO3 yaw lock persists across trajectory replacement');

    yawLockEnabled = false;
    let maximumYawAcceleration = 0;
    let maximumYawRate = 0;
    let previousYawRate = drone._so3YawRateSetpointDeg;
    runSteps(drone, 0.8, input({ yaw: -1 }), null, state => {
        const yawRate = state._so3YawRateSetpointDeg;
        maximumYawAcceleration = Math.max(
            maximumYawAcceleration,
            Math.abs(yawRate - previousYawRate) / DT,
        );
        maximumYawRate = Math.max(maximumYawRate, Math.abs(yawRate));
        previousYawRate = yawRate;
    });
    assert(maximumYawAcceleration <= 20 + 1e-9,
        `unlocked SO3 yaw acceleration ${maximumYawAcceleration}°/s² must be <=20`);
    assert(maximumYawRate <= 60 + 1e-9,
        `unlocked SO3 yaw rate ${maximumYawRate}°/s must be <=60`);
    assert(Math.abs(drone._wrapDegrees(drone._so3YawSetpointDeg - lockedYaw)) > 1,
        'unlocked SO3 yaw turns toward horizontal reference velocity');
    yawLockEnabled = true;
}

for (const replanningHz of [15, 33]) {
    const drone = new Drone();
    drone.flightMode = 'so3';
    drone.update(DT, input(), null);
    const replanEverySteps = Math.max(1, Math.round(200 / replanningHz));
    let accepted = 0;
    let directSamples = 0;
    for (let step = 0; step < 400; step++) {
        if (step % replanEverySteps === 0) {
            accepted += drone.setYopoTrajectory([
                drone.x + 5, 5, 0,
                drone.y, 0, 0,
                drone.z, 0, 0,
            ], 1.125, { generation: step }) ? 1 : 0;
        }
        drone.update(DT, input(), null);
        if (drone.getControlDiagnostics().commandType === ControlCommandType.DIRECT_ACCELERATION) {
            directSamples++;
        }
    }
    assert(accepted >= Math.floor(400 / replanEverySteps),
        `${replanningHz}Hz replacement stream accepts every safe trajectory`);
    assert(directSamples === 400,
        `${replanningHz}Hz replacement stream remains continuously DIRECT_ACCELERATION`);
    assert(finiteDroneState(drone), `${replanningHz}Hz replacement stream remains finite`);
}

// Mode switching may reset controller memory/navigation, but never physical
// position, velocity, attitude, or angular rate at the transition instant.
for (const oldMode of MODES) {
    for (const newMode of MODES) {
        if (oldMode === newMode) continue;
        const drone = new Drone();
        drone.flightMode = oldMode;
        drone._prevFlightMode = oldMode;
        drone.x = 11; drone.y = 22; drone.z = -33;
        drone.vx = 4; drone.vy = -5; drone.vz = 6;
        drone.orientation.setFromEulerAngles(17, -31, 23);
        drone.pitchRate = 71; drone.rollRate = -82; drone.yawRate = 43;
        const before = {
            position: [drone.x, drone.y, drone.z],
            velocity: [drone.vx, drone.vy, drone.vz],
            quaternion: [
                drone.orientation.x,
                drone.orientation.y,
                drone.orientation.z,
                drone.orientation.w,
            ],
            rates: [drone.pitchRate, drone.rollRate, drone.yawRate],
        };

        drone._onFlightModeChanged(oldMode, newMode);
        const after = {
            position: [drone.x, drone.y, drone.z],
            velocity: [drone.vx, drone.vy, drone.vz],
            quaternion: [
                drone.orientation.x,
                drone.orientation.y,
                drone.orientation.z,
                drone.orientation.w,
            ],
            rates: [drone.pitchRate, drone.rollRate, drone.yawRate],
        };
        assert(JSON.stringify(after) === JSON.stringify(before),
            `${oldMode}->${newMode} transition preserves p/v/q/body-rates exactly`);
    }
}

console.log(`\nFlight mode integration: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
