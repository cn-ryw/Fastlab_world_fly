/** FPV legacy-behaviour golden regression under the common 200 Hz scheduler. */
import { createRequire } from 'node:module';

const pc = createRequire(import.meta.url)('../asset/vendor/playcanvas.min.js');
globalThis.pc = pc;
globalThis.document = { getElementById: () => null };

const { Drone } = await import('../src/drone.js');
const { FixedStepScheduler } = await import('../src/fixed-step-scheduler.js');

let passed = 0;
let failed = 0;
function assert(condition, message) {
    if (condition) passed++;
    else { failed++; console.error(`FAIL: ${message}`); }
}

const command = Object.freeze({
    armed: true,
    boost: true,
    pitch: 0.65,
    roll: -0.25,
    yaw: 0.3,
    throttle: 0.2,
    rates: Object.freeze({ pitch: 1.2, roll: 0.9, yaw: 1.1 }),
});

// Frozen from the pre-unification 60 Hz FPV implementation after exactly two
// seconds. This must stay data, not be regenerated through Drone.update(), so
// a shared regression in the legacy and fixed-step paths cannot move the goal
// posts together.
const LEGACY_60HZ_GOLDEN_2S = Object.freeze({
    pitchRate: 291.7199999999727,
    rollRate: -84.1499999999921,
    yawRate: 67.31999999999371,
    orientation: Object.freeze({
        x: -0.7910020087918885,
        y: -0.19094495079864654,
        z: 0.21851668452557937,
        w: 0.5386151747172075,
    }),
});

function makeDrone() {
    const drone = new Drone();
    drone.flightMode = 'fpv';
    drone.setSpawnPoint(0, 100, 0);
    return drone;
}

function runFixed(renderFps, seconds = 2) {
    const drone = makeDrone();
    const scheduler = new FixedStepScheduler();
    for (let frame = 0; frame < Math.round(seconds * renderFps); frame++) {
        scheduler.advance(1 / renderFps, dt => drone.update(dt, command, null));
    }
    return drone;
}

function quaternionDistanceDeg(a, b) {
    const dot = Math.abs(
        a.orientation.x * b.orientation.x
        + a.orientation.y * b.orientation.y
        + a.orientation.z * b.orientation.z
        + a.orientation.w * b.orientation.w
    );
    return 2 * Math.acos(Math.min(1, dot)) * 180 / Math.PI;
}

const fixed60 = runFixed(60);
const expectedThrust = ((command.throttle + 1) * 0.5) * 2600 * 1.7;
assert(Math.abs(fixed60.thrustOutput - expectedThrust) < 1e-9,
    `FPV throttle must remain exact (${expectedThrust}gf), got ${fixed60.thrustOutput}`);

for (const [actual, expected, axis] of [
    [fixed60.pitchRate, LEGACY_60HZ_GOLDEN_2S.pitchRate, 'pitch'],
    [fixed60.rollRate, LEGACY_60HZ_GOLDEN_2S.rollRate, 'roll'],
    [fixed60.yawRate, LEGACY_60HZ_GOLDEN_2S.yawRate, 'yaw'],
]) {
    const relativeError = Math.abs(actual - expected) / Math.max(1, Math.abs(expected));
    assert(relativeError < 0.005,
        `FPV ${axis} rate response error ${(relativeError * 100).toFixed(6)}% must be <0.5%`);
}
const goldenAttitudeError = quaternionDistanceDeg(fixed60, LEGACY_60HZ_GOLDEN_2S);
assert(goldenAttitudeError < 1,
    `FPV 2s fixed-step attitude error ${goldenAttitudeError.toFixed(6)}° must be <1°`);

const fixed30 = runFixed(30);
const fixed120 = runFixed(120);
for (const candidate of [fixed30, fixed120]) {
    assert(Math.hypot(candidate.x - fixed60.x, candidate.y - fixed60.y, candidate.z - fixed60.z) < 1e-9,
        'FPV fixed-step position must be render-FPS invariant');
    assert(Math.hypot(candidate.vx - fixed60.vx, candidate.vy - fixed60.vy, candidate.vz - fixed60.vz) < 1e-9,
        'FPV fixed-step velocity must be render-FPS invariant');
    assert(quaternionDistanceDeg(candidate, fixed60) < 1e-7,
        'FPV fixed-step attitude must be render-FPS invariant');
}

const tenSecond60 = runFixed(60, 10);
for (const candidate of [runFixed(30, 10), runFixed(120, 10)]) {
    assert(Math.hypot(
        candidate.x - tenSecond60.x,
        candidate.y - tenSecond60.y,
        candidate.z - tenSecond60.z,
    ) < 0.5, '10s fixed-step position spread must be <0.5m at 30/60/120 FPS');
    assert(Math.hypot(
        candidate.vx - tenSecond60.vx,
        candidate.vy - tenSecond60.vy,
        candidate.vz - tenSecond60.vz,
    ) < 0.2, '10s fixed-step velocity spread must be <0.2m/s at 30/60/120 FPS');
    assert(quaternionDistanceDeg(candidate, tenSecond60) < 1,
        '10s fixed-step attitude spread must be <1° at 30/60/120 FPS');
}

console.log(`\nFPV fixed-step golden: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
