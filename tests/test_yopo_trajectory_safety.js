/** Continuous-time YOPO trajectory safety-envelope tests. */
import {
    YOPO_MAX_ENDPOINT_ACCEL_MPS2,
    YOPO_MAX_ENDPOINT_DISTANCE_M,
    YOPO_MAX_ENDPOINT_SPEED_MPS,
    YOPO_MAX_TRAJ_TIME_S,
    YOPO_MIN_TRAJ_TIME_S,
    YopoTrajectoryTracker,
    validateYopoTrajectory,
} from '../src/yopo-trajectory.js';

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

function stationaryEndpoint() {
    return [0, 0, 0, 0, 0, 0, 0, 0, 0];
}

// Endpoint values alone are benign, but the stop-to-stop quintics exceed the
// continuous interval envelope at an interior stationary point.
{
    const checked = validateYopoTrajectory([30, 0, 0, 0, 0, 0, 0, 0, 0], 1);
    assert(!checked.valid, 'interior speed overshoot is rejected');
    assert(checked.reason.includes('speed peak 56.25'), 'exact interior speed maximum is reported');
}

{
    const checked = validateYopoTrajectory([15, 0, 0, 0, 0, 0, 0, 0, 0], 1);
    assert(!checked.valid, 'interior acceleration overshoot is rejected');
    assert(checked.reason.includes('acceleration peak 86.60'),
        'exact interior acceleration maximum is reported');
}

// Exact limits are inclusive.  Constant velocity simultaneously exercises the
// 120 m displacement and 50 m/s speed limits without an interpolation peak.
{
    const checked = validateYopoTrajectory(
        [YOPO_MAX_ENDPOINT_DISTANCE_M, YOPO_MAX_ENDPOINT_SPEED_MPS, 0, 0, 0, 0, 0, 0, 0],
        YOPO_MAX_ENDPOINT_DISTANCE_M / YOPO_MAX_ENDPOINT_SPEED_MPS,
        { vx: YOPO_MAX_ENDPOINT_SPEED_MPS },
    );
    assert(checked.valid, 'exact displacement and speed boundaries are accepted');
    assert(Math.abs(checked.maxSpeed - YOPO_MAX_ENDPOINT_SPEED_MPS) < 1e-9,
        'reported boundary speed is exact');
}

// A short constant-acceleration segment reaches exactly 80 m/s^2 while keeping
// both endpoint speeds small.
{
    const duration = YOPO_MIN_TRAJ_TIME_S;
    const initialVelocity = -YOPO_MAX_ENDPOINT_ACCEL_MPS2 * duration / 2;
    const terminalVelocity = -initialVelocity;
    const checked = validateYopoTrajectory(
        [0, terminalVelocity, YOPO_MAX_ENDPOINT_ACCEL_MPS2, 0, 0, 0, 0, 0, 0],
        duration,
        { vx: initialVelocity, ax: YOPO_MAX_ENDPOINT_ACCEL_MPS2 },
    );
    assert(checked.valid, 'exact acceleration and minimum-duration boundaries are accepted');
    assert(Math.abs(checked.maxAcceleration - YOPO_MAX_ENDPOINT_ACCEL_MPS2) < 1e-8,
        'reported boundary acceleration is exact');
}

assert(validateYopoTrajectory(stationaryEndpoint(), YOPO_MAX_TRAJ_TIME_S).valid,
    'maximum duration boundary is accepted');
assert(!validateYopoTrajectory(stationaryEndpoint(), YOPO_MIN_TRAJ_TIME_S - 1e-6).valid,
    'duration below minimum is rejected');
assert(!validateYopoTrajectory(stationaryEndpoint(), YOPO_MAX_TRAJ_TIME_S + 1e-6).valid,
    'duration above maximum is rejected');
assert(!validateYopoTrajectory(stationaryEndpoint(), NaN).valid,
    'NaN duration is rejected');
assert(!validateYopoTrajectory([NaN, 0, 0, 0, 0, 0, 0, 0, 0], 1).valid,
    'NaN endpoint is rejected');
assert(!validateYopoTrajectory(stationaryEndpoint(), 1, { vx: NaN }).valid,
    'NaN current state is rejected');

// Ten exact 200 Hz controller samples must expire a minimum-duration segment;
// binary floating-point accumulation produces 0.049999999999999996 here.
{
    const tracker = new YopoTrajectoryTracker();
    assert(tracker.install(stationaryEndpoint(), YOPO_MIN_TRAJ_TIME_S, {}).valid,
        'minimum-duration tracker test trajectory installs');
    let step;
    for (let i = 0; i < 10; i++) step = tracker.advance(0.005);
    assert(step.expired && !tracker.active,
        'trajectory expires exactly on its final 200 Hz sample');
}

console.log(`\nYOPO trajectory safety: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
