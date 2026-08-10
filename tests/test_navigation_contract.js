/** Navigation safety contract: 4 m arrival, leveled yaw, and YOPO envelopes. */
import { createRequire } from 'node:module';
import { readFile } from 'node:fs/promises';

const pc = createRequire(import.meta.url)('../asset/vendor/playcanvas.min.js');
globalThis.pc = pc;
globalThis.document = { getElementById: () => null };

const {
    ARRIVAL_DISTANCE_M,
    Drone,
    leveledYawFromBackward,
    validateYopoTrajectory,
} = await import('../src/drone.js');

let passed = 0;
let failed = 0;
function assert(condition, message) {
    if (condition) { passed++; return; }
    failed++;
    console.error(`FAIL: ${message}`);
}
function close(actual, expected, message, epsilon = 1e-9) {
    assert(Math.abs(actual - expected) <= epsilon, `${message}: expected ${expected}, got ${actual}`);
}

assert(ARRIVAL_DISTANCE_M === 4, 'arrival radius is the authoritative 4 m');

const mainSource = await readFile(new URL('../src/main.js', import.meta.url), 'utf8');
assert(
    /async function enterPlacementMode[\s\S]*?finishNavigationSession\('mode-exit', \{ cancelDrone: true/.test(mainSource),
    'leaving flight for placement terminates and cancels the navigation session',
);
assert(
    /finishNavigationSession\('goal-changed',[\s\S]{0,300}?cancelDrone: false/.test(mainSource),
    'replacing a goal closes the old session without zeroing physical velocity',
);

// Replacing a goal must clear the prior generation's trajectory immediately;
// physical velocity remains continuous while the new frame-safe response arrives.
{
    const drone = new Drone();
    drone.x = 0; drone.y = 10; drone.z = 0;
    drone.vx = 7; drone.vy = -1; drone.vz = 3;
    drone.setIdealGoal({ x: 20, y: 10, z: 0 });
    assert(drone.setYopoTrajectory(
        [5, 0, 0, 10, 0, 0, 0, 0, 0],
        1.125,
    ) === true, 'test precondition: old trajectory accepted');
    assert(drone._yopoPolyX, 'test precondition: old trajectory installed');
    drone.setIdealGoal({ x: -20, y: 10, z: 0 });
    assert(drone._yopoPolyX === null, 'new goal clears old X trajectory');
    assert(drone._yopoPolyY === null, 'new goal clears old Y trajectory');
    assert(drone._yopoPolyZ === null, 'new goal clears old Z trajectory');
    assert(drone._yopoDecayRef === null, 'new goal clears old decay reference');
    assert(drone._yopoPlanTriggered === false, 'arrival waits for the new generation trajectory');
    assert(drone.vx === 7 && drone.vy === -1 && drone.vz === 3,
        'retargeting preserves the actual velocity state');
}

for (const [distance, shouldArrive] of [[3.9, true], [4.1, false], [8.9, false]]) {
    const drone = new Drone();
    drone.x = 0; drone.y = 10; drone.z = 0;
    drone.setIdealGoal({ x: distance, y: 10, z: 0 });
    drone.update(1 / 60, { armed: false }, null);
    assert((drone._idealGoal === null) === shouldArrive,
        `${distance} m boundary should${shouldArrive ? '' : ' not'} arrive`);
}

{
    const identity = leveledYawFromBackward(0, 1);
    close(identity.yawRad, 0, 'identity orientation faces -Z, not +Z');
    close(identity.fwdZ, -1, 'identity forward is -Z');

    const yaw90 = leveledYawFromBackward(1, 0);
    close(yaw90.yawRad, Math.PI / 2, '+90 degree yaw faces -X');
    close(yaw90.fwdX, -1, '+90 degree forward is -X');
}

{
    const good = validateYopoTrajectory(
        [15, 15, 1, 100, 0, 0, 2, 1, 0],
        1.125,
        { x: 0, y: 100, z: 0 },
    );
    assert(good.valid, 'finite axis-major trajectory inside envelope is accepted');
    assert(!validateYopoTrajectory([1, 2, 3], 1.125).valid, 'wrong endstate length is rejected');
    assert(!validateYopoTrajectory([1, 2, 3, 4, 5, 6, 7, 8, NaN], 1.125).valid,
        'non-finite endstate is rejected');
    assert(!validateYopoTrajectory([1, 80, 0, 0, 0, 0, 0, 0, 0], 1.125).valid,
        'unsafe endpoint speed is rejected');
    assert(!validateYopoTrajectory([1, 0, 0, 0, 0, 0, 0, 0, 0], 0).valid,
        'zero trajectory duration is rejected rather than defaulted');
}

console.log(`\nNavigation contract: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
