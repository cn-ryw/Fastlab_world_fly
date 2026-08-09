import assert from 'node:assert/strict';
import { PerceptionFrame, normalizePlanningState } from '../src/perception-frame.js';

const planning = normalizePlanningState({
    actualState: {
        position: { x: 1, y: 2, z: 3 },
        velocity: { x: 4, y: 5, z: 6 },
    },
    referenceState: {
        position: { x: 10, y: 20, z: 30 },
        velocity: { x: 7, y: 8, z: 9 },
        acceleration: { x: 0.1, y: 0.2, z: 0.3 },
    },
    yaw: 12,
});

const frame = new PerceptionFrame({
    frameId: 7,
    capturedAt: 123.5,
    transform: {
        position: { x: 0, y: 2, z: -1 },
        rotation: { x: 0, y: 12, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
    rgb: Object.freeze({ type: 'image/jpeg', size: 42 }),
    ...planning,
    projectionConfig: { width: 384, height: 192 },
});

const observation = frame.planningObservation({ x: 50, y: 20, z: 30 });
assert.deepEqual(observation.position, { x: 10, y: 20, z: 30 }, 'goal delta origin uses reference position');
assert.deepEqual(observation.velocity, { x: 4, y: 5, z: 6 }, 'observation keeps actual velocity');
assert.deepEqual(observation.acceleration, { x: 0.1, y: 0.2, z: 0.3 }, 'observation uses reference acceleration');
assert(Object.isFrozen(frame) && Object.isFrozen(frame.transform) && Object.isFrozen(frame.actualState));

const legacy = normalizePlanningState({ x: 1, y: 2, z: 3, vx: 4, vy: 5, vz: 6 }, 20);
assert.deepEqual(legacy.referenceState.position, { x: 1, y: 2, z: 3 }, 'legacy flat pose remains compatible');

console.log('\nPerceptionFrame contract: 5 passed, 0 failed');
