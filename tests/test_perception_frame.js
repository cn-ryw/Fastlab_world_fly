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
    simTimeS: 4.25,
});

const frame = new PerceptionFrame({
    frameId: 7,
    capturedAt: 123.5,
    captureSimTimeS: planning.simTimeS,
    transform: {
        position: { x: 0, y: 2, z: -1 },
        rotation: { x: 0, y: 12, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
    rgb: Object.freeze({ type: 'image/jpeg', size: 42 }),
    captureProfile: 'calibration',
    rgbFrameComplete: true,
    rgbTilesReady: false,
    rgbReadyFaces: 2,
    rgbTotalFaces: 6,
    rgbReadinessReason: 'tiles-partial',
    faceTileReadiness: [
        { face: 'front', readyWhenCopied: true },
        { face: 'right', readyWhenCopied: true },
        { face: 'back', readyWhenCopied: false },
    ],
    ...planning,
    projectionConfig: { width: 384, height: 192 },
});

const observation = frame.planningObservation({ x: 50, y: 20, z: 30 });
assert.deepEqual(observation.actualPosition, { x: 1, y: 2, z: 3 }, 'trajectory endpoint origin uses actual position');
assert.deepEqual(observation.referencePosition, { x: 10, y: 20, z: 30 }, 'goal delta origin uses reference position');
assert.deepEqual(observation.velocity, { x: 4, y: 5, z: 6 }, 'observation keeps actual velocity');
assert.deepEqual(observation.acceleration, { x: 0.1, y: 0.2, z: 0.3 }, 'observation uses reference acceleration');
assert.equal(frame.captureProfile, 'calibration', 'capture profile is frozen with RGB provenance');
assert.equal(frame.captureSimTimeS, 4.25, 'capture simulation time is frozen with RGB provenance');
assert.equal(frame.rgbTilesReady, false);
assert.equal(frame.rgbReadyFaces, 2);
assert.equal(frame.rgbTotalFaces, 6);
assert.equal(frame.faceTileReadiness[2].readyWhenCopied, false);
assert(Object.isFrozen(frame.faceTileReadiness) && Object.isFrozen(frame.faceTileReadiness[0]));
assert(Object.isFrozen(frame) && Object.isFrozen(frame.transform) && Object.isFrozen(frame.actualState));

assert.throws(() => new PerceptionFrame({
    frameId: 8,
    capturedAt: 124,
    transform: { position: { x: 0, y: 0, z: 0 } },
    rgb: Object.freeze({ type: 'image/jpeg', size: 1 }),
    ...planning,
    captureProfile: 'unknown',
}), /captureProfile/);

const legacy = normalizePlanningState({ x: 1, y: 2, z: 3, vx: 4, vy: 5, vz: 6 }, 20);
assert.deepEqual(legacy.referenceState.position, { x: 1, y: 2, z: 3 }, 'legacy flat pose remains compatible');
assert.equal(legacy.simTimeS, null, 'legacy state without a simulation clock remains compatible');

assert.throws(() => normalizePlanningState({
    actualState: planning.actualState,
    referenceState: planning.referenceState,
    simTimeS: Number.NaN,
}, 0), /simTimeS/, 'non-finite simulation time is rejected');

console.log('\nPerceptionFrame contract: all passed');
