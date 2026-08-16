/** A cached static no-hit must never suppress per-physics-step swept collision. */
import { TilesCollisionProvider } from '../src/tiles-collision.js';

let passed = 0;
let failed = 0;
function assert(condition, message) {
    if (condition) { passed++; return; }
    failed++;
    console.error(`FAIL: ${message}`);
}

let sweptProbeCount = 0;
const world = {
    ready: true,
    sampleHeightAtLocal: () => NaN,
    pickLocalRay(origin, direction) {
        if (origin.x < -0.9 && direction.x > 0.9) {
            sweptProbeCount++;
            return { distance: 0.5, position: { x: origin.x + 0.5, y: origin.y, z: origin.z } };
        }
        return null;
    },
};

const provider = new TilesCollisionProvider(world, { queryIntervalMs: 1000 });
const first = provider.queryCollisionResponse(0, 0, 0, 0.6, {});
assert(first === null, 'first static query establishes a no-hit cache');
assert(provider._lastNoHit !== null, 'no-hit cache was populated');

const second = provider.queryCollisionResponse(0.1, 0, 0, 0.6, {
    previous: { x: -1, y: 0, z: 0 },
    velocity: { x: 10, y: 0, z: 0 },
});
assert(sweptProbeCount === 5, 'all swept-sphere probes still run while static cache is reusable');
assert(second?.source === 'swept-center', 'wall crossing is returned as a swept collision');

let crossProbeCount = 0;
const crossWorld = {
    ...world,
    pickLocalRay(origin, direction) {
        crossProbeCount++;
        return { distance: 0.5, position: { x: origin.x + 0.5, y: origin.y, z: origin.z } };
    },
};
const crossProvider = new TilesCollisionProvider(crossWorld, { sweepProbeMode: 'cross' });
crossProvider.queryCollisionResponse(0.1, 0, 0, 0.6, {
    previous: { x: -1, y: 0, z: 0 },
    velocity: { x: 10, y: 0, z: 0 },
});
assert(crossProbeCount >= 3, 'cross sweep probes center and both lateral radius offsets');

console.log(`\nCollision sweep cache: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
