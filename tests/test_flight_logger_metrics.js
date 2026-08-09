import assert from 'node:assert/strict';
import { FlightLogger } from '../src/flight-logger.js';

let downloadedBlob = null;
const originalCreateObjectURL = URL.createObjectURL;
const originalRevokeObjectURL = URL.revokeObjectURL;
URL.createObjectURL = blob => { downloadedBlob = blob; return 'blob:test'; };
URL.revokeObjectURL = () => {};
globalThis.document = {
    body: { appendChild() {}, removeChild() {} },
    createElement() { return { click() {}, href: '', download: '' }; },
};

const logger = new FlightLogger();
logger.start({ x: 10, y: 20, z: 30 }, 20);
logger.recordPerception({
    frameId: 1, mode: 'planning', outcome: 'applied', captureMs: 20,
    renderMs: 10, projectMs: 2, jpegMs: 3, networkMs: 30,
    da360Ms: 18, yopoMs: 4, applyMs: 1, captureToApplyMs: 60,
    frameAgeMs: 60, calibrationId: 'cal-1',
});
logger.recordPerception({ frameId: 1, mode: 'planning', outcome: 'applied', frameAgeMs: 62 });
logger.recordPerception({ frameId: 2, mode: 'planning', outcome: 'stale', dropReason: 'old-frame' });
const drone = {
    x: 0, y: 20, z: 0, vx: 0, vy: 0, vz: 0,
    yaw: 0, pitch: 0, roll: 0, pitchRate: 0, rollRate: 0, yawRate: 0,
    thrustOutput: 0, groundSpeed: 0, flightMode: 'so3', _yopoPolyX: {},
};
logger.record(drone, 0, 20, 0);
await new Promise(resolve => setTimeout(resolve, 2));
logger.record(drone, 0, 20, 0);
logger.stop(false);

const log = JSON.parse(await downloadedBlob.text());
assert.equal(log.perf.uniquePlanningFrames, 1, 'duplicate applies for one frame count once');
assert.equal(log.perf.droppedByReason['old-frame'], 1);
assert.equal(log.perf.captureToApplyP95Ms, 60);
assert.deepEqual(log.perf.calibrationIds, ['cal-1']);
assert.ok(log.perf.physicsUpdateIntervalP95Ms >= 0);
assert.equal(log.perception.length, 3);

URL.createObjectURL = originalCreateObjectURL;
URL.revokeObjectURL = originalRevokeObjectURL;
console.log('\nFlight logger metrics: 6 passed, 0 failed');
