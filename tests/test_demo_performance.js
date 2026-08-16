import assert from 'node:assert/strict';
import {
    DemoPerformanceController,
    resolvePerformanceConfig,
} from '../src/demo-performance.js';

const demo = resolvePerformanceConfig('');
assert.equal(demo.profile, 'demo30');
assert.equal(demo.tileRequestsPerServer, 12);
assert.equal(demo.normalFacesPerSlice, 3);
assert.equal(demo.constrainedFacesPerSlice, 2);
assert.equal(demo.preloadRadiusMeters, 400);
assert.equal(demo.preloadTimeoutMs, 60000);
assert.equal(demo.preloadFrameDelayMs, 32);
assert.equal(demo.preloadFaceTileTimeoutMs, 4000);
assert.equal(demo.preloadFaceTileQuietMs, 150);
assert.equal(demo.preloadRequired, false);

const baseline = resolvePerformanceConfig('?perfProfile=baseline');
assert.equal(baseline.profile, 'baseline');
assert.equal(baseline.tileRequestsPerServer, 18);
assert.equal(baseline.preloadRequired, false);

for (const limit of [6, 8, 12, 18]) {
    assert.equal(
        resolvePerformanceConfig(`?tileRequestsPerServer=${limit}`).tileRequestsPerServer,
        limit,
    );
}
assert.equal(resolvePerformanceConfig('?tileRequestsPerServer=7').tileRequestsPerServer, 12);
assert.equal(resolvePerformanceConfig('?dynamicSse=off').dynamicSse, 'off');

const controller = new DemoPerformanceController(demo);
assert.equal(controller.captureIntervalMs('planning', null, 20), 1000 / 15);
assert.equal(controller.captureIntervalMs('preview', { vx: 0, vy: 0, vz: 0 }, 20), 125);
assert.equal(controller.captureIntervalMs('preview', { vx: 1, vy: 0, vz: 0 }, 20), 500);
assert.equal(controller.captureIntervalMs('preview', { vx: 0.5, vy: 0, vz: 0 }, 20), 500,
    'moving preview keeps its state inside the hysteresis band');
assert.equal(controller.captureIntervalMs('preview', { vx: 0.2, vy: 0, vz: 0 }, 20), 125);
assert.equal(controller.facesPerSlice(false, 6), 3);

const scheduler = { maximumRequestsPerServer: 18 };
controller.configureCesium({ RequestScheduler: scheduler });
assert.equal(scheduler.maximumRequestsPerServer, 12);

const viewer = {
    resolutionScale: 1,
    scene: { requestRender() {} },
};
controller.attachViewer(viewer);
assert.equal(viewer.resolutionScale, 0.7);

// Sustained 50 ms frame intervals first consume main-view resolution budget,
// then reduce capture slicing from 3 to 2 without going below the 10 Hz floor.
for (let now = 0; now <= 9000; now += 50) controller.recordFrame(now);
let metrics = controller.snapshotSince(0);
assert.equal(metrics.adaptiveResolutionScaleFinal, 0.6);
assert.equal(metrics.panoramaFacesPerSliceFinal, 2);
assert.equal(metrics.mainFrameIntervalP95Ms, 50);

// A stable 30 fps window recovers perception slicing before visual resolution.
for (let now = 9050; now <= 17500; now += 1000 / 30) controller.recordFrame(now);
metrics = controller.snapshotSince(0);
assert.equal(metrics.panoramaFacesPerSliceFinal, 3);
assert(metrics.adaptiveResolutionScaleFinal >= 0.55);

const baselineController = new DemoPerformanceController(baseline);
assert.equal(baselineController.captureIntervalMs('planning', null, 20), 20);
assert.equal(baselineController.facesPerSlice(false, 6), 6);

console.log('Demo performance controller: all tests passed');
