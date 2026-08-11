import assert from 'node:assert/strict';
import { FlightLogger } from '../src/flight-logger.js';

let downloadedBlob = null;
const originalCreateObjectURL = URL.createObjectURL;
const originalRevokeObjectURL = URL.revokeObjectURL;
const originalLocationDescriptor = Object.getOwnPropertyDescriptor(globalThis, 'location');
URL.createObjectURL = blob => { downloadedBlob = blob; return 'blob:test'; };
URL.revokeObjectURL = () => {};
Object.defineProperty(globalThis, 'location', {
    configurable: true,
    value: {
        href: 'https://pilot:login-secret@127.0.0.1:8080/'
            + '?panoProfile=flight&panoFacesPerSlice=2'
            + '&flightPreloadRadius=800&flightPreloadMinCoverage=0.97'
            + '&flightPreloadViewTimeoutMs=25000&flightPreloadViewAttempts=3'
            + '&flightPreloadStrict=1'
            + '&da360Url=https%3A%2F%2Fnested-user%3Anested-pass%40api.example%2Fdepth%3Ftoken%3Dnested-token'
            + '&unreviewedField=must-not-enter-evidence'
            + '&ionToken=ion-secret&Api_Key=underscore-secret'
            + '&API-KEY=hyphen-secret&apikey=compact-secret'
            + '&auth=auth-secret&Authorization=bearer-secret'
            + '&credential=credential-secret&password=password-secret'
            + '&passwd=passwd-secret&client_secret=client-secret'
            + '#fragment-secret',
    },
});
globalThis.document = {
    body: { appendChild() {}, removeChild() {} },
    createElement() { return { click() {}, href: '', download: '' }; },
};

const logger = new FlightLogger();
logger.start(
    { x: 10, y: 20, z: 30 },
    20,
    { goalId: 'goal-1', generation: 1 },
);
logger.recordPerception({
    frameId: 1, goalId: 'goal-1', generation: 1,
    mode: 'planning', outcome: 'applied', captureMs: 20,
    planningAuthorized: true,
    trajectoryApplied: true, trajectoryAppliedAtMs: performance.now(),
    renderMs: 10, projectMs: 2, jpegMs: 3, networkMs: 30,
    serverMs: 25, da360Ms: 18, yopoMs: 4, applyMs: 1, captureToApplyMs: 60,
    frameAgeMs: 60, calibrationId: 'cal-1',
    rgbTilesReady: false, rgbReadyFaces: 4, rgbTotalFaces: 6,
    rgbTileError: true, rgbReadinessReason: 'tile-error',
});
logger.recordPerception({
    frameId: 1, goalId: 'goal-1', generation: 1,
    mode: 'planning', outcome: 'applied', planningAuthorized: true,
    trajectoryApplied: true, trajectoryAppliedAtMs: performance.now(), frameAgeMs: 62,
});
logger.recordPerception({
    frameId: 2, goalId: 'goal-1', generation: 1,
    mode: 'planning', outcome: 'stale', dropReason: 'old-frame',
});
logger.recordPerception({
    frameId: 3, goalId: 'goal-1', generation: 1,
    mode: 'planning', outcome: 'applied', planningAuthorized: false,
    dropReason: 'da360-relative-is-preview-only',
});
logger.recordPerception({
    frameId: 4, goalId: 'old-goal', generation: 0,
    mode: 'planning', outcome: 'stale', dropReason: 'cross-session',
});
logger.recordPerception({
    frameId: 4, goalId: 'old-goal', generation: 0,
    mode: 'depth-preview', outcome: 'stale', dropReason: 'cross-session-display',
});
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
assert.equal(log.perf.droppedByReason['da360-relative-is-preview-only'], 1);
assert.equal(log.perf.captureToApplyP95Ms, 60);
assert.equal(log.perf.rgbTilesReadyPlanningFrames, 0);
assert.equal(log.perf.rgbTilesPartialPlanningFrames, 1);
assert.equal(log.perf.rgbTilesUnknownPlanningFrames, 0);
assert.equal(log.perf.rgbTileErrorPlanningFrames, 1);
assert.equal(log.perf.rgbTilesReadyPlanningPercent, 0);
assert.equal(log.perception[0].rgbTilesReady, false);
assert.equal(log.perception[0].rgbReadyFaces, 4);
assert.equal(log.perception[0].rgbReadinessReason, 'tile-error');
assert.deepEqual(log.perf.calibrationIds, ['cal-1']);
assert.ok(log.perf.physicsUpdateIntervalP95Ms >= 0);
assert.equal(log.perception.length, 4);
assert.equal(log.perf.crossSessionPerceptionDropped, 2);
assert.deepEqual(log.navigationSession, { goalId: 'goal-1', generation: '1' });
assert.equal(log.schemaVersion, 2);
assert.ok(log.startTime <= log.endTime);
assert.ok(Number.isFinite(log.frames[0].recordedAtMs));
const loggedUrl = new URL(log.resolvedUrl);
assert.equal(loggedUrl.username, '');
assert.equal(loggedUrl.password, '');
assert.equal(loggedUrl.hash, '');
assert.equal(loggedUrl.searchParams.get('panoProfile'), 'flight');
assert.equal(loggedUrl.searchParams.get('panoFacesPerSlice'), '2');
assert.equal(loggedUrl.searchParams.get('flightPreloadRadius'), '800');
assert.equal(loggedUrl.searchParams.get('flightPreloadMinCoverage'), '0.97');
assert.equal(loggedUrl.searchParams.get('flightPreloadViewTimeoutMs'), '25000');
assert.equal(loggedUrl.searchParams.get('flightPreloadViewAttempts'), '3');
assert.equal(loggedUrl.searchParams.get('flightPreloadStrict'), '1');
for (const secret of [
    'login-secret', 'ion-secret', 'underscore-secret', 'hyphen-secret',
    'compact-secret', 'auth-secret', 'bearer-secret', 'credential-secret',
    'password-secret', 'passwd-secret', 'client-secret', 'fragment-secret',
    'nested-user', 'nested-pass', 'nested-token', 'must-not-enter-evidence',
]) {
    assert.ok(!log.resolvedUrl.includes(secret), `resolvedUrl leaked ${secret}`);
}
assert.equal(loggedUrl.searchParams.has('da360Url'), false);
assert.equal(loggedUrl.searchParams.has('unreviewedField'), false);

globalThis.location.href = 'https://127.0.0.1:8080/'
    + '?pano-profile=https%3A%2F%2Fu%3Ap%40api.example%2F%3Ftoken%3DLEAKME'
    + '&panoProfile=https%3A%2F%2Fu%3Ap%40api.example%2F%3Ftoken%3DNESTED'
    + '&panoFacesPerSlice=not-a-number&depthMs=20';
const invalidAllowlistLogger = new FlightLogger();
invalidAllowlistLogger.start({ x: 0, y: 0, z: 0 }, 0);
invalidAllowlistLogger.stop(false);
const invalidAllowlistLog = JSON.parse(await downloadedBlob.text());
const invalidAllowlistUrl = new URL(invalidAllowlistLog.resolvedUrl);
assert.equal(invalidAllowlistUrl.searchParams.has('pano-profile'), false);
assert.equal(invalidAllowlistUrl.searchParams.has('panoProfile'), false);
assert.equal(invalidAllowlistUrl.searchParams.has('panoFacesPerSlice'), false);
assert.equal(invalidAllowlistUrl.searchParams.get('depthMs'), '20');
assert.ok(!invalidAllowlistLog.resolvedUrl.includes('LEAKME'));
assert.ok(!invalidAllowlistLog.resolvedUrl.includes('NESTED'));

for (const unsafeUrl of [
    'https://evil.example/?panoProfile=flight',
    'http://127.0.0.1:8080/private/token-DEMO_PATH_SECRET?panoProfile=flight',
    'http://127.0.0.1:8080/%ZZ?panoProfile=flight',
    'ftp://127.0.0.1:8080/?panoProfile=flight',
]) {
    globalThis.location.href = unsafeUrl;
    const unsafeUrlLogger = new FlightLogger();
    unsafeUrlLogger.start({ x: 0, y: 0, z: 0 }, 0);
    unsafeUrlLogger.stop(false);
    const unsafeUrlLog = JSON.parse(await downloadedBlob.text());
    assert.equal(unsafeUrlLog.resolvedUrl, null, `unsafe entry URL was logged: ${unsafeUrl}`);
}

globalThis.location.href = 'not an absolute URL?ionToken=must-not-leak';
const malformedUrlLogger = new FlightLogger();
malformedUrlLogger.start({ x: 0, y: 0, z: 0 }, 0);
malformedUrlLogger.stop(false);
const malformedUrlLog = JSON.parse(await downloadedBlob.text());
assert.equal(malformedUrlLog.resolvedUrl, null, 'malformed URLs must fail closed');

URL.createObjectURL = originalCreateObjectURL;
URL.revokeObjectURL = originalRevokeObjectURL;
if (originalLocationDescriptor) {
    Object.defineProperty(globalThis, 'location', originalLocationDescriptor);
} else {
    delete globalThis.location;
}
console.log('\nFlight logger metrics: all assertions passed');
