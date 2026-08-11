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
    captureToApplyDisplacementM: 1.5,
    ageAtFetchStartMs: 22, ageAtResponseHeadersMs: 52, ageAtJsonParsedMs: 58,
    responseBytes: 900, gateWaitMs: 59, depthDecodeMs: null,
    planningDiagnosticsSchemaVersion: 1,
    selectedEndstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
    selectedRawEndstate: [-0.9, -0.7, -0.5, -0.3, -0.1, 0.1, 0.3, 0.5, 0.7],
    selectedCandidateId: 17, selectedActionId: 17, selectedLatticeId: 54,
    selectedScore: 0.125, candidateCount: 72,
    terminalSpeedMps: 14, terminalAccelerationMps2: 5,
    endpointDisplacementM: 17, trajectoryTimeS: 1.125,
    planningVelocityScaleMps: 16, planningAccelerationScaleMps2: 12,
    frameAgeMs: 60, calibrationId: 'cal-1',
    rgbTilesReady: false, rgbReadyFaces: 4, rgbTotalFaces: 6,
    rgbTileError: true, rgbReadinessReason: 'tile-error',
});
logger.recordPerception({
    frameId: 1, goalId: 'goal-1', generation: 1,
    mode: 'depth-preview', outcome: 'applied',
    planningAuthorized: null, trajectoryApplied: null,
    depthPreviewCommitted: true, depthDecodeMs: 12, depthDrawMs: 3,
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
    getControlDiagnostics() {
        return {
            commandType: 'DIRECT_ACCELERATION', source: 'yopo', generation: 1,
            planningFrameId: 21, planningRequestId: 9, selectedCandidateId: 17,
            terminalPhase: 'approach', fallbackReason: null,
            referenceState: {
                x: 1, y: 20, z: 2, vx: 12, vy: 0.1, vz: 3,
                ax: 2, ay: 0, az: -1,
            },
            rawAcceleration: { x: 30, y: 0, z: 0 },
            limitedAcceleration: { x: 25, y: 0, z: 0 },
            requestedForce: { x: 5, y: 9.81, z: 0 },
            allocatedForce: { x: 4, y: 9, z: 0 },
            projectionRatio: 0.9,
            tiltDeg: 42,
            thrustGf: 1234,
            saturation: { horizontal: true, vertical: false, direct: true },
            antiWindup: { horizontal: false, vertical: false },
            trajectoryAgeS: 0.2,
            trajectoryOriginalAgeS: 0.3,
            trajectoryRemainingS: 0.825,
            trajectoryApplyPositionErrorM: 0.4,
            trajectoryApplyVelocityErrorMps: 1.2,
            poly5PeakSpeedMps: 18.2,
            poly5PeakAccelerationMps2: 38.5,
            trajectoryEndpointGoalDistanceM: 3.2,
            terminalTrajectoryEligible: true,
            terminalSettledTimeS: 0.2,
            overrunCount: 2,
            overrunDroppedSeconds: 0.15,
        };
    },
};
const schedule = {
    steps: 4, frameSeconds: 0.02, simulatedThisFrameSeconds: 0.02,
    droppedSeconds: 0, totalDroppedSeconds: 0,
};
logger.record(drone, 0, 20, 0, schedule);
await new Promise(resolve => setTimeout(resolve, 2));
logger.record(drone, 0, 20, 0, schedule);
logger.stop(false);

const log = JSON.parse(await downloadedBlob.text());
assert.equal(log.perf.uniquePlanningFrames, 1, 'duplicate applies for one frame count once');
assert.equal(log.perf.droppedByReason['old-frame'], 1);
assert.equal(log.perf.droppedByReason['da360-relative-is-preview-only'], 1);
assert.equal(log.perf.captureToApplyP95Ms, 60);
assert.equal(log.perf.captureToApplyDisplacementP95M, 1.5);
assert.equal(log.perf.ageAtJsonParsedP95Ms, 58);
assert.equal(log.perf.responseBytesP95, 900);
assert.equal(log.perf.depthDecodeP95Ms, 12);
assert.equal(log.perf.depthDrawP95Ms, 3);
assert.equal(log.perf.selectedTerminalSpeedP95Mps, 14);
assert.equal(log.perf.selectedTerminalAccelerationP95Mps2, 5);
assert.equal(log.perf.controlDirectSaturationPercent, 100);
assert.equal(log.perf.controlHorizontalSaturationPercent, 100);
assert.equal(log.perf.controlVerticalSaturationPercent, 0);
assert.equal(log.perf.controlHorizontalArwPercent, 0);
assert.equal(log.perf.controlVerticalArwPercent, 0);
assert.equal(log.perf.controlProjectionRatioP05, 0.9);
assert.equal(log.perf.poly5PeakAccelerationMaxMps2, 38.5);
assert.equal(log.perf.poly5PeakSpeedMaxMps, 18.2);
assert.equal(log.perf.trajectoryApplyPositionErrorP95M, 0.4);
assert.equal(log.perf.trajectoryApplyVelocityErrorP95Mps, 1.2);
assert.equal(log.perf.controlOverrunCountMax, 2);
assert.equal(log.perf.controlOverrunDroppedSecondsMax, 0.15);
assert.equal(log.perf.schedulerTotalDroppedSecondsMax, 0);
assert.equal(log.perf.terminalPhaseCounts.approach, 2);
assert.equal(log.perf.droppedByModeReason.planning['old-frame'], 1);
assert.equal(log.perf.rgbTilesReadyPlanningFrames, 0);
assert.equal(log.perf.rgbTilesPartialPlanningFrames, 1);
assert.equal(log.perf.rgbTilesUnknownPlanningFrames, 0);
assert.equal(log.perf.rgbTileErrorPlanningFrames, 1);
assert.equal(log.perf.rgbTilesReadyPlanningPercent, 0);
assert.equal(log.perception[0].rgbTilesReady, false);
assert.equal(log.perception[0].rgbReadyFaces, 4);
assert.equal(log.perception[0].rgbReadinessReason, 'tile-error');
assert.equal(log.perception[0].planningDiagnosticsSchemaVersion, 1);
assert.deepEqual(log.perception[0].selectedEndstate, [1, 2, 3, 4, 5, 6, 7, 8, 9]);
assert.deepEqual(
    log.perception[0].selectedRawEndstate,
    [-0.9, -0.7, -0.5, -0.3, -0.1, 0.1, 0.3, 0.5, 0.7],
);
assert.equal(log.perception[0].selectedCandidateId, 17);
assert.equal(log.perception[0].selectedActionId, 17);
assert.equal(log.perception[0].selectedLatticeId, 54);
assert.equal(log.perception[0].selectedScore, 0.125);
assert.equal(log.perception[0].terminalAccelerationMps2, 5);
assert.equal(log.perception[0].trajectoryTimeS, 1.125);
assert.deepEqual(log.perf.calibrationIds, ['cal-1']);
assert.ok(log.perf.physicsUpdateIntervalP95Ms >= 0);
assert.equal(log.perception.length, 5);
assert.equal(log.perf.crossSessionPerceptionDropped, 2);
assert.deepEqual(log.navigationSession, { goalId: 'goal-1', generation: '1' });
assert.equal(log.schemaVersion, 2);
assert.ok(log.startTime <= log.endTime);
assert.ok(Number.isFinite(log.frames[0].recordedAtMs));
assert.equal(log.frames[0].control.commandType, 'DIRECT_ACCELERATION');
assert.equal(log.frames[0].control.frame, 'sim-world-y-up');
assert.deepEqual(log.frames[0].control.referenceVelocity, { x: 12, y: 0.1, z: 3 });
assert.deepEqual(log.frames[0].control.rawAcceleration, { x: 30, y: 0, z: 0 });
assert.equal(log.frames[0].control.projectionRatio, 0.9);
assert.equal(log.frames[0].control.saturation.direct, true);
assert.equal(log.frames[0].control.trajectoryOriginalAgeS, 0.3);
assert.equal(log.frames[0].control.trajectoryApplyPositionErrorM, 0.4);
assert.equal(log.frames[0].control.trajectoryApplyVelocityErrorMps, 1.2);
assert.equal(log.frames[0].control.planningFrameId, 21);
assert.equal(log.frames[0].control.selectedCandidateId, 17);
assert.equal(log.frames[0].control.poly5PeakAccelerationMps2, 38.5);
assert.equal(log.frames[0].control.terminalTrajectoryEligible, true);
assert.equal(log.frames[0].scheduler.steps, 4);
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

const missingDiagnosticsLogger = new FlightLogger();
missingDiagnosticsLogger.start({ x: 0, y: 0, z: 0 }, 0);
missingDiagnosticsLogger.record({
    ...drone,
    _yopoPolyX: null,
    getControlDiagnostics() {
        return {
            commandType: 'FAILSAFE_HOLD',
            projectionRatio: null,
            trajectoryAgeS: undefined,
            trajectoryOriginalAgeS: '',
            trajectoryRemainingS: null,
            trajectoryApplyPositionErrorM: undefined,
            trajectoryApplyVelocityErrorMps: '',
            poly5PeakSpeedMps: null,
            poly5PeakAccelerationMps2: undefined,
            trajectoryEndpointGoalDistanceM: '',
            terminalSettledTimeS: null,
            overrunDroppedSeconds: '',
        };
    },
}, null, null, null);
missingDiagnosticsLogger.stop(false);
const missingDiagnosticsLog = JSON.parse(await downloadedBlob.text());
const missingControl = missingDiagnosticsLog.frames[0].control;
assert.equal(missingControl.projectionRatio, null);
assert.equal(missingControl.trajectoryAgeS, null);
assert.equal(missingControl.trajectoryOriginalAgeS, null);
assert.equal(missingControl.trajectoryRemainingS, null);
assert.equal(missingControl.trajectoryApplyPositionErrorM, null);
assert.equal(missingControl.trajectoryApplyVelocityErrorMps, null);
assert.equal(missingControl.poly5PeakSpeedMps, null);
assert.equal(missingControl.poly5PeakAccelerationMps2, null);
assert.equal(missingControl.trajectoryEndpointGoalDistanceM, null);
assert.equal(missingControl.terminalSettledTimeS, null);
assert.equal(missingControl.overrunCount, null);
assert.equal(missingControl.overrunDroppedSeconds, null);
assert.equal(missingDiagnosticsLog.perf.controlProjectionRatioP05, null);
assert.equal(missingDiagnosticsLog.perf.poly5PeakAccelerationMaxMps2, null);
assert.equal(missingDiagnosticsLog.perf.poly5PeakSpeedMaxMps, null);
assert.equal(missingDiagnosticsLog.perf.trajectoryApplyPositionErrorP95M, null);
assert.equal(missingDiagnosticsLog.perf.trajectoryApplyVelocityErrorP95Mps, null);
assert.equal(missingDiagnosticsLog.perf.controlOverrunCountMax, null);
assert.equal(missingDiagnosticsLog.perf.controlOverrunDroppedSecondsMax, null);

const zeroOverrunLogger = new FlightLogger();
zeroOverrunLogger.start({ x: 0, y: 0, z: 0 }, 0);
zeroOverrunLogger.record({
    ...drone,
    _yopoPolyX: null,
    getControlDiagnostics() {
        return { commandType: 'FAILSAFE_HOLD', overrunCount: 0 };
    },
}, null, null, null);
zeroOverrunLogger.stop(false);
const zeroOverrunLog = JSON.parse(await downloadedBlob.text());
assert.equal(zeroOverrunLog.frames[0].control.overrunCount, 0);
assert.equal(zeroOverrunLog.perf.controlOverrunCountMax, 0);

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
