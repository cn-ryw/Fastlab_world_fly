/**
 * Panorama depth display/session regression tests. No browser or GPU is required.
 * Run: node tests/test_panorama_depth_state.js
 */

import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

class FakeElement {
    constructor(id = '') {
        this.id = id;
        this.textContent = '';
        this.title = '';
        this.dataset = {};
        this.style = {};
        this.checked = true;
        this.classList = { toggle() {}, add() {}, remove() {} };
    }
    setAttribute() {}
}

class FakeContext2D {
    constructor() {
        this.drawCalls = [];
    }
    clearRect() {}
    fillRect() {}
    beginPath() {}
    moveTo() {}
    lineTo() {}
    stroke() {}
    fillText() {}
    putImageData() {}
    drawImage(...args) { this.drawCalls.push(args); }
    createImageData(width, height) {
        return { data: new Uint8ClampedArray(width * height * 4) };
    }
    createLinearGradient() { return { addColorStop() {} }; }
}

class FakeCanvas extends FakeElement {
    constructor(id = '', width = 384, height = 192) {
        super(id);
        this.width = width;
        this.height = height;
        this.context = new FakeContext2D();
    }
    getContext() { return this.context; }
    toBlob(callback, type = 'image/jpeg') {
        callback(new Blob(['rgb-frame'], { type }));
    }
}

const elements = new Map([
    ['panorama-sensor-panel', new FakeElement('panorama-sensor-panel')],
    ['panorama-rgb-canvas', new FakeCanvas('panorama-rgb-canvas')],
    ['panorama-depth-canvas', new FakeCanvas('panorama-depth-canvas')],
    ['panorama-rgb-status', new FakeElement('panorama-rgb-status')],
    ['panorama-depth-status', new FakeElement('panorama-depth-status')],
    ['panorama-depth-near-label', new FakeElement('panorama-depth-near-label')],
    ['panorama-depth-far-label', new FakeElement('panorama-depth-far-label')],
    ['panorama-depth-unit', new FakeElement('panorama-depth-unit')],
]);

globalThis.HTMLCanvasElement = FakeCanvas;
globalThis.document = {
    body: { appendChild() {} },
    getElementById(id) { return elements.get(id) || null; },
    createElement(tag) { return tag === 'canvas' ? new FakeCanvas() : new FakeElement(); },
};
globalThis.window = {
    // The hard planning-observation envelope is not user-relaxable. Passing
    // 2000 here must still cap the effective value at 250 ms.
    location: {
        search: '?da360UploadScale=1&yopoMaxFrameAgeMs=2000',
        hostname: '127.0.0.1',
    },
    setTimeout: globalThis.setTimeout.bind(globalThis),
    clearTimeout: globalThis.clearTimeout.bind(globalThis),
};

const { PanoramaSensor, decodeDepthImageSource } = await import('../src/panorama-sensor.js?depth-state-test');
const DEPTH_JPEG = 'data:image/jpeg;base64,AA==';
const RELATIVE_POLAR_SCAN = Object.freeze({
    schema_version: 1,
    depth_mode: 'da360-relative',
    unit: 'x-near-reference',
    radius: 20,
    angle_start_deg: -135,
    angle_step_deg: 90,
    angle_positive: 'body-left',
    normalization: 'per-frame-depth-p02',
    valid_fraction: 1,
    values: [1, 2, 3, 4],
});
const METRIC_POLAR_SCAN = Object.freeze({
    schema_version: 1,
    depth_mode: 'da360-metric',
    unit: 'metres',
    radius: 20,
    angle_start_deg: -135,
    angle_step_deg: 90,
    angle_positive: 'body-left',
    valid_fraction: 1,
    values: [1, 2, 3, 4],
});
const VALID_PLANNING_DIAGNOSTICS = Object.freeze({
    schema_version: 1,
    selected_endstate_raw: Object.freeze([0, 0, 0, 0, 0, 0, 0, 0, 0]),
    selected_candidate_id: 0,
    selected_action_id: 0,
    selected_lattice_id: 0,
    selected_score: 0.25,
    terminal_speed_mps: 1,
    terminal_acceleration_mps2: 0,
    endpoint_displacement_m: 1,
    trajectory_time_s: 1,
    candidate_count: 1,
    velocity_scale_mps: 15,
    acceleration_scale_mps2: 10,
});

function fakeBitmap(label = 'depth') {
    return { width: 8, height: 4, label, closed: false, close() { this.closed = true; } };
}

function response(payload) {
    const normalizedPayload = payload?.planning_authorized === true
        && payload.planning_diagnostics === undefined
        ? { ...payload, planning_diagnostics: { ...VALID_PLANNING_DIAGNOSTICS } }
        : payload;
    const bytes = new TextEncoder().encode(JSON.stringify(normalizedPayload)).byteLength;
    return {
        ok: true,
        status: 200,
        headers: { get(name) { return name === 'Content-Length' ? String(bytes) : null; } },
        async json() { return normalizedPayload; },
    };
}

function newSensorWithFrame() {
    elements.get('panorama-rgb-canvas').context.drawCalls.length = 0;
    elements.get('panorama-depth-canvas').context.drawCalls.length = 0;
    const sensor = new PanoramaSensor();
    const rgbFrame = new FakeCanvas('captured-rgb');
    assert.equal(sensor.primeFromCaptureResult(rgbFrame, 0, {
        transform: {
            position: { x: 0, y: 0, z: 0 },
            rotation: { x: 0, y: 0, z: 0 },
        },
    }), true);
    return sensor;
}

// request-started is UI-only and repeated terminal states are coalesced so
// Firefox DevTools is not flooded at perception rate. Distinct terminal,
// error, stale, blocked, and rejected states remain visible.
{
    const originalLog = console.log;
    const messages = [];
    console.log = (...args) => messages.push(args.join(' '));
    try {
        const sensor = new PanoramaSensor();
        messages.length = 0;
        sensor._setDepthState('preview', 'request-started');
        sensor._setDepthState('preview', 'depth-ready', { outcome: 'applied' });
        sensor._setDepthState('preview', 'request-started');
        sensor._setDepthState('preview', 'depth-ready', { outcome: 'applied' });
        assert.equal(
            messages.filter(message => message.includes('[depth-state]')).length,
            1,
            'healthy per-frame transitions should produce one session-level state log',
        );
        assert.match(messages[0], /reason=depth-ready/);

        sensor._setDepthState('preview', 'response-after-session-change', { outcome: 'stale' });
        sensor._setDepthState('preview', 'response-after-session-change', { outcome: 'stale' });
        assert.equal(
            messages.filter(message => message.includes('[depth-state]')).length,
            2,
            'a distinct stale outcome remains visible without repeating forever',
        );

        sensor._goalId = 'goal-log-test';
        sensor._yopoGeneration = 1;
        for (let frame = 0; frame < 3; frame++) {
            sensor._rgbFrameId = frame + 1;
            sensor._setDepthState('planning', 'request-started');
            sensor._setDepthState('planning', 'trajectory-ready', { outcome: 'applied' });
        }
        assert.equal(
            messages.filter(message => message.includes('reason=trajectory-ready')).length,
            1,
            'successful planning frames should produce one state line per goal generation',
        );

        for (let frame = 0; frame < 3; frame++) {
            sensor._rgbFrameId = frame + 4;
            sensor._setDepthState('planning', 'request-started');
            sensor._setDepthState('planning', 'blocked:preview-only', { outcome: 'blocked' });
        }
        assert.equal(
            messages.filter(message => message.includes('reason=blocked:preview-only')).length,
            1,
            'repeated planning blocks should be logged once instead of once per frame',
        );

        sensor._setDepthState('planning', 'trajectory-apply-rejected', { outcome: 'rejected' });
        sensor._setDepthState('error', 'HTTP 503', { outcome: 'error' });
        assert.ok(messages.some(message => message.includes('reason=trajectory-apply-rejected')));
        assert.ok(messages.some(message => message.includes('reason=HTTP 503')));

        const staleRequest = {
            mode: 'planning', goalId: 'goal-log-test', generation: 1,
            frameId: 9, gateAcquiredAt: performance.now(), frameContext: null,
        };
        for (let index = 0; index < 5; index++) {
            sensor._markStale(staleRequest, 'planning-frame-too-old');
        }
        assert.equal(
            messages.filter(message => message.includes('[depth-stale]')).length,
            1,
            'high-rate planning age drops are summarized once per throttle window',
        );
        assert.equal(
            messages.filter(message => message.includes('reason=stale:planning-frame-too-old')).length,
            0,
            'ready/stale UI alternation must not bypass stale-log throttling',
        );
    } finally {
        console.log = originalLog;
    }
}

function primeCalibrationFrame(sensor, label = 'calibration-rgb', captureMs = 0) {
    assert.equal(sensor.getCaptureProfile(), 'calibration');
    assert.equal(sensor.primeFromCaptureResult(
        new FakeCanvas(label),
        captureMs,
        { captureProfile: 'calibration' },
    ), true);
    assert.equal(sensor._rgbFrameContext.captureProfile, 'calibration');
}

async function nextTask() {
    await new Promise(resolve => setTimeout(resolve, 0));
}

// Preload arguments remain positional: structured capture timings come from
// the capture result, while capture time/state/profile come from the context.
{
    const sensor = new PanoramaSensor();
    const result = {
        complete: true,
        canvas: new FakeCanvas('structured-preload'),
        timings_ms: { total: 27, render: 9, tile_wait: 18 },
    };
    assert.equal(sensor.primeFromCaptureResult(result, 999, {
        capturedAt: 123.5,
        transform: { position: { x: 1, y: 2, z: 3 } },
        captureProfile: 'calibration',
    }), true);
    assert.equal(sensor._rgbFrameContext.capturedAt, 123.5);
    assert.equal(sensor._rgbFrameContext.captureProfile, 'calibration');
    assert.deepEqual(sensor._rgbFrameContext.captureTimings, result.timings_ms);
    const materialized = await sensor._materializePerceptionFrame(sensor._rgbFrameContext);
    assert.equal(materialized.frame.captureProfile, 'calibration');
    assert.deepEqual(materialized.frame.transform.position, { x: 1, y: 2, z: 3 });
}

// Decoder supports a DOM-free Image fallback as well as createImageBitmap.
{
    class GoodImage {
        constructor() { this.width = 4; this.height = 2; this.naturalWidth = 4; this.naturalHeight = 2; }
        set src(_value) { queueMicrotask(() => this.onload()); }
    }
    const decoded = await decodeDepthImageSource(DEPTH_JPEG, { Image: GoodImage });
    assert.equal(decoded.width, 4);

    class BadImage {
        set src(_value) { queueMicrotask(() => this.onerror()); }
    }
    await assert.rejects(
        decodeDepthImageSource(DEPTH_JPEG, { Image: BadImage }),
        /depth JPEG decode failed/
    );
}

// hasDepth changes only after asynchronous JPEG decode and canvas commit.
{
    const sensor = newSensorWithFrame();
    let resolveDecode;
    globalThis.createImageBitmap = () => new Promise(resolve => { resolveDecode = resolve; });
    globalThis.fetch = async () => response({
        depth_image: DEPTH_JPEG,
        depth_mode: 'da360-relative',
        polar_scan: RELATIVE_POLAR_SCAN,
        latency_ms: 12,
    });

    const pending = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    assert.equal(sensor.hasDepth, false, 'network success alone must not mark depth ready');
    assert.equal(sensor.depthCanvas.context.drawCalls.length, 0, 'decode-pending image must not be drawn');

    resolveDecode(fakeBitmap('preview'));
    assert.equal(await pending, true);
    assert.equal(sensor.hasDepth, true);
    assert.equal(sensor.depthCanvas.context.drawCalls.length, 1);
    assert.equal(sensor.getDepthState().mode, 'preview');
    assert.equal(sensor.depthStatusEl.textContent, 'preview 12ms');
    assert.equal(sensor.getDepthPolarScan().frameId, 1);
    assert.equal(sensor.getDepthPolarScan().captureYawDeg, 0);
    assert.equal(sensor.getDepthPolarScan().scan.unit, 'x-near-reference');
    sensor.reset();
    assert.equal(sensor.getDepthPolarScan(), null,
        'reset/teleport must not retain an ego-centred outline from the old pose');
}

// Decode failure keeps the canvas untouched and exposes an error state.
{
    const sensor = newSensorWithFrame();
    globalThis.createImageBitmap = async () => { throw new Error('corrupt JPEG'); };
    globalThis.fetch = async () => response({ depth_image: DEPTH_JPEG, latency_ms: 9 });

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
    assert.equal(sensor.hasDepth, false);
    assert.equal(sensor.depthCanvas.context.drawCalls.length, 0);
    assert.equal(sensor.getDepthState().mode, 'error');
    assert.match(sensor.depthStatusEl.textContent, /^error/);
}

// A profile switch cannot rewrite the provenance of a frame already in flight.
// Metrics must describe the frozen RGB frame, not the sensor's current profile.
{
    const sensor = newSensorWithFrame();
    let resolveDecode;
    let metrics = null;
    globalThis.createImageBitmap = () => new Promise(resolve => { resolveDecode = resolve; });
    globalThis.fetch = async () => response({ depth_image: DEPTH_JPEG, latency_ms: 10 });
    sensor.onPerceptionMetrics = value => { metrics = value; };

    const pending = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    sensor.setCaptureProfile('calibration');
    resolveDecode(fakeBitmap('frozen-flight-profile'));
    assert.equal(await pending, true);
    assert.equal(metrics.captureProfile, 'flight');
    assert.equal(sensor.getCaptureProfile(), 'calibration');
}

// preview -> planning uses a new RGB frame; reset returns to preview and retains pixels.
{
    const sensor = newSensorWithFrame();
    const calls = [];
    let lastRequestOptions = null;
    globalThis.createImageBitmap = async () => fakeBitmap();
    globalThis.fetch = async (url, options) => {
        lastRequestOptions = options;
        calls.push(String(url));
        if (String(url).includes('/yopo/plan_full')) {
            return response({
                depth_image: DEPTH_JPEG,
                latency_ms: 18,
                planning_authorized: true,
                planning_reason: 'experimental-unaccepted-da360-metric',
                endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
                traj_time: 1.125,
                depth_mode: 'da360-metric',
                calibration_id: 'calibration-v1',
                calibration_accuracy_accepted: false,
                service_session_id: 'service-v1',
                planning_diagnostics: {
                    schema_version: 1,
                    selected_endstate_raw: [0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
                    selected_candidate_id: 5,
                    selected_action_id: 5,
                    selected_lattice_id: 58,
                    selected_score: 0.25,
                    terminal_speed_mps: 14.5,
                    terminal_acceleration_mps2: 2.5,
                    endpoint_displacement_m: 13.2,
                    trajectory_time_s: 1.125,
                    candidate_count: 64,
                    velocity_scale_mps: 15,
                    acceleration_scale_mps2: 10,
                },
                frame_id: '2',
                goal_id: 'goal-1',
                generation: '1',
            });
        }
        return response({ depth_image: DEPTH_JPEG, latency_ms: 11 });
    };

    await sensor._requestDepth(sensor.rgbCanvas);
    assert.equal(sensor.getDepthState().mode, 'preview');
    const retainedDrawCount = sensor.depthCanvas.context.drawCalls.length;

    sensor.setYopoPose({
        actualState: {
            position: { x: 1, y: 2, z: 3 },
            velocity: { x: 4, y: 5, z: 6 },
        },
        referenceState: {
            position: { x: 11, y: 12, z: 13 },
            velocity: { x: 7, y: 8, z: 9 },
            acceleration: { x: 0.1, y: 0.2, z: 0.3 },
        },
        yaw: 0.25,
        simTimeS: 12.5,
    });
    const goalId = sensor.setYopoGoal({ x: 10, y: 20, z: 30 });
    assert.equal(sensor.getDepthState().mode, 'planning');
    assert.equal(sensor._shouldRequestDepth(), false, 'old RGB frame must not be reused for a new goal');

    sensor.primeFromCaptureResult(new FakeCanvas('next-rgb'));
    // The capture contract must stay frozen even if the vehicle advances before
    // the response is installed.
    sensor.setYopoPose({
        actualState: {
            position: { x: 2, y: 2, z: 3 },
            velocity: { x: 4, y: 5, z: 6 },
        },
        referenceState: {
            position: { x: 12, y: 12, z: 13 },
            velocity: { x: 7, y: 8, z: 9 },
            acceleration: { x: 0.1, y: 0.2, z: 0.3 },
        },
        yaw: 0.25,
        simTimeS: 12.6,
    });
    let callbackContext = null;
    let planningMetrics = null;
    sensor.onYopoResult = (_endstate, _trajTime, context) => {
        callbackContext = context;
        return true;
    };
    sensor.onPerceptionMetrics = metrics => {
        if (metrics.mode === 'planning') planningMetrics = metrics;
    };
    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);
    await sensor._waitForDepthPreviewIdle();
    assert.match(calls.at(-1), /\/yopo\/plan_full\?/);
    assert.match(calls.at(-1), new RegExp(`goal_id=${goalId}`));
    const planningUrl = new URL(calls.at(-1));
    assert.equal(planningUrl.searchParams.get('px'), '1', 'trajectory endpoint origin uses frame actual position');
    assert.equal(planningUrl.searchParams.get('rpx'), '11', 'YOPO goal delta origin uses frame reference position');
    assert.equal(planningUrl.searchParams.get('vx'), '4', 'YOPO observation keeps frame actual velocity');
    assert.equal(planningUrl.searchParams.get('ax'), '0.1', 'YOPO observation uses frame reference acceleration');
    const runtimeProjection = JSON.parse(lastRequestOptions.headers['X-Projection-Config']);
    assert.equal(runtimeProjection.verticalFovDeg, 180,
        'planning request carries the frozen frame projection contract');
    assert.equal(runtimeProjection.jpegQuality, 0.74,
        'planning request carries the actual JPEG encoder quality');
    assert.equal(callbackContext.goalId, goalId);
    assert.equal(callbackContext.mode, 'planning');
    assert.equal(callbackContext.serviceSessionId, 'service-v1');
    assert.equal(callbackContext.calibrationAccuracyAccepted, false);
    assert.equal(callbackContext.captureSimTimeS, 12.5);
    assert.equal(callbackContext.captureActualState.position.x, 1);
    assert.equal(callbackContext.applyActualState.position.x, 2);
    assert.deepEqual(callbackContext.captureToApplyDelta, { x: 1, y: 0, z: 0 });
    assert.equal(callbackContext.captureToApplyDisplacementM, 1);
    assert.equal(callbackContext.planningDiagnosticsSchemaVersion, 1);
    assert.equal(callbackContext.selectedCandidateId, 5);
    assert.equal(callbackContext.selectedActionId, 5);
    assert.equal(callbackContext.selectedLatticeId, 58);
    assert.equal(callbackContext.terminalSpeedMps, 14.5);
    assert.equal(callbackContext.terminalAccelerationMps2, 2.5);
    assert.equal(callbackContext.endpointDisplacementM, 13.2);
    assert.equal(callbackContext.velocityScaleMps, 15);
    assert.deepEqual(
        callbackContext.planningDiagnostics.selected_endstate_raw,
        [0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
    );
    assert.equal(planningMetrics.trajectoryApplied, true);
    assert.ok(Number.isFinite(planningMetrics.trajectoryAppliedAtMs));
    assert.equal(planningMetrics.depthMode, 'da360-metric');
    assert.equal(planningMetrics.calibrationId, 'calibration-v1');
    assert.equal(planningMetrics.calibrationAccuracyAccepted, false);
    assert.equal(planningMetrics.serviceSessionId, 'service-v1');
    assert.equal(planningMetrics.captureToApplyDisplacementM, 1);
    assert.ok(Number.isFinite(planningMetrics.ageAtFetchStartMs));
    assert.ok(Number.isFinite(planningMetrics.ageAtResponseHeadersMs));
    assert.ok(Number.isFinite(planningMetrics.ageAtJsonParsedMs));
    assert.ok(planningMetrics.responseBytes > 0);
    assert.ok(Number.isFinite(planningMetrics.gateWaitMs));
    assert.equal(planningMetrics.planningDiagnosticsSchemaVersion, 1);

    sensor.resetYopoGoal('cancelled');
    assert.equal(sensor.getDepthState().mode, 'preview');
    assert.equal(sensor.hasDepth, true);
    assert.equal(sensor.depthCanvas.context.drawCalls.length, retainedDrawCount + 1,
        'cancel must retain the last depth pixels');
}

// Planning success is acknowledged and timestamped before the optional depth
// preview JPEG finishes decoding; a rejected install cannot count as applied.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('planning-order-rgb'));
    let resolveDecode;
    let callbackCalled = false;
    const metrics = [];
    globalThis.createImageBitmap = () => new Promise(resolve => { resolveDecode = resolve; });
    globalThis.fetch = async () => response({
        depth_image: DEPTH_JPEG,
        latency_ms: 30,
        planning_authorized: true,
        planning_reason: 'validated-da360-metric',
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-order',
        service_session_id: 'svc-order',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => { callbackCalled = true; return true; };
    sensor.onPerceptionMetrics = value => { metrics.push(value); };
    const pending = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    assert.equal(callbackCalled, true, 'trajectory apply must not wait for preview decode');
    assert.equal(sensor._depthGate, false,
        'trajectory response releases the request gate before preview decode');
    assert.equal(await pending, true,
        'the control request resolves while its optional preview is still pending');
    const appliedMetrics = metrics.find(value => value.mode === 'planning');
    assert.equal(appliedMetrics.trajectoryApplied, true,
        'trajectory evidence is emitted synchronously with the install');
    assert.equal(appliedMetrics.depthPreviewCommitted, null,
        'optional preview outcome is not part of the trajectory acknowledgement');
    resolveDecode(fakeBitmap('planning-order-depth'));
    await sensor._waitForDepthPreviewIdle();
    assert.equal(appliedMetrics.trajectoryApplied, true);
    assert.ok(appliedMetrics.captureToApplyMs <= appliedMetrics.frameAgeMs);
    assert.equal(metrics.filter(value => value.mode === 'planning').length, 1);
    assert.equal(metrics.filter(value => value.mode === 'depth-preview').length, 1);
    const displayMetrics = metrics.find(value => value.mode === 'depth-preview');
    assert.ok(Number.isFinite(displayMetrics.depthDecodeMs));
    assert.ok(Number.isFinite(displayMetrics.depthDrawMs));

    const rejected = newSensorWithFrame();
    rejected.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    rejected.setYopoGoal({ x: 30, y: 100, z: 0 });
    rejected.primeFromCaptureResult(new FakeCanvas('planning-reject-rgb'));
    let rejectedMetrics = null;
    globalThis.createImageBitmap = async () => fakeBitmap('planning-reject-depth');
    rejected.onYopoResult = () => false;
    rejected.onPerceptionMetrics = value => {
        if (value.mode === 'planning') rejectedMetrics = value;
    };
    assert.equal(await rejected._requestDepth(rejected.rgbCanvas), true);
    await rejected._waitForDepthPreviewIdle();
    assert.equal(rejectedMetrics.outcome, 'rejected');
    assert.equal(rejectedMetrics.trajectoryApplied, false);
    assert.equal(rejectedMetrics.dropReason, 'trajectory-apply-rejected');
    assert.match(rejected.depthStatusEl.textContent, /rejected/);

    const ignored = newSensorWithFrame();
    ignored.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    ignored.setYopoGoal({ x: 30, y: 100, z: 0 });
    ignored.primeFromCaptureResult(new FakeCanvas('planning-ignore-rgb'));
    let ignoredMetrics = null;
    ignored.onYopoResult = () => ({
        outcome: 'ignored',
        reason: 'terminal-committed',
    });
    ignored.onPerceptionMetrics = value => {
        if (value.mode === 'planning') ignoredMetrics = value;
    };
    assert.equal(await ignored._requestDepth(ignored.rgbCanvas), true);
    await ignored._waitForDepthPreviewIdle();
    assert.equal(ignoredMetrics.outcome, 'ignored');
    assert.equal(ignoredMetrics.trajectoryApplied, false);
    assert.equal(ignoredMetrics.trajectoryIgnored, true);
    assert.equal(ignoredMetrics.dropReason, 'terminal-committed');
    assert.match(ignored.depthStatusEl.textContent, /paused/);
}

// Terminal ownership pauses network planning without disabling live RGB
// capture. Once released, only a newly captured frame may be planned.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('planning-pause-rgb'));
    assert.equal(sensor.setYopoPlanningPaused(true, 'terminal-committed'), true);
    assert.equal(sensor._shouldRequestDepth(), false,
        'terminal pause suppresses otherwise eligible planning requests');
    assert.equal(sensor.getDepthState().planningPaused, true);
    assert.equal(sensor.setYopoPlanningPaused(false, 'terminal-aborted'), true);
    assert.equal(sensor.getDepthState().planningPaused, false);
    assert.equal(sensor._shouldRequestDepth(), false,
        'unpause does not reuse the RGB frame captured under terminal ownership');
    sensor.primeFromCaptureResult(new FakeCanvas('planning-resumed-rgb'));
    assert.equal(sensor._shouldRequestDepth(), true,
        'a fresh post-terminal RGB frame resumes ordinary rolling planning');
}

// A slow operator preview is never part of the planning request gate. The next
// compact trajectory request starts and installs while the prior JPEG decode is
// still pending; cached UI previews are sampled at most once per 2 s window.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0, simTimeS: 1 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('slow-preview-first-rgb'));
    let resolvePreviewFetch;
    const planningUrls = [];
    const previewUrls = [];
    let applyCalls = 0;
    globalThis.createImageBitmap = async () => fakeBitmap('independent-preview');
    globalThis.fetch = async url => {
        const parsed = new URL(String(url));
        if (parsed.pathname === '/yopo/preview') {
            previewUrls.push(parsed);
            const previewResponse = response({
                depth_image: DEPTH_JPEG,
                preview_included: true,
                latency_ms: 4,
                depth_mode: 'da360-metric',
                frame_id: parsed.searchParams.get('frame_id'),
                goal_id: parsed.searchParams.get('goal_id'),
                generation: parsed.searchParams.get('generation'),
            });
            if (previewUrls.length === 1) {
                return new Promise(resolve => { resolvePreviewFetch = () => resolve(previewResponse); });
            }
            return previewResponse;
        }
        planningUrls.push(parsed);
        return response({
            preview_included: false,
            preview_available: true,
            latency_ms: 30,
            planning_authorized: true,
            planning_reason: 'validated-da360-metric',
            endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
            traj_time: 1,
            depth_mode: 'da360-metric',
            calibration_id: 'cal-slow-preview',
            service_session_id: 'svc-slow-preview',
            frame_id: parsed.searchParams.get('frame_id'),
            goal_id: parsed.searchParams.get('goal_id'),
            generation: parsed.searchParams.get('generation'),
        });
    };
    sensor.onYopoResult = () => { applyCalls++; return true; };

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);
    assert.equal(sensor._depthGate, false);
    assert.equal(applyCalls, 1);
    assert.equal(planningUrls[0].searchParams.get('include_preview'), '0');
    await nextTask();
    assert.equal(previewUrls.length, 1);

    sensor.setYopoPose({ x: 1, y: 100, z: 0, vx: 1, vy: 0, vz: 0, simTimeS: 1.05 }, 0);
    sensor.primeFromCaptureResult(new FakeCanvas('slow-preview-second-rgb'));
    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true,
        'next control request must finish before the prior preview decode');
    assert.equal(applyCalls, 2);
    assert.equal(planningUrls.length, 2);
    assert.equal(planningUrls[1].searchParams.get('include_preview'), '0');
    assert.equal(previewUrls.length, 1,
        'the stalled independent preview neither blocks nor duplicates planning');

    resolvePreviewFetch();
    await sensor._waitForDepthPreviewIdle();
    assert.equal(sensor.depthCanvas.context.drawCalls.length, 1);

    sensor._lastPlanningPreviewFetchAt -= 2001;
    globalThis.createImageBitmap = async () => fakeBitmap('sampled-preview');
    sensor.setYopoPose({ x: 2, y: 100, z: 0, vx: 1, vy: 0, vz: 0, simTimeS: 1.1 }, 0);
    sensor.primeFromCaptureResult(new FakeCanvas('sampled-preview-third-rgb'));
    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);
    await sensor._waitForDepthPreviewIdle();
    assert.equal(planningUrls[2].searchParams.get('include_preview'), '0');
    assert.equal(previewUrls.length, 2,
        'independent preview sampling resumes after the 2 s cadence window');
}

// If preview-producing planning responses outrun decoding, retain only the
// latest queued preview. The in-progress stale bitmap may finish, but it cannot
// draw, and the superseded middle JPEG is never decoded.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    const leveledTransform = {
        position: { x: 0, y: 100, z: 0 },
        rotation: { x: 0, y: 0, z: 0 },
    };
    sensor.primeFromCaptureResult(
        new FakeCanvas('coalesce-first-rgb'),
        0,
        { transform: leveledTransform },
    );
    let resolveFirstDecode;
    let decodeCalls = 0;
    globalThis.createImageBitmap = () => {
        decodeCalls++;
        if (decodeCalls === 1) {
            return new Promise(resolve => { resolveFirstDecode = resolve; });
        }
        return Promise.resolve(fakeBitmap(`coalesce-${decodeCalls}`));
    };
    globalThis.fetch = async url => {
        const parsed = new URL(String(url));
        const responseFrameId = Number(parsed.searchParams.get('frame_id'));
        return response({
            depth_image: DEPTH_JPEG,
            polar_scan: {
                ...METRIC_POLAR_SCAN,
                values: [responseFrameId, 2, 3, 4],
            },
            preview_included: true,
            latency_ms: 30,
            planning_authorized: true,
            endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
            traj_time: 1,
            depth_mode: 'da360-metric',
            calibration_id: 'cal-coalesce',
            service_session_id: 'svc-coalesce',
            frame_id: parsed.searchParams.get('frame_id'),
            goal_id: parsed.searchParams.get('goal_id'),
            generation: parsed.searchParams.get('generation'),
        });
    };
    sensor.onYopoResult = () => true;
    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);

    for (const [position, label] of [[1, 'middle'], [2, 'latest']]) {
        sensor._lastPlanningPreviewFetchAt -= 2001;
        sensor.setYopoPose({ x: position, y: 100, z: 0, vx: 1, vy: 0, vz: 0 }, 0);
        sensor.primeFromCaptureResult(
            new FakeCanvas(`coalesce-${label}-rgb`),
            0,
            { transform: leveledTransform },
        );
        assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);
    }
    assert.equal(decodeCalls, 1,
        'middle/latest previews queue without waiting for the active decoder');
    assert.equal(sensor.getDepthPolarScan(), null,
        'a polar scan must not publish before its matching bitmap wins the latest gate');
    resolveFirstDecode(fakeBitmap('coalesce-stale-first'));
    await sensor._waitForDepthPreviewIdle();
    assert.equal(decodeCalls, 2, 'the superseded middle preview is never decoded');
    assert.equal(sensor.depthCanvas.context.drawCalls.length, 1,
        'only the latest preview may draw after coalescing');
    assert.equal(sensor.getDepthPolarScan().requestId, 3);
    assert.equal(sensor.getDepthPolarScan().scan.values[0], 4,
        'the committed polar scan and bitmap must belong to the same latest response');
}

// Once the trajectory is installed, a new navigation generation makes only
// the old preview JPEG stale. The old image must not draw or overwrite UI
// state, while the real install acknowledgement stays with the old session.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('planning-stale-preview-rgb'));
    let resolveDecode;
    const metrics = [];
    globalThis.createImageBitmap = () => new Promise(resolve => { resolveDecode = resolve; });
    globalThis.fetch = async () => response({
        depth_image: DEPTH_JPEG,
        latency_ms: 30,
        planning_authorized: true,
        planning_reason: 'validated-da360-metric',
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-stale-preview',
        service_session_id: 'svc-stale-preview',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => true;
    sensor.onPerceptionMetrics = value => { metrics.push(value); };
    const drawCount = sensor.depthCanvas.context.drawCalls.length;
    const pending = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    assert.equal(await pending, true);

    sensor.setYopoGoal({ x: 60, y: 100, z: 0 });
    const newerState = sensor.getDepthState();
    resolveDecode(fakeBitmap('stale-depth-preview'));
    await sensor._waitForDepthPreviewIdle();
    assert.equal(sensor.depthCanvas.context.drawCalls.length, drawCount,
        'stale preview must not draw over the newer frame');
    assert.deepEqual(sensor.getDepthState(), newerState,
        'stale preview must not overwrite the newer session UI state');
    const controlMetrics = metrics.filter(value => value.mode === 'planning');
    const displayMetrics = metrics.filter(value => value.mode === 'depth-preview');
    assert.equal(controlMetrics.length, 1, 'actual apply has one control event');
    assert.equal(controlMetrics[0].outcome, 'applied');
    assert.equal(controlMetrics[0].trajectoryApplied, true);
    assert.equal(controlMetrics[0].depthPreviewCommitted, null);
    assert.equal(controlMetrics[0].depthPreviewError, null);
    assert.equal(displayMetrics.length, 1, 'stale display has a separate diagnostic');
    assert.equal(displayMetrics[0].outcome, 'stale');
}

// Blocked and rejected planning decisions are control outcomes too: they must
// be emitted before optional JPEG decoding, exactly once, and survive a goal
// change while the old display work is still pending.
{
    for (const scenario of [
        {
            label: 'blocked',
            response: {
                depth_image: DEPTH_JPEG,
                latency_ms: 30,
                planning_authorized: false,
                planning_reason: 'da360-relative-is-preview-only',
                depth_mode: 'da360-relative',
                frame_id: '2', goal_id: 'goal-1', generation: '1',
            },
            apply: () => { throw new Error('blocked planning must not invoke YOPO'); },
        },
        {
            label: 'rejected',
            response: {
                depth_image: DEPTH_JPEG,
                latency_ms: 30,
                planning_authorized: true,
                planning_reason: 'validated-da360-metric',
                endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
                traj_time: 1,
                depth_mode: 'da360-metric',
                calibration_id: 'cal-rejected-pending',
                service_session_id: 'svc-rejected-pending',
                frame_id: '2', goal_id: 'goal-1', generation: '1',
            },
            apply: () => false,
        },
    ]) {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(new FakeCanvas(`planning-${scenario.label}-pending-rgb`));
        let resolveDecode;
        const metrics = [];
        globalThis.createImageBitmap = () => new Promise(resolve => { resolveDecode = resolve; });
        globalThis.fetch = async () => response(scenario.response);
        sensor.onYopoResult = scenario.apply;
        sensor.onPerceptionMetrics = value => { metrics.push(value); };
        const drawCount = sensor.depthCanvas.context.drawCalls.length;
        const pending = sensor._requestDepth(sensor.rgbCanvas);
        await nextTask();
        assert.equal(await pending, true);

        const controlBeforeDecode = metrics.filter(value => value.mode === 'planning');
        assert.equal(controlBeforeDecode.length, 1, `${scenario.label} has one immediate control event`);
        assert.equal(controlBeforeDecode[0].outcome, scenario.label);
        assert.equal(controlBeforeDecode[0].trajectoryApplied, false);

        sensor.setYopoGoal({ x: 60, y: 100, z: 0 });
        resolveDecode(fakeBitmap(`${scenario.label}-stale-preview`));
        await sensor._waitForDepthPreviewIdle();
        assert.equal(sensor.depthCanvas.context.drawCalls.length, drawCount,
            `${scenario.label} old preview must not draw into the new goal`);
        assert.equal(metrics.filter(value => value.mode === 'planning').length, 1,
            `${scenario.label} control outcome remains unique`);
        const display = metrics.filter(value => value.mode === 'depth-preview');
        assert.equal(display.length, 1);
        assert.equal(display[0].outcome, 'stale');
    }
}

// A planning trajectory may be valid even when its optional display JPEG is
// corrupt. Keep the applied control evidence and report the canvas failure as
// a separate non-planning diagnostic.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('planning-corrupt-preview-rgb'));
    const metrics = [];
    globalThis.createImageBitmap = async () => { throw new Error('corrupt planning preview'); };
    globalThis.fetch = async () => response({
        depth_image: DEPTH_JPEG,
        latency_ms: 30,
        planning_authorized: true,
        planning_reason: 'validated-da360-metric',
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-corrupt-preview',
        service_session_id: 'svc-corrupt-preview',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => true;
    sensor.onPerceptionMetrics = value => { metrics.push(value); };
    const drawCount = sensor.depthCanvas.context.drawCalls.length;

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);
    await sensor._waitForDepthPreviewIdle();
    const control = metrics.filter(value => value.mode === 'planning');
    const display = metrics.filter(value => value.mode === 'depth-preview');
    assert.equal(control.length, 1);
    assert.equal(control[0].outcome, 'applied');
    assert.equal(control[0].trajectoryApplied, true);
    assert.equal(display.length, 1);
    assert.equal(display[0].outcome, 'error');
    assert.match(display[0].depthPreviewError, /corrupt planning preview/);
    assert.equal(sensor.depthCanvas.context.drawCalls.length, drawCount);
    assert.match(display[0].depthPreviewError, /corrupt planning preview/);
}

// A backend authorization bit is insufficient without the full provenance
// contract required by the trusted closed-loop evaluator.
{
    for (const [label, override, expected] of [
        ['relative-mode', { depth_mode: 'da360-relative' }, /untrusted depth mode/],
        ['missing-calibration', { calibration_id: null }, /missing calibration_id/],
        ['missing-service', { service_session_id: null }, /missing service_sess/],
    ]) {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(new FakeCanvas(`planning-${label}`));
        let callbackCalls = 0;
        globalThis.createImageBitmap = async () => fakeBitmap(`depth-${label}`);
        globalThis.fetch = async () => response({
            depth_image: DEPTH_JPEG,
            latency_ms: 30,
            planning_authorized: true,
            planning_reason: 'validated-da360-metric',
            endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
            traj_time: 1,
            depth_mode: 'da360-metric',
            calibration_id: 'cal-trusted',
            service_session_id: 'svc-trusted',
            frame_id: '2', goal_id: 'goal-1', generation: '1',
            ...override,
        });
        sensor.onYopoResult = () => { callbackCalls++; return true; };
        assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
        assert.equal(callbackCalls, 0, `${label} must be rejected before trajectory apply`);
        assert.match(sensor.depthStatusEl.title, expected);
    }
}

// Relative depth may update the canvas, but it is preview-only and must never
// be reported or applied as a successful planning frame.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(
        new FakeCanvas('relative-planning-rgb'),
        0,
        { capturedAt: performance.now() },
    );
    let yopoCallbacks = 0;
    let metrics = null;
    sensor.onYopoResult = () => { yopoCallbacks++; };
    sensor.onPerceptionMetrics = value => {
        if (value.mode === 'planning') metrics = value;
    };
    globalThis.createImageBitmap = async () => fakeBitmap('relative-preview');
    globalThis.fetch = async () => response({
        depth_image: DEPTH_JPEG,
        latency_ms: 35,
        planning_authorized: false,
        planning_reason: 'da360-relative-is-preview-only',
        depth_mode: 'da360-relative',
        frame_id: '2',
        goal_id: 'goal-1',
        generation: '1',
    });

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true,
        'a blocked planning response may still commit its useful depth preview');
    await sensor._waitForDepthPreviewIdle();
    assert.equal(sensor.hasDepth, true);
    assert.equal(sensor.getDepthState().mode, 'planning');
    assert.equal(sensor.getDepthState().outcome, 'blocked');
    assert.match(sensor.depthStatusEl.textContent, /blocked/);
    assert.equal(yopoCallbacks, 0, 'relative depth never installs a YOPO trajectory');
    assert.equal(metrics.outcome, 'blocked');
    assert.equal(metrics.planningAuthorized, false);
    assert.equal(metrics.dropReason, 'da360-relative-is-preview-only');
}

// A delayed preview response cannot overwrite a new planning generation.
{
    const sensor = newSensorWithFrame();
    let resolveDecode;
    let requestSignal;
    let yopoCallbacks = 0;
    globalThis.createImageBitmap = () => new Promise(resolve => { resolveDecode = resolve; });
    globalThis.fetch = async (_url, options) => {
        requestSignal = options.signal;
        return response({ depth_image: DEPTH_JPEG, latency_ms: 20 });
    };
    sensor.onYopoResult = () => { yopoCallbacks++; };

    const oldRequest = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    const generationBeforeGoal = sensor.getDepthState().generation;
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    const goalId = sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    assert.equal(requestSignal.aborted, true, 'setting a goal must abort the in-flight preview request');
    assert.ok(sensor.getDepthState().generation > generationBeforeGoal);

    resolveDecode(fakeBitmap('stale-preview'));
    assert.equal(await oldRequest, false);
    assert.equal(sensor.hasDepth, false);
    assert.equal(sensor.depthCanvas.context.drawCalls.length, 0);
    assert.equal(yopoCallbacks, 0);
    assert.equal(sensor.getDepthState().mode, 'planning');
    assert.equal(sensor.getDepthState().outcome, 'ok',
        'the stale old request must not overwrite the new session outcome');
    assert.match(sensor.getDepthState().reason, /awaiting-new-rgb-frame/);
    assert.equal(sensor.getDepthState().goalId, goalId);
}

// Collision/expiry/overrun replan boundaries invalidate an in-flight planning
// response even when goal and navigation generation are unchanged.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('planning-before-controller-failure'));

    let resolveFetch;
    let requestSignal = null;
    let callbackCalls = 0;
    globalThis.createImageBitmap = async () => fakeBitmap('stale-controller-failure-preview');
    globalThis.fetch = (_url, options) => {
        requestSignal = options.signal;
        return new Promise(resolve => { resolveFetch = resolve; });
    };
    sensor.onYopoResult = () => { callbackCalls++; return true; };

    const staleRequest = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    const frameBeforeFailure = sensor._rgbFrameId;
    const epochBeforeFailure = sensor._planningEpoch;
    assert.ok(requestSignal, 'controller-failure test reaches an in-flight planning request');

    assert.equal(sensor.requestImmediatePlanningFrame('collision'), true);
    assert.equal(requestSignal.aborted, true,
        'controller failure aborts the in-flight planning request');
    assert.equal(sensor._planningEpoch, epochBeforeFailure + 1,
        'controller failure advances the independent planning epoch');
    assert.equal(sensor._minimumRequestFrameId, frameBeforeFailure + 1,
        'controller failure requires a newly captured RGB frame');

    // Simulate a backend/fetch implementation that resolves despite abort.
    // The local epoch gate must still reject the old response before apply.
    resolveFetch(response({
        depth_image: DEPTH_JPEG,
        latency_ms: 30,
        planning_authorized: true,
        planning_reason: 'validated-da360-metric',
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-before-failure',
        service_session_id: 'svc-before-failure',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    }));
    assert.equal(await staleRequest, false,
        'pre-failure response cannot become current after a replan boundary');
    assert.equal(callbackCalls, 0,
        'pre-failure response never reaches the trajectory install callback');
    assert.equal(sensor._shouldRequestDepth(), false,
        'the old RGB frame cannot be immediately reused after invalidation');
}

// A response remains the newest available depth even if RGB capture has moved
// ahead. With one request in flight it may advance the canvas, never overwrite
// a newer depth request.
{
    const sensor = newSensorWithFrame();
    let resolveFetch;
    const outcomes = [];
    globalThis.createImageBitmap = async () => fakeBitmap('old-frame');
    globalThis.fetch = () => new Promise(resolve => { resolveFetch = resolve; });
    sensor.onPerceptionMetrics = metrics => outcomes.push(metrics);

    const oldRequest = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    sensor.primeFromCaptureResult(new FakeCanvas('newer-rgb'));
    sensor._lastRequestedFrameId = sensor._rgbFrameId; // keep this test focused on the stale response
    resolveFetch(response({ depth_image: DEPTH_JPEG, latency_ms: 14 }));

    assert.equal(await oldRequest, true);
    await sensor._waitForDepthPreviewIdle();
    assert.equal(sensor.hasDepth, true, 'newest available ordered depth becomes ready');
    assert.equal(sensor.depthCanvas.context.drawCalls.length, 1,
        'latest depth response may display one frame behind RGB');
    assert.equal(outcomes.length, 1);
    assert.equal(outcomes[0].outcome, 'applied');
    assert.equal(outcomes[0].depthPreviewLagFrames, 1);
}

// Planning inference is pipelined with the next capture. The frozen frame's
// trajectory remains applicable while goal/generation are current and fresh;
// its ordered depth preview may visibly trail RGB by one frame.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('planning-pipeline-rgb'));
    let resolveFetch;
    let callbackCalls = 0;
    const outcomes = [];
    globalThis.createImageBitmap = async () => fakeBitmap('pipeline-old-preview');
    globalThis.fetch = () => new Promise(resolve => { resolveFetch = resolve; });
    sensor.onYopoResult = () => { callbackCalls++; return true; };
    sensor.onPerceptionMetrics = metrics => outcomes.push(metrics);
    const drawCount = sensor.depthCanvas.context.drawCalls.length;

    const oldRequest = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    sensor.primeFromCaptureResult(new FakeCanvas('planning-pipeline-newer-rgb'));
    sensor._lastRequestedFrameId = sensor._rgbFrameId; // prevent an unrelated queued request
    resolveFetch(response({
        depth_image: DEPTH_JPEG,
        latency_ms: 35,
        planning_authorized: true,
        planning_reason: 'validated-da360-metric',
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-pipeline',
        service_session_id: 'svc-pipeline',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    }));

    assert.equal(await oldRequest, true);
    await sensor._waitForDepthPreviewIdle();
    assert.equal(callbackCalls, 1, 'fresh frozen planning frame must still apply');
    assert.equal(sensor.depthCanvas.context.drawCalls.length, drawCount + 1,
        'latest ordered depth response keeps the planning preview advancing');
    const control = outcomes.filter(value => value.mode === 'planning');
    const display = outcomes.filter(value => value.mode === 'depth-preview');
    assert.equal(control.length, 1);
    assert.equal(control[0].outcome, 'applied');
    assert.equal(control[0].trajectoryApplied, true);
    assert.equal(control[0].frameId, 2, 'metrics retain the applied frozen frame identity');
    assert.equal(control[0].depthPreviewCommitted, null);
    assert.equal(control[0].depthPreviewLagFrames, 1);
    assert.equal(display.length, 1);
    assert.equal(display[0].depthPreviewCommitted, true);
}

// Pipelining does not authorize arbitrarily old observations: a response past
// the hard frame-age envelope is dropped before the trajectory callback.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(
        new FakeCanvas('planning-expired-rgb'),
        0,
        { capturedAt: performance.now() - 300 },
    );
    let callbackCalls = 0;
    let metrics = null;
    let fetchCalls = 0;
    globalThis.createImageBitmap = async () => fakeBitmap('expired-preview');
    globalThis.fetch = async () => {
        fetchCalls++;
        return response({
        depth_image: DEPTH_JPEG,
        latency_ms: 35,
        planning_authorized: true,
        planning_reason: 'validated-da360-metric',
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-expired',
        service_session_id: 'svc-expired',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
        });
    };
    sensor.onYopoResult = () => { callbackCalls++; return true; };
    sensor.onPerceptionMetrics = value => { metrics = value; };

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
    assert.equal(callbackCalls, 0);
    assert.equal(fetchCalls, 0,
        'an already exhausted 250 ms age budget must not consume a GPU request');
    assert.equal(metrics.outcome, 'stale');
    assert.equal(metrics.dropReason, 'planning-frame-too-old');
    assert.equal(metrics.fetchStarted, false);
    assert.equal(metrics.captureMs, 0);
    assert.equal(metrics.renderMs, null);
    assert.equal(metrics.jpegMs, null);
    assert.equal(metrics.fetchStartedAtMs, null);
    assert.equal(metrics.responseHeadersAtMs, null);
    assert.equal(metrics.payloadParsedAtMs, null);
    assert.equal(metrics.responseHeadersMs, null);
    assert.equal(metrics.httpBodyMs, null);
    assert.equal(metrics.responseJsonMs, null);
    assert.equal(metrics.networkMs, null);
    assert.equal(metrics.responseBytes, null);
    assert.equal(metrics.serverMs, null);
    assert.equal(metrics.da360Ms, null);
    assert.equal(metrics.yopoMs, null);
    assert.equal(metrics.planningDiagnostics, null);
    assert.equal(metrics.planningDiagnosticsSchemaVersion, null);
}

// At a 60 Hz render cadence, a response that already consumed more than the
// nominal 20 ms interval must hand the newest frozen frame directly to the next
// planning request. Limiting from response completion would add a 20--37 ms
// idle window; limiting from request start produces a zero-gap 50 ms cadence.
{
    const realPerformance = globalThis.performance;
    const originalWindowSetTimeout = window.setTimeout;
    const originalWindowClearTimeout = window.clearTimeout;
    let clockMs = 1000;
    let timerSequence = 0;
    const timers = new Map();
    Object.defineProperty(globalThis, 'performance', {
        configurable: true,
        value: { now: () => clockMs },
    });
    window.setTimeout = (callback, delay = 0) => {
        const id = ++timerSequence;
        timers.set(id, { callback, dueAt: clockMs + Number(delay) });
        return id;
    };
    window.clearTimeout = id => { timers.delete(id); };
    try {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(
            new FakeCanvas('request-start-cadence-first-rgb'),
            0,
            { capturedAt: clockMs },
        );
        sensor._lastPlanningPreviewFetchAt = clockMs;
        const fetchStarts = [];
        let resolveFirstFetch = null;
        const planningResponse = url => {
            const parsed = new URL(String(url));
            return response({
                preview_included: false,
                preview_available: false,
                latency_ms: 30,
                planning_authorized: true,
                endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
                traj_time: 1,
                depth_mode: 'da360-metric',
                calibration_id: 'cal-request-start-cadence',
                service_session_id: 'svc-request-start-cadence',
                frame_id: parsed.searchParams.get('frame_id'),
                goal_id: parsed.searchParams.get('goal_id'),
                generation: parsed.searchParams.get('generation'),
            });
        };
        globalThis.fetch = url => {
            fetchStarts.push(clockMs);
            if (fetchStarts.length === 1) {
                return new Promise(resolve => { resolveFirstFetch = resolve; });
            }
            return Promise.resolve(planningResponse(url));
        };
        sensor.onYopoResult = () => true;

        const firstRequest = sensor._requestDepth(sensor.rgbCanvas);
        await nextTask();
        assert.deepEqual(fetchStarts, [1000]);

        // A new panorama becomes available on the next 60 Hz frame while the
        // first request remains the sole control request in flight.
        clockMs = 1000 + 1000 / 60;
        sensor.setYopoPose({ x: 1, y: 100, z: 0, vx: 1, vy: 0, vz: 0 }, 0);
        sensor.primeFromCaptureResult(
            new FakeCanvas('request-start-cadence-latest-rgb'),
            0,
            { capturedAt: clockMs },
        );

        clockMs = 1050;
        resolveFirstFetch(planningResponse(
            'http://127.0.0.1:5688/yopo/plan_full?frame_id=2&goal_id=goal-1&generation=1'
        ));
        assert.equal(await firstRequest, true);
        await nextTask();
        assert.deepEqual(fetchStarts, [1000, 1050],
            'response completion must immediately start the latest frame without another 20 ms wait');
        assert.equal(sensor._depthGate, false);
        assert.equal(sensor._depthCatchupTimer, null);
    } finally {
        window.setTimeout = originalWindowSetTimeout;
        window.clearTimeout = originalWindowClearTimeout;
        Object.defineProperty(globalThis, 'performance', {
            configurable: true,
            value: realPerformance,
        });
    }
}

// A sub-20 ms response uses one remaining-time timer and repeated scheduling
// attempts do not create a busy loop or duplicate request.
{
    const realPerformance = globalThis.performance;
    const originalWindowSetTimeout = window.setTimeout;
    const originalWindowClearTimeout = window.clearTimeout;
    let clockMs = 1010;
    let scheduled = null;
    let timerCalls = 0;
    Object.defineProperty(globalThis, 'performance', {
        configurable: true,
        value: { now: () => clockMs },
    });
    window.setTimeout = (callback, delay = 0) => {
        timerCalls++;
        scheduled = { callback, delay };
        return 811;
    };
    window.clearTimeout = () => { scheduled = null; };
    try {
        const sensor = newSensorWithFrame();
        sensor._lastDepthRequestStartedAt = 1000;
        sensor._lastRequestedFrameId = 0;
        const starts = [];
        sensor._requestDepth = () => { starts.push(performance.now()); };
        sensor._queueLatestDepthRequest();
        sensor._queueLatestDepthRequest();
        assert.equal(timerCalls, 1);
        assert.equal(scheduled.delay, 10);

        const fire = scheduled.callback;
        clockMs = 1020;
        scheduled = null;
        fire();
        await nextTask();
        assert.deepEqual(starts, [1020]);
        assert.equal(timerCalls, 1);
    } finally {
        window.setTimeout = originalWindowSetTimeout;
        window.clearTimeout = originalWindowClearTimeout;
        Object.defineProperty(globalThis, 'performance', {
            configurable: true,
            value: realPerformance,
        });
    }
}

// Installation reserves 2 ms inside the 250 ms hard age limit. This closes the
// boundary where validation at 249.x ms could previously become a 251 ms apply
// after the synchronous trajectory callback and accounting.
{
    const realPerformance = globalThis.performance;
    let clockMs = 1000;
    Object.defineProperty(globalThis, 'performance', {
        configurable: true,
        value: { now: () => clockMs },
    });
    try {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(
            {
                canvas: new FakeCanvas('planning-apply-boundary-rgb'),
                complete: true,
                timings_ms: {
                    total: 41,
                    render: 20,
                    scene_render: 18,
                    tile_wait: 2,
                    wait_rerender: 1,
                    face_upload: 3,
                    project: 4,
                    scheduler: 5,
                },
            },
            0,
            { capturedAt: clockMs },
        );
        let callbackCalls = 0;
        let staleMetrics = null;
        globalThis.fetch = async () => ({
            ok: true,
            status: 200,
            headers: { get() { return '256'; } },
            async json() {
                clockMs = 1248;
                return {
                    latency_ms: 40,
                    timings_ms: { da360_ms: 30, yopo_ms: 3 },
                    planning_authorized: true,
                    endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
                    traj_time: 1,
                    depth_mode: 'da360-metric',
                    calibration_id: 'cal-apply-boundary',
                    service_session_id: 'svc-apply-boundary',
                    planning_diagnostics: { ...VALID_PLANNING_DIAGNOSTICS },
                    frame_id: '2', goal_id: 'goal-1', generation: '1',
                };
            },
        });
        sensor.onYopoResult = () => { callbackCalls++; return true; };
        sensor.onPerceptionMetrics = value => { staleMetrics = value; };
        assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
        assert.equal(callbackCalls, 0,
            '248 ms boundary must be rejected before entering the apply callback');
        assert.equal(staleMetrics.dropReason, 'planning-frame-too-old');
        assert.equal(staleMetrics.planningApplyReserveMs, 2);
        assert.equal(staleMetrics.captureMs, 41);
        assert.equal(staleMetrics.renderMs, 20);
        assert.equal(staleMetrics.sceneRenderMs, 18);
        assert.equal(staleMetrics.tileWaitMs, 2);
        assert.equal(staleMetrics.waitRerenderMs, 1);
        assert.equal(staleMetrics.faceUploadMs, 3);
        assert.equal(staleMetrics.projectMs, 4);
        assert.equal(staleMetrics.schedulerYieldMs, 5);
        assert.equal(staleMetrics.jpegMs, 0);
        assert.equal(staleMetrics.fetchStarted, true);
        assert.equal(staleMetrics.fetchStartedAtMs, 1000);
        assert.equal(staleMetrics.responseHeadersAtMs, 1000);
        assert.equal(staleMetrics.payloadParsedAtMs, 1248);
        assert.equal(staleMetrics.responseHeadersMs, 0);
        assert.equal(staleMetrics.httpBodyMs, 248);
        assert.equal(staleMetrics.responseJsonMs, 248);
        assert.equal(staleMetrics.networkMs, 248);
        assert.equal(staleMetrics.responseBytes, 256);
        assert.equal(staleMetrics.gateWaitMs, 248);
        assert.equal(staleMetrics.serverMs, 40);
        assert.equal(staleMetrics.da360Ms, 30);
        assert.equal(staleMetrics.yopoMs, 3);
        assert.deepEqual(staleMetrics.serverTimings, { da360_ms: 30, yopo_ms: 3 });
        assert.equal(staleMetrics.ageBudgetRemainingMs, 2);
        assert.equal(staleMetrics.planningDiagnosticsSchemaVersion, 1);
        assert.equal(staleMetrics.selectedCandidateId, 0);
        assert.deepEqual(
            staleMetrics.planningDiagnostics.selected_endstate_raw,
            VALID_PLANNING_DIAGNOSTICS.selected_endstate_raw,
        );
    } finally {
        Object.defineProperty(globalThis, 'performance', {
            configurable: true,
            value: realPerformance,
        });
    }
}

// Parsed-but-unsupported planner diagnostics fail closed while preserving all
// response-stage evidence needed to distinguish transport, server, and schema
// faults. The unaccepted payload is never forwarded to the controller.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('planning-schema-v2-rgb'));
    let callbackCalls = 0;
    let errorMetrics = null;
    globalThis.fetch = async () => response({
        latency_ms: 40,
        timings_ms: { da360_ms: 29, yopo_ms: 3, serialize_ms: 2 },
        planning_authorized: true,
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-schema-v2',
        service_session_id: 'svc-schema-v2',
        planning_diagnostics: {
            ...VALID_PLANNING_DIAGNOSTICS,
            schema_version: 2,
        },
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => { callbackCalls++; return true; };
    sensor.onPerceptionMetrics = metrics => { errorMetrics = metrics; };

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
    assert.equal(callbackCalls, 0);
    assert.equal(sensor.getDepthState().mode, 'error');
    assert.match(sensor.getDepthState().reason, /unsupported planning_diagnostics schema_version=2/);
    assert.equal(errorMetrics.outcome, 'error');
    assert.equal(errorMetrics.planningDiagnosticsSchemaVersion, 2);
    assert.equal(errorMetrics.planningDiagnostics, null);
    assert.deepEqual(errorMetrics.selectedEndstate, [1, 2, 3, 4, 5, 6, 7, 8, 9]);
    assert.equal(errorMetrics.serverMs, 40);
    assert.equal(errorMetrics.da360Ms, 29);
    assert.equal(errorMetrics.yopoMs, 3);
    assert.deepEqual(
        errorMetrics.serverTimings,
        { da360_ms: 29, yopo_ms: 3, serialize_ms: 2 },
    );
    assert.ok(errorMetrics.responseBytes > 0);
    assert.equal(errorMetrics.fetchStarted, true);
    assert.ok(Number.isFinite(errorMetrics.responseHeadersMs));
    assert.ok(Number.isFinite(errorMetrics.responseJsonMs));
    assert.ok(Number.isFinite(errorMetrics.networkMs));
    assert.ok(Number.isFinite(errorMetrics.gateWaitMs));
}

// Authorized planning diagnostics are a versioned, self-consistent contract.
// Numeric schema aliases, missing/non-finite/out-of-range fields, inconsistent
// lattice mappings, and raw endstates outside the tolerance all fail closed.
{
    const invalidDiagnostics = [
        {
            label: 'missing diagnostics',
            value: null,
            reason: /authorized planning response missing planning/,
        },
        {
            label: 'numeric-string schema',
            value: { ...VALID_PLANNING_DIAGNOSTICS, schema_version: '1' },
            reason: /unsupported planning_diagnostics schema_version=1/,
        },
        {
            label: 'candidate/action mismatch',
            value: {
                ...VALID_PLANNING_DIAGNOSTICS,
                selected_candidate_id: 1,
                selected_action_id: 0,
                selected_lattice_id: 1,
                candidate_count: 2,
            },
            reason: /planning_diagnostics candidate\/action/,
        },
        {
            label: 'lattice mismatch',
            value: {
                ...VALID_PLANNING_DIAGNOSTICS,
                selected_candidate_id: 0,
                selected_action_id: 0,
                selected_lattice_id: 0,
                candidate_count: 2,
            },
            reason: /planning_diagnostics candidate\/action/,
        },
        {
            label: 'raw outside tolerance',
            value: {
                ...VALID_PLANNING_DIAGNOSTICS,
                selected_endstate_raw: [1 + 2e-5, 0, 0, 0, 0, 0, 0, 0, 0],
            },
            reason: /planning_diagnostics selected_endstate_raw is out/,
        },
        {
            label: 'missing selected score',
            value: {
                ...VALID_PLANNING_DIAGNOSTICS,
                selected_score: undefined,
            },
            reason: /planning_diagnostics missing\/non-finite number/,
        },
        {
            label: 'non-finite terminal speed',
            value: {
                ...VALID_PLANNING_DIAGNOSTICS,
                terminal_speed_mps: Number.NaN,
            },
            reason: /planning_diagnostics missing\/non-finite number/,
        },
        {
            label: 'negative terminal acceleration',
            value: {
                ...VALID_PLANNING_DIAGNOSTICS,
                terminal_acceleration_mps2: -0.01,
            },
            reason: /planning_diagnostics negative score\/terminal metric/,
        },
        {
            label: 'zero trajectory time',
            value: {
                ...VALID_PLANNING_DIAGNOSTICS,
                trajectory_time_s: 0,
            },
            reason: /planning_diagnostics time\/scale must be positive/,
        },
        {
            label: 'zero acceleration scale',
            value: {
                ...VALID_PLANNING_DIAGNOSTICS,
                acceleration_scale_mps2: 0,
            },
            reason: /planning_diagnostics time\/scale must be positive/,
        },
    ];
    for (const scenario of invalidDiagnostics) {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(new FakeCanvas(`planning-${scenario.label}-rgb`));
        let callbackCalls = 0;
        globalThis.fetch = async () => response({
            planning_authorized: true,
            endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
            traj_time: 1,
            depth_mode: 'da360-metric',
            calibration_id: `cal-${scenario.label}`,
            service_session_id: `svc-${scenario.label}`,
            planning_diagnostics: scenario.value,
            frame_id: '2', goal_id: 'goal-1', generation: '1',
        });
        sensor.onYopoResult = () => { callbackCalls++; return true; };

        assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false, scenario.label);
        assert.equal(callbackCalls, 0, scenario.label);
        assert.equal(sensor.getDepthState().mode, 'error', scenario.label);
        assert.match(sensor.getDepthState().reason, scenario.reason, scenario.label);
    }

    const accepted = newSensorWithFrame();
    accepted.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    accepted.setYopoGoal({ x: 30, y: 100, z: 0 });
    accepted.primeFromCaptureResult(new FakeCanvas('planning-raw-tolerance-rgb'));
    let acceptedContext = null;
    globalThis.fetch = async () => response({
        planning_authorized: true,
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-raw-tolerance',
        service_session_id: 'svc-raw-tolerance',
        planning_diagnostics: {
            ...VALID_PLANNING_DIAGNOSTICS,
            selected_endstate_raw: [1 + 5e-6, 0, 0, 0, 0, 0, 0, 0, 0],
        },
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    accepted.onYopoResult = (_endstate, _duration, context) => {
        acceptedContext = context;
        return true;
    };
    assert.equal(await accepted._requestDepth(accepted.rgbCanvas), true);
    assert.equal(acceptedContext.planningDiagnosticsSchemaVersion, 1);
    assert.equal(acceptedContext.selectedRawEndstate[0], 1 + 5e-6);
}

// Calibration artifacts all originate from one frozen PerceptionFrame.
// The planning deadline owns JPEG materialization too. Even a non-abort-aware
// toBlob/rgbPromise cannot retain the single-request gate after expiry.
{
    const originalWindowSetTimeout = window.setTimeout;
    const originalWindowClearTimeout = window.clearTimeout;
    let fireDeadline = null;
    window.setTimeout = callback => {
        fireDeadline = callback;
        return 991;
    };
    window.clearTimeout = () => {};
    try {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(new FakeCanvas('deadline-jpeg-rgb'));
        let resolveRgb;
        sensor._rgbFrameContext = Object.freeze({
            ...sensor._rgbFrameContext,
            rgbPromise: new Promise(resolve => { resolveRgb = resolve; }),
        });
        let fetchCalls = 0;
        let staleMetrics = null;
        globalThis.fetch = async () => { fetchCalls++; return response({}); };
        sensor.onPerceptionMetrics = metrics => { staleMetrics = metrics; };

        const pending = sensor._requestDepth(sensor.rgbCanvas);
        assert.equal(sensor._depthGate, true);
        assert.equal(typeof fireDeadline, 'function');
        fireDeadline();
        assert.equal(await pending, false);
        assert.equal(fetchCalls, 0);
        assert.equal(sensor._depthGate, false);
        assert.equal(sensor.depthPending, false);
        assert.equal(sensor._activeDepthRequest, null);
        assert.equal(staleMetrics.dropReason, 'planning-frame-too-old');
        resolveRgb({
            blob: new Blob(['late-rgb'], { type: 'image/jpeg' }),
            width: 384,
            height: 192,
            jpegMs: 999,
        });
        await nextTask();
        assert.equal(sensor._depthGate, false,
            'a late JPEG completion cannot reacquire request ownership');
    } finally {
        window.setTimeout = originalWindowSetTimeout;
        window.clearTimeout = originalWindowClearTimeout;
    }
}

// The same deadline releases the gate even if a fetch mock/backend transport
// ignores AbortSignal and never settles on its own.
{
    const originalWindowSetTimeout = window.setTimeout;
    const originalWindowClearTimeout = window.clearTimeout;
    let fireDeadline = null;
    window.setTimeout = callback => {
        fireDeadline = callback;
        return 992;
    };
    window.clearTimeout = () => {};
    try {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(new FakeCanvas('deadline-fetch-rgb'));
        let fetchCalls = 0;
        let staleMetrics = null;
        globalThis.fetch = () => {
            fetchCalls++;
            return new Promise(() => {});
        };
        sensor.onPerceptionMetrics = metrics => { staleMetrics = metrics; };

        const pending = sensor._requestDepth(sensor.rgbCanvas);
        await nextTask();
        assert.equal(fetchCalls, 1);
        assert.equal(sensor._depthGate, true);
        fireDeadline();
        assert.equal(await pending, false);
        assert.equal(sensor._depthGate, false);
        assert.equal(sensor._activeDepthRequest, null);
        assert.equal(staleMetrics.dropReason, 'planning-frame-too-old');
    } finally {
        window.setTimeout = originalWindowSetTimeout;
        window.clearTimeout = originalWindowClearTimeout;
    }
}

// A deadline-aware consumer can defer its final mutation through commitIfFresh.
// If time crosses the hard 250 ms age immediately before that mutation, the
// callback is never executed and the response is reported stale.
{
    const realPerformance = globalThis.performance;
    let clockMs = 1000;
    Object.defineProperty(globalThis, 'performance', {
        configurable: true,
        value: { now: () => clockMs },
    });
    try {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(
            new FakeCanvas('planning-final-commit-deadline-rgb'),
            0,
            { capturedAt: clockMs },
        );
        let finalMutations = 0;
        let controlMetrics = null;
        globalThis.fetch = async () => ({
            ok: true,
            status: 200,
            headers: { get() { return '256'; } },
            async json() {
                clockMs = 1247;
                return {
                    planning_authorized: true,
                    endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
                    traj_time: 1,
                    depth_mode: 'da360-metric',
                    calibration_id: 'cal-final-deadline',
                    service_session_id: 'svc-final-deadline',
                    planning_diagnostics: { ...VALID_PLANNING_DIAGNOSTICS },
                    frame_id: '2', goal_id: 'goal-1', generation: '1',
                };
            },
        });
        sensor.onYopoResult = (_endstate, _duration, context) => {
            assert.equal(context.applyDeadlineWallTimeMs, 1250);
            assert.equal(typeof context.commitIfFresh, 'function');
            clockMs = 1250;
            return context.commitIfFresh(() => {
                finalMutations++;
                return true;
            });
        };
        sensor.onPerceptionMetrics = metrics => {
            if (metrics.mode === 'planning') controlMetrics = metrics;
        };

        assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
        assert.equal(finalMutations, 0,
            'the final mutation must not run at or beyond the hard age deadline');
        assert.equal(controlMetrics.outcome, 'stale');
        assert.equal(controlMetrics.trajectoryApplied, false);
        assert.equal(controlMetrics.dropReason, 'trajectory-apply-deadline-exceeded');
        assert.equal(controlMetrics.applyDeadlineExceeded, true);
        assert.equal(sensor._depthGate, false);
    } finally {
        Object.defineProperty(globalThis, 'performance', {
            configurable: true,
            value: realPerformance,
        });
    }
}

// If a synchronous trajectory install itself crosses the hard boundary, the
// cooperative transaction rolls back that exact request before returning a
// stale result. Production uses a compare-and-clear rollback so N cannot erase
// a newer N+1 command.
{
    const realPerformance = globalThis.performance;
    let clockMs = 1000;
    Object.defineProperty(globalThis, 'performance', {
        configurable: true,
        value: { now: () => clockMs },
    });
    try {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(
            new FakeCanvas('planning-cross-deadline-rollback-rgb'),
            0,
            { capturedAt: clockMs },
        );
        let installed = 0;
        let rollbackCalls = 0;
        let controlMetrics = null;
        globalThis.fetch = async () => ({
            ok: true,
            status: 200,
            headers: { get() { return '256'; } },
            async json() {
                clockMs = 1247;
                return {
                    planning_authorized: true,
                    endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
                    traj_time: 1,
                    depth_mode: 'da360-metric',
                    calibration_id: 'cal-cross-deadline',
                    service_session_id: 'svc-cross-deadline',
                    planning_diagnostics: { ...VALID_PLANNING_DIAGNOSTICS },
                    frame_id: '2', goal_id: 'goal-1', generation: '1',
                };
            },
        });
        sensor.onYopoResult = (_endstate, _duration, context) => context.commitIfFresh(
            () => {
                installed++;
                clockMs = 1250;
                return true;
            },
            () => {
                rollbackCalls++;
                installed--;
                return true;
            },
        );
        sensor.onPerceptionMetrics = metrics => {
            if (metrics.mode === 'planning') controlMetrics = metrics;
        };

        assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
        assert.equal(installed, 0);
        assert.equal(rollbackCalls, 1);
        assert.equal(controlMetrics.outcome, 'stale');
        assert.equal(controlMetrics.dropReason, 'trajectory-apply-deadline-exceeded');
        assert.equal(controlMetrics.applyDeadlineExceeded, true);
    } finally {
        Object.defineProperty(globalThis, 'performance', {
            configurable: true,
            value: realPerformance,
        });
    }
}

// The inner main.js transaction may finish just inside the deadline while the
// surrounding observer transaction crosses it on the very next clock read.
// Its registered compare-and-clear rollback must remain armed until the outer
// post-check commits the whole nested transaction.
{
    const realPerformance = globalThis.performance;
    let clockMs = 1000;
    let advanceAfterInnerPost = false;
    Object.defineProperty(globalThis, 'performance', {
        configurable: true,
        value: {
            now: () => {
                const sampled = clockMs;
                if (advanceAfterInnerPost && clockMs === 1249) {
                    clockMs = 1250;
                    advanceAfterInnerPost = false;
                }
                return sampled;
            },
        },
    });
    try {
        const sensor = newSensorWithFrame();
        sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
        sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
        sensor.primeFromCaptureResult(
            new FakeCanvas('planning-outer-post-deadline-rgb'),
            0,
            { capturedAt: clockMs },
        );
        let installed = 0;
        let rollbackCalls = 0;
        let controlMetrics = null;
        globalThis.fetch = async () => ({
            ok: true,
            status: 200,
            headers: { get() { return '256'; } },
            async json() {
                clockMs = 1247;
                return {
                    planning_authorized: true,
                    endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
                    traj_time: 1,
                    depth_mode: 'da360-metric',
                    calibration_id: 'cal-outer-post-deadline',
                    service_session_id: 'svc-outer-post-deadline',
                    planning_diagnostics: { ...VALID_PLANNING_DIAGNOSTICS },
                    frame_id: '2', goal_id: 'goal-1', generation: '1',
                };
            },
        });
        sensor.onYopoResult = (_endstate, _duration, context) => context.commitIfFresh(
            () => {
                installed++;
                clockMs = 1249;
                advanceAfterInnerPost = true;
                return true;
            },
            () => {
                rollbackCalls++;
                installed--;
                return true;
            },
        );
        sensor.onPerceptionMetrics = metrics => {
            if (metrics.mode === 'planning') controlMetrics = metrics;
        };

        assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
        assert.equal(installed, 0,
            'outer post-deadline accounting must not leave the inner trajectory active');
        assert.equal(rollbackCalls, 1);
        assert.equal(controlMetrics.outcome, 'stale');
        assert.equal(controlMetrics.dropReason, 'trajectory-apply-deadline-exceeded');
        assert.equal(controlMetrics.applyDeadlineExceeded, true);
    } finally {
        Object.defineProperty(globalThis, 'performance', {
            configurable: true,
            value: realPerformance,
        });
    }
}

// Optional logging observers are fault-isolated from the control result. A
// logger exception cannot turn an installed trajectory into error/stale state.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('observer-isolation-rgb'));
    globalThis.fetch = async () => response({
        latency_ms: 30,
        planning_authorized: true,
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-observer-isolation',
        service_session_id: 'svc-observer-isolation',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => true;
    sensor.onYopoLatency = () => { throw new Error('latency logger failed'); };
    sensor.onPerceptionMetrics = () => { throw new Error('metrics logger failed'); };

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);
    assert.equal(sensor.getDepthState().outcome, 'applied');
    assert.match(sensor.getDepthState().reason, /trajectory-ready/);
    assert.equal(sensor._depthGate, false);
}

// Calibration artifacts all originate from one frozen PerceptionFrame.
{
    const sensor = newSensorWithFrame();
    await assert.rejects(
        sensor.captureCalibrationSample({}, { locationId: 'street-a' }),
        /requires the "calibration" panorama capture profile/,
    );
    sensor.setCaptureProfile('calibration');
    await assert.rejects(
        sensor.captureCalibrationSample({}, { locationId: 'street-a' }),
        /wait for a complete calibration-profile panorama frame/,
        'switching profile must not relabel the last flight frame',
    );
    primeCalibrationFrame(sensor, 'calibration-capture-001');
    const materialized = await sensor._materializePerceptionFrame(sensor._rgbFrameContext);
    sensor._perceptionFrame = materialized.frame;
    let sampledTransform = null;
    const world = {
        sampleMetricDepthAnchors(transform, options) {
            sampledTransform = transform;
            return {
                anchors: [{ u: 10, v: 10, distance: 5 }],
                failures: [],
                metadata: {
                    schemaVersion: 1,
                    identity: {
                        sessionId: options.sessionId,
                        locationId: options.locationId,
                        captureId: options.captureId,
                        frameId: options.frameId,
                    },
                    image: { width: options.imageWidth, height: options.imageHeight },
                    validAnchors: 1,
                    failureCount: 0,
                },
            };
        },
    };
    const rawBytes = new Uint8Array([1, 2, 3, 4]);
    let requestedUrl = null;
    globalThis.fetch = async (url, options) => {
        requestedUrl = new URL(url);
        assert.equal(options.headers['X-Frame-ID'], String(materialized.frame.frameId));
        assert.deepEqual(
            JSON.parse(options.headers['X-Projection-Config']),
            materialized.frame.projectionConfig,
            'raw request carries the frozen projection contract',
        );
        assert.equal(options.body, materialized.frame.rgb, 'raw request reuses the frozen frame JPEG');
        return {
            ok: true,
            status: 200,
            headers: { get(name) {
                return ({
                    'X-Frame-ID': String(materialized.frame.frameId),
                    'X-Session-ID': sensor._calibrationSessionId,
                    'X-Capture-ID': 'capture-001',
                    'X-Location-ID': 'street-a',
                    'X-DA360-Model': 'large',
                    'X-DA360-Width': '476',
                    'X-DA360-Height': '238',
                    'X-DA360-Latency-Ms': '30',
                })[name] || null;
            } },
            async arrayBuffer() { return rawBytes.buffer; },
        };
    };
    const artifacts = await sensor.captureCalibrationSample(world, {
        locationId: 'street-a',
        captureId: 'capture-001',
        download: false,
    });
    assert.equal(sampledTransform, materialized.frame.transform, 'anchors use the exact frozen panorama transform');
    assert.equal(artifacts.manifest.schemaVersion, 2);
    assert.equal(artifacts.frame.captureProfile, 'calibration');
    assert.equal(artifacts.manifest.captureProfile, 'calibration');
    assert.equal(artifacts.manifest.frameId, String(materialized.frame.frameId));
    assert.equal(artifacts.anchors.metadata.identity.locationId, 'street-a');
    assert.equal(artifacts.anchors.metadata.identity.sessionId, artifacts.manifest.sessionId);
    assert.equal(artifacts.anchors.metadata.identity.captureId, artifacts.manifest.captureId);
    assert.equal(requestedUrl.searchParams.get('frame_id'), String(materialized.frame.frameId));
    assert.equal(requestedUrl.searchParams.get('session_id'), artifacts.manifest.sessionId);
    assert.equal(requestedUrl.searchParams.get('capture_id'), 'capture-001');
    assert.equal(requestedUrl.searchParams.get('location_id'), 'street-a');
    assert.equal(artifacts.manifest.files.raw.name, 'capture-001-raw.npz');
    assert.equal(artifacts.manifest.files.rgb.name, 'capture-001-rgb.jpg');
    assert.equal(artifacts.manifest.files.anchors.name, 'capture-001-anchors.json');
    assert.equal(artifacts.manifest.files.raw.bytes, artifacts.files['capture-001-raw.npz'].size);
    assert.equal(artifacts.manifest.files.rgb.bytes, artifacts.files['capture-001-rgb.jpg'].size);
    assert.equal(artifacts.manifest.files.anchors.bytes, artifacts.files['capture-001-anchors.json'].size);
    assert.ok(!('rawSha256' in artifacts.manifest));
    assert.equal(artifacts.manifest.rgbWidth, materialized.frame.projectionConfig.rgbWidth);
    assert.equal(artifacts.manifest.projectionConfig.jpegQuality, 0.74);
    assert.equal(artifacts.manifest.projectionConfig.uploadScale, 1);
    assert.ok(artifacts.files['capture-001-raw.npz']);
    assert.ok(artifacts.files['capture-001-rgb.jpg']);
    await assert.rejects(
        sensor.captureCalibrationSample(world, {
            locationId: 'street-a', captureId: 'capture-duplicate', download: false,
        }),
        /already exported/,
        'one frozen perception frame may only be consumed once',
    );
}

// Concurrent exporters are serialized; a pending request cannot consume the
// same frame under a second capture identity.
{
    const sensor = newSensorWithFrame();
    sensor.setCaptureProfile('calibration');
    primeCalibrationFrame(sensor, 'calibration-lock');
    const materialized = await sensor._materializePerceptionFrame(sensor._rgbFrameContext);
    sensor._perceptionFrame = materialized.frame;
    const world = {
        sampleMetricDepthAnchors(_transform, options) {
            return { anchors: [], failures: [], metadata: { ...options } };
        },
    };
    let resolveFetch;
    globalThis.fetch = () => new Promise(resolve => { resolveFetch = resolve; });
    const first = sensor.captureCalibrationSample(world, {
        locationId: 'street-lock', captureId: 'capture-lock-1', download: false,
    });
    await assert.rejects(
        sensor.captureCalibrationSample(world, {
            locationId: 'street-lock', captureId: 'capture-lock-2', download: false,
        }),
        /already in progress/,
    );
    sensor.primeFromCaptureResult(new FakeCanvas('new-rgb-during-export'));
    const newer = await sensor._materializePerceptionFrame(sensor._rgbFrameContext);
    sensor._perceptionFrame = newer.frame;
    resolveFetch({
        ok: true,
        status: 200,
        headers: { get(name) {
            return ({
                'X-Frame-ID': String(materialized.frame.frameId),
                'X-Session-ID': sensor._calibrationSessionId,
                'X-Capture-ID': 'capture-lock-1',
                'X-Location-ID': 'street-lock',
            })[name] || null;
        } },
        async arrayBuffer() { return new Uint8Array([9, 8, 7]).buffer; },
    });
    const artifacts = await first;
    assert.equal(artifacts.frame, materialized.frame,
        'a newer UI frame does not invalidate the already frozen export bundle');
    assert.notEqual(artifacts.frame, sensor._perceptionFrame);
}

// Reset/mode teardown cancels the raw request without releasing the shared
// capture-viewer lock until the exporter has actually unwound.
{
    const sensor = newSensorWithFrame();
    sensor.setCaptureProfile('calibration');
    primeCalibrationFrame(sensor, 'calibration-reset');
    const materialized = await sensor._materializePerceptionFrame(sensor._rgbFrameContext);
    sensor._perceptionFrame = materialized.frame;
    const world = {
        sampleMetricDepthAnchors(_transform, options) {
            return { anchors: [], failures: [], metadata: { ...options } };
        },
    };
    let requestSignal = null;
    let markFetchStarted;
    const fetchStarted = new Promise(resolve => { markFetchStarted = resolve; });
    globalThis.fetch = async (_url, options) => {
        requestSignal = options.signal;
        markFetchStarted();
        return new Promise((_resolve, reject) => {
            options.signal.addEventListener('abort', () => {
                const error = new Error(String(options.signal.reason || 'aborted'));
                error.name = 'AbortError';
                reject(error);
            }, { once: true });
        });
    };

    const pending = sensor.captureCalibrationSample(world, {
        locationId: 'street-reset', captureId: 'capture-reset', download: false,
    });
    await fetchStarted;
    sensor.reset();
    assert.equal(requestSignal.aborted, true, 'sensor reset aborts the raw calibration request');
    assert.equal(sensor._calibrationCapturePending, true,
        'reset does not release the viewer lock before the exporter settles');
    await assert.rejects(pending, /sensor-reset/);
    assert.equal(sensor._calibrationCapturePending, false);
    assert.equal(sensor._consumedCalibrationFrames.size, 0,
        'an aborted exporter never consumes the reset session frame');
}

// Leaving calibration invalidates and aborts an in-flight bundle. This also
// covers setYopoGoal(), which must force the flight profile before planning.
for (const transition of ['profile-switch', 'goal-set']) {
    const sensor = newSensorWithFrame();
    sensor.setCaptureProfile('calibration');
    primeCalibrationFrame(sensor, `calibration-${transition}`);
    const world = {
        sampleMetricDepthAnchors(_transform, options) {
            return { anchors: [], failures: [], metadata: { ...options } };
        },
    };
    let requestSignal = null;
    let markFetchStarted;
    const fetchStarted = new Promise(resolve => { markFetchStarted = resolve; });
    globalThis.fetch = async (_url, options) => {
        requestSignal = options.signal;
        markFetchStarted();
        return new Promise((_resolve, reject) => {
            options.signal.addEventListener('abort', () => {
                const error = new Error(String(options.signal.reason || 'aborted'));
                error.name = 'AbortError';
                reject(error);
            }, { once: true });
        });
    };

    const generationBefore = sensor._calibrationGeneration;
    const pending = sensor.captureCalibrationSample(world, {
        locationId: `street-${transition}`,
        captureId: `capture-${transition}`,
        download: false,
    });
    await fetchStarted;
    const expectedReason = transition === 'profile-switch'
        ? 'test-profile-switch:calibration->flight'
        : 'yopo-goal:calibration->flight';
    if (transition === 'profile-switch') {
        sensor.setCaptureProfile('flight', 'test-profile-switch');
    } else {
        assert.ok(sensor.setYopoGoal({ x: 10, y: 20, z: 30 }));
    }
    assert.equal(sensor.getCaptureProfile(), 'flight');
    assert.equal(sensor._calibrationGeneration, generationBefore + 1);
    assert.equal(requestSignal.aborted, true);
    assert.equal(requestSignal.reason, expectedReason);
    await assert.rejects(pending, new RegExp(expectedReason.replace(/[-/>]/g, '\\$&')));
    assert.equal(sensor._calibrationCapturePending, false);
    assert.equal(sensor._consumedCalibrationFrames.size, 0);
}

// Missing frame echo is a hard error: accepting it would allow a cached or
// reordered raw result into a capture bundle.
{
    const sensor = newSensorWithFrame();
    sensor.setCaptureProfile('calibration');
    primeCalibrationFrame(sensor, 'calibration-echo');
    const materialized = await sensor._materializePerceptionFrame(sensor._rgbFrameContext);
    sensor._perceptionFrame = materialized.frame;
    const world = {
        sampleMetricDepthAnchors(_transform, options) {
            return { anchors: [], failures: [], metadata: { ...options } };
        },
    };
    globalThis.fetch = async () => ({
        ok: true,
        status: 200,
        headers: { get() { return null; } },
        async arrayBuffer() { return new Uint8Array([1]).buffer; },
    });
    await assert.rejects(
        sensor.captureCalibrationSample(world, {
            locationId: 'street-echo', captureId: 'capture-echo', download: false,
        }),
        /raw frame mismatch: missing/,
    );
}

// Calibration waits for the current six-face capture, then freezes the newly
// completed frame while preventing the next automatic capture from starting.
{
    const sensor = newSensorWithFrame();
    sensor.setCaptureProfile('calibration');
    primeCalibrationFrame(sensor, 'calibration-before-idle');
    const oldFrame = await sensor._materializePerceptionFrame(sensor._rgbFrameContext);
    sensor._perceptionFrame = oldFrame.frame;
    let resolveCaptureIdle;
    sensor.capturing = true;
    sensor._captureIdlePromise = new Promise(resolve => { resolveCaptureIdle = resolve; });
    let sampledFrameId = null;
    const world = {
        sampleMetricDepthAnchors(_transform, options) {
            sampledFrameId = options.frameId;
            return { anchors: [], failures: [], metadata: { ...options } };
        },
    };
    globalThis.fetch = async (_url, request) => ({
        ok: true,
        status: 200,
        headers: { get(name) {
            return ({
                'X-Frame-ID': request.headers['X-Frame-ID'],
                'X-Session-ID': sensor._calibrationSessionId,
                'X-Capture-ID': 'capture-after-idle',
                'X-Location-ID': 'street-idle',
            })[name] || null;
        } },
        async arrayBuffer() { return new Uint8Array([4, 3, 2, 1]).buffer; },
    });

    const pending = sensor.captureCalibrationSample(world, {
        locationId: 'street-idle', captureId: 'capture-after-idle', download: false,
    });
    await Promise.resolve();
    assert.equal(sensor._calibrationCapturePending, true);
    assert.equal(sampledFrameId, null, 'anchors must not sample a viewer being captured');

    sensor.primeFromCaptureResult(new FakeCanvas('new-complete-rgb'), 1, {
        capturedAt: performance.now(),
    });
    const expectedFrameId = String(sensor._rgbFrameContext.frameId);
    sensor.capturing = false;
    sensor._captureIdlePromise = null;
    resolveCaptureIdle();

    const artifacts = await pending;
    assert.equal(String(artifacts.frame.frameId), expectedFrameId);
    assert.equal(sampledFrameId, expectedFrameId);
    assert.equal(sensor._calibrationCapturePending, false);
}

// Tile readiness is diagnostic in the current experimental flight workflow:
// a complete but partial RGB canvas remains visible and may be sent to YOPO.
// The exact frame readiness must survive through callback and metrics so the
// operator can decide whether the resulting trajectory is trustworthy.
{
    const sensor = new PanoramaSensor();
    sensor.setYopoPose({
        actualState: {
            position: { x: 1, y: 2, z: 3 },
            velocity: { x: 0, y: 0, z: 0 },
        },
        referenceState: {
            position: { x: 1, y: 2, z: 3 },
            velocity: { x: 0, y: 0, z: 0 },
            acceleration: { x: 0, y: 0, z: 0 },
        },
        yaw: 0,
    }, 0);
    sensor.setYopoGoal({ x: 10, y: 2, z: 3 });
    const faceTileReadiness = [
        { face: 'front', readyWhenCopied: true },
        { face: 'right', readyWhenCopied: true },
        { face: 'back', readyWhenCopied: false },
        { face: 'left', readyWhenCopied: false },
        { face: 'up', readyWhenCopied: false },
        { face: 'down', readyWhenCopied: false },
    ];
    assert.equal(sensor.primeFromCaptureResult({
        canvas: new FakeCanvas('partial-planning-rgb'),
        complete: true,
        // Legacy producers used `ready` for canvas completion. The explicit
        // all-face aggregate must take precedence when both are present.
        ready: true,
        allFacesTileReady: false,
        readyFaces: 2,
        faces: 6,
        faceTileReadiness,
        readinessReason: 'tiles-partial',
        tileError: false,
    }, 5, {
        transform: {
            position: { x: 1, y: 2, z: 3 },
            rotation: { x: 0, y: 0, z: 0 },
        },
    }), true, 'partial RGB canvas should still be committed to the preview');

    const state = sensor.getDepthState();
    assert.equal(state.rgbFrameComplete, true);
    assert.equal(state.rgbTilesReady, false);
    assert.equal(state.rgbReadyFaces, 2);
    assert.equal(state.rgbTotalFaces, 6);
    assert.equal(state.faceTileReadiness.length, 6);

    let planningFetches = 0;
    let callbackContext = null;
    const metrics = [];
    globalThis.createImageBitmap = async () => fakeBitmap('partial-planning-depth');
    globalThis.fetch = async url => {
        planningFetches++;
        assert.match(String(url), /\/yopo\/plan_full\?/);
        return response({
            frame_id: String(sensor._rgbFrameContext.frameId),
            goal_id: sensor._goalId,
            generation: String(sensor._yopoGeneration),
            depth_image: DEPTH_JPEG,
            depth_mode: 'da360-metric',
            calibration_id: 'partial-rgb-test',
            calibration_accuracy_accepted: false,
            service_session_id: 'partial-rgb-service',
            planning_authorized: true,
            planning_reason: 'experimental-unaccepted-da360-metric',
            endstate: [1, 0, 0, 0, 0, 0, 0, 0, 0],
            traj_time: 1,
            latency_ms: 3,
            timings_ms: { da360_ms: 1, yopo_ms: 1 },
        });
    };
    sensor.onYopoResult = (_endstate, _trajTime, context) => {
        callbackContext = context;
        return true;
    };
    sensor.onPerceptionMetrics = value => metrics.push(value);
    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);
    assert.equal(planningFetches, 1, 'partial tile readiness must not silently block YOPO');
    assert.equal(callbackContext.rgbTilesReady, false);
    assert.equal(callbackContext.rgbReadyFaces, 2);
    const planningMetric = metrics.find(value => value.mode === 'planning');
    assert.equal(planningMetric.rgbFrameId, sensor._rgbFrameContext.frameId);
    assert.equal(planningMetric.rgbTilesReady, false);
    assert.equal(planningMetric.rgbReadyFaces, 2);
    assert.equal(planningMetric.rgbReadinessReason, 'tiles-partial');
}

// HTML contract: one visible canvas, no hidden image, and 0.6 m collision defaults.
{
    const html = await readFile(new URL('../index.html', import.meta.url), 'utf8');
    assert.doesNotMatch(html, /id="panorama-depth-image"/);
    assert.match(html, /id="panorama-depth-canvas"/);
    assert.match(html, /id="phys-collision-radius"[^>]*value="0\.6"/);
    assert.match(html, /id="phys-collision-radius-num"[^>]*value="0\.6"/);
    const mainModuleKeys = Array.from(
        html.matchAll(/src\/main\.js\?v=([A-Za-z0-9._-]+)/g),
        match => match[1],
    );
    assert.equal(mainModuleKeys.length, 2,
        'normal and slow-load fallback both import the main module');
    assert.equal(new Set(mainModuleKeys).size, 1,
        'normal and slow-load fallback imports share one ES module cache key');
}

console.log('Panorama depth state tests: all passed');
