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

function fakeBitmap(label = 'depth') {
    return { width: 8, height: 4, label, closed: false, close() { this.closed = true; } };
}

function response(payload) {
    return { ok: true, status: 200, async json() { return payload; } };
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
                service_fingerprint: 'service-v1',
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
    });
    const goalId = sensor.setYopoGoal({ x: 10, y: 20, z: 30 });
    assert.equal(sensor.getDepthState().mode, 'planning');
    assert.equal(sensor._shouldRequestDepth(), false, 'old RGB frame must not be reused for a new goal');

    sensor.primeFromCaptureResult(new FakeCanvas('next-rgb'));
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
    assert.match(calls.at(-1), /\/yopo\/plan_full\?/);
    assert.match(calls.at(-1), new RegExp(`goal_id=${goalId}`));
    const planningUrl = new URL(calls.at(-1));
    assert.equal(planningUrl.searchParams.get('px'), '1', 'trajectory endpoint origin uses frame actual position');
    assert.equal(planningUrl.searchParams.get('rpx'), '11', 'YOPO goal delta origin uses frame reference position');
    assert.equal(planningUrl.searchParams.get('vx'), '4', 'YOPO observation keeps frame actual velocity');
    assert.equal(planningUrl.searchParams.get('ax'), '0.1', 'YOPO observation uses frame reference acceleration');
    const runtimeProjection = JSON.parse(lastRequestOptions.headers['X-Projection-Config']);
    assert.equal(runtimeProjection.verticalFovDeg, 180,
        'planning request carries the frozen frame projection fingerprint');
    assert.equal(runtimeProjection.jpegQuality, 0.74,
        'planning request carries the actual JPEG encoder quality');
    assert.equal(callbackContext.goalId, goalId);
    assert.equal(callbackContext.mode, 'planning');
    assert.equal(callbackContext.serviceFingerprint, 'service-v1');
    assert.equal(callbackContext.calibrationAccuracyAccepted, false);
    assert.equal(planningMetrics.trajectoryApplied, true);
    assert.ok(Number.isFinite(planningMetrics.trajectoryAppliedAtMs));
    assert.equal(planningMetrics.depthMode, 'da360-metric');
    assert.equal(planningMetrics.calibrationId, 'calibration-v1');
    assert.equal(planningMetrics.calibrationAccuracyAccepted, false);
    assert.equal(planningMetrics.serviceFingerprint, 'service-v1');

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
        service_fingerprint: 'svc-order',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => { callbackCalled = true; return true; };
    sensor.onPerceptionMetrics = value => { metrics.push(value); };
    const pending = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();
    assert.equal(callbackCalled, true, 'trajectory apply must not wait for preview decode');
    const appliedMetrics = metrics.find(value => value.mode === 'planning');
    assert.equal(appliedMetrics.trajectoryApplied, true,
        'trajectory evidence is emitted synchronously with the install');
    assert.equal(appliedMetrics.depthPreviewCommitted, null,
        'optional preview outcome is not part of the trajectory acknowledgement');
    resolveDecode(fakeBitmap('planning-order-depth'));
    assert.equal(await pending, true);
    assert.equal(appliedMetrics.trajectoryApplied, true);
    assert.ok(appliedMetrics.captureToApplyMs <= appliedMetrics.frameAgeMs);
    assert.equal(metrics.filter(value => value.mode === 'planning').length, 1);
    assert.equal(metrics.filter(value => value.mode === 'depth-preview').length, 1);

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
    assert.equal(rejectedMetrics.outcome, 'rejected');
    assert.equal(rejectedMetrics.trajectoryApplied, false);
    assert.equal(rejectedMetrics.dropReason, 'trajectory-apply-rejected');
    assert.match(rejected.depthStatusEl.textContent, /rejected/);
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
        service_fingerprint: 'svc-stale-preview',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => true;
    sensor.onPerceptionMetrics = value => { metrics.push(value); };
    const drawCount = sensor.depthCanvas.context.drawCalls.length;
    const pending = sensor._requestDepth(sensor.rgbCanvas);
    await nextTask();

    sensor.setYopoGoal({ x: 60, y: 100, z: 0 });
    const newerState = sensor.getDepthState();
    resolveDecode(fakeBitmap('stale-depth-preview'));
    assert.equal(await pending, true);
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
                service_fingerprint: 'svc-rejected-pending',
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

        const controlBeforeDecode = metrics.filter(value => value.mode === 'planning');
        assert.equal(controlBeforeDecode.length, 1, `${scenario.label} has one immediate control event`);
        assert.equal(controlBeforeDecode[0].outcome, scenario.label);
        assert.equal(controlBeforeDecode[0].trajectoryApplied, false);

        sensor.setYopoGoal({ x: 60, y: 100, z: 0 });
        resolveDecode(fakeBitmap(`${scenario.label}-stale-preview`));
        assert.equal(await pending, true);
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
        service_fingerprint: 'svc-corrupt-preview',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => true;
    sensor.onPerceptionMetrics = value => { metrics.push(value); };
    const drawCount = sensor.depthCanvas.context.drawCalls.length;

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), true);
    const control = metrics.filter(value => value.mode === 'planning');
    const display = metrics.filter(value => value.mode === 'depth-preview');
    assert.equal(control.length, 1);
    assert.equal(control[0].outcome, 'applied');
    assert.equal(control[0].trajectoryApplied, true);
    assert.equal(display.length, 1);
    assert.equal(display[0].outcome, 'error');
    assert.match(display[0].depthPreviewError, /corrupt planning preview/);
    assert.equal(sensor.depthCanvas.context.drawCalls.length, drawCount);
    assert.match(sensor.depthStatusEl.title, /depth-preview-error/);
}

// A backend authorization bit is insufficient without the full provenance
// contract required by the trusted closed-loop evaluator.
{
    for (const [label, override, expected] of [
        ['relative-mode', { depth_mode: 'da360-relative' }, /untrusted depth mode/],
        ['missing-calibration', { calibration_id: null }, /missing calibration_id/],
        ['missing-service', { service_fingerprint: null }, /missing service_fing/],
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
            service_fingerprint: 'svc-trusted',
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
        { capturedAt: performance.now() - 1000 },
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
        'even an old blocked planning response still commits its useful depth preview');
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
    assert.equal(sensor.getDepthState().outcome, 'stale');
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
        service_fingerprint: 'svc-before-failure',
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
        service_fingerprint: 'svc-pipeline',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    }));

    assert.equal(await oldRequest, true);
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
    globalThis.createImageBitmap = async () => fakeBitmap('expired-preview');
    globalThis.fetch = async () => response({
        depth_image: DEPTH_JPEG,
        latency_ms: 35,
        planning_authorized: true,
        planning_reason: 'validated-da360-metric',
        endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
        traj_time: 1,
        depth_mode: 'da360-metric',
        calibration_id: 'cal-expired',
        service_fingerprint: 'svc-expired',
        frame_id: '2', goal_id: 'goal-1', generation: '1',
    });
    sensor.onYopoResult = () => { callbackCalls++; return true; };
    sensor.onPerceptionMetrics = value => { metrics = value; };

    assert.equal(await sensor._requestDepth(sensor.rgbCanvas), false);
    assert.equal(callbackCalls, 0);
    assert.equal(metrics.outcome, 'stale');
    assert.equal(metrics.dropReason, 'planning-frame-too-old');
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
            'raw request carries the frozen projection fingerprint',
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
    assert.equal(artifacts.manifest.files.raw.sha256, artifacts.manifest.rawSha256);
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
            service_fingerprint: 'partial-rgb-service',
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
