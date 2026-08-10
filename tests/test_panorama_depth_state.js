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
    location: { search: '?da360UploadScale=1', hostname: '127.0.0.1' },
    setTimeout: globalThis.setTimeout.bind(globalThis),
    clearTimeout: globalThis.clearTimeout.bind(globalThis),
};

const { PanoramaSensor, decodeDepthImageSource } = await import('../src/panorama-sensor.js?depth-state-test');
const DEPTH_JPEG = 'data:image/jpeg;base64,AA==';

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
    assert.equal(sensor.primeFromCaptureResult(rgbFrame), true);
    return sensor;
}

async function nextTask() {
    await new Promise(resolve => setTimeout(resolve, 0));
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
    globalThis.fetch = async () => response({ depth_image: DEPTH_JPEG, latency_ms: 12 });

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
                planning_reason: 'validated-da360-metric',
                endstate: [1, 2, 3, 4, 5, 6, 7, 8, 9],
                traj_time: 1.125,
                depth_mode: 'da360-relative',
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
    sensor.onYopoResult = (_endstate, _trajTime, context) => { callbackContext = context; };
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

    sensor.resetYopoGoal('cancelled');
    assert.equal(sensor.getDepthState().mode, 'preview');
    assert.equal(sensor.hasDepth, true);
    assert.equal(sensor.depthCanvas.context.drawCalls.length, retainedDrawCount + 1,
        'cancel must retain the last depth pixels');
}

// Relative depth may update the canvas, but it is preview-only and must never
// be reported or applied as a successful planning frame.
{
    const sensor = newSensorWithFrame();
    sensor.setYopoPose({ x: 0, y: 100, z: 0, vx: 0, vy: 0, vz: 0 }, 0);
    sensor.setYopoGoal({ x: 30, y: 100, z: 0 });
    sensor.primeFromCaptureResult(new FakeCanvas('relative-planning-rgb'));
    let yopoCallbacks = 0;
    let metrics = null;
    sensor.onYopoResult = () => { yopoCallbacks++; };
    sensor.onPerceptionMetrics = value => { metrics = value; };
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
        'blocked planning still commits the useful depth preview');
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

// A response for an older RGB frame is stale even within the same generation.
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

    assert.equal(await oldRequest, false);
    assert.equal(sensor.hasDepth, false, 'old frame response must not mark depth ready');
    assert.equal(sensor.depthCanvas.context.drawCalls.length, 0, 'old frame response must not touch canvas');
    assert.ok(outcomes.some(item => item.outcome === 'stale' && item.dropReason === 'response-after-session-change'),
        'old frame is recorded with an explicit stale drop reason');
}

// Calibration artifacts all originate from one frozen PerceptionFrame.
{
    const sensor = newSensorWithFrame();
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

// Missing frame echo is a hard error: accepting it would allow a cached or
// reordered raw result into a capture bundle.
{
    const sensor = newSensorWithFrame();
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

// HTML contract: one visible canvas, no hidden image, and 0.6 m collision defaults.
{
    const html = await readFile(new URL('../index.html', import.meta.url), 'utf8');
    assert.doesNotMatch(html, /id="panorama-depth-image"/);
    assert.match(html, /id="panorama-depth-canvas"/);
    assert.match(html, /id="phys-collision-radius"[^>]*value="0\.6"/);
    assert.match(html, /id="phys-collision-radius-num"[^>]*value="0\.6"/);
    assert.equal(
        html.match(/src\/main\.js\?v=20260807-planfull/g)?.length,
        2,
        'normal and slow-load fallback imports share one ES module cache key',
    );
}

console.log('Panorama depth state tests: all passed');
