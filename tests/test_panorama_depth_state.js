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
    globalThis.createImageBitmap = async () => fakeBitmap();
    globalThis.fetch = async (url) => {
        calls.push(String(url));
        if (String(url).includes('/yopo/plan_full')) {
            return response({
                depth_image: DEPTH_JPEG,
                latency_ms: 18,
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
    assert.equal(callbackContext.goalId, goalId);
    assert.equal(callbackContext.mode, 'planning');

    sensor.resetYopoGoal('cancelled');
    assert.equal(sensor.getDepthState().mode, 'preview');
    assert.equal(sensor.hasDepth, true);
    assert.equal(sensor.depthCanvas.context.drawCalls.length, retainedDrawCount + 1,
        'cancel must retain the last depth pixels');
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
                metadata: { ...options, validAnchors: 1, failureCount: 0 },
            };
        },
    };
    const rawBytes = new Uint8Array([1, 2, 3, 4]);
    globalThis.fetch = async (_url, options) => {
        assert.equal(options.headers['X-Frame-ID'], String(materialized.frame.frameId));
        assert.equal(options.body, materialized.frame.rgb, 'raw request reuses the frozen frame JPEG');
        return {
            ok: true,
            status: 200,
            headers: { get(name) {
                return ({
                    'X-Frame-ID': String(materialized.frame.frameId),
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
    assert.equal(artifacts.manifest.frameId, String(materialized.frame.frameId));
    assert.equal(artifacts.anchors.metadata.locationId, 'street-a');
    assert.ok(artifacts.files['capture-001-raw.npz']);
    assert.ok(artifacts.files['capture-001-rgb.jpg']);
}

// HTML contract: one visible canvas, no hidden image, and 0.6 m collision defaults.
{
    const html = await readFile(new URL('../index.html', import.meta.url), 'utf8');
    assert.doesNotMatch(html, /id="panorama-depth-image"/);
    assert.match(html, /id="panorama-depth-canvas"/);
    assert.match(html, /id="phys-collision-radius"[^>]*value="0\.6"/);
    assert.match(html, /id="phys-collision-radius-num"[^>]*value="0\.6"/);
}

console.log('Panorama depth state tests: all passed');
