/** Six-face capture yields after every two faces and honors AbortSignal. */
let yieldCount = 0;
globalThis.window = {
    location: { search: '', hostname: '127.0.0.1' },
    setTimeout: globalThis.setTimeout.bind(globalThis),
    clearTimeout: globalThis.clearTimeout.bind(globalThis),
    requestAnimationFrame(callback) {
        yieldCount++;
        queueMicrotask(() => callback(performance.now()));
    },
};

class FakeElement {
    constructor() {
        this.textContent = '';
        this.title = '';
        this.dataset = {};
        this.checked = true;
        this.classList = { toggle() {}, add() {}, remove() {} };
    }
}

class FakeCanvas extends FakeElement {
    constructor(width = 384, height = 192) {
        super();
        this.width = width;
        this.height = height;
        this.context = {
            clearRect() {}, drawImage() {}, fillRect() {}, beginPath() {},
            moveTo() {}, lineTo() {}, stroke() {}, fillText() {},
            createLinearGradient() { return { addColorStop() {} }; },
        };
    }
    getContext() { return this.context; }
}

const elements = new Map([
    ['panorama-sensor-panel', new FakeElement()],
    ['panorama-rgb-canvas', new FakeCanvas()],
    ['panorama-depth-canvas', new FakeCanvas()],
    ['panorama-rgb-status', new FakeElement()],
    ['panorama-depth-status', new FakeElement()],
    ['panorama-depth-near-label', new FakeElement()],
    ['panorama-depth-far-label', new FakeElement()],
    ['panorama-depth-unit', new FakeElement()],
]);
globalThis.HTMLCanvasElement = FakeCanvas;
globalThis.document = {
    getElementById(id) { return elements.get(id) || null; },
    createElement(tag) { return tag === 'canvas' ? new FakeCanvas() : new FakeElement(); },
};

const { CesiumWorld } = await import('../src/cesium-world.js?scheduler-test');

function harness() {
    const faces = [];
    const projector = {
        readyFaces: new Set(),
        updateFace(name) { faces.push(name); },
        render() { return { width: 384, height: 192 }; },
    };
    const world = Object.create(CesiumWorld.prototype);
    world._getPanoramaProjector = () => projector;
    world.getTransformBasisFixed = () => ({ right: {}, up: {}, back: {} });
    world.localToCartesian = value => value;
    world._componentDirectionToFixed = (_basis, value) => value;
    world._renderViewerNow = () => {};
    const viewer = {
        camera: { frustum: {}, setView() {} },
        scene: { canvas: { width: 96, height: 96 }, requestRender() {} },
    };
    world._panoramaViewer = viewer;
    world._panoramaTileset = { tilesLoaded: true };
    world._panoramaTileLoadState = { pending: 0, processing: 0 };
    const transform = {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
    };
    return { world, viewer, transform, faces };
}

for (const [facesPerSlice, expectedYields] of [[2, 2], [3, 1], [6, 0]]) {
    yieldCount = 0;
    globalThis.window.requestAnimationFrame = callback => {
        yieldCount++;
        queueMicrotask(() => callback(performance.now()));
    };
    const { world, viewer, transform, faces } = harness();
    const result = await world._capturePanoramaHybridWithViewerAsync(
        viewer, transform, 384, 192, 96, 180,
        { captureAnyway: true, facesPerSlice },
    );
    if (faces.length !== 6) throw new Error(`expected 6 captured faces, got ${faces.length}`);
    if (yieldCount !== expectedYields) {
        throw new Error(`facesPerSlice=${facesPerSlice}: expected ${expectedYields} yields, got ${yieldCount}`);
    }
    for (const key of [
        'scene_render', 'tile_wait', 'wait_rerender', 'face_upload',
        'project', 'scheduler', 'total', 'render',
    ]) {
        if (!(result.timings_ms[key] >= 0)) throw new Error(`capture timing ${key} missing`);
    }
    if (result.timings_ms.render !== result.timings_ms.scene_render + result.timings_ms.wait_rerender) {
        throw new Error('render timing must contain only actual scene renders');
    }
    if (!result.allFacesTileReady || !world._lastCompletedPanoramaCapture?.allFacesTileReady) {
        throw new Error('per-face tile readiness was not retained with the completed RGB capture');
    }
}

// A dedicated viewer has no default render loop. Tile-idle polling must execute
// a real render on every poll rather than only setting Scene.requestRender().
{
    const world = Object.create(CesiumWorld.prototype);
    const tileset = { tilesLoaded: false };
    const loadState = { pending: 1, processing: 0 };
    let requestRenderCount = 0;
    let actualRenderCount = 0;
    const viewer = {
        isDestroyed: () => false,
        scene: { requestRender() { requestRenderCount++; } },
    };
    world._renderViewerNow = renderedViewer => {
        if (renderedViewer !== viewer) throw new Error('tile wait rendered the wrong viewer');
        actualRenderCount++;
        if (actualRenderCount === 2) {
            tileset.tilesLoaded = true;
            loadState.pending = 0;
        }
    };
    const renderTimings = { renderMs: 0, renderCount: 0 };
    const ready = await world.waitForTilesIdle(
        500, 0, tileset, loadState, viewer, renderTimings,
    );
    if (!ready) throw new Error('real-render tile wait did not become ready');
    if (actualRenderCount !== 2 || requestRenderCount !== actualRenderCount) {
        throw new Error(`expected a real render on each poll, got request=${requestRenderCount}, actual=${actualRenderCount}`);
    }
    if (renderTimings.renderCount !== actualRenderCount || renderTimings.renderMs < 0) {
        throw new Error('wait re-render timings were not accumulated');
    }
}

{
    const controller = new AbortController();
    const { world, viewer, transform, faces } = harness();
    globalThis.window.requestAnimationFrame = callback => {
        controller.abort('test-cancel');
        queueMicrotask(() => callback(performance.now()));
    };
    await world._capturePanoramaHybridWithViewerAsync(
        viewer, transform, 384, 192, 96, 180,
        { captureAnyway: true, facesPerSlice: 2, signal: controller.signal },
    ).then(
        () => { throw new Error('aborted capture unexpectedly resolved'); },
        error => {
            if (error.name !== 'AbortError') throw error;
        },
    );
    if (faces.length !== 2) throw new Error(`abort should stop after first slice, got ${faces.length} faces`);
}

// The public wrappers must preserve the 80/64 performance-sweep candidates;
// silently clamping them to 96 would make the benchmark labels untruthful.
{
    const world = Object.create(CesiumWorld.prototype);
    world.viewer = {};
    world.ready = true;
    const requestedSizes = [];
    world._ensurePanoramaCaptureViewer = async size => {
        requestedSizes.push(size);
        return { size };
    };
    world._capturePanoramaHybridWithViewerAsync = async (_viewer, _transform, _width, _height, faceSize) => ({
        faceSize,
    });
    const transform = {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
    };
    const low = await world.capturePanoramaIncrementalAsync(transform, { faceSize: 64 });
    const mid = await world.preloadPanoramaAtTransform(transform, { faceSize: 80 });
    if (low.faceSize !== 64 || mid.faceSize !== 80) {
        throw new Error(`performance sweep face sizes were clamped: ${low.faceSize}, ${mid.faceSize}`);
    }
    if (requestedSizes.join(',') !== '64,80') {
        throw new Error(`capture viewers received wrong face sizes: ${requestedSizes.join(',')}`);
    }
}

// Profiles default to flight, recognize the legacy strict URL, abort an active
// capture when switched, and force YOPO goals back to the non-blocking profile.
{
    globalThis.window.location.search = '';
    const { PanoramaSensor: FlightSensor } = await import('../src/panorama-sensor.js?profile-flight-test');
    const flightSensor = new FlightSensor();
    if (flightSensor.getCaptureProfile() !== 'flight') throw new Error('capture profile must default to flight');
    const flightOptions = flightSensor.getCaptureOptions();
    if (!flightOptions.captureAnyway
        || flightOptions.frameDelayMs !== 0
        || flightOptions.tileTimeoutMs !== 0
        || flightOptions.tileQuietMs !== 0) {
        throw new Error('flight profile must force a zero-wait capture');
    }

    globalThis.window.location.search = '?panoProfile=calibration&panoCaptureAnyway=1'
        + '&panoFrameDelayMs=12&panoFaceTileTimeoutMs=1234&panoFaceTileQuietMs=321';
    const { PanoramaSensor: ExplicitCalibrationSensor } = await import('../src/panorama-sensor.js?profile-explicit-test');
    const explicitSensor = new ExplicitCalibrationSensor();
    const explicitOptions = explicitSensor.getCaptureOptions();
    if (explicitSensor.getCaptureProfile() !== 'calibration'
        || explicitOptions.captureAnyway
        || explicitOptions.frameDelayMs !== 12
        || explicitOptions.tileTimeoutMs !== 1234
        || explicitOptions.tileQuietMs !== 321) {
        throw new Error('explicit calibration profile did not honor strict query parameters');
    }

    globalThis.window.location.search = '?panoCaptureAnyway=0';
    const { PanoramaSensor: LegacyCalibrationSensor } = await import('../src/panorama-sensor.js?profile-legacy-test');
    const sensor = new LegacyCalibrationSensor();
    if (sensor.getCaptureProfile() !== 'calibration') {
        throw new Error('legacy strict URL was not recognized as calibration');
    }
    const calibrationOptions = sensor.getCaptureOptions();
    if (calibrationOptions.captureAnyway
        || calibrationOptions.tileTimeoutMs !== 6000
        || calibrationOptions.tileQuietMs !== 650) {
        throw new Error('calibration profile did not retain strict 6000/650 defaults');
    }

    const activeCapture = new AbortController();
    sensor._captureAbortController = activeCapture;
    sensor.setCaptureProfile('flight', 'test-switch');
    if (!activeCapture.signal.aborted) throw new Error('profile switch did not abort the active capture');
    if (sensor.getCaptureProfile() !== 'flight') throw new Error('profile switch did not commit');

    sensor.setCaptureProfile('calibration');
    sensor._captureAbortController = null;
    sensor.setYopoGoal({ x: 1, y: 2, z: 3 });
    if (sensor.getCaptureProfile() !== 'flight') throw new Error('YOPO goal did not force flight profile');
    let calibrationRejected = false;
    try {
        sensor.setCaptureProfile('calibration', 'test-active-navigation');
    } catch (error) {
        calibrationRejected = /navigation is active/.test(String(error?.message));
    }
    if (!calibrationRejected || sensor.getCaptureProfile() !== 'flight') {
        throw new Error('active navigation must reject the blocking calibration profile');
    }
}

// The automatic capture path passes the frozen profile as an explicit option
// and stores the same profile/timings on the completed RGB frame context.
{
    globalThis.window.location.search = '';
    const { PanoramaSensor } = await import('../src/panorama-sensor.js?profile-capture-args-test');
    const sensor = new PanoramaSensor();
    sensor.setCaptureProfile('calibration');
    sensor._requestDepth = () => false;
    const transform = { position: { x: 4, y: 5, z: 6 } };
    let receivedTransform = null;
    let receivedOptions = null;
    const captureResult = {
        complete: true,
        canvas: new FakeCanvas(),
        timings_ms: { total: 41, render: 11, tile_wait: 30 },
    };
    await sensor._capture({
        async capturePanoramaIncrementalAsync(actualTransform, options) {
            receivedTransform = actualTransform;
            receivedOptions = options;
            return captureResult;
        },
    }, transform);
    if (receivedTransform !== transform) throw new Error('capture transform argument shifted');
    if (receivedOptions.profile !== 'calibration'
        || receivedOptions.captureAnyway
        || receivedOptions.tileTimeoutMs !== 6000
        || receivedOptions.tileQuietMs !== 650
        || !(receivedOptions.signal instanceof AbortSignal)) {
        throw new Error('automatic capture arguments did not preserve calibration profile/options');
    }
    if (sensor._rgbFrameContext.captureProfile !== 'calibration') {
        throw new Error('completed capture lost its frozen profile');
    }
    if (sensor._rgbFrameContext.captureTimings.total !== captureResult.timings_ms.total) {
        throw new Error('completed capture lost structured timings');
    }
}

console.log('\nPanorama scheduler/profile tests: all passed');
