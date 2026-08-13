/** Six-face capture yields between configurable face slices and honors AbortSignal. */
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
            getImageData(_x, _y, width, height) {
                return { data: new Uint8ClampedArray(width * height * 4) };
            },
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

// The 500 m startup preload must retain the complete 160 m grid. Truncating
// the nearest-first list at the old 420 m count drops most of the outer ring
// and makes coverage depend on iteration order.
{
    const world = Object.create(CesiumWorld.prototype);
    const targets = world._buildPreloadTargets(500, 160, 30);
    if (targets.length !== 29) {
        throw new Error(`expected the complete 500 m preload grid, got ${targets.length} targets`);
    }
    const keys = new Set(targets.map(({ x, z }) => `${x},${z}`));
    for (const { x, z } of targets) {
        if (!keys.has(`${-x},${-z}`)) {
            throw new Error(`500 m preload grid is asymmetric at ${x},${z}`);
        }
    }
    const maxRadius = Math.max(...targets.map(({ x, z }) => Math.hypot(x, z)));
    if (maxRadius < 450 || maxRadius > 500) {
        throw new Error(`500 m preload outer ring is incomplete: ${maxRadius.toFixed(2)} m`);
    }
}

// API credentials must be carried by the supported Google header, never in a
// browser-visible root or derived tile URL.
{
    let receivedResource = null;
    let receivedOptions = null;
    class FakeResource {
        constructor(options) { Object.assign(this, options); }
    }
    const world = Object.create(CesiumWorld.prototype);
    world.assetId = 123;
    world.Cesium = {
        IonResource: {
            async fromAssetId(assetId) {
                if (assetId !== 123) throw new Error('wrong asset ID');
                return {
                    url: 'https://tile.googleapis.com/v1/3dtiles/root.json?key=test-secret',
                    headers: { 'X-Test': 'retained' },
                    credits: ['credit'],
                    proxy: null,
                };
            },
        },
        Resource: FakeResource,
        Cesium3DTileset: {
            async fromUrl(resource, options) {
                receivedResource = resource;
                receivedOptions = options;
                return { resource };
            },
        },
    };
    await world._createGoogleTileset();
    if (!receivedResource
        || receivedResource.url.includes('test-secret')
        || receivedResource.url.includes('?')
        || receivedResource.headers['X-Goog-Api-Key'] !== 'test-secret'
        || receivedResource.headers['X-Test'] !== 'retained'
        || receivedOptions.enableCollision !== true) {
        throw new Error('Google Tiles API key was not moved from URL to header');
    }
}

// Hidden-viewer defaults avoid sibling/leaf over-fetch, while the explicit
// rollback profile retains the previous Cesium streaming flags for live A/B.
{
    const propertyNames = [
        'maximumScreenSpaceError', 'cullRequestsWhileMoving',
        'preloadWhenHidden', 'preloadFlightDestinations',
        'foveatedScreenSpaceError', 'dynamicScreenSpaceError',
        'dynamicScreenSpaceErrorDensity', 'dynamicScreenSpaceErrorFactor',
        'loadSiblings', 'skipLevelOfDetail', 'baseScreenSpaceError',
        'skipScreenSpaceErrorFactor', 'skipLevels',
        'immediatelyLoadDesiredLevelOfDetail', 'preferLeaves',
    ];
    const makeTileset = () => Object.fromEntries(propertyNames.map(name => [name, null]));
    const world = Object.create(CesiumWorld.prototype);
    world.panoramaTileSSE = 512;
    world.panoramaLeanStreaming = true;
    const lean = makeTileset();
    world._configurePanoramaTileset(lean);
    if (lean.loadSiblings || lean.preloadWhenHidden || lean.preloadFlightDestinations
        || lean.immediatelyLoadDesiredLevelOfDetail || lean.preferLeaves
        || lean.skipLevelOfDetail !== true) {
        throw new Error('lean panorama tileset profile retained an over-fetch flag');
    }

    world.panoramaLeanStreaming = false;
    const legacy = makeTileset();
    world._configurePanoramaTileset(legacy);
    if (!legacy.loadSiblings || !legacy.preloadWhenHidden || !legacy.preloadFlightDestinations
        || !legacy.immediatelyLoadDesiredLevelOfDetail || !legacy.preferLeaves
        || legacy.skipLevelOfDetail !== false) {
        throw new Error('legacy panorama tileset rollback profile was not preserved');
    }
}

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
    if (!result.complete || !result.ready || result.readyFaces !== 6
        || result.faceTileReadiness?.length !== 6) {
        throw new Error('ready capture did not expose complete 6/6 readiness provenance');
    }
}

// A zero-wait flight capture still returns its projected canvas for live use,
// while `ready` truthfully remains false until all six tile views are settled.
{
    const { world, viewer, transform } = harness();
    world._panoramaTileset.tilesLoaded = false;
    world._panoramaTileLoadState.pending = 2;
    const result = await world._capturePanoramaHybridWithViewerAsync(
        viewer, transform, 384, 192, 96, 180,
        { captureAnyway: true, facesPerSlice: 6 },
    );
    if (!result.canvas || !result.complete) {
        throw new Error('partial flight capture must retain the completed ERP canvas');
    }
    if (result.ready || result.allFacesTileReady || result.readyFaces !== 0
        || result.faceTileReadiness?.length !== 6
        || result.readinessReason !== 'tiles-partial') {
        throw new Error('partial flight capture was mislabeled as tiles-ready');
    }
}

// The one-shot flight preload must visit all six directions even when an
// individual tile wait times out. Otherwise a slow first face prevents the
// other five directions from ever issuing requests and warm-up is ineffective.
{
    const { world, viewer, transform, faces } = harness();
    world.waitForTilesIdle = async () => false;
    const result = await world._capturePanoramaHybridWithViewerAsync(
        viewer, transform, 384, 192, 96, 180,
        {
            captureAnyway: false,
            continueOnTileTimeout: true,
            tileTimeoutMs: 1,
            facesPerSlice: 6,
        },
    );
    if (faces.length !== 6 || !result.canvas || !result.complete) {
        throw new Error('preload timeout must continue through and project all six faces');
    }
    if (result.ready || result.allFacesTileReady || result.readyFaces !== 0) {
        throw new Error('timed-out preload was incorrectly marked tiles-ready');
    }
}

// Calibration keeps its strict semantics: a face timeout returns immediately
// and must never export a partial capture as a calibration sample.
{
    const { world, viewer, transform, faces } = harness();
    world.waitForTilesIdle = async () => false;
    const result = await world._capturePanoramaHybridWithViewerAsync(
        viewer, transform, 384, 192, 96, 180,
        { captureAnyway: false, tileTimeoutMs: 1, facesPerSlice: 6 },
    );
    if (faces.length !== 0 || result.canvas || result.complete || result.faceIndex !== 0) {
        throw new Error('strict calibration timeout must reject the first incomplete face');
    }
}

// Recent tile request failures remain visible even if Cesium's aggregate
// `tilesLoaded` flag has already bounced back to true.
{
    const { world, viewer, transform } = harness();
    world._panoramaTileLoadState.errorCount = 1;
    world._panoramaTileLoadState.lastErrorAt = performance.now();
    const result = await world._capturePanoramaHybridWithViewerAsync(
        viewer, transform, 384, 192, 96, 180,
        { captureAnyway: true, facesPerSlice: 6 },
    );
    if (!result.complete || result.ready || !result.tileError
        || result.readinessReason !== 'tile-error') {
        throw new Error('recent panorama tile failure was hidden by tilesLoaded=true');
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
        || flightOptions.tileQuietMs !== 0
        || flightOptions.facesPerSlice !== 2) {
        throw new Error('flight profile must force zero-wait capture with the 2-face default');
    }
    const flightPreloadOptions = flightSensor.getCaptureOptions({ preload: true });
    if (flightPreloadOptions.captureAnyway
        || !flightPreloadOptions.continueOnTileTimeout
        || flightPreloadOptions.frameDelayMs !== 96
        || flightPreloadOptions.tileTimeoutMs !== 6000
        || flightPreloadOptions.tileQuietMs !== 650
        || flightPreloadOptions.timeoutMs !== 60000) {
        throw new Error('flight preload must settle hidden-viewer tiles before zero-wait live capture');
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
        || calibrationOptions.continueOnTileTimeout
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

// Chrome planning pipelines the next six-face capture while one immutable
// upload/inference request remains in flight. The capture lock must be released,
// while the single-request gate prevents a second inference from queuing.
{
    const originalOffscreenCanvas = globalThis.OffscreenCanvas;
    let encodeCalls = 0;
    class FakeOffscreenCanvas extends FakeCanvas {
        async convertToBlob({ type }) {
            encodeCalls++;
            return new Blob(['frozen-rgb'], { type });
        }
    }
    globalThis.OffscreenCanvas = FakeOffscreenCanvas;
    try {
        const { PanoramaSensor } = await import('../src/panorama-sensor.js?chrome-pipeline-test');
        const sensor = new PanoramaSensor();
        let releaseRequest;
        let requestCalls = 0;
        let selectedRequest = null;
        sensor._shouldRequestDepth = () => !sensor._depthGate;
        sensor._requestDepth = () => {
            if (sensor._depthGate) return false;
            sensor._depthGate = true;
            requestCalls++;
            const frameContext = sensor._rgbFrameContext;
            selectedRequest = {
                requestId: requestCalls,
                frameId: frameContext.frameId,
                generation: sensor._yopoGeneration,
                planningEpoch: frameContext.planningEpoch,
                goalId: sensor._goalId,
                mode: sensor._desiredDepthMode(),
                frameContext,
            };
            sensor._activeDepthRequest = selectedRequest;
            // Production materialization reads the selected context exactly
            // once. This must be the only point that starts JPEG encoding.
            void frameContext.rgbPromise;
            return new Promise(resolve => { releaseRequest = resolve; })
                .finally(() => {
                    sensor._depthGate = false;
                    sensor._activeDepthRequest = null;
                });
        };
        const world = {
            async capturePanoramaIncrementalAsync() {
                return { complete: true, canvas: new FakeCanvas(), timings_ms: { total: 35 } };
            },
        };
        const transform = { position: { x: 0, y: 0, z: 0 } };

        await sensor._capture(world, transform);
        if (sensor.capturing || requestCalls !== 1 || encodeCalls !== 1 || !sensor._depthGate) {
            throw new Error('capture did not release independently of the in-flight inference');
        }
        await sensor._capture(world, transform);
        if (requestCalls !== 1 || encodeCalls !== 1) {
            throw new Error('a busy request gate allowed an intermediate JPEG/inference to queue');
        }
        if (sensor._rgbFrameId === selectedRequest.frameId
            || !sensor._isRequestSourceCurrent(selectedRequest)) {
            throw new Error('a newer capture invalidated an independently frozen RGB request');
        }
        const raw = await sensor._materializePerceptionFrame(
            selectedRequest.frameContext,
            'rgba8',
        );
        const rawPixels = raw.frame.projectionConfig.rgbWidth
            * raw.frame.projectionConfig.rgbHeight;
        if (raw.uploadEncoding !== 'rgba8'
            || raw.frame.rgb.type !== 'application/x-mindcloud-rgba8'
            || raw.frame.rgb.size !== rawPixels * 4
            || encodeCalls !== 1) {
            throw new Error('planning upload did not use one exact lossless RGBA8 frame');
        }
        sensor._planningEpoch++;
        if (sensor._isRequestSourceCurrent(selectedRequest)) {
            throw new Error('a frozen RGB request survived a real planning-session change');
        }
        sensor._planningEpoch--;
        releaseRequest(true);
        await Promise.resolve();
    } finally {
        if (originalOffscreenCanvas === undefined) delete globalThis.OffscreenCanvas;
        else globalThis.OffscreenCanvas = originalOffscreenCanvas;
    }
}

console.log('\nPanorama scheduler/profile tests: all passed');
