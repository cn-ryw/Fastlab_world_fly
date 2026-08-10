/** Six-face capture yields after every two faces and honors AbortSignal. */
let yieldCount = 0;
globalThis.window = {
    location: { search: '' },
    setTimeout: globalThis.setTimeout.bind(globalThis),
    requestAnimationFrame(callback) {
        yieldCount++;
        queueMicrotask(() => callback(performance.now()));
    },
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

{
    yieldCount = 0;
    const { world, viewer, transform, faces } = harness();
    const result = await world._capturePanoramaHybridWithViewerAsync(
        viewer, transform, 384, 192, 96, 180,
        { captureAnyway: true, facesPerSlice: 2 },
    );
    if (faces.length !== 6) throw new Error(`expected 6 captured faces, got ${faces.length}`);
    if (yieldCount !== 2) throw new Error(`expected 2 scheduler yields, got ${yieldCount}`);
    if (!(result.timings_ms.total >= 0 && result.timings_ms.project >= 0)) {
        throw new Error('segmented capture timings missing');
    }
    if (!result.allFacesTileReady || !world._lastCompletedPanoramaCapture?.allFacesTileReady) {
        throw new Error('per-face tile readiness was not retained with the completed RGB capture');
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

console.log('\nPanorama scheduler: 8 passed, 0 failed');
