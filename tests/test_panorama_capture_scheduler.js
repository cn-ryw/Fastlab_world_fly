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

console.log('\nPanorama scheduler: 5 passed, 0 failed');
