/**
 * Production CesiumWorld metric-anchor sampler tests with ray-cast stubs.
 * Run: node tests/test_metric_anchor_sampler.js
 */

globalThis.window = { location: { search: '' } };

const { CesiumWorld } = await import('../src/cesium-world.js');
const { erpPixelToComponentDirection } = await import('../src/erp-geometry.js');

let passed = 0, failed = 0;
function assert(cond, msg) { cond ? passed++ : (failed++, console.error(`FAIL: ${msg}`)); }
function assertClose(a, b, msg, eps = 1e-9) {
    assert(Math.abs(a - b) < eps, `${msg}: expected ${b}, got ${a}`);
}
function assertVector(actual, expected, msg) {
    assertClose(actual.x, expected.x, `${msg}.x`);
    assertClose(actual.y, expected.y, `${msg}.y`);
    assertClose(actual.z, expected.z, `${msg}.z`);
}

function makeWorld() {
    const rays = [];
    const world = Object.create(CesiumWorld.prototype);
    const panoramaViewer = { scene: {} };
    const panoramaTileset = { tilesLoaded: true };
    world._panoramaViewer = panoramaViewer;
    world._panoramaTileset = panoramaTileset;
    world._panoramaCaptureActiveCount = 0;
    world._panoramaFaceSize = 96;
    world._lastCompletedPanoramaCapture = {
        revision: 7,
        viewer: panoramaViewer,
        tileset: panoramaTileset,
        transform: {
            position,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        width: 768,
        height: 384,
        faceSize: 96,
        verticalFovDeg: 180,
        faceTileReadiness: ['px', 'nx', 'py', 'ny', 'pz', 'nz'].map(
            face => ({ face, readyWhenCopied: true })
        ),
        allFacesTileReady: true,
    };
    world.pickLocalRay = () => {
        throw new Error('metric anchors must not use the main viewer ray caster');
    };
    world._pickLocalRayFromViewer = (viewer, origin, direction) => {
        rays.push({ viewer, origin, direction });
        return { distance: 10, position: {
            x: origin.x + direction.x * 10,
            y: origin.y + direction.y * 10,
            z: origin.z + direction.z * 10,
        } };
    };
    return { world, rays, panoramaViewer, panoramaTileset };
}

const baseOptions = {
    gridCols: 1,
    gridRows: 1,
    imageWidth: 384,
    imageHeight: 192,
    verticalFovDeg: 180,
    excludeTopDeg: 0,
    excludeBottomDeg: 0,
    sessionId: 'session-20260809',
    locationId: 'street-a',
    captureId: 'capture-001',
    frameId: 'frame-9',
};
const position = { x: 2, y: 3, z: 4 };

// Identity panorama transform: ERP centre is sensor-forward and therefore
// component/world -Z, exactly like the projector shader.
{
    const { world, rays, panoramaViewer } = makeWorld();
    const result = world.sampleMetricDepthAnchors({
        position,
        orientation: { x: 0, y: 0, z: 0, w: 1 },
    }, baseOptions);
    assert(result.anchors.length === 1, 'identity sampler yields one anchor');
    assert(rays.length === 1, 'identity sampler casts one ray');
    assert(rays[0].viewer === panoramaViewer, 'sampler ray-casts against the RGB capture viewer');
    assertVector(rays[0].direction, { x: 0, y: 0, z: -1 }, 'identity centre ray');
    assertVector(result.anchors[0].componentDirection,
        erpPixelToComponentDirection(191.5, 95.5, 384, 192),
        'sampler uses canonical shader component helper');
    assertClose(result.anchors[0].u, 191.5, 'sampler centre u');
    assertClose(result.anchors[0].v, 95.5, 'sampler centre v');
    assert(result.metadata.schemaVersion === 1, 'sampler records strict schema version');
    assert(result.metadata.identity.sessionId === 'session-20260809', 'sampler records session ID');
    assert(result.metadata.identity.locationId === 'street-a', 'sampler records location ID');
    assert(result.metadata.identity.captureId === 'capture-001', 'sampler records capture ID');
    assert(result.metadata.identity.frameId === 'frame-9', 'sampler records perception frame ID');
    assert(result.metadata.image.width === 384 && result.metadata.image.height === 192,
        'sampler records the exact RGB image dimensions');
    assert(result.metadata.erp.componentFrame === '(+x right,+y up,+z back)',
        'sampler records the projector component convention');
    assert(result.metadata.raycastSource === 'panorama-capture-viewer',
        'sampler records the dedicated RGB capture viewer as ray source');
    assert(result.metadata.tilesetSharedWithRgb === true,
        'sampler records RGB/anchor tileset parity');
    assert(result.metadata.panoramaFaceSize === 96,
        'sampler records capture-viewer face size');
    assert(result.metadata.panoramaCaptureRevision === 7,
        'sampler records the matched completed RGB capture revision');
    assert(result.metadata.panoramaFaceTileReadiness.every(face => face.readyWhenCopied),
        'sampler records per-face tile readiness at RGB copy time');
    assert(result.metadata.panoramaSourceImage.width === 768
        && result.metadata.panoramaSourceImage.height === 384,
        'sampler records the source panorama projection dimensions');
}

{
    const { world } = makeWorld();
    world._lastCompletedPanoramaCapture.allFacesTileReady = false;
    world._lastCompletedPanoramaCapture.faceTileReadiness[2].readyWhenCopied = false;
    let message = '';
    try {
        world.sampleMetricDepthAnchors({
            position,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        }, baseOptions);
    } catch (error) {
        message = String(error.message || error);
    }
    assert(message.includes('before every cubemap face reported tiles ready'),
        'sampler rejects RGB with any face copied before tiles were ready');
}

// Width and height are rounded independently by the upload canvas. A shared
// scale whose two rounding intervals overlap is valid; a distorted ERP is not.
{
    const { world } = makeWorld();
    const result = world.sampleMetricDepthAnchors({
        position,
        orientation: { x: 0, y: 0, z: 0, w: 1 },
    }, { ...baseOptions, imageWidth: 115, imageHeight: 58 });
    assert(result.anchors.length === 1, 'half-pixel upload rounding preserves a valid ERP');
}

{
    const { world } = makeWorld();
    let message = '';
    try {
        world.sampleMetricDepthAnchors({
            position,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        }, { ...baseOptions, imageWidth: 115, imageHeight: 60 });
    } catch (error) {
        message = String(error.message || error);
    }
    assert(message.includes('image aspect does not match'),
        'sampler rejects an upload with genuine ERP distortion');
}

// Identity and transform fields are mandatory; incomplete artifacts must never
// reach the metric fitter.
{
    const { world } = makeWorld();
    let message = '';
    try {
        world.sampleMetricDepthAnchors({
            position,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        }, { ...baseOptions, sessionId: '' });
    } catch (error) {
        message = String(error.message || error);
    }
    assert(message.includes('sessionId is required'), 'sampler rejects missing session identity');
}

{
    const { world } = makeWorld();
    world._pickLocalRayFromViewer = () => ({ distance: 0, position });
    const result = world.sampleMetricDepthAnchors({
        position,
        orientation: { x: 0, y: 0, z: 0, w: 1 },
    }, baseOptions);
    assert(result.anchors.length === 0, 'non-positive metric hit cannot become an anchor');
    assert(result.failures[0]?.reason === 'out_of_range', 'non-positive hit has explicit failure reason');
}

// Calibration must fail closed rather than silently falling back to the main
// viewer or sampling while the shared capture viewer is being mutated.
{
    const { world } = makeWorld();
    world._panoramaViewer = null;
    let message = '';
    try {
        world.sampleMetricDepthAnchors({
            position,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        }, baseOptions);
    } catch (error) {
        message = String(error.message || error);
    }
    assert(message.includes('panorama capture viewer unavailable'),
        'sampler rejects missing RGB capture viewer without fallback');
}

{
    const { world } = makeWorld();
    world._panoramaCaptureActiveCount = 1;
    let message = '';
    try {
        world.sampleMetricDepthAnchors({
            position,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        }, baseOptions);
    } catch (error) {
        message = String(error.message || error);
    }
    assert(message.includes('capture is in progress'),
        'sampler rejects mutable in-progress capture state');
}

{
    const { world, panoramaViewer } = makeWorld();
    panoramaViewer.scene.primitives = { contains: () => false };
    let message = '';
    try {
        world.sampleMetricDepthAnchors({
            position,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        }, baseOptions);
    } catch (error) {
        message = String(error.message || error);
    }
    assert(message.includes('not attached'),
        'sampler rejects a tileset belonging to a different viewer');
}

{
    const { world } = makeWorld();
    world._lastCompletedPanoramaCapture = null;
    let message = '';
    try {
        world.sampleMetricDepthAnchors({
            position,
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        }, baseOptions);
    } catch (error) {
        message = String(error.message || error);
    }
    assert(message.includes('no completed RGB panorama capture matches'),
        'sampler rejects an unproven capture source');
}

{
    const { world } = makeWorld();
    let message = '';
    try {
        world.sampleMetricDepthAnchors({
            position: { ...position, x: position.x + 1 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        }, baseOptions);
    } catch (error) {
        message = String(error.message || error);
    }
    assert(message.includes('transform does not match'),
        'sampler rejects a transform from a different RGB capture');
}

// +90° world-Y capture orientation rotates component/world -Z to -X.
{
    const half = Math.PI / 4;
    const { world, rays } = makeWorld();
    world._lastCompletedPanoramaCapture.transform = {
        position,
        orientation: { x: 0, y: Math.sin(half), z: 0, w: Math.cos(half) },
    };
    world.sampleMetricDepthAnchors({
        position,
        orientation: { x: 0, y: Math.sin(half), z: 0, w: Math.cos(half) },
    }, baseOptions);
    assertVector(rays[0].direction, { x: -1, y: 0, z: 0 }, 'yaw-rotated centre ray');
}

console.log(`\nMetric anchor sampler tests: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
