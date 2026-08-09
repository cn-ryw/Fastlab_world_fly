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
    world.pickLocalRay = (origin, direction) => {
        rays.push({ origin, direction });
        return { distance: 10, position: {
            x: origin.x + direction.x * 10,
            y: origin.y + direction.y * 10,
            z: origin.z + direction.z * 10,
        } };
    };
    world._tilesReady = () => true;
    return { world, rays };
}

const baseOptions = {
    gridCols: 1,
    gridRows: 1,
    imageWidth: 384,
    imageHeight: 192,
    verticalFovDeg: 180,
    excludeTopDeg: 0,
    excludeBottomDeg: 0,
};
const position = { x: 2, y: 3, z: 4 };

// Identity panorama transform: ERP centre is sensor-forward and therefore
// component/world -Z, exactly like the projector shader.
{
    const { world, rays } = makeWorld();
    const result = world.sampleMetricDepthAnchors({
        position,
        orientation: { x: 0, y: 0, z: 0, w: 1 },
    }, { ...baseOptions, locationId: 'street-a', captureId: 'capture-001', frameId: 'frame-9' });
    assert(result.anchors.length === 1, 'identity sampler yields one anchor');
    assert(rays.length === 1, 'identity sampler casts one ray');
    assertVector(rays[0].direction, { x: 0, y: 0, z: -1 }, 'identity centre ray');
    assertVector(result.anchors[0].componentDirection,
        erpPixelToComponentDirection(191.5, 95.5, 384, 192),
        'sampler uses canonical shader component helper');
    assertClose(result.anchors[0].u, 191.5, 'sampler centre u');
    assertClose(result.anchors[0].v, 95.5, 'sampler centre v');
    assert(result.metadata.locationId === 'street-a', 'sampler records location ID');
    assert(result.metadata.captureId === 'capture-001', 'sampler records capture ID');
    assert(result.metadata.frameId === 'frame-9', 'sampler records perception frame ID');
}

// +90° world-Y capture orientation rotates component/world -Z to -X.
{
    const half = Math.PI / 4;
    const { world, rays } = makeWorld();
    world.sampleMetricDepthAnchors({
        position,
        orientation: { x: 0, y: Math.sin(half), z: 0, w: Math.cos(half) },
    }, baseOptions);
    assertVector(rays[0].direction, { x: -1, y: 0, z: 0 }, 'yaw-rotated centre ray');
}

console.log(`\nMetric anchor sampler tests: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
