/**
 * ERP geometry unit tests — pure math, no DOM/Cesium required.
 * Run: node tests/test_erp_geometry.js
 */

import { erpPixelToDirection, erpDirectionToPixel, sampleAnchorDirections, normalize3 } from '../src/erp-geometry.js';

const EPS = 1e-10;
let passed = 0, failed = 0;

function assert(cond, msg) {
    if (cond) { passed++; return; }
    failed++;
    console.error(`FAIL: ${msg}`);
}

function assertClose(a, b, msg) {
    assert(Math.abs(a - b) < 1e-8, `${msg}: expected ${b}, got ${a}`);
}

// ── pixel → direction → pixel round-trip ──
for (const [u, v, W, H] of [[192, 96, 384, 192], [50, 30, 672, 336], [0, 0, 384, 192], [383, 191, 384, 192]]) {
    const dir = erpPixelToDirection(u, v, W, H, Math.PI);
    const { u: u2, v: v2 } = erpDirectionToPixel(dir.dx, dir.dy, dir.dz, W, H, Math.PI);
    assertClose(u2, u, `round-trip u ${u}→${u2}`);
    assertClose(v2, v, `round-trip v ${v}→${v2}`);
}

// ── direction should be unit-length ──
for (let u = 0; u < 384; u += 32) {
    for (let v = 0; v < 192; v += 16) {
        const d = erpPixelToDirection(u, v, 384, 192, Math.PI);
        const len = Math.hypot(d.dx, d.dy, d.dz);
        assertClose(len, 1.0, `unit-length at (${u},${v})`);
    }
}

// ── cardinal directions ──
function testCardinal(u, v, W, H, expectedDir) {
    const d = erpPixelToDirection(u, v, W, H, Math.PI);
    assertClose(d.dx, expectedDir.dx, `dx at (${u},${v})`);
    assertClose(d.dy, expectedDir.dy, `dy at (${u},${v})`);
    assertClose(d.dz, expectedDir.dz, `dz at (${u},${v})`);
}

// Center → forward (+x)
{
    const d = erpPixelToDirection(191.5, 95.5, 384, 192, Math.PI);
    assert(d.dx > 0.99 && Math.abs(d.dy) < 0.01 && Math.abs(d.dz) < 0.01, 'center → forward');
}

// Left edge → backward (-x) due to ERP yaw wrapping (tolerance 0.01 due to pixel offset)
{
    const d = erpPixelToDirection(0, 95.5, 384, 192, Math.PI);
    assert(d.dx < -0.99, 'left edge dx ~ -1');
    assert(Math.abs(d.dy) < 0.01, 'left edge dy ~ 0');
}

// Bottom center → straight down (-z), tolerance 0.02
{
    const d = erpPixelToDirection(191.5, 191.5, 384, 192, Math.PI);
    assert(d.dz < -0.98, `bottom dz ~ -1, got ${d.dz}`);
}
// Top center → straight up (+z), tolerance 0.02
{
    const d = erpPixelToDirection(191.5, 0.5, 384, 192, Math.PI);
    assert(d.dz > 0.98, `top dz ~ +1, got ${d.dz}`);
}

// Right side → +y (ERP left edge is right/down from center pixel)
{
    const dR = erpPixelToDirection(383.5, 95.5, 384, 192, Math.PI);
    assert(Math.abs(dR.dy) < 0.02, 'right edge dy ~ 0 (backward, not left)');
}

// ── normalize3 ──
const n = normalize3({ x: 3, y: 4, z: 0 });
assertClose(n.x, 0.6, 'norm3 x');
assertClose(n.y, 0.8, 'norm3 y');
assertClose(n.z, 0, 'norm3 z');

// ── sampleAnchorDirections grid ──
const anchors = sampleAnchorDirections(16, 8, 384, 192);
assert(anchors.length === 128, `anchor count: expected 128, got ${anchors.length}`);
assert(anchors[0].col === 0 && anchors[0].row === 0, `first anchor should be (0,0)`);
assert(anchors[127].col === 15 && anchors[127].row === 7, `last anchor should be (15,7)`);
for (const a of anchors) {
    const len = Math.hypot(a.dx, a.dy, a.dz);
    assertClose(len, 1.0, `anchor (${a.col},${a.row}) unit`);
}

// ── seam continuity (ERP left/right edges should wrap) ──
const dLeftEdge = erpPixelToDirection(383, 95.5, 384, 192, Math.PI);
const dRightEdge = erpPixelToDirection(0, 95.5, 384, 192, Math.PI);
// Both should point roughly backward (-x), continuity across seam
assert(dLeftEdge.dx < -0.99 && dRightEdge.dx < -0.99, 'seam continuity: both edges should point ~backward');

// ── vfov parameter ──
const d90 = erpPixelToDirection(191.5, 95.5, 384, 192, Math.PI / 2); // 90° vfov
assertClose(d90.dz, 0, 'center row at 90° vfov should have dz=0');

console.log(`\nERP geometry tests: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
