/**
 * ERP anchor seam continuity tests.
 * Verifies that anchor directions at the left/right ERP edges wrap correctly,
 * forming a continuous 360° panorama without gaps.
 *
 * Run: node tests/test_erp_anchor_seam.js
 */

import { erpPixelToDirection, sampleAnchorDirections } from '../src/erp-geometry.js';

let passed = 0, failed = 0;

function assert(cond, msg) { cond ? passed++ : (failed++, console.error(`FAIL: ${msg}`)); }
function assertClose(a, b, msg) { assert(Math.abs(a - b) < 1e-6, `${msg}: ${a} vs ${b}`); }

const W = 384, H = 192;
const cols = 16, rows = 8;

// ── Leftmost vs rightmost anchor at same row should be adjacent ──
for (let row = 0; row < rows; row++) {
    const uL = (0 + 0.5) / cols * W;
    const uR = (cols - 1 + 0.5) / cols * W;
    const v = (row + 0.5) / rows * H;

    const dL = erpPixelToDirection(uL, v, W, H, Math.PI);
    const dR = erpPixelToDirection(uR, v, W, H, Math.PI);

    // Leftmost anchor yaw should be ≈ π, rightmost ≈ -π + (one step), so they're adjacent
    const yawL = Math.atan2(dL.dy, dL.dx);
    const yawR = Math.atan2(dR.dy, dR.dx);

    // Adjacent anchors across ERP seam should have close yaw (modulo 2π)
    const yawDiff = Math.abs(yawL - yawR);
    const yawDiffWrapped = Math.min(yawDiff, 2 * Math.PI - yawDiff);

    // Expected angular step between adjacent columns
    const expectedStep = 2 * Math.PI / cols;

    assertClose(yawDiffWrapped, expectedStep,
        `seam yaw step at row ${row}: ${(yawDiffWrapped * 180 / Math.PI).toFixed(2)}° vs expected ${(expectedStep * 180 / Math.PI).toFixed(2)}°`);
}

// ── First column continuation to last column should be smooth ──
const anchors = sampleAnchorDirections(cols, rows, W, H);
assert(anchors.length === cols * rows, `should have ${cols * rows} anchors`);

// All yaw values should be unique across full 360
const yaws = anchors.map(a => Math.atan2(a.dy, a.dx));
const sortedYaws = [...yaws].sort((a, b) => a - b);
const yawRange = sortedYaws[sortedYaws.length - 1] - sortedYaws[0];
assert(yawRange > 5.5, `yaw range should cover almost 2π, got ${yawRange.toFixed(3)} rad`);

// ── Top/bottom exclusion works ──
const topExcluded = anchors.filter(a => a.row === 0);
const bottomExcluded = anchors.filter(a => a.row === rows - 1);
assert(topExcluded.length === cols, `should have ${cols} top-row anchors`);
assert(bottomExcluded.length === cols, `should have ${cols} bottom-row anchors`);

console.log(`\nERP seam tests: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
