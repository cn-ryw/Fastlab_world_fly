/**
 * Metric anchor direction validation — pure math, verifies anchor grid covers
 * the expected ERP area (excluding pole guards) and that all directions are
 * valid unit vectors.
 *
 * Run: node tests/test_metric_anchor_direction.js
 */

import { erpPixelToDirection } from '../src/erp-geometry.js';

const EPS = 1e-10;
let passed = 0, failed = 0;

function assert(cond, msg) { cond ? passed++ : (failed++, console.error(`FAIL: ${msg}`)); }
function assertClose(a, b, msg) { assert(Math.abs(a - b) < 1e-8, `${msg}: ${a} vs ${b}`); }

const W = 384, H = 192;
const cols = 16, rows = 8;
const vfovRad = Math.PI;
const topExclude = 15, bottomExclude = 5;

for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
        const u = (col + 0.5) / cols * W;
        const v = (row + 0.5) / rows * H;
        const pitchRad = vfovRad / 2.0 - (v + 0.5) / H * vfovRad;
        const pitchDeg = pitchRad * 180 / Math.PI;

        // Pole exclusion check
        const excluded = pitchDeg > (90 - topExclude) || pitchDeg < (-90 + bottomExclude);

        const d = erpPixelToDirection(u, v, W, H, vfovRad);
        const len = Math.hypot(d.dx, d.dy, d.dz);

        // All anchors should be unit vectors regardless of exclusion
        assertClose(len, 1.0, `anchor (${col},${row}) unit-length`);

        // Top row should be excluded due to top pole guard (pitch > 90-15=75°)
        if (row === 0) {
            assert(excluded, `top row anchor (${col},0) should be pole-excluded (pitch ${pitchDeg.toFixed(1)}°)`);
        }
        // Bottom row with 8 rows spans ~84.4°–61.9°, all above -85° exclusion
        // so bottom exclusion only triggers with more rows or larger guard
    }
}

// Count total excluded anchors (pole guard coverage)
let excludedCount = 0;
for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
        const v = (row + 0.5) / rows * H;
        const pitchRad = vfovRad / 2.0 - (v + 0.5) / H * vfovRad;
        const pitchDeg = pitchRad * 180 / Math.PI;
        if (pitchDeg > (90 - topExclude) || pitchDeg < (-90 + bottomExclude))
            excludedCount++;
    }
}
// With 8 rows (22.5°/row), only top row exceeds top 15° guard; bottom guard (5°) doesn't catch any row
const expectedExcluded = cols * 1; // only top row
assert(excludedCount === expectedExcluded,
    `expected ${expectedExcluded} pole-excluded anchors, got ${excludedCount}`);

// Verify regular (non-excluded) anchors have valid pitch ranges
let minPitch = Infinity, maxPitch = -Infinity;
for (let row = 1; row < rows - 1; row++) {
    const v = (row + 0.5) / rows * H;
    const pitchDeg = (vfovRad / 2.0 - (v + 0.5) / H * vfovRad) * 180 / Math.PI;
    minPitch = Math.min(minPitch, pitchDeg);
    maxPitch = Math.max(maxPitch, pitchDeg);
}
assert(minPitch >= -85, `min pitch ${minPitch} should be >= -85`);
assert(maxPitch <= 75, `max pitch ${maxPitch} should be <= 75`);

console.log(`\nMetric anchor direction tests: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
