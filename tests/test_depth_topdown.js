/** Pure contract tests for the north-up 360-depth top-down renderer. */
import assert from 'node:assert/strict';
import {
    bodyAzimuthToWorld,
    depthTopdownLabels,
    drawDepthTopdown,
    normalizeDepthPolarScan,
    polarScanCanvasPoints,
    RELATIVE_TEST_CLICK_RADIUS_M,
    simYawToCompassBearingDeg,
    topdownClickToGoalOffset,
} from '../src/depth-topdown.js';

function close(actual, expected, label, tolerance = 1e-9) {
    assert.ok(Math.abs(actual - expected) <= tolerance,
        `${label}: ${actual} != ${expected}`);
}

const relative = normalizeDepthPolarScan({
    schema_version: 1,
    depth_mode: 'da360-relative',
    unit: 'x-near-reference',
    radius: 20,
    angle_start_deg: -135,
    angle_step_deg: 90,
    angle_positive: 'body-left',
    normalization: 'per-frame-depth-p02',
    valid_fraction: 1,
    values: [20, 10, 5, 2],
});
assert.ok(relative);
assert.equal(relative.metric, false);
assert.equal(depthTopdownLabels(relative).mode, 'RELATIVE · CLICK TEST ONLY');
assert.equal(depthTopdownLabels(relative).range, 'DEPTH 20×p02 · CLICK R20m');

let clickGoal = topdownClickToGoalOffset(relative, 95, 11, 190, 190);
assert.equal(clickGoal.mapping, 'relative-test');
assert.equal(clickGoal.radiusM, RELATIVE_TEST_CLICK_RADIUS_M);
close(clickGoal.east, 0, 'relative test click north east');
close(clickGoal.north, 20, 'relative test click uses nominal 20m radius');
assert.equal(topdownClickToGoalOffset(relative, 190, 95, 190, 190), null,
    'click outside circular plot is ignored');

assert.equal(normalizeDepthPolarScan({
    schema_version: 1,
    depth_mode: 'da360-relative',
    unit: 'metres',
    radius: 20,
    angle_start_deg: 0,
    angle_step_deg: 90,
    values: [1, 1, 1, 1],
}), null, 'relative data must never be accepted as metric');

let direction = bodyAzimuthToWorld(0, 0);
close(direction.east, 0, 'yaw zero forward east');
close(direction.north, -1, 'yaw zero is fixed south');

direction = bodyAzimuthToWorld(0, 180);
close(direction.east, 0, 'yaw 180 forward east', 1e-8);
close(direction.north, 1, 'yaw 180 faces north');

direction = bodyAzimuthToWorld(90, 0);
close(direction.east, 1, 'body-left from south points east');
close(direction.north, 0, 'body-left from south north');
assert.equal(simYawToCompassBearingDeg(0), 180, 'identity yaw has south bearing');
assert.equal(simYawToCompassBearingDeg(180), 0, 'yaw + 180 degrees faces north');
assert.equal(simYawToCompassBearingDeg(-90), 90, 'negative yaw faces east');

const metric = normalizeDepthPolarScan({
    schema_version: 1,
    depth_mode: 'da360-metric',
    unit: 'metres',
    radius: 20,
    angle_start_deg: 0,
    angle_step_deg: 90,
    angle_positive: 'body-left',
    valid_fraction: 0.75,
    values: [10, 20, null, 40],
});
assert.ok(metric?.metric);
assert.equal(depthTopdownLabels(metric).range, 'R 20m');
clickGoal = topdownClickToGoalOffset(metric, 179, 95, 190, 190);
assert.equal(clickGoal.mapping, 'metric');
close(clickGoal.east, 20, 'metric click uses physical scan radius');
close(clickGoal.north, 0, 'metric click east has no north offset');
const points = polarScanCanvasPoints(metric, 180, 50, 50, 40);
close(points[0].x, 50, 'north point x', 1e-8);
close(points[0].y, 30, '10m north maps to half radius');
assert.equal(points[1].clipped, true);
assert.equal(points[2], null);
assert.equal(points[3].clipped, true, 'values beyond radius remain visibly clipped');

const shifted = polarScanCanvasPoints(metric, 180, 50, 50, 40, { east: 5, north: 0 });
close(shifted[0].x, 60, 'metric endpoint includes capture-to-current east offset');
close(shifted[0].y, 30, 'metric endpoint retains north range after translation');
assert.equal(shifted[1].clipped, true,
    'a sensor-range-clipped ray stays clipped after translating its endpoint');

class FakeContext {
    constructor() { this.moves = []; this.texts = []; }
    save() {}
    restore() {}
    clearRect() {}
    fillRect() {}
    beginPath() {}
    arc() {}
    clip() {}
    stroke() {}
    fill() {}
    fillText(text) { this.texts.push(text); }
    lineTo() {}
    closePath() {}
    setLineDash() {}
    moveTo(x, y) { this.moves.push({ x, y }); }
}
const fakeContext = new FakeContext();
drawDepthTopdown(fakeContext, { width: 100, height: 100 }, metric, {
    captureYawDeg: 180,
    currentYawDeg: 180,
    originOffset: { east: 5, north: 0 },
});
close(fakeContext.moves[0].x, 59.75,
    'metric free-space fan begins at translated capture origin');
close(fakeContext.moves[0].y, 50,
    'translated capture origin retains north coordinate');

const relativeGoalContext = new FakeContext();
drawDepthTopdown(relativeGoalContext, { width: 190, height: 190 }, relative, {
    captureYawDeg: 0,
    currentYawDeg: 0,
    goalOffset: { east: 20, north: 0 },
});
assert(relativeGoalContext.texts.includes('TEST'),
    'relative click goal receives explicit visible TEST feedback');

assert.equal(normalizeDepthPolarScan({
    schema_version: 1,
    depth_mode: 'unknown-mode',
    unit: 'x-near-reference',
    radius: 20,
    angle_start_deg: 0,
    angle_step_deg: 90,
    angle_positive: 'body-left',
    values: [1, 1, 1, 1],
}), null, 'unknown depth modes fail closed');

assert.equal(normalizeDepthPolarScan({
    schema_version: 1,
    depth_mode: 'da360-relative',
    unit: 'x-near-reference',
    radius: 20,
    angle_start_deg: 0,
    angle_step_deg: 0.5,
    angle_positive: 'body-left',
    values: Array(721).fill(1),
}), null, 'oversized scans fail closed');

console.log('depth top-down tests: passed');
