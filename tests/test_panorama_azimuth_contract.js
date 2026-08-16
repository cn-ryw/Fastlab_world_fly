/**
 * End-to-end horizontal ERP direction contract (pure math + shader source).
 *
 * Authoritative training reference:
 *   YOPO_360_v15/Simulator/src/src/sensor_simulator.cu
 *   yaw = pi - (u + 0.5) / W * 2pi
 *   d_body = (cos(pitch)cos(yaw), cos(pitch)sin(yaw), sin(pitch))
 * where body is NWU (+x forward, +y left, +z up).
 *
 * Consequently, from left to right, the horizon is:
 *   seam/back -> body-left -> front -> body-right -> seam/back.
 * The 2026-08-09 regression changed component x from +sin(yaw) to
 * -sin(yaw).  That swaps W/4 and 3W/4 and mirrors the actual DA360/YOPO
 * input; it is not a display-only transform.
 *
 * Run: node tests/test_panorama_azimuth_contract.js
 */

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
    erpPixelToComponentDirection,
    erpPixelToDirection,
} from '../src/erp-geometry.js';

const W = 384;
const H = 192;
const horizonV = H / 2 - 0.5;

function close(actual, expected, label, tolerance = 1e-10) {
    assert.ok(Math.abs(actual - expected) <= tolerance,
        `${label}: expected ${expected}, got ${actual}`);
}

const cardinals = [
    {
        label: 'back/seam', u: -0.5,
        body: { dx: -1, dy: 0, dz: 0 },
        component: { x: 0, y: 0, z: 1 },
    },
    {
        label: 'body-left', u: W / 4 - 0.5,
        body: { dx: 0, dy: 1, dz: 0 },
        component: { x: 1, y: 0, z: 0 },
    },
    {
        label: 'front', u: W / 2 - 0.5,
        body: { dx: 1, dy: 0, dz: 0 },
        component: { x: 0, y: 0, z: -1 },
    },
    {
        label: 'body-right', u: 3 * W / 4 - 0.5,
        body: { dx: 0, dy: -1, dz: 0 },
        component: { x: -1, y: 0, z: 0 },
    },
];

// Treat each captured cubemap face as a solid colour.  The face labels are
// legacy algebraic names (`right` is local +X), so annotate their physical
// meaning explicitly.  A correctly projected ERP horizon must produce this
// colour order; the 2026-08-09 sign regression swaps green and yellow.
const faceColours = Object.freeze({
    front: 'red',
    right: 'green', // local +X = body-left
    back: 'blue',
    left: 'yellow', // local -X = body-right
});

function dominantFace(component) {
    const ax = Math.abs(component.x);
    const ay = Math.abs(component.y);
    const az = Math.abs(component.z);
    if (ay >= ax && ay >= az) return component.y >= 0 ? 'up' : 'down';
    if (ax >= az) return component.x >= 0 ? 'right' : 'left';
    return component.z >= 0 ? 'back' : 'front';
}

for (const expected of cardinals) {
    const body = erpPixelToDirection(expected.u, horizonV, W, H);
    const component = erpPixelToComponentDirection(expected.u, horizonV, W, H);
    for (const axis of ['dx', 'dy', 'dz']) {
        close(body[axis], expected.body[axis], `${expected.label} body.${axis}`);
    }
    for (const axis of ['x', 'y', 'z']) {
        close(component[axis], expected.component[axis], `${expected.label} component.${axis}`);
    }
}

assert.deepEqual(
    cardinals.map(({ u }) => faceColours[
        dominantFace(erpPixelToComponentDirection(u, horizonV, W, H))
    ]),
    ['blue', 'green', 'red', 'yellow'],
    'solid-colour faces must appear back, body-left, front, body-right',
);

// Within the front face, Cesium screen-right is local body-right (-X) because
// local (east, up, north) is remapped to ENU (east, north, up).  The shader's
// front-face right vector is therefore -X: a body-left ray (+X) samples the
// left half of the captured face rather than introducing a second mirror.
const nearFrontBodyLeft = erpPixelToComponentDirection(W / 2 - 16.5, horizonV, W, H);
const frontTextureU = 0.5 * (
    -nearFrontBodyLeft.x / Math.max(1e-12, -nearFrontBodyLeft.z)
) + 0.5;
assert.ok(frontTextureU < 0.5, 'body-left detail stays left within the front face');

// UI preview and JPEG upload both draw the projector canvas without a
// horizontal transform.  Therefore the projector shader itself must use the
// training sign; correcting only the UI would leave DA360/YOPO mirrored.
const worldSource = readFileSync(new URL('../src/cesium-world.js', import.meta.url), 'utf8');
assert.match(worldSource, /float left = cosPitch \* sin\(yaw\);/);
assert.match(worldSource, /return normalize\(vec3\(left, sin\(pitch\), -forward\)\);/);
assert.doesNotMatch(worldSource, /vec3\(-right, sin\(pitch\), -forward\)/);

const sensorSource = readFileSync(new URL('../src/panorama-sensor.js', import.meta.url), 'utf8');
assert.match(sensorSource,
    /ctx\.drawImage\(panoCanvas, 0, 0, this\.rgbCanvas\.width, this\.rgbCanvas\.height\);/,
    'the UI draws the projected ERP without a display-only mirror');
assert.match(sensorSource, /this\._requestDepth\(this\.rgbCanvas\);/,
    'the exact displayed RGB canvas is also the DA360\/YOPO request source');
assert.doesNotMatch(sensorSource, /scale\s*\(\s*-1\s*,/,
    'the sensor pipeline must not hide a second horizontal mirror');

console.log('panorama azimuth contract tests: passed');
