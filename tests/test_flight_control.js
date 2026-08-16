/**
 * Pure flight-control math tests.
 * Run: node tests/test_flight_control.js
 */

import {
    ControlCommandType,
    allocateEasyForce,
    capDirectForce,
    desiredAttitudeFromForce,
    firstOrderRateServo,
    integrateBodyRates,
    limitVector,
    piecewiseHoverThrottle,
    reducedQuaternionBodyRateSetpoint,
    rotateVectorByQuaternion,
    shapeAssistedAxis,
    vectorNorm,
} from '../src/flight-control.js';

let passed = 0;
let failed = 0;

function assert(condition, message) {
    if (condition) {
        passed++;
        return;
    }
    failed++;
    console.error(`FAIL: ${message}`);
}

function near(actual, expected, tolerance, message) {
    assert(Math.abs(actual - expected) <= tolerance,
        `${message}: expected ${expected}, got ${actual}`);
}

function finiteVector(vector, message) {
    assert(Number.isFinite(vector.x) && Number.isFinite(vector.y) && Number.isFinite(vector.z), message);
}

const DEG2RAD = Math.PI / 180;

// Command values are stable wire/debug identifiers.
assert(Object.isFrozen(ControlCommandType), 'command type table is immutable');
assert(ControlCommandType.DIRECT_ACCELERATION === 'DIRECT_ACCELERATION', 'direct acceleration command is exported');

// Assisted-axis shaping is continuous, symmetric, bounded, and deterministic.
near(shapeAssistedAxis(0.1, 0.1, 0), 0, 1e-12, 'deadzone edge maps to zero');
near(shapeAssistedAxis(0.55, 0.1, 0), 0.5, 1e-12, 'linear deadzone remap');
near(shapeAssistedAxis(-0.55, 0.1, 0), -0.5, 1e-12, 'axis shaping is odd symmetric');
near(shapeAssistedAxis(0.55, 0.1, 1), 0.125, 1e-12, 'full expo is cubic');
near(shapeAssistedAxis(2, 0.1, 0.4), 1, 1e-12, 'axis is bounded');
near(shapeAssistedAxis(NaN, 0.1, 0), 0, 0, 'invalid axis safely centers');

// Stabilized throttle has no lower-half dead region.
near(piecewiseHoverThrottle(-1, 980, 2600), 0, 1e-12, 'bottom stick is zero thrust');
near(piecewiseHoverThrottle(-0.5, 980, 2600), 490, 1e-12, 'lower half interpolates to hover');
near(piecewiseHoverThrottle(0, 980, 2600), 980, 1e-12, 'center stick is hover');
near(piecewiseHoverThrottle(0.5, 980, 2600), 1790, 1e-12, 'upper half interpolates to maximum');
near(piecewiseHoverThrottle(1, 980, 2600), 2600, 1e-12, 'top stick is maximum thrust');
near(piecewiseHoverThrottle(NaN, 980, 2600), 980, 1e-12, 'invalid stick safely commands hover');
near(piecewiseHoverThrottle(0, NaN, NaN), 0, 0, 'invalid thrust bounds collapse to zero');

// Vector limiting preserves direction and never emits NaN.
near(vectorNorm({ x: 3, y: 4, z: 0 }), 5, 1e-12, 'vector norm');
const limited = limitVector({ x: 3, y: 4, z: 0 }, 2);
near(limited.x, 1.2, 1e-12, 'limited vector x');
near(limited.y, 1.6, 1e-12, 'limited vector y');
finiteVector(limitVector({ x: NaN, y: 0, z: 0 }, 2), 'invalid vector limit is finite');

// Desired attitude: identity hover and yaw convention.
const hoverAttitude = desiredAttitudeFromForce({ x: 0, y: 9.81, z: 0 }, 0);
near(hoverAttitude.x, 0, 1e-12, 'hover quaternion x');
near(hoverAttitude.y, 0, 1e-12, 'hover quaternion y');
near(hoverAttitude.z, 0, 1e-12, 'hover quaternion z');
near(hoverAttitude.w, 1, 1e-12, 'hover quaternion w');

const yaw90 = desiredAttitudeFromForce({ x: 0, y: 9.81, z: 0 }, Math.PI / 2);
const yaw90Forward = rotateVectorByQuaternion(yaw90, { x: 0, y: 0, z: -1 });
near(yaw90Forward.x, -1, 1e-9, '+90 yaw faces west');
near(yaw90Forward.z, 0, 1e-9, '+90 yaw forward has zero north component');

const tiltedForce = { x: 3, y: 9, z: -2 };
const tiltedAttitude = desiredAttitudeFromForce(tiltedForce, 0.7);
const tiltedUp = rotateVectorByQuaternion(tiltedAttitude, { x: 0, y: 1, z: 0 });
const tiltedNorm = vectorNorm(tiltedForce);
near(tiltedUp.x, tiltedForce.x / tiltedNorm, 1e-9, 'body up aligns with force x');
near(tiltedUp.y, tiltedForce.y / tiltedNorm, 1e-9, 'body up aligns with force y');
near(tiltedUp.z, tiltedForce.z / tiltedNorm, 1e-9, 'body up aligns with force z');

// Golden samples from the authoritative get_Q_from_ACC construction in the
// simulator's Y-up/body-+Y convention.
const goldenRight45 = desiredAttitudeFromForce({ x: 9.81, y: 9.81, z: 0 }, 0);
near(goldenRight45.x, 0, 1e-12, 'get_Q golden right45 qx');
near(goldenRight45.y, 0, 1e-12, 'get_Q golden right45 qy');
near(goldenRight45.z, -0.3826834323650898, 1e-12, 'get_Q golden right45 qz');
near(goldenRight45.w, 0.9238795325112867, 1e-12, 'get_Q golden right45 qw');
const goldenForward45 = desiredAttitudeFromForce({ x: 0, y: 9.81, z: -9.81 }, 0);
near(goldenForward45.x, -0.3826834323650898, 1e-12, 'get_Q golden forward45 qx');
near(goldenForward45.y, 0, 1e-12, 'get_Q golden forward45 qy');
near(goldenForward45.z, 0, 1e-12, 'get_Q golden forward45 qz');
near(goldenForward45.w, 0.9238795325112867, 1e-12, 'get_Q golden forward45 qw');

const invalidAttitude = desiredAttitudeFromForce({ x: NaN, y: Infinity, z: 0 }, NaN);
near(invalidAttitude.w, 1, 1e-12, 'invalid desired force falls back to level yaw zero');

// Reduced-quaternion control is zero at target, prioritizes tilt, and limits rates.
const identity = { x: 0, y: 0, z: 0, w: 1 };
const zeroRates = reducedQuaternionBodyRateSetpoint(identity, identity);
near(vectorNorm(zeroRates), 0, 1e-12, 'no attitude error gives zero rate');

const yawRate = reducedQuaternionBodyRateSetpoint(identity, yaw90);
near(yawRate.x, 0, 1e-9, 'pure yaw produces no pitch rate');
near(yawRate.z, 0, 1e-9, 'pure yaw produces no roll rate');
near(yawRate.y, 120 * DEG2RAD, 1e-9, 'pure yaw respects 120 deg/s limit');

const horizontalForceAttitude = desiredAttitudeFromForce({ x: 100, y: 0, z: 0 }, 0);
const tiltRate = reducedQuaternionBodyRateSetpoint(identity, horizontalForceAttitude);
assert(Math.hypot(tiltRate.x, tiltRate.z) > 0, 'thrust-axis error produces tilt rate');
assert(Math.abs(tiltRate.x) <= 220 * DEG2RAD + 1e-12, 'pitch rate is limited');
assert(Math.abs(tiltRate.z) <= 220 * DEG2RAD + 1e-12, 'roll rate is limited');
near(vectorNorm(reducedQuaternionBodyRateSetpoint(
    { x: NaN, y: 0, z: 0, w: 1 }, identity,
)), 0, 0, 'invalid attitude safely produces zero rates');

// Rate servo uses exact first-order response and handles invalid targets safely.
const servo = firstOrderRateServo(
    { x: 0, y: 0, z: 0 },
    { x: 1, y: -2, z: 3 },
    0.1,
    15,
);
const alpha = 1 - Math.exp(-1.5);
near(servo.x, alpha, 1e-12, 'rate servo x exact response');
near(servo.y, -2 * alpha, 1e-12, 'rate servo y exact response');
near(firstOrderRateServo({ x: 1, y: 2, z: 3 }, { x: NaN, y: 0, z: 0 }, 0.1).x,
    1 - alpha, 1e-12, 'invalid target decays safely toward zero');

// Quaternion exponential integrates constant body rates without Euler ordering.
const integratedYaw = integrateBodyRates(identity, { x: 0, y: Math.PI / 2, z: 0 }, 1);
const integratedForward = rotateVectorByQuaternion(integratedYaw, { x: 0, y: 0, z: -1 });
near(integratedForward.x, -1, 1e-9, 'body yaw integration rotates forward west');
near(integratedForward.z, 0, 1e-9, 'body yaw integration reaches exactly 90 degrees');
near(integrateBodyRates(identity, { x: NaN, y: 0, z: 0 }, 1).w, 1, 1e-12,
    'invalid body rate preserves attitude');

// Easy allocator reserves 30% horizontal authority before saturating vertical.
const marginLimited = allocateEasyForce({ x: 100, y: 100, z: 0 }, 10);
near(marginLimited.force.y, Math.sqrt(91), 1e-9, 'vertical limit reserves 3 N horizontal margin');
near(Math.hypot(marginLimited.force.x, marginLimited.force.z), 3, 1e-9,
    'horizontal output receives reserved margin');
near(vectorNorm(marginLimited.force), 10, 1e-9, 'allocated force respects total maximum');
assert(marginLimited.saturatedVertical && marginLimited.saturatedHorizontal,
    'allocator reports both saturated axes');

const verticalOnly = allocateEasyForce({ x: 0, y: 10, z: 0 }, 10);
near(verticalOnly.force.y, Math.sqrt(91), 1e-9,
    'vertical-only saturation still reserves 30% horizontal authority');
near(Math.hypot(verticalOnly.force.x, verticalOnly.force.z), 0, 0,
    'unused horizontal reserve does not create a lateral force');
assert(verticalOnly.saturatedVertical && !verticalOnly.saturatedHorizontal,
    'vertical-only reserve reports only vertical saturation');

const verticalPriority = allocateEasyForce({ x: 8, y: 8, z: 0 }, 10);
near(verticalPriority.force.y, 8, 1e-12, 'feasible vertical request is retained');
near(verticalPriority.force.x, 6, 1e-12, 'horizontal uses remaining thrust after vertical priority');

const allocatorFallback = allocateEasyForce(
    { x: NaN, y: 0, z: 0 },
    10,
    { fallbackForce: { x: 0, y: 5, z: 0 } },
);
near(allocatorFallback.force.y, 5, 1e-12, 'invalid requested force uses deterministic fallback');
finiteVector(allocatorFallback.force, 'allocator fallback is finite');

// Direct force cap must preserve the planner's force direction.
const direct = capDirectForce({ x: 3, y: 4, z: 0 }, 2);
near(direct.force.x, 1.2, 1e-12, 'direct cap x is uniform');
near(direct.force.y, 1.6, 1e-12, 'direct cap y is uniform');
assert(direct.saturated, 'direct cap reports saturation');
const directInvalid = capDirectForce({ x: NaN, y: 4, z: 0 }, 2);
finiteVector(directInvalid.force, 'invalid direct force safely returns finite zero');
near(vectorNorm(directInvalid.force), 0, 0, 'invalid direct force is zero');

console.log(`\nFlight control math: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
