/*
 * Copyright 2026 Manifold Tech Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Pure flight-control math for the browser's ideal rigid-body plant.
 *
 * Coordinate contract:
 *   world: +X east, +Y up, +Z north
 *   body:  +X right, +Y thrust/up, +Z backward (forward is -Z)
 *
 * Vectors and quaternions are plain objects. A quaternion is a body-to-world
 * rotation `{ x, y, z, w }`. Angular-rate inputs and outputs are rad/s.
 * No function in this module depends on PlayCanvas, the DOM, or mutable global
 * state, which keeps the controller deterministic and directly unit-testable.
 */

const EPSILON = 1e-9;
const DEG2RAD = Math.PI / 180;

export const ControlCommandType = Object.freeze({
    BODY_RATE_THRUST: 'BODY_RATE_THRUST',
    ATTITUDE_THRUST: 'ATTITUDE_THRUST',
    POSITION_VELOCITY_HOLD: 'POSITION_VELOCITY_HOLD',
    DIRECT_ACCELERATION: 'DIRECT_ACCELERATION',
    FAILSAFE_HOLD: 'FAILSAFE_HOLD',
});

export const DEFAULT_ATTITUDE_CONTROL = Object.freeze({
    gains: Object.freeze({ roll: 4, pitch: 4, yaw: 2.8 }),
    yawWeight: 0.4,
    rateLimitsDeg: Object.freeze({ roll: 220, pitch: 220, yaw: 120 }),
    rateBandwidth: 15,
});

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function finiteOr(value, fallback) {
    return Number.isFinite(value) ? value : fallback;
}

export function isFiniteVector(vector) {
    return !!vector
        && Number.isFinite(vector.x)
        && Number.isFinite(vector.y)
        && Number.isFinite(vector.z);
}

function zeroVector() {
    return { x: 0, y: 0, z: 0 };
}

function sanitizeVector(vector, fallback = zeroVector()) {
    if (isFiniteVector(vector)) {
        return { x: vector.x, y: vector.y, z: vector.z };
    }
    return isFiniteVector(fallback)
        ? { x: fallback.x, y: fallback.y, z: fallback.z }
        : zeroVector();
}

/**
 * Continuously maps an assisted-mode axis from the calibrated raw range into
 * [-1, 1]. The deadzone edge maps exactly to zero, avoiding the old minimum
 * velocity step. `expo=0` is linear and `expo=1` is cubic.
 */
export function shapeAssistedAxis(raw, deadzone = 0.05, expo = 0) {
    if (!Number.isFinite(raw)) return 0;

    const dz = clamp(finiteOr(deadzone, 0.05), 0, 1 - 1e-6);
    const expoWeight = clamp(finiteOr(expo, 0), 0, 1);
    const bounded = clamp(raw, -1, 1);
    const magnitude = Math.abs(bounded);
    if (magnitude <= dz) return 0;

    const linear = (magnitude - dz) / (1 - dz);
    const shaped = linear * (1 - expoWeight) + linear * linear * linear * expoWeight;
    return Math.sign(bounded) * shaped;
}

/**
 * PX4 Stabilized-style manual throttle around hover:
 *   [-1, 0] -> [0, hover], [0, 1] -> [hover, max].
 * A non-finite stick is neutral. Invalid thrust bounds collapse safely to 0.
 */
export function piecewiseHoverThrottle(input, hoverGf, maxGf) {
    const maximum = Math.max(0, finiteOr(maxGf, 0));
    const hover = clamp(finiteOr(hoverGf, 0), 0, maximum);
    const stick = clamp(finiteOr(input, 0), -1, 1);

    if (stick <= 0) return hover * (stick + 1);
    return hover + stick * (maximum - hover);
}

/** Returns a finite Euclidean norm. Invalid vectors have the safe norm 0. */
export function vectorNorm(vector) {
    if (!isFiniteVector(vector)) return 0;
    return Math.hypot(vector.x, vector.y, vector.z);
}

export const norm3 = vectorNorm;

/**
 * Uniformly limits a vector magnitude without changing its direction.
 * Invalid vectors or limits return zero.
 */
export function limitVector(vector, maxNorm) {
    if (!isFiniteVector(vector) || !Number.isFinite(maxNorm) || maxNorm <= 0) {
        return zeroVector();
    }

    const magnitude = vectorNorm(vector);
    if (magnitude <= maxNorm) return { x: vector.x, y: vector.y, z: vector.z };
    if (magnitude <= EPSILON) return zeroVector();

    const scale = maxNorm / magnitude;
    return { x: vector.x * scale, y: vector.y * scale, z: vector.z * scale };
}

export const limitVectorNorm = limitVector;

export function isFiniteQuaternion(quaternion) {
    return !!quaternion
        && Number.isFinite(quaternion.x)
        && Number.isFinite(quaternion.y)
        && Number.isFinite(quaternion.z)
        && Number.isFinite(quaternion.w);
}

export function normalizeQuaternion(quaternion) {
    if (!isFiniteQuaternion(quaternion)) return { x: 0, y: 0, z: 0, w: 1 };
    const magnitude = Math.hypot(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    if (magnitude <= EPSILON) return { x: 0, y: 0, z: 0, w: 1 };

    const inverse = 1 / magnitude;
    let result = {
        x: quaternion.x * inverse,
        y: quaternion.y * inverse,
        z: quaternion.z * inverse,
        w: quaternion.w * inverse,
    };

    // Canonicalization makes antipodal inputs produce one deterministic value.
    if (result.w < 0) {
        result = { x: -result.x, y: -result.y, z: -result.z, w: -result.w };
    }
    return result;
}

export function multiplyQuaternions(left, right) {
    const a = normalizeQuaternion(left);
    const b = normalizeQuaternion(right);
    return normalizeQuaternion({
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    });
}

export function inverseQuaternion(quaternion) {
    const q = normalizeQuaternion(quaternion);
    return { x: -q.x, y: -q.y, z: -q.z, w: q.w };
}

export function rotateVectorByQuaternion(quaternion, vector) {
    if (!isFiniteQuaternion(quaternion) || !isFiniteVector(vector)) return zeroVector();
    const q = normalizeQuaternion(quaternion);

    // q * [v, 0] * q^-1, expanded to avoid allocating intermediate quaternions.
    const tx = 2 * (q.y * vector.z - q.z * vector.y);
    const ty = 2 * (q.z * vector.x - q.x * vector.z);
    const tz = 2 * (q.x * vector.y - q.y * vector.x);
    return {
        x: vector.x + q.w * tx + (q.y * tz - q.z * ty),
        y: vector.y + q.w * ty + (q.z * tx - q.x * tz),
        z: vector.z + q.w * tz + (q.x * ty - q.y * tx),
    };
}

function quaternionFromMatrix(
    m00, m01, m02,
    m10, m11, m12,
    m20, m21, m22,
) {
    let x;
    let y;
    let z;
    let w;
    const trace = m00 + m11 + m22;

    if (trace > 0) {
        const scale = Math.sqrt(trace + 1) * 2;
        w = 0.25 * scale;
        x = (m21 - m12) / scale;
        y = (m02 - m20) / scale;
        z = (m10 - m01) / scale;
    } else if (m00 > m11 && m00 > m22) {
        const scale = Math.sqrt(Math.max(0, 1 + m00 - m11 - m22)) * 2;
        w = (m21 - m12) / scale;
        x = 0.25 * scale;
        y = (m01 + m10) / scale;
        z = (m02 + m20) / scale;
    } else if (m11 > m22) {
        const scale = Math.sqrt(Math.max(0, 1 + m11 - m00 - m22)) * 2;
        w = (m02 - m20) / scale;
        x = (m01 + m10) / scale;
        y = 0.25 * scale;
        z = (m12 + m21) / scale;
    } else {
        const scale = Math.sqrt(Math.max(0, 1 + m22 - m00 - m11)) * 2;
        w = (m10 - m01) / scale;
        x = (m02 + m20) / scale;
        y = (m12 + m21) / scale;
        z = 0.25 * scale;
    }

    return normalizeQuaternion({ x, y, z, w });
}

/**
 * Builds the desired body-to-world attitude from world force and heading.
 * The returned quaternion aligns body +Y with the force while keeping body -Z
 * as close as possible to the requested horizontal forward direction.
 */
export function desiredAttitudeFromForce(force, yawRad = 0) {
    const forceVector = sanitizeVector(force);
    const magnitude = vectorNorm(forceVector);
    const yaw = finiteOr(yawRad, 0);

    if (magnitude <= EPSILON) {
        return normalizeQuaternion({ x: 0, y: Math.sin(yaw * 0.5), z: 0, w: Math.cos(yaw * 0.5) });
    }

    // Desired thrust/up axis b2.
    const b2x = forceVector.x / magnitude;
    const b2y = forceVector.y / magnitude;
    const b2z = forceVector.z / magnitude;

    // yaw=0 faces world -Z. Project that forward vector onto b2's plane.
    const forwardX = -Math.sin(yaw);
    const forwardZ = -Math.cos(yaw);
    const forwardDotUp = forwardX * b2x + forwardZ * b2z;
    let b3x = -(forwardX - forwardDotUp * b2x);
    let b3y = forwardDotUp * b2y;
    let b3z = -(forwardZ - forwardDotUp * b2z);
    let backwardLength = Math.hypot(b3x, b3y, b3z);

    if (backwardLength <= EPSILON) {
        // Force is parallel to the heading. Choose the least-parallel cardinal
        // axis and project it to obtain a deterministic, finite body backward.
        const candidates = [
            { x: 1, y: 0, z: 0 },
            { x: 0, y: 0, z: 1 },
            { x: 0, y: 1, z: 0 },
        ];
        let reference = candidates[0];
        let smallestDot = Infinity;
        for (const candidate of candidates) {
            const dot = Math.abs(candidate.x * b2x + candidate.y * b2y + candidate.z * b2z);
            if (dot < smallestDot) {
                smallestDot = dot;
                reference = candidate;
            }
        }
        const dot = reference.x * b2x + reference.y * b2y + reference.z * b2z;
        b3x = reference.x - dot * b2x;
        b3y = reference.y - dot * b2y;
        b3z = reference.z - dot * b2z;
        backwardLength = Math.hypot(b3x, b3y, b3z);
    }

    b3x /= backwardLength;
    b3y /= backwardLength;
    b3z /= backwardLength;

    // right = up x backward. Re-orthogonalize backward to suppress drift.
    let b1x = b2y * b3z - b2z * b3y;
    let b1y = b2z * b3x - b2x * b3z;
    let b1z = b2x * b3y - b2y * b3x;
    const rightLength = Math.hypot(b1x, b1y, b1z);
    b1x /= rightLength;
    b1y /= rightLength;
    b1z /= rightLength;

    b3x = b1y * b2z - b1z * b2y;
    b3y = b1z * b2x - b1x * b2z;
    b3z = b1x * b2y - b1y * b2x;

    // Rotation matrix rows for columns [right | thrust/up | backward].
    return quaternionFromMatrix(
        b1x, b2x, b3x,
        b1y, b2y, b3y,
        b1z, b2z, b3z,
    );
}

export const desiredForceToQuaternion = desiredAttitudeFromForce;

function quaternionFromTwoVectors(from, to) {
    const a = sanitizeVector(from);
    const b = sanitizeVector(to);
    const aNorm = vectorNorm(a);
    const bNorm = vectorNorm(b);
    if (aNorm <= EPSILON || bNorm <= EPSILON) return { x: 0, y: 0, z: 0, w: 1 };

    const ax = a.x / aNorm;
    const ay = a.y / aNorm;
    const az = a.z / aNorm;
    const bx = b.x / bNorm;
    const by = b.y / bNorm;
    const bz = b.z / bNorm;
    const dot = clamp(ax * bx + ay * by + az * bz, -1, 1);

    if (dot > 1 - 1e-10) return { x: 0, y: 0, z: 0, w: 1 };
    if (dot < -1 + 1e-6) return null;

    return normalizeQuaternion({
        x: ay * bz - az * by,
        y: az * bx - ax * bz,
        z: ax * by - ay * bx,
        w: 1 + dot,
    });
}

function canonicalQuaternion(quaternion) {
    const q = normalizeQuaternion(quaternion);
    return q.w < 0 ? { x: -q.x, y: -q.y, z: -q.z, w: -q.w } : q;
}

/**
 * PX4-style reduced-quaternion attitude P controller adapted to body +Y thrust.
 * Tilt is aligned first; the residual heading error is blended by yawWeight.
 * Returned body rates use `{x:pitch, y:yaw, z:roll}` in rad/s.
 */
export function reducedQuaternionBodyRateSetpoint(current, desired, options = {}) {
    if (!isFiniteQuaternion(current) || !isFiniteQuaternion(desired)) return zeroVector();
    const currentNorm = Math.hypot(current.x, current.y, current.z, current.w);
    const desiredNorm = Math.hypot(desired.x, desired.y, desired.z, desired.w);
    if (currentNorm <= EPSILON || desiredNorm <= EPSILON) return zeroVector();

    const q = normalizeQuaternion(current);
    const qDesired = normalizeQuaternion(desired);
    const currentUp = rotateVectorByQuaternion(q, { x: 0, y: 1, z: 0 });
    const desiredUp = rotateVectorByQuaternion(qDesired, { x: 0, y: 1, z: 0 });
    const align = quaternionFromTwoVectors(currentUp, desiredUp);

    // Opposite thrust directions are the reduced-attitude singularity. As PX4
    // does, use the full desired attitude in this infinitesimal corner case.
    let reducedDesired = align ? multiplyQuaternions(align, q) : qDesired;

    const yawWeight = clamp(finiteOr(options.yawWeight, DEFAULT_ATTITUDE_CONTROL.yawWeight), 0, 1);
    if (align && yawWeight > 0) {
        const residual = canonicalQuaternion(multiplyQuaternions(inverseQuaternion(reducedDesired), qDesired));
        const yawError = 2 * Math.atan2(residual.y, residual.w);
        const halfWeightedYaw = yawError * yawWeight * 0.5;
        const weightedYaw = {
            x: 0,
            y: Math.sin(halfWeightedYaw),
            z: 0,
            w: Math.cos(halfWeightedYaw),
        };
        reducedDesired = multiplyQuaternions(reducedDesired, weightedYaw);
    }

    const error = canonicalQuaternion(multiplyQuaternions(inverseQuaternion(q), reducedDesired));
    const gains = options.gains || DEFAULT_ATTITUDE_CONTROL.gains;
    const rollGain = Math.max(0, finiteOr(gains.roll, DEFAULT_ATTITUDE_CONTROL.gains.roll));
    const pitchGain = Math.max(0, finiteOr(gains.pitch, DEFAULT_ATTITUDE_CONTROL.gains.pitch));
    const yawGain = Math.max(0, finiteOr(gains.yaw, DEFAULT_ATTITUDE_CONTROL.gains.yaw));

    // Match PX4's yaw-weight compensation: yawWeight changes large-error path
    // prioritization without weakening the configured small-error yaw gain.
    const compensatedYawGain = yawWeight > 1e-4 ? yawGain / yawWeight : 0;
    let bodyRates = {
        x: 2 * error.x * pitchGain,
        y: 2 * error.y * compensatedYawGain,
        z: 2 * error.z * rollGain,
    };

    const limitsDeg = options.rateLimitsDeg || DEFAULT_ATTITUDE_CONTROL.rateLimitsDeg;
    const limitsRad = options.rateLimitsRad || {
        roll: Math.max(0, finiteOr(limitsDeg.roll, DEFAULT_ATTITUDE_CONTROL.rateLimitsDeg.roll)) * DEG2RAD,
        pitch: Math.max(0, finiteOr(limitsDeg.pitch, DEFAULT_ATTITUDE_CONTROL.rateLimitsDeg.pitch)) * DEG2RAD,
        yaw: Math.max(0, finiteOr(limitsDeg.yaw, DEFAULT_ATTITUDE_CONTROL.rateLimitsDeg.yaw)) * DEG2RAD,
    };
    const pitchLimit = Math.max(0, finiteOr(limitsRad.pitch, DEFAULT_ATTITUDE_CONTROL.rateLimitsDeg.pitch * DEG2RAD));
    const yawLimit = Math.max(0, finiteOr(limitsRad.yaw, DEFAULT_ATTITUDE_CONTROL.rateLimitsDeg.yaw * DEG2RAD));
    const rollLimit = Math.max(0, finiteOr(limitsRad.roll, DEFAULT_ATTITUDE_CONTROL.rateLimitsDeg.roll * DEG2RAD));
    bodyRates = {
        x: clamp(finiteOr(bodyRates.x, 0), -pitchLimit, pitchLimit),
        y: clamp(finiteOr(bodyRates.y, 0), -yawLimit, yawLimit),
        z: clamp(finiteOr(bodyRates.z, 0), -rollLimit, rollLimit),
    };
    return bodyRates;
}

/** Exact zero-order-hold first-order body-rate servo. */
export function firstOrderRateServo(current, target, dt, bandwidth = DEFAULT_ATTITUDE_CONTROL.rateBandwidth) {
    const currentRate = sanitizeVector(current);
    const targetRate = sanitizeVector(target);
    if (!Number.isFinite(dt) || dt <= 0) return currentRate;

    const frequency = Math.max(0, finiteOr(bandwidth, DEFAULT_ATTITUDE_CONTROL.rateBandwidth));
    const alpha = 1 - Math.exp(-frequency * dt);
    return {
        x: currentRate.x + (targetRate.x - currentRate.x) * alpha,
        y: currentRate.y + (targetRate.y - currentRate.y) * alpha,
        z: currentRate.z + (targetRate.z - currentRate.z) * alpha,
    };
}

/**
 * Integrates a constant body-frame rate exactly over dt using the quaternion
 * exponential: q_next = q * exp(omega_body * dt / 2).
 */
export function integrateBodyRates(orientation, bodyRates, dt) {
    const q = normalizeQuaternion(orientation);
    if (!isFiniteQuaternion(orientation)
        || Math.hypot(orientation.x, orientation.y, orientation.z, orientation.w) <= EPSILON
        || !isFiniteVector(bodyRates)
        || !Number.isFinite(dt)
        || dt <= 0) {
        return q;
    }

    const rateMagnitude = vectorNorm(bodyRates);
    if (rateMagnitude <= EPSILON) return q;
    const halfAngle = rateMagnitude * dt * 0.5;
    const scale = Math.sin(halfAngle) / rateMagnitude;
    const delta = {
        x: bodyRates.x * scale,
        y: bodyRates.y * scale,
        z: bodyRates.z * scale,
        w: Math.cos(halfAngle),
    };
    return multiplyQuaternions(q, delta);
}

/**
 * PX4-style force allocation for Easy/hold control, adapted to Y-up.
 * Vertical force is prioritized while always reserving
 * `horizontalMarginFraction * maxThrustN` for horizontal authority.
 */
export function allocateEasyForce(requestedForce, maxThrustN, options = {}) {
    const fallback = sanitizeVector(options.fallbackForce);
    const maximum = finiteOr(maxThrustN, 0);
    if (!isFiniteVector(requestedForce) || maximum <= 0) {
        const fallbackAllocation = maximum > 0
            ? allocateEasyForce(fallback, maximum, { ...options, fallbackForce: zeroVector() })
            : null;
        return {
            force: fallbackAllocation ? fallbackAllocation.force : zeroVector(),
            requested: isFiniteVector(requestedForce) ? { ...requestedForce } : fallback,
            saturatedVertical: true,
            saturatedHorizontal: true,
            horizontalMarginN: 0,
        };
    }

    const marginFraction = clamp(finiteOr(options.horizontalMarginFraction, 0.3), 0, 1);
    const horizontalMarginN = marginFraction * maximum;
    const minimumVertical = clamp(finiteOr(options.minVerticalN, 0), 0, maximum);
    const horizontalNorm = Math.hypot(requestedForce.x, requestedForce.z);
    const verticalMaximum = Math.sqrt(Math.max(
        0,
        maximum * maximum - horizontalMarginN * horizontalMarginN,
    ));
    const vertical = clamp(requestedForce.y, minimumVertical, verticalMaximum);

    const horizontalMaximum = Math.sqrt(Math.max(0, maximum * maximum - vertical * vertical));
    const horizontalScale = horizontalNorm > horizontalMaximum && horizontalNorm > EPSILON
        ? horizontalMaximum / horizontalNorm
        : 1;
    const force = {
        x: requestedForce.x * horizontalScale,
        y: vertical,
        z: requestedForce.z * horizontalScale,
    };

    return {
        force,
        requested: { x: requestedForce.x, y: requestedForce.y, z: requestedForce.z },
        saturatedVertical: Math.abs(vertical - requestedForce.y) > 1e-9,
        saturatedHorizontal: horizontalScale < 1 - 1e-9,
        horizontalMarginN,
    };
}

/** Uniform cap for active YOPO direct force; preserves the planner direction. */
export function capDirectForce(requestedForce, maxThrustN) {
    const requested = sanitizeVector(requestedForce);
    const force = limitVector(requestedForce, maxThrustN);
    return {
        force,
        requested,
        saturated: !isFiniteVector(requestedForce)
            || vectorNorm(requested) > Math.max(0, finiteOr(maxThrustN, 0)) + 1e-9,
    };
}
