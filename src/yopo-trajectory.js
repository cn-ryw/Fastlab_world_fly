/*
 * Copyright 2026 Manifold Tech Ltd.
 * Licensed under the Apache License, Version 2.0.
 */

/**
 * YOPO axis-major Poly5 trajectory contract and lifecycle.
 *
 * This module deliberately has no DOM, PlayCanvas, or Drone dependency. It is
 * the boundary between asynchronous planning results and the deterministic
 * 200 Hz controller clock.
 */

export const YOPO_DEFAULT_TRAJ_TIME_S = 1.125;
export const YOPO_MIN_TRAJ_TIME_S = 0.05;
export const YOPO_MAX_TRAJ_TIME_S = 5.0;
export const YOPO_MAX_ENDPOINT_DISTANCE_M = 120.0;
export const YOPO_MAX_ENDPOINT_SPEED_MPS = 50.0;
export const YOPO_MAX_ENDPOINT_ACCEL_MPS2 = 80.0;

// All root finding is performed in normalized trajectory time u=t/T.  This
// keeps the degree-seven stationary-point polynomial well conditioned even at
// the 50 ms minimum trajectory duration.
const POLYNOMIAL_TRIM_EPSILON = 1e-13;
const ROOT_VALUE_EPSILON = 2e-11;
const ROOT_POSITION_EPSILON = 2e-12;
const LIMIT_EPSILON = 1e-8;
const TRAJECTORY_TIME_EPSILON_S = 1e-12;

export class Poly5Solver {
    constructor(pos0, vel0, acc0, pos1, vel1, acc1, duration) {
        const t = duration;
        const inverse = [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0.5, 0, 0, 0],
            [-10/t**3, -6/t**2, -1.5/t, 10/t**3, -4/t**2, 0.5/t],
            [15/t**4, 8/t**3, 1.5/t**2, -15/t**4, 7/t**3, -1/t**2],
            [-6/t**5, -3/t**4, -0.5/t**3, 6/t**5, -3/t**4, 0.5/t**3],
        ];
        const boundary = [pos0, vel0, acc0, pos1, vel1, acc1];
        this.A = inverse.map(row => row.reduce(
            (sum, coefficient, index) => sum + coefficient * boundary[index],
            0,
        ));
    }

    position(t) {
        return this.A[0] + this.A[1]*t + this.A[2]*t*t + this.A[3]*t**3
            + this.A[4]*t**4 + this.A[5]*t**5;
    }

    velocity(t) {
        return this.A[1] + 2*this.A[2]*t + 3*this.A[3]*t*t
            + 4*this.A[4]*t**3 + 5*this.A[5]*t**4;
    }

    acceleration(t) {
        return 2*this.A[2] + 6*this.A[3]*t + 12*this.A[4]*t*t + 20*this.A[5]*t**3;
    }
}

function finiteState(state = {}) {
    const number = (value, fallback = 0) => Number.isFinite(Number(value)) ? Number(value) : fallback;
    return {
        x: number(state.x), y: number(state.y), z: number(state.z),
        vx: number(state.vx), vy: number(state.vy), vz: number(state.vz),
        ax: number(state.ax), ay: number(state.ay), az: number(state.az),
    };
}

function sample(polynomials, t) {
    return {
        x: polynomials.x.position(t),
        y: polynomials.y.position(t),
        z: polynomials.z.position(t),
        vx: polynomials.x.velocity(t),
        vy: polynomials.y.velocity(t),
        vz: polynomials.z.velocity(t),
        ax: polynomials.x.acceleration(t),
        ay: polynomials.y.acceleration(t),
        az: polynomials.z.acceleration(t),
    };
}

function polynomialDerivative(coefficients) {
    if (coefficients.length <= 1) return [0];
    return coefficients.slice(1).map((coefficient, degree) => coefficient * (degree + 1));
}

function polynomialProduct(lhs, rhs) {
    const result = Array(lhs.length + rhs.length - 1).fill(0);
    for (let i = 0; i < lhs.length; i++) {
        for (let j = 0; j < rhs.length; j++) result[i + j] += lhs[i] * rhs[j];
    }
    return result;
}

function polynomialSum(polynomials) {
    const size = Math.max(...polynomials.map(polynomial => polynomial.length));
    const result = Array(size).fill(0);
    for (const polynomial of polynomials) {
        for (let i = 0; i < polynomial.length; i++) result[i] += polynomial[i];
    }
    return result;
}

function normalizedPolynomial(coefficients) {
    if (!coefficients.every(Number.isFinite)) return null;
    const scale = Math.max(...coefficients.map(Math.abs), 0);
    if (scale === 0) return [0];
    const result = coefficients.map(coefficient => coefficient / scale);
    while (result.length > 1 && Math.abs(result[result.length - 1]) <= POLYNOMIAL_TRIM_EPSILON) {
        result.pop();
    }
    return result;
}

function evaluatePolynomial(coefficients, u) {
    let value = 0;
    for (let i = coefficients.length - 1; i >= 0; i--) value = value * u + coefficients[i];
    return value;
}

function appendDistinctRoot(roots, root) {
    const clamped = Math.max(0, Math.min(1, root));
    if (!roots.some(existing => Math.abs(existing - clamped) <= ROOT_POSITION_EPSILON)) {
        roots.push(clamped);
    }
}

function bisectPolynomialRoot(coefficients, lower, upper, lowerValue, upperValue) {
    let lo = lower;
    let hi = upper;
    let flo = lowerValue;
    let fhi = upperValue;
    for (let iteration = 0; iteration < 80; iteration++) {
        const middle = (lo + hi) * 0.5;
        const fm = evaluatePolynomial(coefficients, middle);
        if (Math.abs(fm) <= ROOT_VALUE_EPSILON || hi - lo <= ROOT_POSITION_EPSILON) {
            return middle;
        }
        if (Math.sign(flo) === Math.sign(fm)) {
            lo = middle;
            flo = fm;
        } else {
            hi = middle;
            fhi = fm;
        }
    }
    // Referencing fhi here also makes an accidental non-bracketing caller
    // visible to static analysers without changing the deterministic midpoint.
    return Number.isFinite(fhi) ? (lo + hi) * 0.5 : NaN;
}

/**
 * Isolate every distinct real root of a low-degree polynomial on [0, 1].
 *
 * The roots of p' partition the interval into regions where p is monotonic.
 * Recursing on p' therefore leaves at most one sign-changing root per open
 * region.  Evaluating p at the partition points also retains even-multiplicity
 * roots, which a sign-only scan would miss.  Degrees here are bounded by seven.
 */
function polynomialRootsOnUnitInterval(rawCoefficients) {
    const coefficients = normalizedPolynomial(rawCoefficients);
    if (!coefficients) return null;
    const degree = coefficients.length - 1;
    if (degree <= 0) return [];
    if (degree === 1) {
        const root = -coefficients[0] / coefficients[1];
        return Number.isFinite(root) && root >= -ROOT_POSITION_EPSILON && root <= 1 + ROOT_POSITION_EPSILON
            ? [Math.max(0, Math.min(1, root))]
            : [];
    }

    const criticalPoints = polynomialRootsOnUnitInterval(polynomialDerivative(coefficients));
    if (!criticalPoints) return null;
    const partitions = [0, ...criticalPoints, 1]
        .sort((lhs, rhs) => lhs - rhs)
        .filter((value, index, values) => index === 0
            || Math.abs(value - values[index - 1]) > ROOT_POSITION_EPSILON);
    const roots = [];

    for (const point of partitions) {
        const value = evaluatePolynomial(coefficients, point);
        if (!Number.isFinite(value)) return null;
        if (Math.abs(value) <= ROOT_VALUE_EPSILON) appendDistinctRoot(roots, point);
    }

    for (let i = 0; i + 1 < partitions.length; i++) {
        const lower = partitions[i];
        const upper = partitions[i + 1];
        const lowerValue = evaluatePolynomial(coefficients, lower);
        const upperValue = evaluatePolynomial(coefficients, upper);
        if (lowerValue === 0 || upperValue === 0 || Math.sign(lowerValue) === Math.sign(upperValue)) continue;
        const root = bisectPolynomialRoot(coefficients, lower, upper, lowerValue, upperValue);
        if (!Number.isFinite(root)) return null;
        appendDistinctRoot(roots, root);
    }
    return roots.sort((lhs, rhs) => lhs - rhs);
}

function axisDerivativeInNormalizedTime(polynomial, duration, derivativeOrder) {
    const result = [];
    for (let degree = derivativeOrder; degree < polynomial.A.length; degree++) {
        let multiplier = 1;
        for (let order = 0; order < derivativeOrder; order++) multiplier *= degree - order;
        result.push(polynomial.A[degree] * multiplier * duration ** (degree - derivativeOrder));
    }
    return result;
}

function squaredNormPolynomial(polynomials, duration, derivativeOrder) {
    const components = ['x', 'y', 'z'].map(axis =>
        axisDerivativeInNormalizedTime(polynomials[axis], duration, derivativeOrder));
    return polynomialSum(components.map(component => polynomialProduct(component, component)));
}

function stationaryTimesForSquaredNorm(polynomials, duration, derivativeOrder) {
    const squaredNorm = squaredNormPolynomial(polynomials, duration, derivativeOrder);
    return polynomialRootsOnUnitInterval(polynomialDerivative(squaredNorm));
}

function currentStateHasNonFiniteValue(state) {
    if (!state || typeof state !== 'object') return false;
    return ['x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az'].some(key =>
        state[key] !== undefined && !Number.isFinite(Number(state[key])));
}

function exceedsLimit(value, limit) {
    return value > limit + LIMIT_EPSILON * Math.max(1, limit);
}

export function buildYopoPolynomials(values, duration, startState = {}) {
    const start = finiteState(startState);
    return {
        x: new Poly5Solver(start.x, start.vx, start.ax, values[0], values[1], values[2], duration),
        y: new Poly5Solver(start.y, start.vy, start.ay, values[3], values[4], values[5], duration),
        z: new Poly5Solver(start.z, start.vz, start.az, values[6], values[7], values[8], duration),
    };
}

/**
 * Validate the public axis-major endpoint and the fitted trajectory interval.
 * Speed and acceleration extrema are evaluated at both interval endpoints and
 * at every real stationary point of their squared vector norms.  Unlike a
 * sampled scan, this covers the complete continuous quintic interval.
 */
export function validateYopoTrajectory(endpoint, trajTime, currentState = {}) {
    if (!Array.isArray(endpoint) && !ArrayBuffer.isView(endpoint)) {
        return { valid: false, reason: 'endstate must be an array' };
    }
    if (endpoint.length !== 9) {
        return { valid: false, reason: `endstate must contain 9 values, got ${endpoint.length}` };
    }
    const values = Array.from(endpoint, Number);
    if (!values.every(Number.isFinite)) {
        return { valid: false, reason: 'endstate contains a non-finite value' };
    }
    if (currentStateHasNonFiniteValue(currentState)) {
        return { valid: false, reason: 'current state contains a non-finite value' };
    }

    const duration = trajTime == null ? YOPO_DEFAULT_TRAJ_TIME_S : Number(trajTime);
    if (!Number.isFinite(duration) || duration < YOPO_MIN_TRAJ_TIME_S || duration > YOPO_MAX_TRAJ_TIME_S) {
        return {
            valid: false,
            reason: `trajTime ${trajTime} is outside ${YOPO_MIN_TRAJ_TIME_S}-${YOPO_MAX_TRAJ_TIME_S}s`,
        };
    }

    const start = finiteState(currentState);
    const positions = [values[0], values[3], values[6]];
    const endpointVelocity = [values[1], values[4], values[7]];
    const endpointAcceleration = [values[2], values[5], values[8]];
    const displacement = Math.hypot(
        positions[0] - start.x,
        positions[1] - start.y,
        positions[2] - start.z,
    );
    if (exceedsLimit(displacement, YOPO_MAX_ENDPOINT_DISTANCE_M)) {
        return {
            valid: false,
            reason: `endpoint displacement ${displacement.toFixed(2)}m exceeds ${YOPO_MAX_ENDPOINT_DISTANCE_M}m`,
        };
    }
    if (exceedsLimit(Math.hypot(...endpointVelocity), YOPO_MAX_ENDPOINT_SPEED_MPS)) {
        return { valid: false, reason: `endpoint speed exceeds ${YOPO_MAX_ENDPOINT_SPEED_MPS}m/s` };
    }
    if (exceedsLimit(Math.hypot(...endpointAcceleration), YOPO_MAX_ENDPOINT_ACCEL_MPS2)) {
        return {
            valid: false,
            reason: `endpoint acceleration exceeds ${YOPO_MAX_ENDPOINT_ACCEL_MPS2}m/s²`,
        };
    }

    const polynomials = buildYopoPolynomials(values, duration, start);
    const speedStationaryTimes = stationaryTimesForSquaredNorm(polynomials, duration, 1);
    const accelerationStationaryTimes = stationaryTimesForSquaredNorm(polynomials, duration, 2);
    if (!speedStationaryTimes || !accelerationStationaryTimes) {
        return { valid: false, reason: 'fitted trajectory extrema could not be resolved' };
    }

    let maxSpeed = 0;
    let maxAcceleration = 0;
    for (const normalizedTime of [0, 1, ...speedStationaryTimes]) {
        const reference = sample(polynomials, duration * normalizedTime);
        const speed = Math.hypot(reference.vx, reference.vy, reference.vz);
        if (!Number.isFinite(speed)) {
            return { valid: false, reason: 'fitted trajectory contains a non-finite value' };
        }
        maxSpeed = Math.max(maxSpeed, speed);
    }
    for (const normalizedTime of [0, 1, ...accelerationStationaryTimes]) {
        const reference = sample(polynomials, duration * normalizedTime);
        const acceleration = Math.hypot(reference.ax, reference.ay, reference.az);
        if (!Number.isFinite(acceleration)) {
            return { valid: false, reason: 'fitted trajectory contains a non-finite value' };
        }
        maxAcceleration = Math.max(maxAcceleration, acceleration);
    }
    if (exceedsLimit(maxSpeed, YOPO_MAX_ENDPOINT_SPEED_MPS)) {
        return {
            valid: false,
            reason: `trajectory speed peak ${maxSpeed.toFixed(2)}m/s exceeds ${YOPO_MAX_ENDPOINT_SPEED_MPS}m/s`,
        };
    }
    if (exceedsLimit(maxAcceleration, YOPO_MAX_ENDPOINT_ACCEL_MPS2)) {
        return {
            valid: false,
            reason: `trajectory acceleration peak ${maxAcceleration.toFixed(2)}m/s² exceeds ${YOPO_MAX_ENDPOINT_ACCEL_MPS2}m/s²`,
        };
    }

    return { valid: true, values, trajTime: duration, polynomials, maxSpeed, maxAcceleration };
}

export class YopoTrajectoryTracker {
    constructor() {
        this.clear();
    }

    clear(reason = 'cleared') {
        this.polynomials = null;
        this.duration = 0;
        this.time = 0;
        this.context = null;
        this.lastReference = null;
        this.lastReason = reason;
    }

    get active() {
        return !!this.polynomials;
    }

    install(endpoint, trajTime, startState, context = null) {
        const checked = validateYopoTrajectory(endpoint, trajTime, startState);
        if (!checked.valid) return checked;
        this.polynomials = checked.polynomials;
        this.duration = checked.trajTime;
        this.time = 0;
        this.context = context && typeof context === 'object'
            ? Object.freeze({ ...context })
            : null;
        this.lastReference = sample(this.polynomials, 0);
        this.lastReason = 'installed';
        return checked;
    }

    referenceAt(time) {
        if (!this.polynomials) return null;
        const clampedTime = Math.max(0, Math.min(Number(time) || 0, this.duration));
        return sample(this.polynomials, clampedTime);
    }

    advance(dt) {
        if (!this.polynomials) return { active: false, expired: false, reference: null };
        const nextTime = this.time + Math.max(0, Number(dt) || 0);
        if (nextTime >= this.duration - TRAJECTORY_TIME_EPSILON_S) {
            const reference = this.referenceAt(this.duration);
            this.lastReference = reference;
            this.clear('expired');
            this.lastReference = reference;
            return { active: false, expired: true, reference };
        }
        this.time = nextTime;
        const reference = this.referenceAt(this.time);
        this.lastReference = reference;
        return { active: true, expired: false, reference };
    }
}
