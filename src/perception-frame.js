/**
 * One immutable sensor/control snapshot.  Keeping pose and RGB under the same
 * frame ID prevents a slow panorama capture from being paired with a newer
 * vehicle state while the drone continues to move.
 */

function finiteNumber(value, label) {
    const number = Number(value);
    if (!Number.isFinite(number)) throw new TypeError(`${label} must be finite`);
    return number;
}

function freezeVector(vector, label) {
    if (!vector || typeof vector !== 'object') throw new TypeError(`${label} is required`);
    return Object.freeze({
        x: finiteNumber(vector.x, `${label}.x`),
        y: finiteNumber(vector.y, `${label}.y`),
        z: finiteNumber(vector.z, `${label}.z`),
    });
}

function freezeTransform(transform) {
    if (!transform || typeof transform !== 'object') throw new TypeError('transform is required');
    const frozen = { position: freezeVector(transform.position, 'transform.position') };
    if (transform.rotation) frozen.rotation = freezeVector(transform.rotation, 'transform.rotation');
    if (transform.orientation) {
        frozen.orientation = Object.freeze({
            x: finiteNumber(transform.orientation.x, 'transform.orientation.x'),
            y: finiteNumber(transform.orientation.y, 'transform.orientation.y'),
            z: finiteNumber(transform.orientation.z, 'transform.orientation.z'),
            w: finiteNumber(transform.orientation.w, 'transform.orientation.w'),
        });
    }
    return Object.freeze(frozen);
}

function freezeState(state, label, requireAcceleration = false) {
    if (!state || typeof state !== 'object') throw new TypeError(`${label} is required`);
    const frozen = {
        position: freezeVector(state.position, `${label}.position`),
        velocity: freezeVector(state.velocity, `${label}.velocity`),
    };
    if (state.acceleration || requireAcceleration) {
        frozen.acceleration = freezeVector(
            state.acceleration || { x: 0, y: 0, z: 0 },
            `${label}.acceleration`,
        );
    }
    return Object.freeze(frozen);
}

export class PerceptionFrame {
    constructor({
        frameId,
        capturedAt,
        transform,
        rgb,
        actualState,
        referenceState,
        yaw,
        captureProfile = 'flight',
        projectionConfig = {},
    }) {
        const id = Number(frameId);
        if (!Number.isSafeInteger(id) || id < 0) throw new TypeError('frameId must be a non-negative integer');
        if (!rgb) throw new TypeError('rgb is required');

        this.frameId = id;
        this.capturedAt = finiteNumber(capturedAt, 'capturedAt');
        this.transform = freezeTransform(transform);
        this.rgb = rgb;
        this.actualState = freezeState(actualState, 'actualState');
        this.referenceState = freezeState(referenceState, 'referenceState', true);
        this.yaw = finiteNumber(yaw, 'yaw');
        if (captureProfile !== 'flight' && captureProfile !== 'calibration') {
            throw new TypeError('captureProfile must be "flight" or "calibration"');
        }
        this.captureProfile = captureProfile;
        this.projectionConfig = Object.freeze({ ...projectionConfig });
        Object.freeze(this);
    }

    /** Fields consumed by /yopo/plan_full. */
    planningObservation(goal) {
        const frozenGoal = freezeVector(goal, 'goal');
        return Object.freeze({
            actualPosition: this.actualState.position,
            referencePosition: this.referenceState.position,
            velocity: this.actualState.velocity,
            acceleration: this.referenceState.acceleration,
            goal: frozenGoal,
            yaw: this.yaw,
        });
    }
}

export function normalizePlanningState(state, yawFallback = 0) {
    if (state?.actualState?.position) {
        return Object.freeze({
            actualState: freezeState(state.actualState, 'actualState'),
            referenceState: freezeState(state.referenceState || state.actualState, 'referenceState', true),
            yaw: finiteNumber(state.yaw ?? yawFallback, 'yaw'),
        });
    }

    const position = { x: state?.x, y: state?.y, z: state?.z };
    const velocity = { x: state?.vx || 0, y: state?.vy || 0, z: state?.vz || 0 };
    const actualState = { position, velocity };
    return Object.freeze({
        actualState: freezeState(actualState, 'actualState'),
        referenceState: freezeState({ ...actualState, acceleration: { x: 0, y: 0, z: 0 } }, 'referenceState', true),
        yaw: finiteNumber(yawFallback || 0, 'yaw'),
    });
}
