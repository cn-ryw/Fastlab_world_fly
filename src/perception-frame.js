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

function finiteCount(value, label, fallback) {
    const number = value == null ? fallback : Number(value);
    if (!Number.isSafeInteger(number) || number < 0) {
        throw new TypeError(`${label} must be a non-negative integer`);
    }
    return number;
}

function freezeFaceTileReadiness(readiness) {
    if (readiness == null) return Object.freeze([]);
    if (!Array.isArray(readiness)) throw new TypeError('faceTileReadiness must be an array');
    return Object.freeze(readiness.map((entry, index) => {
        if (!entry || typeof entry !== 'object') {
            throw new TypeError(`faceTileReadiness[${index}] must be an object`);
        }
        return Object.freeze({
            face: String(entry.face ?? index),
            readyWhenCopied: entry.readyWhenCopied === true,
        });
    }));
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
        rgbFrameComplete = true,
        rgbTilesReady = true,
        rgbReadyFaces = null,
        rgbTotalFaces = 6,
        rgbReadinessReason = null,
        rgbTileError = false,
        faceTileReadiness = [],
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
        this.rgbFrameComplete = rgbFrameComplete === true;
        this.rgbTilesReady = rgbTilesReady === true;
        this.rgbTotalFaces = finiteCount(rgbTotalFaces, 'rgbTotalFaces', 6);
        this.faceTileReadiness = freezeFaceTileReadiness(faceTileReadiness);
        const inferredReadyFaces = this.faceTileReadiness.reduce(
            (count, entry) => count + (entry.readyWhenCopied ? 1 : 0),
            0,
        );
        this.rgbReadyFaces = finiteCount(
            rgbReadyFaces,
            'rgbReadyFaces',
            this.faceTileReadiness.length > 0
                ? inferredReadyFaces
                : this.rgbTilesReady
                ? this.rgbTotalFaces
                : 0,
        );
        if (this.rgbReadyFaces > this.rgbTotalFaces) {
            throw new TypeError('rgbReadyFaces cannot exceed rgbTotalFaces');
        }
        this.rgbTileError = rgbTileError === true;
        this.rgbReadinessReason = String(
            rgbReadinessReason
            || (this.rgbTilesReady
                ? 'tiles-ready'
                : this.rgbTileError
                ? 'tile-error'
                : this.rgbFrameComplete
                ? 'tiles-partial'
                : 'capture-incomplete'),
        );
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
