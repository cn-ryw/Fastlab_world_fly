/**
 * ERP (Equirectangular Projection) geometry helpers.
 *
 * Pixel ↔ direction mapping MUST match the PanoramaSensor shader:
 *   yaw   = Math.PI - (u + 0.5) / W * 2 * Math.PI
 *   pitch = vfov / 2 - (v + 0.5) / H * vfov
 * where vfov = verticalFovDeg in radians.
 *
 * The body/camera frame is NWU (x=forward, y=left, z=up).
 */

/**
 * Continuous pixel coordinate of the centre of an equally-sized grid cell.
 *
 * ERP helpers use the common image convention in which integer coordinates
 * identify pixel centres.  The shader samples normalised texture coordinates,
 * so a cell centre at normalised coordinate `t` maps to `t * size - 0.5`.
 */
export function erpGridCellCenter(cell, cells, size) {
    if (!Number.isInteger(cell) || !Number.isInteger(cells) || cells <= 0
        || cell < 0 || cell >= cells || !Number.isFinite(size) || size <= 0) {
        throw new RangeError('invalid ERP grid cell');
    }
    return (cell + 0.5) / cells * size - 0.5;
}

/** Return the shader yaw/pitch associated with an ERP pixel centre. */
export function erpPixelToAngles(u, v, W, H, vfovRad = Math.PI) {
    if (![u, v, W, H, vfovRad].every(Number.isFinite) || W <= 0 || H <= 0 || vfovRad <= 0) {
        throw new RangeError('invalid ERP geometry');
    }
    return {
        yaw: Math.PI - (u + 0.5) / W * 2.0 * Math.PI,
        pitch: vfovRad / 2.0 - (v + 0.5) / H * vfovRad,
    };
}

/**
 * ERP pixel (u, v) → unit direction in the sensor/body frame (NWU).
 *
 * @param {number} u       – horizontal pixel coordinate (0..W-1)
 * @param {number} v       – vertical pixel coordinate (0..H-1)
 * @param {number} W       – image width
 * @param {number} H       – image height
 * @param {number} vfovRad – vertical field of view in radians (default Math.PI = 180°)
 * @returns {{dx: number, dy: number, dz: number}} unit-length direction vector
 */
export function erpPixelToDirection(u, v, W, H, vfovRad = Math.PI) {
    const { yaw, pitch } = erpPixelToAngles(u, v, W, H, vfovRad);
    const cosPitch = Math.cos(pitch);
    // dy negated to mirror left/right: matches YOPO training ERP layout
    return {
        dx: cosPitch * Math.cos(yaw),
        dy: -cosPitch * Math.sin(yaw),
        dz: Math.sin(pitch),
    };
}

/**
 * Convert the canonical sensor/body NWU direction to the component axes used
 * by the panorama cubemap renderer: +X right, +Y up, +Z back.
 *
 * Keep this mapping numerically identical to the projector shader's
 * `directionFromPitchYaw`: `vec3(-right, up, -forward)`.
 */
export function erpDirectionToComponent(direction) {
    if (!direction || ![direction.dx, direction.dy, direction.dz].every(Number.isFinite)) {
        throw new TypeError('invalid ERP direction');
    }
    return normalize3({
        x: direction.dy,
        y: direction.dz,
        z: -direction.dx,
    });
}

/** ERP pixel directly to the panorama renderer's component direction. */
export function erpPixelToComponentDirection(u, v, W, H, vfovRad = Math.PI) {
    return erpDirectionToComponent(erpPixelToDirection(u, v, W, H, vfovRad));
}

/**
 * Unit direction → nearest ERP pixel (inverse of erpPixelToDirection).
 *
 * Returns {u, v} in continuous pixel coordinates (not snapped to integer).
 */
export function erpDirectionToPixel(dx, dy, dz, W, H, vfovRad = Math.PI) {
    // negate dy to match mirrored erpPixelToDirection
    const yaw = Math.atan2(-dy, dx);               // (-π, π]
    const pitch = Math.asin(Math.max(-1, Math.min(1, dz)));
    const u = (Math.PI - yaw) / (2.0 * Math.PI) * W - 0.5;
    const v = (vfovRad / 2.0 - pitch) / vfovRad * H - 0.5;
    return { u, v };
}

/**
 * Sample directions at grid anchor centres.
 *
 * @param {number} cols     – horizontal grid cells
 * @param {number} rows     – vertical grid cells
 * @param {number} W        – ERP image width (for geometry, not direction)
 * @param {number} H        – ERP image height
 * @param {number} vfovRad  – vertical FOV in radians
 * @returns {Array<{dx: number, dy: number, dz: number, u: number, v: number, col: number, row: number}>}
 */
export function sampleAnchorDirections(cols, rows, W, H, vfovRad = Math.PI) {
    const anchors = [];
    for (let row = 0; row < rows; row++) {
        for (let col = 0; col < cols; col++) {
            const u = erpGridCellCenter(col, cols, W);
            const v = erpGridCellCenter(row, rows, H);
            const { yaw, pitch } = erpPixelToAngles(u, v, W, H, vfovRad);
            const dir = erpPixelToDirection(u, v, W, H, vfovRad);
            anchors.push({ ...dir, u, v, col, row, yaw, pitch });
        }
    }
    return anchors;
}

/**
 * Return a normalised copy of a 3D vector.
 */
export function normalize3(v) {
    const len = Math.hypot(v.x, v.y, v.z);
    if (len < 1e-12) return { x: 1, y: 0, z: 0 };
    return { x: v.x / len, y: v.y / len, z: v.z / len };
}
