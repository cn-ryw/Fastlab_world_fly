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
    const yaw = Math.PI - (u + 0.5) / W * 2.0 * Math.PI;
    const pitch = vfovRad / 2.0 - (v + 0.5) / H * vfovRad;
    const cosPitch = Math.cos(pitch);
    return {
        dx: cosPitch * Math.cos(yaw),
        dy: cosPitch * Math.sin(yaw),
        dz: Math.sin(pitch),
    };
}

/**
 * Unit direction → nearest ERP pixel (inverse of erpPixelToDirection).
 *
 * Returns {u, v} in continuous pixel coordinates (not snapped to integer).
 */
export function erpDirectionToPixel(dx, dy, dz, W, H, vfovRad = Math.PI) {
    const yaw = Math.atan2(dy, dx);               // (-π, π]
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
            const u = (col + 0.5) / cols * W;
            const v = (row + 0.5) / rows * H;
            const dir = erpPixelToDirection(u, v, W, H, vfovRad);
            anchors.push({ ...dir, u, v, col, row });
        }
    }
    return anchors;
}

/**
 * Normalise a 3D vector in-place (mutates).
 */
export function normalize3(v) {
    const len = Math.hypot(v.x, v.y, v.z);
    if (len < 1e-12) return { x: 1, y: 0, z: 0 };
    return { x: v.x / len, y: v.y / len, z: v.z / len };
}
