/**
 * North-up top-down visualization for a compact 360-degree depth polar scan.
 *
 * Coordinate contracts:
 *   simulation world: +X east, +Z north, +Y up
 *   drone yaw: 0 faces -Z (south), positive yaw turns toward -X (west)
 *   polar angle: 0 is sensor forward, positive points body-left
 *
 * The renderer never decodes the false-colour JPEG and never assigns metres
 * to a relative DA360 scan.
 */

const METRIC_DEPTH_MODES = new Set(['da360-metric', 'cesium-truth']);
const DEPTH_MODES = new Set(['da360-relative', ...METRIC_DEPTH_MODES]);
export const RELATIVE_TEST_CLICK_RADIUS_M = 20;

function finiteNumber(value, fallback = null) {
    const number = Number(value);
    return Number.isFinite(number) ? number : fallback;
}

export function normalizeDepthPolarScan(value) {
    if (!value || typeof value !== 'object' || value.schema_version !== 1) return null;
    if (!Array.isArray(value.values) || value.values.length < 4 || value.values.length > 720) return null;

    const depthMode = String(value.depth_mode || '');
    if (!DEPTH_MODES.has(depthMode)) return null;
    const metric = METRIC_DEPTH_MODES.has(depthMode);
    const expectedUnit = metric ? 'metres' : 'x-near-reference';
    if (value.unit !== expectedUnit) return null;
    if (value.angle_positive !== 'body-left') return null;

    const radius = finiteNumber(value.radius);
    const angleStartDeg = finiteNumber(value.angle_start_deg);
    const angleStepDeg = finiteNumber(value.angle_step_deg);
    if (!(radius > 0) || !(angleStepDeg > 0) || angleStartDeg === null) return null;
    if (Math.abs(angleStepDeg * value.values.length - 360) > 1e-6) return null;

    const values = value.values.map(item => {
        const distance = finiteNumber(item);
        return distance !== null && distance > 0 ? distance : null;
    });
    if (!values.some(item => item !== null)) return null;

    return Object.freeze({
        schemaVersion: 1,
        depthMode,
        metric,
        unit: expectedUnit,
        radius,
        angleStartDeg,
        angleStepDeg,
        anglePositive: value.angle_positive === 'body-left' ? 'body-left' : null,
        pitchBandDeg: Array.isArray(value.pitch_band_deg)
            ? Object.freeze(value.pitch_band_deg.slice(0, 2).map(Number))
            : null,
        distancePercentile: finiteNumber(value.distance_percentile),
        normalization: metric ? null : String(value.normalization || 'unknown'),
        validFraction: finiteNumber(value.valid_fraction, 0),
        values: Object.freeze(values),
    });
}

/** Simulator yaw converted to a conventional compass bearing (N=0, E=90). */
export function simYawToCompassBearingDeg(droneYawDeg) {
    const yaw = finiteNumber(droneYawDeg, 0);
    return ((yaw + 180) % 360 + 360) % 360;
}

/** Convert a body-relative bearing to a world-horizontal unit vector. */
export function bodyAzimuthToWorld(bodyAngleDeg, droneYawDeg) {
    const theta = finiteNumber(bodyAngleDeg, 0) * Math.PI / 180;
    const yaw = finiteNumber(droneYawDeg, 0) * Math.PI / 180;
    const forward = { east: -Math.sin(yaw), north: -Math.cos(yaw) };
    const left = { east: Math.cos(yaw), north: -Math.sin(yaw) };
    return Object.freeze({
        east: Math.cos(theta) * forward.east + Math.sin(theta) * left.east,
        north: Math.cos(theta) * forward.north + Math.sin(theta) * left.north,
    });
}

/** Project all valid scan samples into a north-up canvas. */
export function polarScanCanvasPoints(scan, captureYawDeg, cx, cy, plotRadius, originOffset = null) {
    const normalized = scan?.schemaVersion === 1 ? scan : normalizeDepthPolarScan(scan);
    if (!normalized) return [];
    const originEast = normalized.metric ? finiteNumber(originOffset?.east, 0) : 0;
    const originNorth = normalized.metric ? finiteNumber(originOffset?.north, 0) : 0;
    return normalized.values.map((distance, index) => {
        if (distance === null) return null;
        const angleDeg = normalized.angleStartDeg + index * normalized.angleStepDeg;
        const direction = bodyAzimuthToWorld(angleDeg, captureYawDeg);
        const east = originEast + direction.east * distance;
        const north = originNorth + direction.north * distance;
        const worldDistance = Math.hypot(east, north);
        const clipFactor = worldDistance > normalized.radius
            ? normalized.radius / worldDistance
            : 1;
        return Object.freeze({
            x: cx + east * clipFactor / normalized.radius * plotRadius,
            y: cy - north * clipFactor / normalized.radius * plotRadius,
            distance,
            clipped: distance >= normalized.radius || worldDistance >= normalized.radius,
        });
    });
}

export function depthTopdownLabels(scan) {
    const normalized = scan?.schemaVersion === 1 ? scan : normalizeDepthPolarScan(scan);
    if (!normalized) return Object.freeze({ mode: 'WAITING FOR DEPTH', range: 'R --' });
    return Object.freeze({
        mode: normalized.metric ? 'METRIC DEPTH' : 'RELATIVE · CLICK TEST ONLY',
        range: normalized.metric
            ? `R ${normalized.radius.toFixed(0)}m`
            : `DEPTH ${normalized.radius.toFixed(0)}×p02 · CLICK R${RELATIVE_TEST_CLICK_RADIUS_M}m`,
    });
}

/**
 * Convert one click in the circular plot into a world-horizontal goal offset.
 *
 * Metric scans use their physical radius. Relative scans deliberately use a
 * separate nominal test radius: this enables UI/goal-direction testing without
 * pretending that the normalized depth values are metres.
 */
export function topdownClickToGoalOffset(
    scan,
    offsetX,
    offsetY,
    canvasWidth,
    canvasHeight,
    relativeTestRadiusM = RELATIVE_TEST_CLICK_RADIUS_M,
) {
    const normalized = scan?.schemaVersion === 1 ? scan : normalizeDepthPolarScan(scan);
    const width = finiteNumber(canvasWidth);
    const height = finiteNumber(canvasHeight);
    const x = finiteNumber(offsetX);
    const y = finiteNumber(offsetY);
    if (!normalized || !(width > 22) || !(height > 22) || x === null || y === null) return null;

    const plotRadius = Math.min(width, height) / 2 - 11;
    const dx = x - width / 2;
    const dy = y - height / 2;
    if (Math.hypot(dx, dy) > plotRadius) return null;

    const nominalRadiusM = normalized.metric
        ? normalized.radius
        : finiteNumber(relativeTestRadiusM);
    if (!(nominalRadiusM > 0)) return null;
    return Object.freeze({
        east: dx / plotRadius * nominalRadiusM,
        north: -dy / plotRadius * nominalRadiusM,
        radiusM: nominalRadiusM,
        mapping: normalized.metric ? 'metric' : 'relative-test',
    });
}

function drawCompass(ctx, cx, cy, radius) {
    ctx.save();
    ctx.strokeStyle = 'rgba(125, 211, 252, 0.20)';
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 4]);
    ctx.beginPath();
    ctx.moveTo(cx, cy - radius); ctx.lineTo(cx, cy + radius);
    ctx.moveTo(cx - radius, cy); ctx.lineTo(cx + radius, cy);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = '#e2e8f0';
    ctx.font = '700 10px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = '#fb7185';
    ctx.fillText('N', cx, cy - radius + 8);
    ctx.fillStyle = '#94a3b8';
    ctx.fillText('E', cx + radius - 7, cy);
    ctx.fillText('S', cx, cy + radius - 7);
    ctx.fillText('W', cx - radius + 7, cy);
    ctx.restore();
}

function drawDroneHeading(ctx, cx, cy, yawDeg) {
    const heading = bodyAzimuthToWorld(0, yawDeg);
    const right = { east: heading.north, north: -heading.east };
    const nose = 9;
    const tail = 5;
    ctx.save();
    ctx.fillStyle = '#22c55e';
    ctx.strokeStyle = '#dcfce7';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx + heading.east * nose, cy - heading.north * nose);
    ctx.lineTo(
        cx - heading.east * tail + right.east * 4,
        cy + heading.north * tail - right.north * 4,
    );
    ctx.lineTo(
        cx - heading.east * tail - right.east * 4,
        cy + heading.north * tail + right.north * 4,
    );
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
}

/**
 * Draw the latest scan. ``captureYawDeg`` rotates the captured body-relative
 * scan into north-up world coordinates; ``currentYawDeg`` draws live heading.
 */
export function drawDepthTopdown(ctx, canvas, scan, {
    captureYawDeg = 0,
    currentYawDeg = captureYawDeg,
    originOffset = null,
    goalOffset = null,
} = {}) {
    if (!ctx || !canvas) return false;
    const normalized = scan?.schemaVersion === 1 ? scan : normalizeDepthPolarScan(scan);
    const width = Number(canvas.width || 0);
    const height = Number(canvas.height || 0);
    const cx = width / 2;
    const cy = height / 2;
    const radius = Math.max(1, Math.min(width, height) / 2 - 11);

    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = 'rgba(3, 7, 18, 0.98)';
    ctx.fillRect(0, 0, width, height);

    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.clip();
    ctx.fillStyle = 'rgba(15, 23, 42, 0.96)';
    ctx.fillRect(cx - radius, cy - radius, radius * 2, radius * 2);

    for (const ringFraction of [0.25, 0.5, 0.75, 1]) {
        ctx.beginPath();
        ctx.arc(cx, cy, radius * ringFraction, 0, Math.PI * 2);
        ctx.strokeStyle = ringFraction === 1
            ? 'rgba(125, 211, 252, 0.42)'
            : 'rgba(125, 211, 252, 0.13)';
        ctx.lineWidth = 1;
        ctx.stroke();
    }

    if (normalized) {
        const originEast = normalized.metric ? finiteNumber(originOffset?.east, 0) : 0;
        const originNorth = normalized.metric ? finiteNumber(originOffset?.north, 0) : 0;
        const originDistance = Math.hypot(originEast, originNorth);
        const originClip = originDistance > normalized.radius
            ? normalized.radius / originDistance
            : 1;
        const scanOrigin = {
            x: cx + originEast * originClip / normalized.radius * radius,
            y: cy - originNorth * originClip / normalized.radius * radius,
        };
        const points = polarScanCanvasPoints(
            normalized,
            captureYawDeg,
            cx,
            cy,
            radius,
            originOffset,
        );
        for (let index = 0; index < points.length; index++) {
            const nextIndex = (index + 1) % points.length;
            const point = points[index];
            const next = points[nextIndex];
            if (!point || !next) continue;
            ctx.beginPath();
            ctx.moveTo(scanOrigin.x, scanOrigin.y);
            ctx.lineTo(point.x, point.y);
            ctx.lineTo(next.x, next.y);
            ctx.closePath();
            ctx.fillStyle = normalized.metric
                ? 'rgba(34, 211, 238, 0.18)'
                : 'rgba(167, 139, 250, 0.18)';
            ctx.fill();
            ctx.beginPath();
            ctx.moveTo(point.x, point.y);
            ctx.lineTo(next.x, next.y);
            ctx.strokeStyle = point.clipped && next.clipped
                ? 'rgba(34, 197, 94, 0.45)'
                : 'rgba(251, 113, 133, 0.82)';
            ctx.lineWidth = 1.4;
            ctx.stroke();
        }

        if (goalOffset) {
            const east = finiteNumber(goalOffset.east);
            const north = finiteNumber(goalOffset.north);
            if (east !== null && north !== null) {
                const goalRadiusM = normalized.metric
                    ? normalized.radius
                    : RELATIVE_TEST_CLICK_RADIUS_M;
                const distance = Math.hypot(east, north);
                const factor = distance > goalRadiusM
                    ? goalRadiusM / distance
                    : 1;
                const gx = cx + east * factor / goalRadiusM * radius;
                const gy = cy - north * factor / goalRadiusM * radius;
                ctx.save();
                ctx.strokeStyle = normalized.metric
                    ? 'rgba(250, 204, 21, 0.72)'
                    : 'rgba(251, 146, 60, 0.90)';
                if (!normalized.metric) ctx.setLineDash([3, 3]);
                ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(gx, gy); ctx.stroke();
                ctx.setLineDash([]);
                ctx.fillStyle = normalized.metric ? '#facc15' : '#fb923c';
                ctx.beginPath(); ctx.arc(gx, gy, 3, 0, Math.PI * 2); ctx.fill();
                if (!normalized.metric) {
                    ctx.font = '700 8px monospace';
                    ctx.textAlign = 'center';
                    ctx.fillText('TEST', gx, gy - 7);
                }
                ctx.restore();
            }
        }
    } else {
        ctx.fillStyle = '#64748b';
        ctx.font = '10px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('WAITING FOR 360 DEPTH', cx, cy - 18);
    }
    ctx.restore();

    drawCompass(ctx, cx, cy, radius);
    drawDroneHeading(ctx, cx, cy, currentYawDeg);
    return !!normalized;
}
