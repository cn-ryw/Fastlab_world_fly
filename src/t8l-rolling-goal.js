export const T8L_GOAL_RADIUS_M = 50;
export const T8L_GOAL_DEADZONE = 0.25;

export function applyT8LGoalDeadzone(value, deadzone = T8L_GOAL_DEADZONE) {
    const normalized = Math.max(-1, Math.min(1, Number(value) || 0));
    if (normalized > deadzone) return (normalized - deadzone) / (1 - deadzone);
    if (normalized < -deadzone) return (normalized + deadzone) / (1 - deadzone);
    return 0;
}

/**
 * Build a body-heading-relative rolling goal in the simulator's XZ plane.
 * yaw=0 faces -Z; positive yaw turns the nose toward -X.
 * Channel inversion belongs to Controller mapping, not this transform.
 */
export function computeT8LRollingGoal(
    position,
    channels,
    yawDeg = 0,
    radius = T8L_GOAL_RADIUS_M,
) {
    if (!position || !Array.isArray(channels) || channels.length < 2) return null;
    const origin = [position.x, position.y, position.z].map(Number);
    const heading = Number(yawDeg);
    if (!origin.every(Number.isFinite) || !Number.isFinite(heading)) return null;
    const roll = applyT8LGoalDeadzone(channels[0]);
    // The transmitter's globally calibrated pitch sign is correct for direct
    // flight modes, while the rolling-goal convention defines stick-forward
    // as a positive body-forward displacement. Convert only at this SO3
    // navigation boundary so FPV/Easy pitch remain unchanged.
    const pitch = -applyT8LGoalDeadzone(channels[1]);
    const yawRad = heading * Math.PI / 180;
    const forwardX = -Math.sin(yawRad);
    const forwardZ = -Math.cos(yawRad);
    const rightX = -Math.cos(yawRad);
    const rightZ = Math.sin(yawRad);
    let dx = (forwardX * pitch + rightX * roll) * radius;
    let dz = (forwardZ * pitch + rightZ * roll) * radius;
    const magnitude = Math.hypot(dx, dz);
    if (magnitude > radius) {
        dx *= radius / magnitude;
        dz *= radius / magnitude;
    }
    return Object.freeze({ x: origin[0] + dx, y: origin[1], z: origin[2] + dz });
}
