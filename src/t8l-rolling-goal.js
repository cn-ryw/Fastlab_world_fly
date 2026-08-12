export const T8L_GOAL_RADIUS_M = 8;
export const T8L_GOAL_DEADZONE = 0.25;

export function applyT8LGoalDeadzone(value, deadzone = T8L_GOAL_DEADZONE) {
    const normalized = Math.max(-1, Math.min(1, Number(value) || 0));
    if (normalized > deadzone) return (normalized - deadzone) / (1 - deadzone);
    if (normalized < -deadzone) return (normalized + deadzone) / (1 - deadzone);
    return 0;
}

/** ROS start_rc_goal.sh semantics adapted from ROS XY-up to Cesium XZ-horizontal. */
export function computeT8LRollingGoal(position, channels, radius = T8L_GOAL_RADIUS_M) {
    if (!position || !Array.isArray(channels) || channels.length < 2) return null;
    const origin = [position.x, position.y, position.z].map(Number);
    if (!origin.every(Number.isFinite)) return null;
    // CH1 roll and CH2 pitch are both inverted; swap_xy maps pitch to world X
    // and roll to the other horizontal world axis (Cesium Z).
    const roll = -applyT8LGoalDeadzone(channels[0]);
    const pitch = -applyT8LGoalDeadzone(channels[1]);
    let dx = pitch * radius;
    let dz = roll * radius;
    const magnitude = Math.hypot(dx, dz);
    if (magnitude > radius) {
        dx *= radius / magnitude;
        dz *= radius / magnitude;
    }
    return Object.freeze({ x: origin[0] + dx, y: origin[1], z: origin[2] + dz });
}
