import assert from 'node:assert/strict';
import { applyT8LGoalDeadzone, computeT8LRollingGoal } from '../src/t8l-rolling-goal.js';

assert.equal(applyT8LGoalDeadzone(0.24), 0);
assert.equal(applyT8LGoalDeadzone(-0.25), 0);
assert.equal(applyT8LGoalDeadzone(1), 1);
assert.equal(applyT8LGoalDeadzone(-1), -1);

const origin = { x: 10, y: 123, z: -4 };
assert.deepEqual(computeT8LRollingGoal(origin, [0, 0]), origin, 'center stick follows current position');
assert.deepEqual(computeT8LRollingGoal(origin, [0, -1]), { x: 10, y: 123, z: -54 },
    'transmitter stick-forward pitch follows the nose at yaw zero');
assert.deepEqual(computeT8LRollingGoal(origin, [1, 0]), { x: -40, y: 123, z: -4 },
    'positive roll follows body right at yaw zero');
const westFacingForward = computeT8LRollingGoal(origin, [0, -1], 90);
assert.ok(Math.abs(westFacingForward.x + 40) < 1e-12);
assert.ok(Math.abs(westFacingForward.z + 4) < 1e-12);
const diagonal = computeT8LRollingGoal(origin, [1, -1]);
assert.ok(Math.abs(Math.hypot(diagonal.x - origin.x, diagonal.z - origin.z) - 50) < 1e-12);
assert.equal(diagonal.y, origin.y, 'rolling target preserves current altitude');

console.log('T8L rolling goal: all tests passed');
