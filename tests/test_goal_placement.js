/** Final three-dimensional goal placement validation contract. */
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

globalThis.window = { location: { search: '' } };

const { CesiumWorld } = await import('../src/cesium-world.js?goal-placement-test');

function makeWorld(surfaceY) {
    const samples = [];
    const world = Object.create(CesiumWorld.prototype);
    world.sampleHeightAtLocal = (x, z, width) => {
        samples.push({ x, z, width });
        return typeof surfaceY === 'function' ? surfaceY(samples.length) : surfaceY;
    };
    return { world, samples };
}

function validate(surfaceY, goal, options) {
    const fixture = makeWorld(surfaceY);
    return {
        ...fixture,
        result: fixture.world.validateGoalPlacement(goal, options),
    };
}

{
    const { result, samples } = validate(40, { x: 12, y: 41, z: -7 });
    assert.equal(result.valid, true, 'a goal with clearance above a building roof is valid');
    assert.equal(result.surfaceY, 40);
    assert.deepEqual(samples, [{ x: 12, z: -7, width: 0.6 }],
        'validation samples only the final goal horizontal position once');
}

for (const [goalY, label] of [
    [25, 'inside a building'],
    [40, 'on the building surface'],
    [40.8, 'at the required clearance boundary'],
]) {
    const { result, samples } = validate(40, { x: 3, y: goalY, z: 9 });
    assert.equal(result.valid, false, `a goal ${label} is rejected`);
    assert.equal(result.reason, 'goal-inside-surface');
    assert.equal(result.surfaceY, 40);
    assert.equal(result.requiredClearance, 0.8);
    assert.equal(samples.length, 1, `${label} uses one surface sample`);
}

{
    const { result } = validate(0, { x: -5, y: 10, z: 2 });
    assert.equal(result.valid, true, 'a goal safely above the ground is valid');
}

{
    const { result, samples } = validate(10, { x: 1, y: 11.5, z: 2 }, {
        collisionRadius: 1.3,
    });
    assert.equal(result.valid, false, 'custom collision radius participates in clearance');
    assert.equal(result.requiredClearance, 1.5);
    assert.deepEqual(samples, [{ x: 1, z: 2, width: 1.3 }]);
}

for (const surfaceY of [null, undefined, NaN, Infinity]) {
    const { result, samples } = validate(surfaceY, { x: 1, y: 20, z: 2 });
    assert.equal(result.valid, false, `unresolved surface ${String(surfaceY)} fails closed`);
    assert.equal(result.reason, 'surface-unresolved');
    assert.equal(samples.length, 1);
}

for (const goal of [
    null,
    { x: NaN, y: 10, z: 0 },
    { x: 0, y: Infinity, z: 0 },
    { x: 0, y: 10, z: undefined },
    { x: null, y: 10, z: 0 },
    { x: '0', y: 10, z: 0 },
]) {
    const { result, samples } = validate(0, goal);
    assert.equal(result.valid, false, 'a non-finite goal fails closed');
    assert.equal(result.reason, 'invalid-position');
    assert.equal(samples.length, 0, 'an invalid goal is rejected before surface sampling');
}

{
    const { result, samples } = validate(
        callNumber => {
            if (callNumber > 1) throw new Error('unexpected neighborhood sample');
            return 25;
        },
        { x: 100, y: 26, z: -100 },
    );
    assert.equal(result.valid, true, 'an elevated surface can provide horizontal coordinates');
    assert.equal(samples.length, 1, 'building classification never performs neighborhood sampling');
}

// Main-scene and depth-minimap clicks must share one final-goal submission
// path so neither input can bypass validation before starting navigation.
const main = await readFile(new URL('../src/main.js', import.meta.url), 'utf8');
const helperStart = main.indexOf('function tryBeginFixedGoal(');
const setupStart = main.indexOf('function setupFlightGoalClickHandler()');
const setupEnd = main.indexOf('function beginNavigationSession(', setupStart);
assert.ok(helperStart >= 0 && setupStart > helperStart && setupEnd > setupStart,
    'fixed-goal helper and click-handler boundaries are present');

const helper = main.slice(helperStart, setupStart);
assert.match(helper, /world\.validateGoalPlacement\(goal,/,
    'shared helper validates the composed three-dimensional goal');
assert.match(helper, /if \(!placement\.valid\)[\s\S]*?return false;/,
    'shared helper stops rejected goals');
assert.match(helper, /beginNavigationSession\(goal\);[\s\S]*?return true;/,
    'shared helper starts navigation only after validation');
assert.equal((main.match(/validateGoalPlacement\(/g) || []).length, 1,
    'main has one centralized goal-placement validation call');

const setup = main.slice(setupStart, setupEnd);
assert.equal((setup.match(/tryBeginFixedGoal\(/g) || []).length, 2,
    'scene and minimap handlers both use the shared helper');
const sceneHandler = setup.slice(
    setup.indexOf('const onMouseDown ='),
    setup.indexOf('// Track G key state'),
);
assert.match(sceneHandler, /scene\.pickPosition\(clickPos\)/,
    'scene handler still picks visible roofs, facades, and terrain');
assert.match(sceneHandler, /tryBeginFixedGoal\(\{ x: local\.x, z: local\.z \}\)/);
assert.doesNotMatch(sceneHandler, /beginNavigationSession\(/,
    'scene handler cannot bypass final-goal validation');

const radarHandler = setup.slice(
    setup.indexOf('const onRadarMouseDown ='),
    setup.indexOf('if (radarCanvas)'),
);
assert.match(radarHandler, /topdownClickToGoalOffset\(/);
assert.match(radarHandler, /tryBeginFixedGoal\(/);
assert.doesNotMatch(radarHandler, /beginNavigationSession\(/,
    'minimap handler cannot bypass final-goal validation');
assert.doesNotMatch(radarHandler, /_goalGdown|placementKeysDown\.has\('KeyG'\)/,
    'minimap left-click remains independent of the G key');

console.log('goal placement tests passed');
