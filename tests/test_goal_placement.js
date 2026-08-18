/** Final three-dimensional goal placement validation contract. */
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

globalThis.window = { location: { search: '' } };

const { CesiumWorld } = await import('../src/cesium-world.js?goal-placement-test');

function makeWorld(surfaceY, clearanceProbe = null) {
    const samples = [];
    const clearanceSamples = [];
    const world = Object.create(CesiumWorld.prototype);
    world.sampleHeightAtLocal = (x, z, width) => {
        samples.push({ x, z, width });
        return typeof surfaceY === 'function' ? surfaceY(samples.length) : surfaceY;
    };
    world.probeGoalClearance = (goal, maxDistance) => {
        clearanceSamples.push({ goal: { ...goal }, maxDistance });
        return typeof clearanceProbe === 'function'
            ? clearanceProbe(goal, maxDistance, clearanceSamples.length)
            : { completed: true, hit: null, probeCount: 10 };
    };
    return { world, samples, clearanceSamples };
}

function validate(surfaceY, goal, options, clearanceProbe) {
    const fixture = makeWorld(surfaceY, clearanceProbe);
    return {
        ...fixture,
        result: fixture.world.validateGoalPlacement(goal, options),
    };
}

{
    const { result, samples, clearanceSamples } = validate(40, { x: 12, y: 41, z: -7 });
    assert.equal(result.valid, true, 'a goal with clearance above a building roof is valid');
    assert.equal(result.surfaceY, 40);
    assert.equal(result.requiredClearance, 0.8);
    assert.equal(result.clearanceProbeCount, 10);
    assert.deepEqual(samples, [{ x: 12, z: -7, width: 0.6 }],
        'validation samples the final goal horizontal position once');
    assert.deepEqual(clearanceSamples, [{
        goal: { x: 12, y: 41, z: -7 },
        maxDistance: 0.8,
    }], 'validation also probes the final three-dimensional neighborhood');
}

for (const [goalY, label] of [
    [25, 'inside a building'],
    [40, 'on the building surface'],
    [40.8, 'at the required clearance boundary'],
]) {
    const { result, samples, clearanceSamples } = validate(40, { x: 3, y: goalY, z: 9 });
    assert.equal(result.valid, false, `a goal ${label} is rejected`);
    assert.equal(result.reason, 'goal-inside-surface');
    assert.equal(result.surfaceY, 40);
    assert.equal(result.requiredClearance, 0.8);
    assert.equal(samples.length, 1, `${label} uses one surface sample`);
    assert.equal(clearanceSamples.length, 0, `${label} is rejected before ray probing`);
}

{
    const { result, clearanceSamples } = validate(0, { x: -5, y: 10, z: 2 });
    assert.equal(result.valid, true, 'a goal safely above the ground is valid');
    assert.equal(clearanceSamples.length, 1);
}

{
    const { result } = validate(
        0,
        { x: 4, y: 10, z: 8 },
        undefined,
        () => ({
            completed: true,
            hit: {
                distance: 0.45,
                position: { x: 4.45, y: 10, z: 8 },
            },
            probeCount: 1,
        }),
    );
    assert.equal(result.valid, false, 'a goal whose sphere overlaps a nearby wall is rejected');
    assert.equal(result.reason, 'goal-clearance-obstructed');
    assert.equal(result.obstacleDistance, 0.45);
}

{
    const { result } = validate(
        0,
        { x: 4, y: 10, z: 8 },
        undefined,
        () => ({ completed: false, hit: null, probeCount: 1 }),
    );
    assert.equal(result.valid, false, 'an unavailable neighborhood query fails closed');
    assert.equal(result.reason, 'clearance-query-failed');
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
    const { result, samples, clearanceSamples } = validate(surfaceY, { x: 1, y: 20, z: 2 });
    assert.equal(result.valid, false, `unresolved surface ${String(surfaceY)} fails closed`);
    assert.equal(result.reason, 'surface-unresolved');
    assert.equal(samples.length, 1);
    assert.equal(clearanceSamples.length, 0);
}

for (const goal of [
    null,
    { x: NaN, y: 10, z: 0 },
    { x: 0, y: Infinity, z: 0 },
    { x: 0, y: 10, z: undefined },
    { x: null, y: 10, z: 0 },
    { x: '0', y: 10, z: 0 },
]) {
    const { result, samples, clearanceSamples } = validate(0, goal);
    assert.equal(result.valid, false, 'a non-finite goal fails closed');
    assert.equal(result.reason, 'invalid-position');
    assert.equal(samples.length, 0, 'an invalid goal is rejected before surface sampling');
    assert.equal(clearanceSamples.length, 0);
}

{
    const { result, samples, clearanceSamples } = validate(
        callNumber => {
            if (callNumber > 1) throw new Error('unexpected height sample');
            return 25;
        },
        { x: 100, y: 26, z: -100 },
    );
    assert.equal(result.valid, true, 'an elevated surface can provide horizontal coordinates');
    assert.equal(samples.length, 1, 'building classification performs one height sample');
    assert.equal(clearanceSamples.length, 1, 'building classification still checks nearby geometry');
}

{
    const rays = [];
    const world = Object.create(CesiumWorld.prototype);
    world.pickLocalRay = (origin, direction, maxDistance, queryStatus) => {
        queryStatus.completed = true;
        rays.push({ origin: { ...origin }, direction: { ...direction }, maxDistance });
        if (direction.x === 1 && direction.y === 0 && direction.z === 0) {
            return {
                distance: 0.4,
                position: { x: origin.x + 0.4, y: origin.y, z: origin.z },
            };
        }
        return null;
    };
    const probe = world.probeGoalClearance({ x: 1, y: 2, z: 3 }, 0.8);
    assert.equal(probe.completed, true);
    assert.equal(probe.probeCount, 10, 'goal clearance checks cardinal, diagonal, up and down rays');
    assert.equal(rays.length, 10);
    assert.equal(probe.hit.distance, 0.4);
    assert.ok(rays.some(({ direction }) => direction.y === 1));
    assert.ok(rays.some(({ direction }) => direction.y === -1));
    assert.ok(rays.some(({ direction }) => direction.x !== 0 && direction.z !== 0));
}

{
    const world = Object.create(CesiumWorld.prototype);
    world.pickLocalRay = (_origin, _direction, _maxDistance, queryStatus) => {
        queryStatus.completed = false;
        return null;
    };
    const probe = world.probeGoalClearance({ x: 1, y: 2, z: 3 }, 0.8);
    assert.equal(probe.completed, false, 'a failed ray query stops the neighborhood check');
    assert.equal(probe.probeCount, 1);
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
