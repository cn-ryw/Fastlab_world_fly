/** Flight-path sampling, reset, visibility, and Clear-button regression contract. */
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

globalThis.window = { location: { search: '' } };

const { CesiumWorld } = await import('../src/cesium-world.js?flight-path-history-test');

let entityAddCount = 0;
let renderRequestCount = 0;
class CallbackProperty {
    constructor(callback) {
        this.getValue = callback;
    }
}
class PolylineGlowMaterialProperty {
    constructor(options) {
        this.options = options;
    }
}

const world = new CesiumWorld('test-container', { token: 'test-token' });
world.Cesium = {
    CallbackProperty,
    PolylineGlowMaterialProperty,
    Color: {
        CYAN: { withAlpha: (alpha) => ({ color: 'cyan', alpha }) },
    },
    ArcType: { NONE: 'none' },
};
world.viewer = {
    entities: {
        add(definition) {
            entityAddCount++;
            return { ...definition };
        },
    },
    scene: {
        requestRender() {
            renderRequestCount++;
        },
    },
};
world.ready = true;
world.localToCartesian = (point) => ({ ...point });

// Sampling begins immediately, ignores sub-threshold motion, and accepts 0.5 m.
world.updateFlightPath({ x: 0, y: 10, z: 0 });
const pathEntity = world._flightPathEntity;
assert.equal(world._flightPathPositions.length, 1);
assert.equal(entityAddCount, 1);
world.updateFlightPath({ x: 0.49, y: 10, z: 0 });
assert.equal(world._flightPathPositions.length, 1);
world.updateFlightPath({ x: 0.5, y: 10, z: 0 });
assert.equal(world._flightPathPositions.length, 2);

// Clearing replaces all old samples with the current position without
// rebuilding the Cesium entity or changing the user's visibility setting.
world.setFlightPathVisible(false);
world.resetFlightPath({ x: 20, y: 30, z: 40 });
assert.deepEqual(world._flightPathPositions, [{ x: 20, y: 30, z: 40 }]);
assert.deepEqual(world._flightPathLastLocal, { x: 20, y: 30, z: 40 });
assert.equal(world._flightPathEntity, pathEntity);
assert.equal(entityAddCount, 1);
assert.equal(world._flightPathVisible, false);
assert.equal(pathEntity.show, false);
assert.deepEqual(pathEntity.polyline.positions.getValue(), [{ x: 20, y: 30, z: 40 }]);

world.updateFlightPath({ x: 20.49, y: 30, z: 40 });
assert.equal(world._flightPathPositions.length, 1);
world.updateFlightPath({ x: 20.5, y: 30, z: 40 });
assert.deepEqual(world._flightPathPositions, [
    { x: 20, y: 30, z: 40 },
    { x: 20.5, y: 30, z: 40 },
]);

// Invalid and repeated clears are safe, leave the entity hidden, and do not
// recreate it. A later valid clear starts a fresh history again.
world.resetFlightPath({ x: Number.NaN, y: 1, z: 2 });
assert.deepEqual(world._flightPathPositions, []);
assert.equal(world._flightPathLastLocal, null);
world.resetFlightPath(null);
world.resetFlightPath(null);
assert.deepEqual(world._flightPathPositions, []);
assert.equal(world._flightPathEntity, pathEntity);
assert.equal(entityAddCount, 1);
assert.equal(pathEntity.show, false);
world.resetFlightPath({ x: -1, y: 2, z: -3 });
assert.deepEqual(world._flightPathPositions, [{ x: -1, y: 2, z: -3 }]);
assert.ok(renderRequestCount > 0);

// Static UI contract: the settings action is a normal button, is bound once,
// passes only a fully finite drone position, and otherwise clears with null.
const html = await readFile(new URL('../index.html', import.meta.url), 'utf8');
const main = await readFile(new URL('../src/main.js', import.meta.url), 'utf8');
assert.match(
    html,
    /<button\s+type="button"\s+class="assign-btn"\s+id="clear-flight-path-btn"[\s\S]*?aria-label="Clear flight path history and restart from the current position">Clear<\/button>/,
);
assert.equal((html.match(/main\.js\?v=20260817-final-goal-banner-a1/g) || []).length, 2);
assert.match(main, /getElementById\('clear-flight-path-btn'\)/);
assert.match(main, /!clearFlightPathBtn\._flightPathClearBound/);
assert.match(main, /clearFlightPathBtn\.addEventListener\('click'/);
assert.match(
    main,
    /\[drone\.x, drone\.y, drone\.z\]\.every\(Number\.isFinite\)[\s\S]{0,180}?world\?\.resetFlightPath\?\.\(currentPosition\)/,
);
const clearBindingStart = main.indexOf("const clearFlightPathBtn = document.getElementById('clear-flight-path-btn')");
const clearBindingEnd = main.indexOf('// Drone model scale slider', clearBindingStart);
assert.ok(clearBindingStart >= 0 && clearBindingEnd > clearBindingStart);
const clearBinding = main.slice(clearBindingStart, clearBindingEnd);
assert.doesNotMatch(
    clearBinding,
    /applyDisplaySettings|finishNavigationSession|flightLogger|controller|clean-mode-toggle|flight-path-toggle|confirm\s*\(/,
);

console.log('flight path history tests passed');
