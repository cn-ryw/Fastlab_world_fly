/* Cesium Ion token persistence and precedence regression contract. */
import assert from 'node:assert/strict';

const {
    CESIUM_ION_TOKEN_STORAGE_KEY,
    resolveCesiumIonToken,
    storeCesiumIonToken,
    storedCesiumIonToken,
} = await import('../src/cesium-token.js?contract-test');

const values = new Map();
const storage = {
    getItem(key) { return values.get(key) ?? null; },
    setItem(key, value) { values.set(key, value); },
};

assert.equal(storedCesiumIonToken(storage), '');
assert.equal(storeCesiumIonToken('  test-token  ', storage), 'test-token');
assert.equal(values.get(CESIUM_ION_TOKEN_STORAGE_KEY), 'test-token');
assert.equal(storedCesiumIonToken(storage), 'test-token');

assert.equal(resolveCesiumIonToken({
    explicitToken: ' explicit ',
    storage,
}), 'explicit');
assert.equal(resolveCesiumIonToken({ storage }), 'test-token');

const emptyStorage = { getItem() { return null; } };
const defaultValues = new Map();
const defaultStorage = {
    getItem(key) { return defaultValues.get(key) ?? null; },
    setItem(key, value) { defaultValues.set(key, value); },
};
const bundledDefault = resolveCesiumIonToken({ storage: defaultStorage });
assert.ok(bundledDefault.length > 80);
assert.equal(defaultValues.get(CESIUM_ION_TOKEN_STORAGE_KEY), bundledDefault);
assert.equal(resolveCesiumIonToken({ storage: emptyStorage }), bundledDefault);
assert.equal(storedCesiumIonToken({ getItem() { throw new Error('blocked'); } }), '');
assert.throws(() => storeCesiumIonToken('   ', storage), /non-empty/);
assert.throws(
    () => storeCesiumIonToken('secret', { setItem() { throw new Error('blocked'); } }),
    /normal, non-private browser profile/,
);


globalThis.window = { location: { search: '' }, Cesium: {} };
globalThis.localStorage = storage;
const { CesiumWorld } = await import('../src/cesium-world.js?token-contract-test');
assert.equal(new CesiumWorld('cesium-container').token, 'test-token');

values.clear();
const missingTokenWorld = new CesiumWorld('cesium-container');
assert.equal(missingTokenWorld.token, bundledDefault);

console.log('cesium token contract tests passed');
