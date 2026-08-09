/** Persisted 0.15 m default migrates to 0.6 m without clobbering user tuning. */
globalThis.performance ||= { now: () => Date.now() };
globalThis.document ||= { getElementById: () => null };
globalThis.window ||= {};

const { Controller } = await import('../src/controller.js');
const migrate = (config) => Controller.prototype._migrateConfig.call({}, structuredClone(config));

let passed = 0;
let failed = 0;
function assert(condition, message) {
    if (condition) { passed++; return; }
    failed++;
    console.error(`FAIL: ${message}`);
}

const legacy = migrate({ configVersion: 4, settings: { 'phys-collision-radius': '0.15' } });
assert(legacy.configVersion === 5, 'config version advances to v5');
assert(legacy.settings['phys-collision-radius'] === '0.6', 'old shipped 0.15 m default migrates to 0.6 m');

const custom = migrate({ configVersion: 4, settings: { 'phys-collision-radius': '0.42' } });
assert(custom.settings['phys-collision-radius'] === '0.42', 'explicit custom radius is preserved');

console.log(`\nController config migration: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
