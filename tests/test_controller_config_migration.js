/** Controller config v1-v5 migration and v6 safety validation. */
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
assert(legacy.configVersion === 6, 'config version advances to v6');
assert(legacy.settings['phys-collision-radius'] === '0.6', 'old shipped 0.15 m default migrates to 0.6 m');

const custom = migrate({ configVersion: 4, settings: { 'phys-collision-radius': '0.42' } });
assert(custom.settings['phys-collision-radius'] === '0.42', 'explicit custom radius is preserved');

const oldPidDefaults = {
    'ctrl-pos-kp': '2.0', 'ctrl-pos-ki': '0.3', 'ctrl-pos-kd': '0.1',
    'ctrl-vel-kp': '3.0', 'ctrl-vel-ki': '1.0', 'ctrl-vel-kd': '0.05',
    'ctrl-alt-kp': '4.0', 'ctrl-alt-ki': '2.0', 'ctrl-alt-kd': '0.1',
};
const v5 = migrate({
    configVersion: 5,
    currentMode: 'fpv',
    mapping: {
        roll: { axisIndex: 5, inverted: true, deadzone: 0.12, rate: 3, expo: 0.7 },
    },
    settings: {
        'flight-mode-select': 'fpv',
        'so3-kx': '99', 'so3-kv': '99', 'so3-kr': '150', 'so3-komega': '20',
        ...oldPidDefaults,
    },
    modeRateExpo: {
        drone: { roll: { rate: 2, expo: 0.2 } },
        fpv: { roll: { rate: 8, expo: 0.8 } },
    },
    modePidSettings: {
        drone: oldPidDefaults,
        fpv: { ...oldPidDefaults, 'ctrl-pos-kp': '1.25' },
    },
});

for (const mode of ['drone', 'fpv', 'stabilized', 'so3']) {
    assert(!!v5.modeRateExpo[mode], `${mode} has a complete rate/expo profile`);
    assert(!!v5.modePidSettings[mode], `${mode} has a complete PID profile`);
    assert(Object.keys(v5.modeRateExpo[mode]).length === 5, `${mode} rate/expo includes every axis`);
    assert(Object.keys(v5.modePidSettings[mode]).length === 9, `${mode} PID includes every gain`);
}
assert(v5.modeRateExpo.fpv.roll.rate === 8, 'legacy FPV tuning is preserved');
assert(v5.modeRateExpo.stabilized.roll.rate === 1, 'Level does not inherit FPV rates');
assert(v5.modeRateExpo.so3.roll.expo === 0, 'SO3 does not inherit FPV expo');
assert(v5.modePidSettings.drone['ctrl-pos-kp'] === '0.95', 'old shipped Easy PID profile migrates to v6 baseline');
assert(v5.modePidSettings.fpv['ctrl-pos-kp'] === '1.25', 'custom legacy PID profile is preserved');
assert(!('so3-kx' in v5.settings) && !('so3-kr' in v5.settings), 'obsolete scalar SO3 gains are discarded');

// Profiles must be independent objects: modifying one cannot mutate another.
v5.modeRateExpo.stabilized.roll.rate = 4;
assert(v5.modeRateExpo.so3.roll.rate === 1, 'mode rate/expo profiles are not aliased');

const damaged = migrate({
    configVersion: 6,
    currentMode: 'warp-drive',
    settings: { 'flight-mode-select': 'also-invalid' },
    mapping: {
        roll: { axisIndex: null, inverted: 'yes', deadzone: 'NaN', rate: 999, expo: -4 },
        pitch: 'broken',
    },
    buttonMapping: { arm: { source: 'shell', buttonIndex: 9999, triggerMode: 'maybe' } },
    hidCalibration: [{ min: 100, center: 20, max: 10 }],
    modeRateExpo: { fpv: { roll: { rate: Infinity, expo: 'bad' } } },
    modePidSettings: { drone: { 'ctrl-pos-kp': Infinity, 'ctrl-vel-ki': 999 } },
});
assert(damaged.currentMode === 'drone', 'invalid mode falls back to Easy');
assert(damaged.settings['flight-mode-select'] === 'drone', 'invalid saved mode select is repaired');
assert(damaged.mapping.roll.axisIndex === 0 && damaged.mapping.roll.inverted === false, 'damaged axis mapping uses safe defaults');
assert(damaged.mapping.roll.rate === 10 && damaged.mapping.roll.expo === 0, 'finite mapping ranges are clamped');
assert(damaged.mapping.pitch.axisIndex === 1, 'non-object mapping action uses its default');
assert(damaged.buttonMapping.arm.source === 'button' && damaged.buttonMapping.arm.buttonIndex === 0, 'damaged button binding uses defaults');
assert(damaged.hidCalibration.length === 16 && damaged.hidCalibration[0].min === null, 'invalid HID calibration is cleared safely');
assert(damaged.modeRateExpo.fpv.roll.rate === 1, 'invalid per-mode rate falls back safely');
assert(damaged.modePidSettings.drone['ctrl-pos-kp'] === '0.95', 'non-finite PID falls back to v6 default');
assert(damaged.modePidSettings.drone['ctrl-vel-ki'] === '5', 'finite PID is clamped to its UI range');

console.log(`\nController config migration: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
