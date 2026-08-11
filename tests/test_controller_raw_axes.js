/** Raw assisted-axis contract: calibrated/inverted, but pre-deadzone/pre-expo. */
globalThis.performance ||= { now: () => Date.now() };
globalThis.document ||= {
    getElementById: () => null,
    querySelectorAll: () => [],
};
globalThis.window ||= {
    addEventListener: () => {},
};
globalThis.localStorage ||= {
    getItem: () => null,
    setItem: () => {},
};

const { Controller } = await import('../src/controller.js');

let passed = 0;
let failed = 0;
function assert(condition, message) {
    if (condition) { passed++; return; }
    failed++;
    console.error(`FAIL: ${message}`);
}
function near(actual, expected, eps = 1e-9) {
    return Math.abs(actual - expected) <= eps;
}

const controller = new Controller();
controller._getHIDAxes = () => null;
controller._getGamepad = () => ({
    id: 'raw-axis-test',
    index: 0,
    connected: true,
    axes: [0.5, 0.1, -0.25, 0.4],
    buttons: [{ pressed: false }],
});
controller._updateGamepadDisplay = () => {};

controller.mapping.roll.deadzone = 0.2;
controller.mapping.roll.expo = 0.5;
controller.mapping.pitch.deadzone = 0.2;
controller.mapping.pitch.expo = 0.8;
controller.mapping.pitch.inverted = true;
controller.mapping.throttle.deadzone = 0;
controller.mapping.throttle.expo = 0;
controller.mapping.yaw.deadzone = 0.05;
controller.mapping.yaw.expo = 0.25;

const input = controller.update();

assert(near(input.rawAxes.roll, 0.5), 'raw roll is captured before deadzone/expo');
assert(near(input.rawAxes.pitch, -0.1), 'raw pitch includes axis inversion');
assert(near(input.rawAxes.throttle, -0.25), 'raw throttle is preserved');
assert(near(input.rawAxes.yaw, 0.4), 'raw yaw is preserved');
assert(near(input.rawAxes.cameraTilt, 0), 'unassigned raw camera tilt is zero');

// Legacy FPV-shaped axes remain unchanged: roll gets expo, pitch is removed by
// deadzone, and throttle retains the original linear path.
assert(near(input.roll, 0.3125), 'legacy roll still uses the existing expo curve');
assert(near(input.pitch, 0), 'legacy pitch still uses the existing deadzone');
assert(near(input.throttle, -0.25), 'legacy throttle behavior is unchanged');
assert(near(input.yaw, 0.316), 'legacy yaw still uses the existing expo curve');

assert(input.axisConfig.roll.deadzone === 0.2 && input.axisConfig.roll.expo === 0.5,
    'axisConfig exposes roll deadzone/expo to assisted controllers');
assert(input.axisConfig.pitch.deadzone === 0.2 && input.axisConfig.pitch.expo === 0.8,
    'axisConfig exposes pitch deadzone/expo to assisted controllers');

input.rawAxes.roll = -1;
assert(near(controller.rawAxes.roll, 0.5), 'returned rawAxes is a read-only-style snapshot, not internal storage');

console.log(`\nController raw axes: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
