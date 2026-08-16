/** Controller integration contract for RadioMaster T8L serial ownership. */
import assert from 'node:assert/strict';

globalThis.performance ||= { now: () => Date.now() };
globalThis.document ||= {
    getElementById: () => null,
    querySelectorAll: () => [],
};
globalThis.window ||= { addEventListener: () => {} };
globalThis.localStorage ||= { getItem: () => null, setItem: () => {} };

const { Controller } = await import('../src/controller.js?t8l-controller-test');

const controller = new Controller();
controller._buildSettingsUI = () => {};
controller._updateT8LDisplay = () => {};
controller._updateHIDDisplay = () => {};
controller._updateGamepadDisplay = () => {};
for (const [action, axisIndex] of Object.entries({
    roll: 0,
    pitch: 1,
    throttle: 2,
    yaw: 3,
})) {
    controller.mapping[action].source = 'axis';
    controller.mapping[action].axisIndex = axisIndex;
    controller.mapping[action].buttonIndex = -1;
}
controller._getHIDAxes = () => new Array(10).fill(0.9);
controller._getGamepad = () => ({
    id: 'fallback-gamepad', connected: true, axes: new Array(10).fill(0.8), buttons: [],
});

let t8l = {
    connected: true,
    fresh: true,
    ageMs: 0,
    frameRateHz: 50,
    rawChannels: [1600, 1300, 1800, 1100, 2000, 2000, 1500, 1500, 1500, 1500],
    axes: [0.2, -0.4, 0.6, -0.8, 1, 1, 0, 0, 0, 0],
};
controller._t8lSerial = { snapshot: () => t8l };
controller.buttonMapping.arm = {
    source: 'axis', buttonIndex: -1, axisIndex: 4,
    axisThreshold: 0.5, inverted: false, triggerMode: 'level',
};
controller.buttonMapping.modeSwitch = {
    source: 'axis', buttonIndex: -1, axisIndex: 5,
    axisThreshold: 0.5, inverted: false, triggerMode: 'level',
};
const modeChanges = [];
controller._onModeSwitch = mode => {
    controller._currentMode = mode;
    modeChanges.push(mode);
};

let input = controller.update();
assert.equal(input.inputSource, 't8l-serial');
assert.deepEqual(
    [input.rawAxes.roll, input.rawAxes.pitch, input.rawAxes.throttle, input.rawAxes.yaw],
    [0.2, -0.4, 0.6, -0.8],
    'T8L CH1-CH4 own the mapped flight axes ahead of HID/gamepad',
);
assert.equal(input.armed, false, 'first valid frame cannot arm from a held switch');
assert.deepEqual(modeChanges, [], 'first valid frame cannot change mode from a held switch');

t8l = { ...t8l, axes: [...t8l.axes.slice(0, 4), -1, -1, ...t8l.axes.slice(6)] };
controller.update();
t8l = { ...t8l, axes: [...t8l.axes.slice(0, 4), 1, 1, ...t8l.axes.slice(6)] };
input = controller.update();
assert.equal(input.armed, true, 'Arm level follows a physical T8L switch transition');
assert.equal(modeChanges.at(-1), 'so3', 'T8L Mode high selects SO3');

t8l = { ...t8l, axes: [...t8l.axes.slice(0, 5), -1, ...t8l.axes.slice(6)] };
controller.update();
assert.equal(modeChanges.at(-1), 'fpv', 'T8L Mode low selects FPV');
t8l = { ...t8l, axes: [...t8l.axes.slice(0, 5), 0, ...t8l.axes.slice(6)] };
controller.update();
assert.equal(modeChanges.at(-1), 'drone', 'T8L Mode middle selects Easy');

t8l = { ...t8l, fresh: false, ageMs: 251 };
input = controller.update();
assert.equal(input.inputSource, 't8l-serial', 'a stale open T8L does not fall through to HID/gamepad');
assert.deepEqual(
    [input.rawAxes.roll, input.rawAxes.pitch, input.rawAxes.throttle, input.rawAxes.yaw],
    [0, 0, 0, 0],
    'stale T8L axes are zeroed',
);
assert.equal(input.t8l.failsafeTriggered, true, '250 ms stale transition emits one failsafe');
assert.equal(controller.update().t8l.failsafeTriggered, false, 'stale failsafe is edge-triggered');

t8l = { ...t8l, fresh: true, ageMs: 0, axes: [0.1, 0.2, 0.3, 0.4, 1, 1, 0, 0, 0, 0] };
const changesBeforeRecovery = modeChanges.length;
input = controller.update();
assert.equal(input.armed, true, 'first recovered frame preserves the current arm state');
assert.equal(modeChanges.length, changesBeforeRecovery,
    'first recovered frame suppresses a held mode-switch transition');

t8l = { ...t8l, connected: false, fresh: false, ageMs: Infinity };
controller._t8lFailsafePending = true;
input = controller.update();
assert.equal(input.inputSource, 'webhid', 'WebHID becomes active after explicit T8L disconnect');
assert.deepEqual(
    [input.rawAxes.roll, input.rawAxes.pitch, input.rawAxes.throttle, input.rawAxes.yaw],
    [0, 0, 0, 0],
    'disconnect failsafe zeros the first fallback-device frame',
);
assert.equal(input.armed, true, 'disconnect failsafe preserves the current arm state');
input = controller.update();
assert.equal(input.rawAxes.roll, 0.9, 'fallback control resumes after the failsafe boundary frame');

console.log('Controller T8L integration: all tests passed');
