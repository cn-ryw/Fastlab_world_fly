/*
 * Copyright 2026 Manifold Tech Ltd.
 * Author: MENG Guotao <mengguotao@manifoldtech.cn>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Controller input system — RC transmitter via Gamepad API + keyboard.
 * Supports channel assignment, axis inversion, dead zones, and listen-mode mapping.
 */

import { editPath } from './path-editor.js';
import { formatLap } from './gates.js';
import { reportUserError } from './error-report.js';

const ACTIONS = ['roll', 'pitch', 'throttle', 'yaw', 'cameraTilt'];
const BUTTON_ACTIONS = ['arm', 'modeSwitch'];

// All persisted slider/select IDs
const SETTINGS_IDS = [
    'flight-mode-select',
    'drone-max-speed',
    'drone-max-vspeed',
    'cam-hfov',
    'cam-mount-angle',
    'ctrl-pos-kp', 'ctrl-pos-ki', 'ctrl-pos-kd', 'ctrl-vel-kp', 'ctrl-vel-ki', 'ctrl-vel-kd', 'ctrl-alt-kp', 'ctrl-alt-ki', 'ctrl-alt-kd',
    'so3-kx', 'so3-kv', 'so3-kr', 'so3-komega',
    'phys-mass', 'phys-thrust', 'phys-drag-cd', 'phys-drag-area',
    'phys-drone-size', 'phys-collision-radius',
    'clean-mode-toggle', 'osd-toggle',
    'pano-level-toggle', 'yaw-lock-toggle',
];

const CONFIG_VERSION = 5;
const DEFAULT_EASY_MAX_SPEED = '83.333';
const DEFAULT_EASY_MAX_VSPEED = '8';
const DEFAULT_DRAG_AREA = '0.0015';
const LEGACY_EASY_MAX_SPEED = 9;
const PREVIOUS_EASY_MAX_SPEED = 18;
const LEGACY_EASY_MAX_VSPEED = 6;
const PREVIOUS_DRAG_AREA = 0.01;

// Settings that are stored separately per flight mode (drone vs fpv)
const PER_MODE_SETTINGS_IDS = [
    'ctrl-pos-kp', 'ctrl-pos-ki', 'ctrl-pos-kd',
    'ctrl-vel-kp', 'ctrl-vel-ki', 'ctrl-vel-kd',
    'ctrl-alt-kp', 'ctrl-alt-ki', 'ctrl-alt-kd',
];

const DEFAULT_MAPPING = {
    roll:       { axisIndex: 0, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
    pitch:      { axisIndex: 1, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
    throttle:   { axisIndex: 2, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
    yaw:        { axisIndex: 3, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
    cameraTilt: { axisIndex: -1, inverted: false, deadzone: 0, rate: 1.0, expo: 0.0 },
};

/**
 * Strict-but-fuzzy comparison for gate-path control-point arrays.
 * Used to decide whether an editor commit actually changed the layout
 * and therefore invalidates the per-scene best-lap record. Any change
 * in length, or any coordinate that moved more than ~1 mm on any axis,
 * counts as a redesign.
 */
function _pointsEqual(a, b) {
    if (!Array.isArray(a) || !Array.isArray(b)) return false;
    if (a.length !== b.length) return false;
    const EPS = 1e-3; // metres — gate positions are stored in metres
    for (let i = 0; i < a.length; i++) {
        const p = a[i], q = b[i];
        if (!p || !q) return false;
        if (Math.abs(Number(p.x) - Number(q.x)) > EPS) return false;
        if (Math.abs(Number(p.y) - Number(q.y)) > EPS) return false;
        if (Math.abs(Number(p.z) - Number(q.z)) > EPS) return false;
    }
    return true;
}

const DEFAULT_BUTTON_MAPPING = {
    // triggerMode applies only to axis-source bindings (RC switches / gamepad
    // axes) and selects between 'toggle' (rising edge flips state) and 'level'
    // (switch position directly reflects state). Button-source bindings and
    // the keyboard always use edge-based toggle regardless of this field.
    arm:        { source: 'button', buttonIndex: 0,  axisIndex: -1, axisThreshold: 0.5, inverted: false, triggerMode: 'toggle' },
    // Unassigned by default — user can bind any channel/button via the settings panel.
    modeSwitch: { source: 'button', buttonIndex: -1, axisIndex: -1, axisThreshold: 0.5, inverted: false, triggerMode: 'toggle' },
};

const KEYBOARD_MAP = {
    'KeyW':       { action: 'throttle', value: 1 },
    'KeyS':       { action: 'throttle', value: -1 },
    'KeyA':       { action: 'yaw',      value: -1 },
    'KeyD':       { action: 'yaw',      value: 1 },
    // Arrow keys = drone position control (Easy/Drone mode):
    //   pitch → fwd/back velocity,  roll → left/right velocity
    'ArrowUp':    { action: 'pitch',    value: -1 },
    'ArrowDown':  { action: 'pitch',    value: 1 },
    'ArrowLeft':  { action: 'roll',     value: -1 },
    'ArrowRight': { action: 'roll',     value: 1 },
    'KeyQ':       { action: 'cameraTilt', value: 1 },
    'KeyE':       { action: 'cameraTilt', value: -1 },
};

export class Controller {
    constructor() {
        this.mapping = JSON.parse(JSON.stringify(DEFAULT_MAPPING));
        this.buttonMapping = JSON.parse(JSON.stringify(DEFAULT_BUTTON_MAPPING));
        this.gamepadIndex = -1;
        this.gamepadName = '';
        this.connected = false;

        // Per-mode rate/expo snapshots (mapping.rate and mapping.expo per axis)
        // Initialized as null; _loadConfig will fill from saved data or
        // the post-init block will snapshot from restored this.mapping.
        this._modeRateExpo = { drone: null, fpv: null };
        // Per-mode PID settings (slider values keyed by element id)
        this._modePidSettings = { drone: null, fpv: null };
        this._currentMode = 'drone';

        // WebHID support for RC transmitters
        this._hidDevice = null;
        this._hidAxes = new Array(16).fill(0); // Up to 16 channels, normalized to -1..1
        this._hidRawAxes = new Array(16).fill(0); // raw 16-bit values before calibration
        this._hidCalibration = Array.from({length: 16}, () => ({ min: null, center: null, max: null }));
        this._hidConnected = false;
        this._hidDeviceName = '';
        
        // Option to disable Gamepad API (allows Chrome WebHID to claim the device)
        this._gamepadApiDisabled = false;

        // Current input state (merged keyboard + gamepad, range [-1, 1])
        this.axes = { roll: 0, pitch: 0, throttle: 0, yaw: 0, cameraTilt: 0 };
        this._cameraTiltKeyboard = 0;
        this._cameraTiltAxis = 0;
        this._prevCameraTiltAxis = 0;
        this.buttons = { arm: false, modeSwitch: false };
        this.boost = false;

        // Separate current/previous state for keyboard and gamepad edge detection.
        // Reset is keyboard-only (R key); it has no gamepad/HID binding, so it is
        // not tracked in these per-frame button-state objects.
        this._gpButtons     = { arm: false, modeSwitch: false };
        this._prevKbButtons = { arm: false, reset: false, modeSwitch: false };
        this._prevGpButtons = { arm: false, modeSwitch: false };

        // Keyboard state
        this._keysDown = new Set();

        // Listen mode (axes)
        this._listenAction = null;
        this._listenCallback = null;
        this._listenBaseline = null;

        // Listen mode (buttons)
        this._listenButtonAction = null;
        this._listenButtonCallback = null;
        this._listenButtonBaseline = null;

        // Armed state
        this.armed = false;

        // Startup / hot-reconnect guard. When the input device transitions
        // from disconnected to connected we seed _prevGpButtons with the
        // current polled state so a switch that is already in its “pressed”
        // position does not produce a phantom rising-edge on the first frame.
        this._wasConnected = false;

        // Gate-path settings. See src/gates.js for runtime behaviour.
        // `gateSize` and `clearance` are global knobs shared by all gates.
        // `path` is either null (no course drawn) or the user-edited
        // closed-loop description that gets fed to GateCourse.rebuild().
        // Persistence of the path itself is per-map (see src/path-store.js);
        // the localStorage copy here is a runtime fallback — on scene load
        // main.js overwrites `path` from the per-map file if one exists.
        this.gatePathSettings = {
            gateSize:  1.2,
            clearance: 0.8,
            path: null,   // { closed: true, points: [{x,y,z}, ...], yMin, yMax }
        };
        // Wired from main.js after the GateCourse is constructed.
        //   _gatePathApplyCb()         → called whenever path/size/clearance
        //                                changes; main.js rebuilds entities
        //                                and persists to asset/gate-paths/.
        //   _gatePathCtxProvider()     → returns { octree, bounds, spawnPoint }
        //                                snapshot for the path editor backdrop.
        this._gateCourse = null;
        this._gatePathApplyCb = null;
        this._gatePathCtxProvider = null;

        // Load saved config
        this._loadConfig();

        // Setup event listeners
        this._setupKeyboard();
        this._setupGamepad();
        this._buildSettingsUI();

        // Ensure both modes have valid rate/expo + PID snapshots.
        // After _loadConfig, this.mapping has the restored rate/expo values
        // and DOM sliders have the restored PID values.
        // For legacy configs (no per-mode data), initialize both modes
        // from these restored values so nothing is lost.
        const curSnap = this._snapshotRateExpo();
        for (const mode of ['drone', 'fpv']) {
            if (!this._modeRateExpo[mode]) this._modeRateExpo[mode] = JSON.parse(JSON.stringify(curSnap));
            if (!this._modePidSettings[mode]) this._modePidSettings[mode] = this._snapshotPidSettings();
        }
    }

    /**
     * Call once per frame to poll gamepad and update axes.
     */
    update() {
        // Reset axes and buttons to 0/false each frame
        for (const action of ACTIONS) {
            this.axes[action] = 0;
        }
        this.buttons.arm = false;
        this._gpButtons.arm = false;
        this.boost = false;
        this._cameraTiltKeyboard = 0;
        this._prevCameraTiltAxis = this._cameraTiltAxis;
        this._cameraTiltAxis = 0;

        // Keyboard input
        for (const [code, map] of Object.entries(KEYBOARD_MAP)) {
            if (this._keysDown.has(code)) {
                this.axes[map.action] += map.value;
                if (map.action === 'cameraTilt') {
                    this._cameraTiltKeyboard += map.value;
                }
            }
        }
        if (this._keysDown.has('ShiftLeft') || this._keysDown.has('ShiftRight')) {
            this.boost = true;
        }

        // Gamepad input (prefer WebHID if connected)
        const hidAxes = this._getHIDAxes();
        const gp = this._getGamepad();
        
        if (hidAxes) {
            // Use WebHID input
            this.connected = true;
            this.gamepadName = this._hidDeviceName + ' (HID)';

            for (const action of ACTIONS) {
                const m = this.mapping[action];
                if (m.axisIndex >= 0 && m.axisIndex < hidAxes.length) {
                    let val = hidAxes[m.axisIndex];
                    if (m.inverted) val = -val;
                    if (Math.abs(val) < m.deadzone) val = 0;
                    // Apply expo curve
                    const e = m.expo || 0;
                    if (e > 0) {
                        val = Math.sign(val) * Math.abs(val) * (1 - e + e * val * val);
                    }
                    this.axes[action] += val;
                    if (action === 'cameraTilt') {
                        this._cameraTiltAxis = val;
                    }
                }
            }

            // HID button handling — transmitters expose switches as axes.
            // Reset to `false` each frame and only raise if the axis binding
            // exists and its (optionally inverted) value crosses the
            // directional threshold. One-sided so the user picks which end
            // of the switch counts as “pressed” via the Inv toggle.
            for (const bAction of BUTTON_ACTIONS) {
                const bm = this.buttonMapping[bAction];
                let pressed = false;
                if (bm.source === 'axis' && bm.axisIndex >= 0 && bm.axisIndex < hidAxes.length) {
                    let v = hidAxes[bm.axisIndex];
                    if (bm.inverted) v = -v;
                    pressed = v > bm.axisThreshold;
                }
                this._gpButtons[bAction] = pressed;
            }

            // Listen mode for HID: detect axis movement
            if (this._listenAction && this._listenBaseline) {
                let maxDelta = 0;
                let bestAxis = -1;
                let bestSign = 1;
                for (let i = 0; i < hidAxes.length; i++) {
                    const delta = hidAxes[i] - this._listenBaseline[i];
                    if (Math.abs(delta) > Math.abs(maxDelta)) {
                        maxDelta = delta;
                        bestAxis = i;
                        bestSign = Math.sign(delta);
                    }
                }
                if (Math.abs(maxDelta) > 0.3) { // Lower threshold for HID (more sensitive)
                    if (this.mapping[this._listenAction]) {
                        this.mapping[this._listenAction].axisIndex = bestAxis;
                        this.mapping[this._listenAction].inverted = bestSign < 0;
                    }
                    const action = this._listenAction;
                    this._listenAction = null;
                    this._listenBaseline = null;
                    if (this._listenCallback) this._listenCallback(action, bestAxis, bestSign < 0);
                    this._saveConfig();
                    this._buildSettingsUI();
                }
            }

            // Update HID display
            this._updateHIDDisplay(hidAxes);
        } else if (gp) {
            this.connected = true;
            this.gamepadName = gp.id;

            for (const action of ACTIONS) {
                const m = this.mapping[action];
                if (m.axisIndex >= 0 && m.axisIndex < gp.axes.length) {
                    let val = gp.axes[m.axisIndex];
                    if (m.inverted) val = -val;
                    if (Math.abs(val) < m.deadzone) val = 0;
                    // Apply expo curve: output = val * (1 - expo + expo * val²)
                    const e = m.expo || 0;
                    if (e > 0) {
                        val = Math.sign(val) * Math.abs(val) * (1 - e + e * val * val);
                    }
                    this.axes[action] += val;
                    if (action === 'cameraTilt') {
                        this._cameraTiltAxis = val;
                    }
                }
            }

            // Listen mode: detect axis movement
            if (this._listenAction && this._listenBaseline) {
                let maxDelta = 0;
                let bestAxis = -1;
                let bestSign = 1;
                for (let i = 0; i < gp.axes.length; i++) {
                    const delta = gp.axes[i] - this._listenBaseline[i];
                    if (Math.abs(delta) > Math.abs(maxDelta)) {
                        maxDelta = delta;
                        bestAxis = i;
                        bestSign = Math.sign(delta);
                    }
                }
                if (Math.abs(maxDelta) > 0.5) {
                    // Axis detected — update mapping only for stick actions
                    if (this.mapping[this._listenAction]) {
                        this.mapping[this._listenAction].axisIndex = bestAxis;
                        this.mapping[this._listenAction].inverted = bestSign < 0;
                    }
                    const action = this._listenAction;
                    this._listenAction = null;
                    this._listenBaseline = null;
                    // Also cancel a concurrent button-listen for the same
                    // action (Gamepad dual-listen path) so a later press
                    // won't overwrite the binding we just committed.
                    if (this._listenButtonAction === action) {
                        this._listenButtonAction = null;
                        this._listenButtonBaseline = null;
                        this._listenButtonCallback = null;
                    }
                    if (this._listenCallback) this._listenCallback(action, bestAxis, bestSign < 0);
                    this._saveConfig();
                    this._buildSettingsUI();
                }
            }

            // Button listen mode: detect button press
            if (this._listenButtonAction && this._listenButtonBaseline) {
                for (let i = 0; i < gp.buttons.length; i++) {
                    if (gp.buttons[i].pressed && !this._listenButtonBaseline[i]) {
                        const action = this._listenButtonAction;
                        this._listenButtonAction = null;
                        this._listenButtonBaseline = null;
                        // Cross-cancel any concurrent axis-listen for the
                        // same action so stick movement after the button
                        // press doesn't replace the fresh button binding.
                        if (this._listenAction === action) {
                            this._listenAction = null;
                            this._listenBaseline = null;
                            this._listenCallback = null;
                        }
                        if (this._listenButtonCallback) this._listenButtonCallback(action, i);
                        this._saveConfig();
                        this._buildSettingsUI();
                        break;
                    }
                }
            }

            // Gamepad buttons. Axis source uses a directional threshold
            // (with optional inversion) so only one end of a trigger/switch
            // counts as pressed; button source respects `inverted` to flip
            // the active-low / active-high sense.
            for (const bAction of BUTTON_ACTIONS) {
                const bm = this.buttonMapping[bAction];
                let pressed = false;
                if (bm.source === 'axis' && bm.axisIndex >= 0 && bm.axisIndex < gp.axes.length) {
                    let v = gp.axes[bm.axisIndex];
                    if (bm.inverted) v = -v;
                    pressed = v > bm.axisThreshold;
                } else if (bm.source === 'button' && bm.buttonIndex >= 0 && bm.buttonIndex < gp.buttons.length) {
                    pressed = gp.buttons[bm.buttonIndex].pressed;
                    if (bm.inverted) pressed = !pressed;
                }
                this._gpButtons[bAction] = pressed;
            }

            // Update gamepad display
            this._updateGamepadDisplay(gp);
        } else {
            this.connected = false;
        }

        // Keyboard buttons. The mode-switch key (M) is suppressed while the
        // settings panel is open so that typing inside any settings control
        // can never accidentally toggle the flight mode — the user must use
        // the dropdown (mouse / arrow keys) in that case.
        const kbArm = this._keysDown.has('Space');
        const kbReset = this._keysDown.has('KeyR');
        const kbModeSwitch = this._keysDown.has('KeyM') && !this.isSettingsOpen();

        // Clamp axes
        for (const action of ACTIONS) {
            this.axes[action] = Math.max(-1, Math.min(1, this.axes[action]));
        }

        // Edge detection: gamepad and keyboard evaluated independently.
        const gpArmRising    = this._gpButtons.arm        && !this._prevGpButtons.arm;
        const gpModeRising   = this._gpButtons.modeSwitch && !this._prevGpButtons.modeSwitch;
        const kbArmRising    = kbArm        && !this._prevKbButtons.arm;
        const kbResetRising  = kbReset      && !this._prevKbButtons.reset;
        const kbModeRising   = kbModeSwitch && !this._prevKbButtons.modeSwitch;

        // Suppress rising-edges on the frame the input device becomes
        // connected so a switch already held in its “pressed” position at
        // startup / hot-reconnect does not spuriously fire arm or modeSwitch.
        // Level-mode bindings are exempt because their whole purpose is to
        // reflect the switch position at all times, including on load.
        const justConnected = this.connected && !this._wasConnected;
        this._wasConnected = this.connected;

        // A binding is "level-mode active" only when it is an axis source
        // that is actually bound (axisIndex >= 0) and its triggerMode is
        // 'level'. Otherwise we fall back to edge-based toggle.
        const armBm  = this.buttonMapping.arm;
        const modeBm = this.buttonMapping.modeSwitch;
        const armAxisLevel  = armBm.source  === 'axis' && armBm.axisIndex  >= 0 && armBm.triggerMode  === 'level';
        const modeAxisLevel = modeBm.source === 'axis' && modeBm.axisIndex >= 0 && modeBm.triggerMode === 'level';

        // Keyboard arm / mode-switch are keyboard-exclusive edge-toggles —
        // their behaviour is independent of the settings panel's button
        // mapping (source, axis/button index, triggerMode) and of whether a
        // gamepad / RC transmitter is plugged in. Space and M always flip
        // their respective state on rising edge; nothing else fights them.
        if (kbArmRising)  this.armed = !this.armed;
        if (kbModeRising) this._toggleFlightMode();

        // Gamepad / HID arm. Level mode follows switch TRANSITIONS (rising /
        // falling edge of `_gpButtons.arm`), not the absolute switch position
        // every frame. This way a static switch cannot silently undo the
        // keyboard Space toggle above between physical switch moves; the two
        // inputs coexist as orthogonal controls, each winning on the frame
        // it's actually used.
        //
        // `this.connected` guards against a stale axis+level binding saved in
        // localStorage firing while no device is polling. `justConnected`
        // suppresses the first-frame phantom edge on hot-reconnect so a
        // switch sitting in its active position at connect time doesn't
        // spuriously override the current armed state.
        if (armAxisLevel && this.connected) {
            const gpArmChanged = this._gpButtons.arm !== this._prevGpButtons.arm;
            if (!justConnected && gpArmChanged) this.armed = this._gpButtons.arm;
        } else if (!justConnected && gpArmRising) {
            this.armed = !this.armed;
        }

        // Gamepad / HID mode switch — same transition semantics as arm above.
        if (modeAxisLevel && this.connected) {
            const gpModeChanged = this._gpButtons.modeSwitch !== this._prevGpButtons.modeSwitch;
            if (!justConnected && gpModeChanged) {
                const targetMode = this._gpButtons.modeSwitch ? 'fpv' : 'drone';
                if (this._currentMode !== targetMode) {
                    const ms = document.getElementById('flight-mode-select');
                    if (ms) ms.value = targetMode;
                    this._onModeSwitch(targetMode);
                }
            }
        } else if (!justConnected && gpModeRising) {
            this._toggleFlightMode();
        }

        this._prevGpButtons.arm        = this._gpButtons.arm;
        this._prevGpButtons.modeSwitch = this._gpButtons.modeSwitch;
        this._prevKbButtons.arm        = kbArm;
        this._prevKbButtons.reset      = kbReset;
        this._prevKbButtons.modeSwitch = kbModeSwitch;

        return {
            roll: this.axes.roll,
            pitch: this.axes.pitch,
            throttle: this.axes.throttle,
            yaw: this.axes.yaw,
            cameraTilt: this.axes.cameraTilt,
            cameraTiltKeyboard: this._cameraTiltKeyboard,
            cameraTiltAxis: this._cameraTiltAxis,
            cameraTiltAxisChanged: Math.abs(this._cameraTiltAxis - this._prevCameraTiltAxis) > 0.01,
            boost: this.boost,
            armed: this.armed,
            resetTriggered: kbResetRising,
            rates: {
                roll:  this.mapping.roll.rate  !== undefined ? this.mapping.roll.rate  : 1.0,
                pitch: this.mapping.pitch.rate !== undefined ? this.mapping.pitch.rate : 1.0,
                yaw:   this.mapping.yaw.rate   !== undefined ? this.mapping.yaw.rate   : 1.0,
            },
        };
    }

    startListening(action, callback) {
        // Support both Gamepad API and WebHID
        if (this._hidConnected) {
            this._listenAction = action;
            this._listenCallback = callback;
            this._listenBaseline = [...this._hidAxes];
            return true;
        }
        const gp = this._getGamepad();
        if (!gp) return false;
        this._listenAction = action;
        this._listenCallback = callback;
        this._listenBaseline = Array.from(gp.axes);
        return true;
    }

    cancelListening() {
        this._listenAction = null;
        this._listenCallback = null;
        this._listenBaseline = null;
    }

    startButtonListening(action, callback) {
        const gp = this._getGamepad();
        if (!gp) return false;
        this._listenButtonAction = action;
        this._listenButtonCallback = callback;
        this._listenButtonBaseline = Array.from(gp.buttons.map(b => b.pressed));
        return true;
    }

    cancelButtonListening() {
        this._listenButtonAction = null;
        this._listenButtonCallback = null;
        this._listenButtonBaseline = null;
    }

    /**
     * Wire the gate-path subsystem. Called from main.js once the
     * GateCourse is constructed and the scene-context provider + apply
     * callback are available.
     *
     * @param {GateCourse} course
     * @param {() => void} applyCb - main.js function that rebuilds the
     *   gate entities from the current gatePathSettings (and persists the
     *   result to the per-map JSON file). Fire-and-forget; safe to call
     *   when no scene is loaded (it will no-op).
     * @param {() => { octree, bounds, spawnPoint }} ctxProvider - returns
     *   a fresh snapshot of the runtime scene context; the path editor
     *   uses it for its point-cloud backdrop and spawn marker.
     */
    attachGateCourse(course, applyCb, ctxProvider) {
        this._gateCourse = course || null;
        this._gatePathApplyCb     = typeof applyCb === 'function' ? applyCb : null;
        this._gatePathCtxProvider = typeof ctxProvider === 'function' ? ctxProvider : null;
        if (this._gateCourse) {
            this._gateCourse.configure(this.gatePathSettings);
        }
        this._buildSettingsUI();
    }

    refreshSettingsUI() {
        this._buildSettingsUI();
    }

    /**
     * Push `gatePathSettings` into the running GateCourse and trigger
     * main.js to rebuild + persist. Called from the settings UI after any
     * gate-path-related change (size, clearance, or a fresh path from
     * the editor).
     */
    _applyGatePath() {
        if (this._gateCourse) {
            this._gateCourse.configure(this.gatePathSettings);
        }
        if (this._gatePathApplyCb) {
            try { this._gatePathApplyCb(); }
            catch (e) {
                reportUserError('Gate path apply failed', e, {
                    key: 'gate-path-apply',
                    intervalMs: 10000,
                });
            }
        }
    }

    /**
     * Populate the Gate Path section of the settings panel. Three rows
     * only: gate size, clearance, and path (status + Edit / Clear
     * buttons). No enable-toggle, no seed — `G` in flight mode toggles
     * visibility, and the path is explicitly edited via the modal.
     */
    _buildRaceCourseSection() {
        const container = document.getElementById('race-course-settings');
        if (!container) return;
        container.innerHTML = '';

        const S = this.gatePathSettings;

        // --- Shared slider factory ---------------------------------
        // Same visual pattern as the old UI but writes to gatePathSettings
        // and triggers the apply callback (main.js rebuilds + persists).
        const mkSlider = (key, label, min, max, step, unit) => {
            const row = document.createElement('div');
            row.className = 'setting-row';
            const l = document.createElement('label');
            l.textContent = unit ? `${label} (${unit})` : label;
            row.appendChild(l);
            const ctrls = document.createElement('div');
            ctrls.className = 'controls';

            const slider = document.createElement('input');
            slider.type = 'range';
            slider.min = String(min);
            slider.max = String(max);
            slider.step = String(step);
            slider.value = String(S[key]);
            slider.style.width = '100px';

            const num = document.createElement('input');
            num.type = 'number';
            num.min = String(min);
            num.max = String(max);
            num.step = String(step);
            num.value = String(S[key]);
            num.style.cssText = 'width:54px;background:#223;color:#ddd;border:1px solid #446;border-radius:3px;padding:2px 4px;font-size:12px;';

            const sync = (v) => {
                const clamped = Math.max(min, Math.min(max, v));
                S[key] = clamped;
                slider.value = String(clamped);
                num.value = String(clamped);
            };
            slider.addEventListener('input', () => {
                const v = parseFloat(slider.value);
                if (!Number.isFinite(v)) return;
                S[key] = v;
                num.value = String(v);
                this._saveConfig();
                this._applyGatePath();
            });
            num.addEventListener('change', () => {
                const v = parseFloat(num.value);
                if (!Number.isFinite(v)) return;
                sync(v);
                this._saveConfig();
                this._applyGatePath();
            });
            ctrls.appendChild(slider);
            ctrls.appendChild(num);
            row.appendChild(ctrls);
            return row;
        };

        container.appendChild(mkSlider('gateSize',  'Gate size', 0.4, 5.0, 0.1, 'm'));
        container.appendChild(mkSlider('clearance', 'Clearance', 0.1, 3.0, 0.1, 'm'));

        // --- Path row ---------------------------------------------
        // Single row that shows current path status (gate count + best
        // lap if any) and exposes Edit / Clear buttons. The editor is
        // launched asynchronously so the user can tweak and re-accept
        // without leaving the settings panel.
        const pathRow = document.createElement('div');
        pathRow.className = 'setting-row';
        const pathLbl = document.createElement('label');
        pathLbl.textContent = 'Path';
        pathRow.appendChild(pathLbl);

        const pathCtrls = document.createElement('div');
        pathCtrls.className = 'controls';
        pathCtrls.style.cssText = 'display:flex;gap:6px;align-items:center;';

        const status = document.createElement('span');
        status.style.cssText = 'font-size:12px;color:#888;margin-right:4px;min-width:140px;';
        const pathObj = S.path;
        const nPts = (pathObj && Array.isArray(pathObj.points)) ? pathObj.points.length : 0;
        const bestLapMs = this._gateCourse && this._gateCourse.bestLapMs;
        if (nPts >= 3) {
            const best = Number.isFinite(bestLapMs) ? ` · best ${formatLap(bestLapMs)}` : '';
            status.textContent = `${nPts} gates${best}`;
        } else if (nPts > 0) {
            status.textContent = `${nPts} / 3 gates — editor to finish`;
        } else {
            status.textContent = 'no path drawn';
        }

        const editBtn = document.createElement('button');
        editBtn.className = 'assign-btn';
        editBtn.textContent = nPts > 0 ? 'Edit\u2026' : 'Edit path\u2026';
        editBtn.addEventListener('click', async () => {
            // Scene context required for the editor backdrop + clearance.
            const provider = this._gatePathCtxProvider;
            const ctx = provider ? provider() : null;
            if (!ctx || !ctx.octree || !ctx.bounds) {
                status.textContent = 'load a scene first';
                status.style.color = '#f77';
                setTimeout(() => this._buildRaceCourseSection(), 1500);
                return;
            }
            const initial = S.path ? {
                closed:    true,
                points:    S.path.points,
                yMin:      S.path.yMin,
                yMax:      S.path.yMax,
            } : null;
            const result = await editPath({
                octree:      ctx.octree,
                bounds:      ctx.bounds,
                spawnPoint:  ctx.spawnPoint || null,
                initialPath: initial,
                gateSize:    S.gateSize,
                clearance:   S.clearance,
            });
            if (result) {
                // Layout-change detection: if the new control points
                // differ from the previously-saved set (any point moved,
                // added, or removed) the old best-lap record no longer
                // applies to this course, so we clear it. Re-opening the
                // editor and pressing Save without touching anything
                // keeps the current PB intact. Tolerance is ~1 mm which
                // is well below anything the user can nudge by eye.
                const prevPts = (S.path && Array.isArray(S.path.points)) ? S.path.points : [];
                const layoutChanged = !_pointsEqual(prevPts, result.points);

                // Editor returns latest gate/clearance sliders too — sync
                // them so the settings panel stays consistent with what
                // the user saw in the modal.
                S.gateSize  = result.gateSize;
                S.clearance = result.clearance;
                S.path = {
                    closed: true,
                    points: result.points,
                    yMin:   result.yMin,
                    yMax:   result.yMax,
                };
                if (layoutChanged && this._gateCourse) {
                    this._gateCourse.bestLapMs = null;
                }
                this._saveConfig();
                this._applyGatePath();
                this._buildRaceCourseSection();
            }
        });

        const clearBtn = document.createElement('button');
        clearBtn.className = 'assign-btn';
        clearBtn.textContent = 'Clear';
        clearBtn.style.opacity = S.path ? '1' : '0.4';
        clearBtn.addEventListener('click', () => {
            if (!S.path) return;
            if (!window.confirm('Clear the current gate path for this scene? (best lap will also reset)')) return;
            S.path = null;
            if (this._gateCourse) this._gateCourse.bestLapMs = null;
            this._saveConfig();
            this._applyGatePath();
            this._buildRaceCourseSection();
        });
        pathCtrls.append(status, editBtn, clearBtn);
        pathRow.appendChild(pathCtrls);
        container.appendChild(pathRow);

        // --- Hint line -------------------------------------------
        const hintRow = document.createElement('div');
        hintRow.className = 'setting-row';
        hintRow.style.cssText = 'opacity:0.7;';
        const hintLbl = document.createElement('label');
        hintLbl.textContent = '';
        hintRow.appendChild(hintLbl);
        const hintText = document.createElement('span');
        hintText.style.cssText = 'font-size:11px;color:#889;font-style:italic;';
        hintText.textContent = nPts >= 3
            ? 'Press G in flight mode to show / hide gates.'
            : 'Draw ≥ 3 gates in the editor; then press G in flight.';
        hintRow.appendChild(hintText);
        container.appendChild(hintRow);
    }

    getConfig() {
        const settings = {};
        for (const id of SETTINGS_IDS) {
            const el = document.getElementById(id);
            if (!el) continue;
            settings[id] = el.type === 'checkbox' ? el.checked : el.value;
        }

        // Snapshot current mode before saving so both modes are up-to-date
        this._modeRateExpo[this._currentMode] = this._snapshotRateExpo();
        this._modePidSettings[this._currentMode] = this._snapshotPidSettings();

        return {
            configVersion: CONFIG_VERSION,
            mapping: JSON.parse(JSON.stringify(this.mapping)),
            buttonMapping: JSON.parse(JSON.stringify(this.buttonMapping)),
            hidCalibration: JSON.parse(JSON.stringify(this._hidCalibration)),
            settings,
            modeRateExpo: JSON.parse(JSON.stringify(this._modeRateExpo)),
            modePidSettings: JSON.parse(JSON.stringify(this._modePidSettings)),
            currentMode: this._currentMode,
            gatePathSettings: JSON.parse(JSON.stringify(this.gatePathSettings)),
        };
    }

    loadConfig(config) {
        config = this._migrateConfig(config);
        if (config.mapping) this.mapping = config.mapping;
        if (config.buttonMapping) this.buttonMapping = config.buttonMapping;
        if (config.hidCalibration) this._hidCalibration = config.hidCalibration;
        if (config.modeRateExpo) this._modeRateExpo = config.modeRateExpo;
        if (config.modePidSettings) this._modePidSettings = config.modePidSettings;
        if (config.currentMode) this._currentMode = config.currentMode;
        if (config.settings) this._restoreSettings(config.settings);
        // Accept both the new `gatePathSettings` key and the legacy
        // `raceCourseSettings` key (we only need gateSize + clearance from
        // the legacy schema; everything else — seed / region / straight —
        // is dropped now).
        if (config.gatePathSettings)        this._mergeGatePathSettings(config.gatePathSettings);
        else if (config.raceCourseSettings) this._mergeGatePathSettings(config.raceCourseSettings);
        if (this._gateCourse) this._gateCourse.configure(this.gatePathSettings);
        this._saveConfig();
        this._buildSettingsUI();
    }

    /**
     * Merge saved gate-path settings onto defaults, clamping the numeric
     * fields and validating the optional `path` object. Tolerant of:
     *   - Missing fields (newer defaults kept for any absent key).
     *   - Legacy `raceCourseSettings` blobs (seed/count/region silently
     *     dropped; gateSize + clearance preserved).
     *   - Hand-edited localStorage dumps with wrong types.
     *
     * Note: path persistence is per-map via src/path-store.js. The copy
     * in localStorage here is purely a fallback that lets the Gate Path
     * section show sensible UI before a scene is loaded.
     */
    _mergeGatePathSettings(saved) {
        if (!saved || typeof saved !== 'object') return;
        const S = this.gatePathSettings;
        if (Number.isFinite(Number(saved.gateSize)))  S.gateSize  = Number(saved.gateSize);
        if (Number.isFinite(Number(saved.clearance))) S.clearance = Number(saved.clearance);
        S.gateSize  = Math.max(0.4, Math.min(5.0, S.gateSize  || 1.2));
        S.clearance = Math.max(0.1, Math.min(3.0, S.clearance || 0.8));

        // Path validation — require closed + >= 3 points of finite xyz.
        const raw = saved.path;
        if (raw && typeof raw === 'object' && Array.isArray(raw.points) && raw.points.length >= 3) {
            const pts = [];
            let ok = true;
            for (const p of raw.points) {
                if (!p || !Number.isFinite(Number(p.x)) || !Number.isFinite(Number(p.y)) || !Number.isFinite(Number(p.z))) {
                    ok = false; break;
                }
                pts.push({ x: Number(p.x), y: Number(p.y), z: Number(p.z) });
            }
            if (ok) {
                const yMin = Number.isFinite(Number(raw.yMin)) ? Number(raw.yMin) : -4;
                const yMax = Number.isFinite(Number(raw.yMax)) ? Number(raw.yMax) :  4;
                S.path = {
                    closed: true,
                    points: pts,
                    yMin:   Math.min(yMin, yMax),
                    yMax:   Math.max(yMin, yMax),
                };
            } else {
                S.path = null;
            }
        } else {
            S.path = null;
        }
    }

    _restoreSettings(settings) {
        for (const [id, val] of Object.entries(settings)) {
            const el = document.getElementById(id);
            if (!el) continue;
            if (el.type === 'checkbox') {
                el.checked = !!val;
                el.dispatchEvent(new Event('change'));
            } else {
                el.value = val;
                // Sync paired number input if present
                const numEl = document.getElementById(id + '-num');
                if (numEl) numEl.value = val;
                // Sync paired span display if present
                const spanEl = document.getElementById(id + '-val');
                if (spanEl) spanEl.textContent = parseFloat(val).toFixed(el.step && el.step.includes('.') ? 2 : 0);
                el.dispatchEvent(new Event('input'));
            }
        }
    }

    // ---- Per-mode helpers ----

    _snapshotRateExpo() {
        const snap = {};
        for (const action of ACTIONS) {
            const m = this.mapping[action];
            snap[action] = { rate: m.rate !== undefined ? m.rate : 1.0, expo: m.expo !== undefined ? m.expo : 0.0 };
        }
        return snap;
    }

    _restoreRateExpo(snap) {
        if (!snap) return;
        for (const action of ACTIONS) {
            if (snap[action]) {
                this.mapping[action].rate = snap[action].rate;
                this.mapping[action].expo = snap[action].expo;
            }
        }
    }

    _snapshotPidSettings() {
        const snap = {};
        for (const id of PER_MODE_SETTINGS_IDS) {
            const el = document.getElementById(id);
            if (el) snap[id] = el.value;
        }
        return snap;
    }

    _restorePidSettings(snap) {
        if (!snap) return;
        for (const [id, val] of Object.entries(snap)) {
            const el = document.getElementById(id);
            if (!el) continue;
            el.value = val;
            const numEl = document.getElementById(id + '-num');
            if (numEl) numEl.value = val;
            el.dispatchEvent(new Event('input'));
        }
    }

    /**
     * Called when the flight-mode-select dropdown changes.
     * Saves current mode's rate/expo + PID, restores the new mode's values.
     */
    _onModeSwitch(newMode) {
        const oldMode = this._currentMode;
        if (newMode === oldMode) return;

        // Save current mode's values
        this._modeRateExpo[oldMode] = this._snapshotRateExpo();
        this._modePidSettings[oldMode] = this._snapshotPidSettings();

        // Update current mode BEFORE restoring, so any _saveConfig calls
        // triggered by input events during restore snapshot to the correct mode
        this._currentMode = newMode;

        // Restore new mode's values
        this._restoreRateExpo(this._modeRateExpo[newMode]);
        this._restorePidSettings(this._modePidSettings[newMode]);

        this._saveConfig();
        this._buildSettingsUI();
    }

    /**
     * Toggle flight mode between 'drone' and 'fpv'. Invoked from the in-flight
     * M key or from a mapped modeSwitch RC channel; the settings-panel
     * dropdown path still goes through _onModeSwitch via its change event.
     * Updates the dropdown so the UI reflects the change on next open.
     */
    _toggleFlightMode() {
        const newMode = this._currentMode === 'drone' ? 'fpv' : 'drone';
        const modeSelect = document.getElementById('flight-mode-select');
        if (modeSelect) modeSelect.value = newMode;
        // Programmatic .value changes don't fire 'change', so call the handler
        // directly to snapshot/restore per-mode rate-expo and PID gains.
        this._onModeSwitch(newMode);
    }

    // ---- Private methods ----

    _getGamepad() {
        // If Gamepad API is disabled, return null to allow Chrome WebHID to claim the device.
        if (this._gamepadApiDisabled) return null;
        
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        for (const gp of gamepads) {
            if (gp && gp.connected) {
                this.gamepadIndex = gp.index;
                return gp;
            }
        }
        return null;
    }

    _setupKeyboard() {
        window.addEventListener('keydown', (e) => {
            if (e.code === 'Tab') {
                e.preventDefault();
                this._toggleSettings();
                return;
            }
            if (e.code === 'Escape' && this.isSettingsOpen()) {
                e.preventDefault();
                e.stopImmediatePropagation(); // prevent main.js handler from seeing this Esc
                this.closeSettings();
                return;
            }
            this._keysDown.add(e.code);
        });
        window.addEventListener('keyup', (e) => {
            this._keysDown.delete(e.code);
        });
        // Clear keys on blur
        window.addEventListener('blur', () => {
            this._keysDown.clear();
        });
    }

    _setupGamepad() {
        window.addEventListener('gamepadconnected', (e) => {
            this.connected = true;
            this.gamepadIndex = e.gamepad.index;
            this.gamepadName = e.gamepad.id;
            console.log(`Gamepad connected: ${e.gamepad.id}`);
            this._buildSettingsUI();
        });
        window.addEventListener('gamepaddisconnected', (e) => {
            if (e.gamepad.index === this.gamepadIndex) {
                this.connected = false;
                this.gamepadIndex = -1;
                this.gamepadName = '';
            }
            this._buildSettingsUI();
        });
    }

    // ---- WebHID Support for RC Transmitters ----

    async connectHID() {
        if (!navigator.hid) {
            alert('WebHID is not supported in this browser. Please use Chrome or Edge.');
            return false;
        }

        try {
            // Request HID device - filter for common RC transmitter vendor IDs
            const devices = await navigator.hid.requestDevice({
                filters: [
                    // RadioMaster transmitters (may vary by model)
                    { vendorId: 0x1209 }, // Generic HID
                    { vendorId: 0x0483 }, // STMicroelectronics (common in RC transmitters)
                    { vendorId: 0x239A }, // Adafruit
                    { vendorId: 0x2341 }, // Arduino
                ],
            });

            if (devices.length === 0) {
                // Try without filter if no device found
                const allDevices = await navigator.hid.requestDevice({ filters: [] });
                if (allDevices.length === 0) {
                    console.log('No HID device selected');
                    return false;
                }
                return this._openHIDDevice(allDevices[0]);
            }

            return this._openHIDDevice(devices[0]);
        } catch (error) {
            reportUserError('HID connection failed', error, {
                key: 'hid-connection',
                intervalMs: 10000,
            });
            return false;
        }
    }

    async _openHIDDevice(device) {
        try {
            if (!device.opened) {
                await device.open();
            }

            this._hidDevice = device;
            this._hidConnected = true;
            this._hidDeviceName = device.productName || 'HID Device';

            console.log('HID Device connected:', this._hidDeviceName);
            console.log('  Vendor ID:', device.vendorId.toString(16));
            console.log('  Product ID:', device.productId.toString(16));
            console.log('  Collections:', device.collections);

            // Set up input report handler
            device.addEventListener('inputreport', (event) => {
                this._handleHIDInputReport(event);
            });

            this._buildSettingsUI();
            return true;
        } catch (error) {
            reportUserError('Failed to open HID device', error, {
                key: 'hid-open-device',
                intervalMs: 10000,
            });
            return false;
        }
    }

    _handleHIDInputReport(event) {
        const { data } = event;
        const bytes = new Uint8Array(data.buffer);

        this._parseHIDReport(bytes);
    }

    _parseHIDReport(bytes) {
        const channelCount = Math.min(16, Math.floor(bytes.length / 2));
        for (let i = 0; i < channelCount; i++) {
            const raw = bytes[i * 2] | (bytes[i * 2 + 1] << 8);
            this._hidRawAxes[i] = raw;
            this._hidAxes[i] = this._applyCalibration(i, raw);
        }
    }

    _applyCalibration(i, raw) {
        const cal = this._hidCalibration[i];
        if (cal.min === null || cal.max === null || cal.center === null) {
            // No calibration: assume 0-2047 range, center 1024
            return Math.max(-1, Math.min(1, (raw - 1024) / 1024));
        }
        const center = cal.center;
        const span = raw >= center
            ? Math.max(1, cal.max - center)
            : Math.max(1, center - cal.min);
        return Math.max(-1, Math.min(1, (raw - center) / span));
    }

    _hasCalibration() {
        return this._hidCalibration.some(c => c.min !== null);
    }

    startCalibration() {
        if (!this._hidConnected) {
            alert('Connect a HID device first.');
            return;
        }

        const calMin = new Array(16).fill(null);
        const calMax = new Array(16).fill(null);

        // ── overlay ──
        const overlay = document.createElement('div');
        Object.assign(overlay.style, {
            position: 'fixed', top: 0, left: 0, width: '100%', height: '100%',
            background: 'rgba(0,0,0,0.8)', zIndex: 20000,
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif",
        });

        const dialog = document.createElement('div');
        Object.assign(dialog.style, {
            background: 'rgba(14,18,28,0.98)', border: '1px solid #f80',
            borderRadius: '12px', padding: '24px', width: '460px',
            color: '#ddd', userSelect: 'none',
        });

        const titleEl = document.createElement('h3');
        Object.assign(titleEl.style, { color: '#f80', margin: '0 0 6px', fontSize: '1.1em' });
        titleEl.textContent = '● Recording Calibration…';

        const instrEl = document.createElement('p');
        Object.assign(instrEl.style, { fontSize: '13px', color: '#aaa', margin: '0 0 14px', lineHeight: '1.5' });
        instrEl.innerHTML = 'Move <b>all sticks and dials</b> to their full extremes in every direction, then center them and click <b>Stop & Save</b>.';

        const barsEl = document.createElement('div');
        Object.assign(barsEl.style, { fontFamily: 'monospace', fontSize: '11px', marginBottom: '14px' });

        const btnRow = document.createElement('div');
        Object.assign(btnRow.style, { display: 'flex', gap: '8px', justifyContent: 'flex-end' });

        const cancelBtn = document.createElement('button');
        cancelBtn.textContent = 'Cancel';
        Object.assign(cancelBtn.style, {
            padding: '7px 18px', background: 'transparent', border: '1px solid #555',
            borderRadius: '6px', color: '#888', cursor: 'pointer', fontSize: '13px',
        });

        const saveBtn = document.createElement('button');
        saveBtn.textContent = 'Stop & Save';
        Object.assign(saveBtn.style, {
            padding: '7px 22px', background: '#f80', border: 'none',
            borderRadius: '6px', color: '#000', cursor: 'pointer', fontSize: '13px', fontWeight: 'bold',
        });

        btnRow.appendChild(cancelBtn);
        btnRow.appendChild(saveBtn);
        dialog.appendChild(titleEl);
        dialog.appendChild(instrEl);
        dialog.appendChild(barsEl);
        dialog.appendChild(btnRow);
        overlay.appendChild(dialog);
        document.body.appendChild(overlay);

        let rafId = null;

        const tick = () => {
            for (let i = 0; i < 16; i++) {
                const raw = this._hidRawAxes[i];
                if (calMin[i] === null || raw < calMin[i]) calMin[i] = raw;
                if (calMax[i] === null || raw > calMax[i]) calMax[i] = raw;
            }

            let html = '';
            for (let i = 0; i < 16; i++) {
                const raw = this._hidRawAxes[i];
                const mn = calMin[i] ?? raw;
                const mx = calMax[i] ?? raw;
                const range = Math.max(1, mx - mn);
                const pct = Math.max(0, Math.min(100, ((raw - mn) / range) * 100));
                const active = (mx - mn) > 10;
                const barColor = active ? '#f80' : '#334';
                html += `<div style="display:flex;align-items:center;gap:4px;margin-bottom:2px;">` +
                    `<span style="width:32px;text-align:right;color:${active ? '#aaa' : '#445'};">CH${i+1}</span>` +
                    `<div style="flex:1;height:8px;background:#1a2030;border-radius:2px;overflow:hidden;">` +
                    `<div style="width:${pct}%;height:100%;background:${barColor};border-radius:2px;"></div></div>` +
                    `<span style="width:90px;text-align:right;color:${active ? '#888' : '#334'};font-size:10px;">${mn}…${mx}</span>` +
                    `</div>`;
            }
            barsEl.innerHTML = html;
            rafId = requestAnimationFrame(tick);
        };
        rafId = requestAnimationFrame(tick);

        saveBtn.addEventListener('click', () => {
            cancelAnimationFrame(rafId);
            // Validate: at least one axis with meaningful range
            const valid = calMin.filter((mn, i) => mn !== null && calMax[i] - mn > 10).length;
            if (valid === 0) {
                instrEl.innerHTML = '<span style="color:#f44">⚠ No movement detected — move sticks first, then click Stop & Save.</span>';
                rafId = requestAnimationFrame(tick);
                return;
            }
            // Center = midpoint of recorded range (works for both self-centering sticks and throttle)
            for (let i = 0; i < 16; i++) {
                const mn = calMin[i]  ?? 0;
                const mx = calMax[i]  ?? 2047;
                this._hidCalibration[i] = {
                    min:    mn,
                    max:    mx,
                    center: Math.round((mn + mx) / 2),
                };
            }
            this._saveConfig();
            this._buildSettingsUI();
            document.body.removeChild(overlay);
        });

        cancelBtn.addEventListener('click', () => {
            cancelAnimationFrame(rafId);
            document.body.removeChild(overlay);
        });
    }

    clearCalibration() {
        this._hidCalibration = Array.from({length: 16}, () => ({ min: null, center: null, max: null }));
        this._saveConfig();
        this._buildSettingsUI();
    }

    disconnectHID() {
        if (this._hidDevice) {
            this._hidDevice.close();
            this._hidDevice = null;
            this._hidConnected = false;
            this._hidDeviceName = '';
            this._hidAxes.fill(0);
            this._buildSettingsUI();
        }
    }

    _getHIDAxes() {
        return this._hidConnected ? this._hidAxes : null;
    }

    isSettingsOpen() {
        const panel = document.getElementById('settings-panel');
        return panel && panel.classList.contains('visible');
    }

    closeSettings() {
        const panel = document.getElementById('settings-panel');
        if (panel) panel.classList.remove('visible');
    }

    _toggleSettings() {
        const panel = document.getElementById('settings-panel');
        panel.classList.toggle('visible');
    }

    _buildSettingsUI() {
        // Race Course is independent of RC state, so render it before the
        // early-return below.
        this._buildRaceCourseSection();

        const container = document.getElementById('channel-assignments');
        if (!container) return;
        container.innerHTML = '';

        for (const action of ACTIONS) {
            const m = this.mapping[action];
            const row = document.createElement('div');
            row.className = 'setting-row';

            const label = document.createElement('label');
            label.textContent = action.charAt(0).toUpperCase() + action.slice(1);
            row.appendChild(label);

            const controls = document.createElement('div');
            controls.className = 'controls';

            // Axis label
            const axisLabel = document.createElement('span');
            axisLabel.className = 'axis-label';
            axisLabel.textContent = m.axisIndex >= 0 ? `Axis ${m.axisIndex}` : 'None';
            controls.appendChild(axisLabel);

            // Invert checkbox
            const invertLabel = document.createElement('label');
            invertLabel.style.fontSize = '12px';
            invertLabel.style.color = '#aaa';
            const invertCb = document.createElement('input');
            invertCb.type = 'checkbox';
            invertCb.checked = m.inverted;
            invertCb.addEventListener('change', () => {
                this.mapping[action].inverted = invertCb.checked;
                this._saveConfig();
            });
            invertLabel.appendChild(invertCb);
            invertLabel.appendChild(document.createTextNode(' Inv'));
            controls.appendChild(invertLabel);

            // Dead zone slider
            const dzLabel = document.createElement('span');
            dzLabel.style.cssText = 'font-size:11px;color:#888;margin-left:6px;';
            dzLabel.textContent = 'DZ';
            controls.appendChild(dzLabel);
            const dzSlider = document.createElement('input');
            dzSlider.type = 'range';
            dzSlider.min = '0'; dzSlider.max = '0.5'; dzSlider.step = '0.01';
            dzSlider.value = m.deadzone;
            dzSlider.style.width = '60px';
            const dzVal = document.createElement('span');
            dzVal.className = 'deadzone-val';
            dzVal.textContent = m.deadzone.toFixed(2);
            dzSlider.addEventListener('input', () => {
                this.mapping[action].deadzone = parseFloat(dzSlider.value);
                dzVal.textContent = parseFloat(dzSlider.value).toFixed(2);
                this._saveConfig();
            });
            controls.appendChild(dzSlider);
            controls.appendChild(dzVal);

            // Assign button
            const assignBtn = document.createElement('button');
            assignBtn.className = 'assign-btn';
            assignBtn.textContent = 'Assign';
            assignBtn.addEventListener('click', () => {
                if (this._listenAction === action) {
                    this.cancelListening();
                    assignBtn.classList.remove('listening');
                    assignBtn.textContent = 'Assign';
                    return;
                }
                // Cancel any other listening
                this.cancelListening();
                document.querySelectorAll('.assign-btn.listening').forEach(b => {
                    b.classList.remove('listening');
                    b.textContent = b._origText || 'Assign';
                });

                const started = this.startListening(action, (a, axis, inverted) => {
                    assignBtn.classList.remove('listening');
                    assignBtn.textContent = 'Assign';
                    axisLabel.textContent = `Axis ${axis}`;
                    invertCb.checked = inverted;
                });
                if (started) {
                    assignBtn.classList.add('listening');
                    assignBtn.textContent = 'Move stick...';
                } else {
                    alert('No gamepad detected. Connect your RC transmitter first.');
                }
            });
            assignBtn._origText = 'Assign';
            controls.appendChild(assignBtn);

            // None button (unassign)
            const noneBtn = document.createElement('button');
            noneBtn.className = 'assign-btn';
            noneBtn.textContent = 'None';
            noneBtn.style.cssText = 'font-size:11px;padding:2px 6px;';
            noneBtn.addEventListener('click', () => {
                this.cancelListening();
                this.mapping[action].axisIndex = -1;
                axisLabel.textContent = 'None';
                this._saveConfig();
            });
            controls.appendChild(noneBtn);

            row.appendChild(controls);
            container.appendChild(row);
        }

        // Rates & Expo section
        this._buildRatesExpoUI();

        // Button assignments (support button or axis source)
        const btnContainer = document.getElementById('button-assignments');
        if (btnContainer) {
            btnContainer.innerHTML = '';
            for (const bAction of BUTTON_ACTIONS) {
                const bm = this.buttonMapping[bAction];
                // Ensure new fields exist (migration from old config)
                if (!bm.source) bm.source = 'button';
                if (bm.axisIndex === undefined) bm.axisIndex = -1;
                if (bm.axisThreshold === undefined) bm.axisThreshold = 0.5;
                if (bm.inverted === undefined) bm.inverted = false;
                if (bm.triggerMode === undefined) bm.triggerMode = 'toggle';

                const row = document.createElement('div');
                row.className = 'setting-row';

                const label = document.createElement('label');
                // Human-readable label; fall back to capitalised action name.
                const LABELS = { arm: 'Arm', modeSwitch: 'Mode Switch' };
                label.textContent = LABELS[bAction] || (bAction.charAt(0).toUpperCase() + bAction.slice(1));
                row.appendChild(label);

                const controls = document.createElement('div');
                controls.className = 'controls';

                // Current assignment label
                const srcLabel = document.createElement('span');
                srcLabel.className = 'axis-label';
                if (bm.source === 'axis') {
                    srcLabel.textContent = `Axis ${bm.axisIndex}`;
                } else {
                    srcLabel.textContent = bm.buttonIndex >= 0 ? `Btn ${bm.buttonIndex}` : 'None';
                }
                controls.appendChild(srcLabel);

                // --- DOM element construction (no DOM insertion yet) -------
                // All children below are appended at the end of this block in
                // the desired visual order:
                //   [Src] [Trigger (axis only)] [Inv] [Assign] [None]
                // srcLabel is appended earlier above; the others are gathered
                // here first so the final append sequence stays readable.

                // Unified Assign button. The user thinks in terms of a single
                // "button" binding; the code picks the right underlying source
                // automatically: HID → axis (switch), Gamepad → whichever of
                // axis-movement or button-press happens first during listen.
                const assignBtn2 = document.createElement('button');
                assignBtn2.className = 'assign-btn';
                assignBtn2.textContent = 'Assign';
                assignBtn2.title = 'Press a button, flick a switch, or move a channel to bind it';
                assignBtn2.addEventListener('click', () => {
                    const isListening = this._listenAction === bAction || this._listenButtonAction === bAction;
                    if (isListening) {
                        this.cancelListening();
                        this.cancelButtonListening();
                        assignBtn2.classList.remove('listening');
                        assignBtn2.textContent = 'Assign';
                        return;
                    }
                    this.cancelListening();
                    this.cancelButtonListening();
                    document.querySelectorAll('.assign-btn.listening').forEach(b => {
                        b.classList.remove('listening');
                        b.textContent = b._origText || 'Assign';
                    });

                    const viaHid = this._hidConnected;
                    const onAxis = (a, axisIdx, inverted) => {
                        bm.source = 'axis';
                        bm.axisIndex = axisIdx;
                        bm.inverted = inverted;
                        this._saveConfig();
                        this._buildSettingsUI();
                    };
                    const onButton = (a, btnIdx) => {
                        bm.source = 'button';
                        bm.buttonIndex = btnIdx;
                        bm.inverted = false;
                        this._saveConfig();
                        this._buildSettingsUI();
                    };

                    let started;
                    if (viaHid) {
                        // HID has no button concept; axis-listen only.
                        started = this.startListening(bAction, onAxis);
                    } else {
                        // Gamepad: arm both axis- and button-listen. The detection
                        // loops cross-cancel, so whichever input the user touches
                        // first decides the binding type.
                        const a = this.startListening(bAction, onAxis);
                        const b = this.startButtonListening(bAction, onButton);
                        started = a || b;
                    }

                    if (started) {
                        assignBtn2.classList.add('listening');
                        assignBtn2.textContent = viaHid ? 'Flick…' : 'Press / Move…';
                    } else {
                        alert('No gamepad or HID device detected.');
                    }
                });
                assignBtn2._origText = 'Assign';

                // Inv toggle — flips which side of the switch / button state
                // counts as "pressed". For an axis source this swaps the
                // high/low end; for a button source it flips active-high ↔
                // active-low (useful if a 3-pos switch is wired inverted).
                const invLabel = document.createElement('label');
                invLabel.style.cssText = 'display:inline-flex;align-items:center;gap:3px;font-size:11px;color:#aaa;margin-left:2px;cursor:pointer;';
                const invCb = document.createElement('input');
                invCb.type = 'checkbox';
                invCb.checked = !!bm.inverted;
                invCb.title = 'Invert: flip which end of the switch counts as pressed';
                invCb.addEventListener('change', () => {
                    bm.inverted = invCb.checked;
                    this._saveConfig();
                });
                invLabel.appendChild(invCb);
                invLabel.appendChild(document.createTextNode('Inv'));

                // None button (unassign)
                const noneBtnB = document.createElement('button');
                noneBtnB.className = 'assign-btn';
                noneBtnB.textContent = 'None';
                noneBtnB.style.cssText = 'font-size:11px;padding:2px 6px;';
                noneBtnB.addEventListener('click', () => {
                    this.cancelButtonListening();
                    this.cancelListening();
                    bm.source = 'button';
                    bm.buttonIndex = -1;
                    bm.axisIndex = -1;
                    bm.inverted = false;
                    srcLabel.textContent = 'None';
                    this._saveConfig();
                    this._buildSettingsUI();
                });

                // Trigger-mode dropdown (axis source only).
                // Toggle: rising edge of the switch flips the action state
                //         (matches a momentary keyboard / gamepad button press).
                // Level:  switch position *is* the action state, re-evaluated
                //         every frame (matches a real 2-position arm switch).
                // Keyboard input always uses toggle regardless of this setting.
                // The axis threshold is kept fixed at its default (0.5) — these
                // are two-state button actions, so a user-tunable threshold has
                // no meaningful effect on behaviour.
                let trigSelect = null;
                if (bm.source === 'axis') {
                    trigSelect = document.createElement('select');
                    trigSelect.style.cssText = 'background:#223;color:#ddd;border:1px solid #446;border-radius:3px;padding:2px 4px;font-size:11px;margin-right:6px;';
                    trigSelect.title = 'Toggle: rising edge flips state. Level: switch position directly drives state.';
                    const optT = document.createElement('option');
                    optT.value = 'toggle'; optT.textContent = 'Toggle';
                    const optL = document.createElement('option');
                    optL.value = 'level'; optL.textContent = 'Level';
                    trigSelect.appendChild(optT);
                    trigSelect.appendChild(optL);
                    trigSelect.value = bm.triggerMode;
                    trigSelect.addEventListener('change', () => {
                        bm.triggerMode = trigSelect.value;
                        this._saveConfig();
                    });
                }

                // --- Final append order: Src | Trigger | Inv | Assign | None
                if (trigSelect) controls.appendChild(trigSelect);
                controls.appendChild(invLabel);
                controls.appendChild(assignBtn2);
                controls.appendChild(noneBtnB);

                row.appendChild(controls);
                btnContainer.appendChild(row);
            }
        }

        // Gamepad status and Chrome WebHID button
        const statusEl = document.getElementById('gamepad-status');
        if (statusEl) {
            let statusHtml = '';
            
            // Disable Gamepad API checkbox
            const disabledChecked = this._gamepadApiDisabled ? 'checked' : '';
            statusHtml += `<div style="margin-bottom:8px;">
                <label style="display:flex;align-items:center;gap:8px;cursor:pointer;">
                    <input type="checkbox" id="disable-gamepad-api" ${disabledChecked}>
                    <span style="color:#fa0;">Disable Gamepad API (use Chrome WebHID)</span>
                </label>
            </div>`;
            
            if (this._hidConnected) {
                statusHtml += `<span style="color:#4f4;">HID Connected: ${this._hidDeviceName}</span>` +
                    `<button id="disconnect-hid-btn" style="margin-left:12px;padding:4px 12px;background:#533;border:1px solid #f44;color:#f44;border-radius:4px;cursor:pointer;font-size:12px;">Disconnect</button>`;
            } else if (this.connected && !this._gamepadApiDisabled) {
                statusHtml += `<span style="color:#4af;">Gamepad: ${this.gamepadName}</span>`;
            } else if (this._gamepadApiDisabled) {
                statusHtml += `<span style="color:#fa0;">Gamepad API disabled - use Chrome WebHID</span>`;
            } else {
                statusHtml += `<span style="color:#888;">No gamepad detected</span>`;
            }
            
            // Always show Connect HID button if not HID connected
            if (!this._hidConnected) {
                statusHtml += `<button id="connect-hid-btn" style="margin-left:12px;padding:4px 12px;background:#335;border:1px solid #4f4;color:#4f4;border-radius:4px;cursor:pointer;font-size:12px;">Connect HID</button>`;
            }

            // Calibration status + buttons (only when HID connected)
            if (this._hidConnected) {
                const calStatus = this._hasCalibration()
                    ? `<span style="color:#4f4;font-size:12px;">✓ Calibrated</span>`
                    : `<span style="color:#fa0;font-size:12px;">⚠ Not calibrated (using defaults)</span>`;
                statusHtml += `<div style="margin-top:10px;display:flex;align-items:center;gap:8px;flex-wrap:wrap;">` +
                    calStatus +
                    `<button id="calibrate-hid-btn" style="padding:4px 12px;background:#223;border:1px solid #4272F5;color:#4272F5;border-radius:4px;cursor:pointer;font-size:12px;">Calibrate…</button>` +
                    (this._hasCalibration() ? `<button id="clear-cal-btn" style="padding:4px 10px;background:transparent;border:1px solid #555;color:#888;border-radius:4px;cursor:pointer;font-size:11px;">Clear Cal</button>` : '') +
                    `</div>`;
            }
            
            statusEl.innerHTML = statusHtml;
            
            // Bind disable checkbox
            const disableCheckbox = document.getElementById('disable-gamepad-api');
            if (disableCheckbox) {
                disableCheckbox.addEventListener('change', (e) => {
                    this._gamepadApiDisabled = e.target.checked;
                    this._buildSettingsUI();
                });
            }
            
            // Bind HID buttons
            const connectBtn = document.getElementById('connect-hid-btn');
            if (connectBtn) {
                connectBtn.addEventListener('click', () => this.connectHID());
            }
            const disconnectBtn = document.getElementById('disconnect-hid-btn');
            if (disconnectBtn) {
                disconnectBtn.addEventListener('click', () => this.disconnectHID());
            }
            const calibrateBtn = document.getElementById('calibrate-hid-btn');
            if (calibrateBtn) {
                calibrateBtn.addEventListener('click', () => this.startCalibration());
            }
            const clearCalBtn = document.getElementById('clear-cal-btn');
            if (clearCalBtn) {
                clearCalBtn.addEventListener('click', () => this.clearCalibration());
            }
        }

        // Settings panel buttons
        this._setupSettingsButtons();
    }

    _buildRatesExpoUI() {
        const container = document.getElementById('rates-expo');
        if (!container) return;
        container.innerHTML = '';

        const RATE_EXPO_AXES = ['roll', 'pitch', 'throttle', 'yaw'];
        const RATE_AXES = ['roll', 'pitch', 'yaw']; // throttle has no rate
        const AXIS_COLORS = {
            roll: '#4af', pitch: '#f44', throttle: '#4f4', yaw: '#fa4'
        };

        for (const action of RATE_EXPO_AXES) {
            const m = this.mapping[action];
            // Ensure rate/expo exist (migration from old config)
            if (m.rate === undefined) m.rate = 1.0;
            if (m.expo === undefined) m.expo = 0.0;

            const row = document.createElement('div');
            row.className = 'setting-row';
            row.style.flexWrap = 'wrap';

            const label = document.createElement('label');
            label.style.minWidth = '80px';
            const dot = document.createElement('span');
            dot.style.cssText = `display:inline-block;width:8px;height:8px;border-radius:50%;background:${AXIS_COLORS[action]};margin-right:6px;`;
            label.appendChild(dot);
            label.appendChild(document.createTextNode(action.charAt(0).toUpperCase() + action.slice(1)));
            row.appendChild(label);

            const controls = document.createElement('div');
            controls.className = 'controls';

            // Rate slider + number input (only for roll/pitch/yaw)
            if (RATE_AXES.includes(action)) {
                const rateLabel = document.createElement('span');
                rateLabel.style.cssText = 'font-size:11px;color:#888;';
                rateLabel.textContent = 'Rate';
                controls.appendChild(rateLabel);
                const rateSlider = document.createElement('input');
                rateSlider.type = 'range';
                rateSlider.min = '0'; rateSlider.max = '10'; rateSlider.step = '0.1';
                rateSlider.value = m.rate;
                rateSlider.style.width = '70px';
                const rateNum = document.createElement('input');
                rateNum.type = 'number';
                rateNum.min = '0'; rateNum.max = '10'; rateNum.step = '0.1';
                rateNum.value = m.rate;
                rateNum.style.cssText = 'width:48px;background:#223;color:#ddd;border:1px solid #446;border-radius:3px;padding:2px 4px;font-size:11px;';
                rateSlider.addEventListener('input', () => {
                    const v = parseFloat(rateSlider.value);
                    this.mapping[action].rate = v;
                    rateNum.value = v;
                    this._saveConfig();
                });
                rateNum.addEventListener('change', () => {
                    const v = Math.max(0, Math.min(10, parseFloat(rateNum.value) || 0));
                    this.mapping[action].rate = v;
                    rateSlider.value = v;
                    rateNum.value = v;
                    this._saveConfig();
                });
                controls.appendChild(rateSlider);
                controls.appendChild(rateNum);
            }

            // Expo slider + number input
            const expoLabel = document.createElement('span');
            expoLabel.style.cssText = `font-size:11px;color:#888;${RATE_AXES.includes(action) ? 'margin-left:8px;' : ''}`;
            expoLabel.textContent = 'Expo';
            controls.appendChild(expoLabel);
            const expoSlider = document.createElement('input');
            expoSlider.type = 'range';
            expoSlider.min = '0'; expoSlider.max = '1.0'; expoSlider.step = '0.05';
            expoSlider.value = m.expo;
            expoSlider.style.width = '70px';
            const expoNum = document.createElement('input');
            expoNum.type = 'number';
            expoNum.min = '0'; expoNum.max = '1'; expoNum.step = '0.05';
            expoNum.value = m.expo;
            expoNum.style.cssText = 'width:48px;background:#223;color:#ddd;border:1px solid #446;border-radius:3px;padding:2px 4px;font-size:11px;';
            expoSlider.addEventListener('input', () => {
                const v = parseFloat(expoSlider.value);
                this.mapping[action].expo = v;
                expoNum.value = v;
                this._saveConfig();
                this._drawExpoCurve(action);
            });
            expoNum.addEventListener('change', () => {
                const v = Math.max(0, Math.min(1, parseFloat(expoNum.value) || 0));
                this.mapping[action].expo = v;
                expoSlider.value = v;
                expoNum.value = v;
                this._saveConfig();
                this._drawExpoCurve(action);
            });
            controls.appendChild(expoSlider);
            controls.appendChild(expoNum);

            row.appendChild(controls);
            container.appendChild(row);
        }

        // Create 4 canvases in the grid container
        const gridContainer = document.getElementById('expo-curves-container');
        if (gridContainer) {
            gridContainer.innerHTML = '';
            for (const action of RATE_EXPO_AXES) {
                const canvas = document.createElement('canvas');
                canvas.id = `expo-curve-${action}`;
                canvas.width = 220;
                canvas.height = 220;
                canvas.style.cssText = 'width:100%;background:#112;border:1px solid #334;border-radius:6px;';
                gridContainer.appendChild(canvas);
                this._drawExpoCurve(action);
            }
        }
    }

    _drawExpoCurve(action) {
        const AXIS_COLORS = {
            roll: '#4af', pitch: '#f44', throttle: '#4f4', yaw: '#fa4'
        };
        const canvas = document.getElementById(`expo-curve-${action}`);
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const W = canvas.width;
        const H = canvas.height;
        const pad = 24;
        const plotW = W - pad * 2;
        const plotH = H - pad * 2;

        ctx.clearRect(0, 0, W, H);

        // Title
        ctx.fillStyle = AXIS_COLORS[action] || '#4af';
        ctx.font = 'bold 11px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(action.charAt(0).toUpperCase() + action.slice(1), W / 2, 14);

        // Grid
        ctx.strokeStyle = '#334';
        ctx.lineWidth = 0.5;
        for (let i = 0; i <= 4; i++) {
            const x = pad + (plotW * i / 4);
            const y = pad + (plotH * i / 4);
            ctx.beginPath(); ctx.moveTo(x, pad); ctx.lineTo(x, pad + plotH); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(pad + plotW, y); ctx.stroke();
        }

        // Tick labels (-1, 0, 1)
        ctx.fillStyle = '#555';
        ctx.font = '9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('-1', pad, pad + plotH + 12);
        ctx.fillText('0', pad + plotW / 2, pad + plotH + 12);
        ctx.fillText('1', pad + plotW, pad + plotH + 12);
        ctx.textAlign = 'right';
        ctx.fillText('-1', pad - 4, pad + plotH + 3);
        ctx.fillText('0', pad - 4, pad + plotH / 2 + 3);
        ctx.fillText('1', pad - 4, pad + 3);

        // Linear reference line (dashed)
        ctx.strokeStyle = '#444';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(pad, pad + plotH);
        ctx.lineTo(pad + plotW, pad);
        ctx.stroke();
        ctx.setLineDash([]);

        // Expo curve (pure expo, no rate scaling)
        const m = this.mapping[action];
        const expo = m.expo || 0;

        ctx.strokeStyle = AXIS_COLORS[action] || '#4af';
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let px = 0; px <= plotW; px++) {
            const input = (px / plotW) * 2 - 1; // -1..1
            const absIn = Math.abs(input);
            const output = Math.sign(input) * absIn * (1 - expo + expo * absIn * absIn);
            const x = pad + px;
            const y = pad + plotH / 2 - output * (plotH / 2);
            if (px === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }

    _updateGamepadDisplay(gp) {
        const el = document.getElementById('gamepad-axes-display');
        if (!el) return;

        let html = '';
        // Axes as bars
        const numAxes = Math.min(gp.axes.length, 8);
        for (let i = 0; i < numAxes; i++) {
            const val = gp.axes[i];
            const pct = ((val + 1) / 2) * 100; // -1..1 → 0..100%
            html += `<div style="display:flex;align-items:center;gap:4px;margin-bottom:2px;">` +
                `<span style="width:24px;text-align:right;color:#aaa;">A${i}</span>` +
                `<div style="flex:1;height:10px;background:#223;border-radius:3px;overflow:hidden;position:relative;">` +
                `<div style="position:absolute;left:50%;top:0;width:1px;height:100%;background:#444;"></div>` +
                `<div style="width:${pct}%;height:100%;background:#4af;border-radius:3px;transition:width 0.05s;"></div>` +
                `</div>` +
                `<span style="width:40px;text-align:right;font-size:10px;">${val.toFixed(2)}</span>` +
                `</div>`;
        }
        // Buttons as bars
        const numBtns = Math.min(gp.buttons.length, 16);
        for (let i = 0; i < numBtns; i++) {
            const val = gp.buttons[i].value;
            const pct = val * 100;
            const color = gp.buttons[i].pressed ? '#4af' : '#335';
            html += `<div style="display:flex;align-items:center;gap:4px;margin-bottom:2px;">` +
                `<span style="width:24px;text-align:right;color:#aaa;">B${i}</span>` +
                `<div style="flex:1;height:10px;background:#223;border-radius:3px;overflow:hidden;">` +
                `<div style="width:${pct}%;height:100%;background:${color};border-radius:3px;transition:width 0.05s;"></div>` +
                `</div>` +
                `<span style="width:40px;text-align:right;font-size:10px;">${val.toFixed(2)}</span>` +
                `</div>`;
        }
        el.innerHTML = html;
    }

    _updateHIDDisplay(hidAxes) {
        const el = document.getElementById('gamepad-axes-display');
        if (!el) return;

        let html = '<div style="color:#4f4;margin-bottom:8px;font-size:12px;">WebHID Mode - Full Precision</div>';
        // Show all HID channels with high precision
        const numChannels = Math.min(hidAxes.length, 8);
        for (let i = 0; i < numChannels; i++) {
            const val = hidAxes[i];
            // Calculate bar position: center is 50%, val -1 to 1 maps to 0% to 100%
            const pct = ((val + 1) / 2) * 100;
            // Color based on magnitude
            const mag = Math.abs(val);
            const barColor = mag > 0.01 ? '#4f4' : '#335';
            html += `<div style="display:flex;align-items:center;gap:4px;margin-bottom:3px;">` +
                `<span style="width:30px;text-align:right;color:#aaa;">CH${i + 1}</span>` +
                `<div style="flex:1;height:12px;background:#223;border-radius:3px;overflow:hidden;position:relative;">` +
                `<div style="position:absolute;left:50%;top:0;width:1px;height:100%;background:#555;"></div>` +
                `<div style="position:absolute;left:${Math.min(pct, 50)}%;width:${Math.abs(pct - 50)}%;height:100%;background:${barColor};"></div>` +
                `</div>` +
                `<span style="width:70px;text-align:right;font-size:11px;font-family:monospace;color:${mag > 0.01 ? '#4f4' : '#666'};">${val >= 0 ? '+' : ''}${val.toFixed(4)}</span>` +
                `</div>`;
        }
        el.innerHTML = html;
    }

    _setupSettingsButtons() {
        const closeBtn = document.getElementById('close-settings-btn');
        if (closeBtn && !closeBtn._bound) {
            closeBtn._bound = true;
            closeBtn.addEventListener('click', () => this._toggleSettings());
        }

        const gearBtn = document.getElementById('gear-btn');
        if (gearBtn && !gearBtn._bound) {
            gearBtn._bound = true;
            gearBtn.addEventListener('click', () => this._toggleSettings());
        }

        const exportBtn = document.getElementById('export-config-btn');
        if (exportBtn && !exportBtn._bound) {
            exportBtn._bound = true;
            exportBtn.addEventListener('click', () => {
                const config = this.getConfig();
                const blob = new Blob([JSON.stringify(config, null, 2)], { type: 'application/json' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = 'drone_controller_config.json';
                a.click();
                URL.revokeObjectURL(url);
            });
        }

        const importBtn = document.getElementById('import-config-btn');
        const importInput = document.getElementById('import-config-input');
        if (importBtn && importInput && !importBtn._bound) {
            importBtn._bound = true;
            importBtn.addEventListener('click', () => importInput.click());
            importInput.addEventListener('change', (e) => {
                const file = e.target.files[0];
                if (!file) return;
                const reader = new FileReader();
                reader.onload = (ev) => {
                    try {
                        const config = JSON.parse(ev.target.result);
                        this.loadConfig(config);
                    } catch (err) {
                        reportUserError('Invalid controller config file', err, { intervalMs: 0 });
                        alert(`Invalid config file: ${err && err.message ? err.message : String(err)}`);
                    }
                };
                reader.readAsText(file);
            });
        }

        // Flight mode select — swap per-mode settings on change.
        //
        // Two guards against accidental mode switches:
        //   1. Block letter-prefix navigation: native <select> lets you jump
        //      to an option by pressing its first letter (D → "Drone", F →
        //      "FPV"). We swallow any key that isn't a navigation key, so
        //      pressing D no longer flips the mode.
        //   2. Blur the select after a commit so that a later arrow-press
        //      on the (focused-but-closed) control can't silently cycle the
        //      value without the dropdown being explicitly re-opened.
        const modeSelect = document.getElementById('flight-mode-select');
        if (modeSelect && !modeSelect._bound) {
            modeSelect._bound = true;
            const ALLOWED_KEYS = new Set([
                'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight',
                'Enter', 'Escape', 'Tab', 'Space',
            ]);
            modeSelect.addEventListener('keydown', (e) => {
                if (!ALLOWED_KEYS.has(e.code)) {
                    e.preventDefault();
                    e.stopPropagation();
                }
            });
            modeSelect.addEventListener('change', () => {
                this._onModeSwitch(modeSelect.value);
                modeSelect.blur();
            });
        }

        // Physics slider+number pairs (bidirectional sync)
        this._bindSliderNum('drone-max-speed', 'drone-max-speed-num');
        this._bindSliderNum('drone-max-vspeed', 'drone-max-vspeed-num');
        this._bindSliderNum('phys-mass', 'phys-mass-num');
        this._bindSliderNum('phys-thrust', 'phys-thrust-num');
        this._bindSliderNum('phys-drag-cd', 'phys-drag-cd-num');
        this._bindSliderNum('phys-drag-area', 'phys-drag-area-num');
        this._bindSliderNum('phys-drone-size', 'phys-drone-size-num');
        this._bindSliderNum('phys-collision-radius', 'phys-collision-radius-num');
        this._bindSliderNum('cam-hfov', 'cam-hfov-num');
        this._bindSliderNum('cam-mount-angle', 'cam-mount-angle-num');

        // Controller gain sliders
        this._bindSliderNum('ctrl-pos-kp', 'ctrl-pos-kp-num');
        this._bindSliderNum('ctrl-pos-ki', 'ctrl-pos-ki-num');
        this._bindSliderNum('ctrl-vel-kp', 'ctrl-vel-kp-num');
        this._bindSliderNum('ctrl-vel-ki', 'ctrl-vel-ki-num');
        this._bindSliderNum('ctrl-alt-kp', 'ctrl-alt-kp-num');
        this._bindSliderNum('ctrl-alt-ki', 'ctrl-alt-ki-num');
        this._bindSliderNum('ctrl-pos-kd', 'ctrl-pos-kd-num');
        this._bindSliderNum('ctrl-vel-kd', 'ctrl-vel-kd-num');
        this._bindSliderNum('ctrl-alt-kd', 'ctrl-alt-kd-num');

        // SO3 geometric controller gains
        this._bindSliderNum('so3-kx', 'so3-kx-num');
        this._bindSliderNum('so3-kv', 'so3-kv-num');
        this._bindSliderNum('so3-kr', 'so3-kr-num');
        this._bindSliderNum('so3-komega', 'so3-komega-num');

        // Display toggle checkboxes
        for (const cbId of ['clean-mode-toggle', 'osd-toggle', 'pano-level-toggle', 'yaw-lock-toggle']) {
            const cb = document.getElementById(cbId);
            if (cb && !cb._bound) {
                cb._bound = true;
                cb.addEventListener('change', () => this._saveConfig());
            }
        }
    }

    _bindSliderNum(sliderId, numId) {
        const slider = document.getElementById(sliderId);
        const numEl = document.getElementById(numId);
        if (slider && numEl && !slider._boundSN) {
            slider._boundSN = true;
            slider.addEventListener('input', () => {
                numEl.value = slider.value;
                this._saveConfig();
            });
            numEl.addEventListener('input', () => {
                const min = parseFloat(slider.min);
                const max = parseFloat(slider.max);
                const v = Math.max(min, Math.min(max, parseFloat(numEl.value) || min));
                slider.value = v;
                numEl.value = v;
                this._saveConfig();
            });
        }
    }

    _bindPhysicsSlider(sliderId, valId) {
        const slider = document.getElementById(sliderId);
        const valEl = document.getElementById(valId);
        if (slider && valEl && !slider._bound) {
            slider._bound = true;
            slider.addEventListener('input', () => {
                valEl.textContent = parseFloat(slider.value).toFixed(slider.step.includes('.') ? 2 : 0);
                this._saveConfig();
            });
        }
    }

    _saveConfig() {
        try {
            localStorage.setItem('drone_sim_controller_config', JSON.stringify(this.getConfig()));
        } catch (e) {
            reportUserError('Controller config save failed', e, {
                key: 'controller-config-save',
                intervalMs: 10000,
            });
        }
    }

    _migrateConfig(config) {
        if (!config || typeof config !== 'object') return {};
        const version = Number(config.configVersion || 1);
        if (version < 2 && config.settings && typeof config.settings === 'object') {
            const settings = config.settings;
            if (Number(settings['drone-max-speed']) === LEGACY_EASY_MAX_SPEED) {
                settings['drone-max-speed'] = DEFAULT_EASY_MAX_SPEED;
            }
            if (Number(settings['drone-max-vspeed']) === LEGACY_EASY_MAX_VSPEED) {
                settings['drone-max-vspeed'] = DEFAULT_EASY_MAX_VSPEED;
            }
        }
        if (version < 3 && config.settings && typeof config.settings === 'object') {
            const settings = config.settings;
            const savedMaxSpeed = Number(settings['drone-max-speed']);
            if (!Number.isFinite(savedMaxSpeed) || savedMaxSpeed <= PREVIOUS_EASY_MAX_SPEED) {
                settings['drone-max-speed'] = DEFAULT_EASY_MAX_SPEED;
            }
            const savedDragArea = Number(settings['phys-drag-area']);
            if (!Number.isFinite(savedDragArea) || savedDragArea === PREVIOUS_DRAG_AREA) {
                settings['phys-drag-area'] = DEFAULT_DRAG_AREA;
            }
        }
        // v4: mass 500→980g, thrust 1000→2600gf (YOPO Hummingbird params)
        if (version < 4 && config.settings && typeof config.settings === 'object') {
            const settings = config.settings;
            const savedMass = Number(settings['phys-mass']);
            if (!Number.isFinite(savedMass) || savedMass <= 510) {
                settings['phys-mass'] = '980';
            }
            const savedThrust = Number(settings['phys-thrust']);
            if (!Number.isFinite(savedThrust) || savedThrust <= 1010) {
                settings['phys-thrust'] = '2600';
            }
        }
        // v5: restore the YOPO training vehicle radius. Only migrate the old
        // shipped default (0.15 m); preserve an explicit user-tuned value.
        if (version < 5 && config.settings && typeof config.settings === 'object') {
            const settings = config.settings;
            const savedRadius = Number(settings['phys-collision-radius']);
            if (!Number.isFinite(savedRadius) || Math.abs(savedRadius - 0.15) < 1e-9) {
                settings['phys-collision-radius'] = '0.6';
            }
        }
        config.configVersion = CONFIG_VERSION;
        return config;
    }

    _loadConfig() {
        try {
            const saved = localStorage.getItem('drone_sim_controller_config');
            if (saved) {
                const config = this._migrateConfig(JSON.parse(saved));
                if (config.mapping) {
                    for (const action of ACTIONS) {
                        if (config.mapping[action]) {
                            this.mapping[action] = { ...this.mapping[action], ...config.mapping[action] };
                        }
                    }
                }
                if (config.buttonMapping) {
                    for (const bAction of BUTTON_ACTIONS) {
                        if (config.buttonMapping[bAction]) {
                            this.buttonMapping[bAction] = { ...this.buttonMapping[bAction], ...config.buttonMapping[bAction] };
                        }
                    }
                }
                if (config.hidCalibration) {
                    this._hidCalibration = config.hidCalibration;
                }
                if (config.modeRateExpo) {
                    this._modeRateExpo = config.modeRateExpo;
                }
                if (config.modePidSettings) {
                    this._modePidSettings = config.modePidSettings;
                }
                if (config.currentMode) {
                    this._currentMode = config.currentMode;
                }
                if (config.settings) {
                    this._restoreSettings(config.settings);
                }
                if (config.gatePathSettings) {
                    this._mergeGatePathSettings(config.gatePathSettings);
                } else if (config.raceCourseSettings) {
                    // Legacy blob — keep gateSize + clearance, drop everything else.
                    this._mergeGatePathSettings(config.raceCourseSettings);
                }
            }
        } catch (e) {
            reportUserError('Controller config load failed', e, {
                key: 'controller-config-load',
                intervalMs: 10000,
            });
        }
    }
}
