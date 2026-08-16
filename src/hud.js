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
 * HUD overlay — updates HTML elements with drone telemetry, FPS, controller status.
 */

import { formatLap } from './gates.js';

/**
 * Render a race-panel readout string as a sequence of fixed-width
 * `<span>` slots, so the whole line stays rock-still even when the
 * active font is proportional (Orbitron, Rajdhani, …). Without this
 * the millisecond digits in `01:40.718` would shift every frame
 * because '8' and '1' render at different widths and the line is
 * `text-align: center`.
 *
 * Three slot widths cover every character we actually emit:
 *   digits / dashes       → 0.60 em (widest slot, keeps 0-9 + '-' aligned)
 *   colons / periods      → 0.30 em (narrow punctuation)
 *   spaces / slashes      → 0.45 em (for the GATE "1 / 16" separator)
 *
 * Returns a ready-to-assign HTML string. Input is assumed to be one of
 * our own format outputs (digits / ':' / '.' / '-' / ' ' / '/') so we
 * don't bother HTML-escaping — the small allow-list rules out the
 * injection vectors on the way in.
 */
function _monoDigits(str) {
    let out = '';
    for (let i = 0; i < str.length; i++) {
        const c = str[i];
        let cls;
        if (c === ':' || c === '.')      cls = 'hrp-char hrp-char-sep';
        else if (c === ' ' || c === '/') cls = 'hrp-char hrp-char-sp';
        else                              cls = 'hrp-char hrp-char-d';
        out += `<span class="${cls}">${c}</span>`;
    }
    return out;
}

export class HUD {
    constructor() {
        this.altitudeEl = document.getElementById('hud-altitude');
        this.vspeedEl = document.getElementById('hud-vspeed');
        this.gspeedEl = document.getElementById('hud-gspeed');
        this.speedCommandEl = document.getElementById('hud-speed-command');
        this.speedCommandLabelEl = document.getElementById('hud-speed-command-label');
        this.speedHintEl = document.getElementById('hud-speed-hint');
        this.fpsEl = document.getElementById('hud-fps');
        this.controllerEl = document.getElementById('hud-controller');
        this.collisionWarnEl = document.getElementById('hud-collision-warn');
        this.armedEl = document.getElementById('armed-indicator');
        this.collisionFlashEl = document.getElementById('collision-flash');
        this.hudContainer = document.getElementById('hud');

        // F1-style race timing panel (left side of the viewport).
        this.racePanelEl     = document.getElementById('hud-race-panel');
        this.hrpLapTimeEl    = document.getElementById('hrp-lap-time');
        this.hrpBestTimeEl   = document.getElementById('hrp-best-time');
        this.hrpGateProgEl   = document.getElementById('hrp-gate-progress');

        // FPS tracking
        this._frameTimes = [];
        this._lastTime = performance.now();
        this._lastDomUpdate = 0;
        this._lastFpsDisplay = 0;
        this._lastDisplayedFps = 0;
        this._collisionFlashTimer = 0;

        // Race HUD flash: briefly tint green + enlarge on each gate-pass.
        // The timer counts down in milliseconds inside update().
        this._raceFlashTimer = 0;
        this._raceLastPassed = 0;
    }

    show() {
        this.hudContainer.classList.remove('hidden');
    }

    hide() {
        this.hudContainer.classList.add('hidden');
    }

    /**
     * Update HUD each frame.
     * @param {Drone} drone
     * @param {Controller} controller
     * @param {GateCourse} [gateCourse] - optional; when present and
     *   enabled with at least one placed gate, renders a
     *   "Gate X / N" progress readout in the bottom-center HUD zone.
     */
    update(drone, controller, gateCourse) {
        const now = performance.now();
        const dt = now - this._lastTime;
        this._lastTime = now;

        // FPS calculation (rolling average)
        this._frameTimes.push(dt);
        if (this._frameTimes.length > 60) this._frameTimes.shift();
        const avgDt = this._frameTimes.reduce((a, b) => a + b, 0) / this._frameTimes.length;
        const fps = Math.round(1000 / avgDt);
        const forceDomUpdate = drone.isColliding || this._collisionFlashTimer > 0;
        const shouldUpdateDom = forceDomUpdate || now - this._lastDomUpdate >= 100;
        const shouldUpdateFps = now - this._lastFpsDisplay >= 250 ||
            Math.abs(fps - this._lastDisplayedFps) >= 5;

        if (this.fpsEl && shouldUpdateFps) {
            this.fpsEl.textContent = fps;
            this._lastDisplayedFps = fps;
            this._lastFpsDisplay = now;
        }
        if (!shouldUpdateDom) return;
        this._lastDomUpdate = now;

        // Update values
        if (this.altitudeEl) this.altitudeEl.textContent = drone.y.toFixed(1);
        if (this.vspeedEl) {
            this.vspeedEl.textContent = drone.verticalSpeed.toFixed(1);
            this.vspeedEl.style.color = drone.verticalSpeed < -2 ? '#f80' : '#0f0';
        }
        const groundSpeed = Number.isFinite(drone.groundSpeed) ? drone.groundSpeed : drone.speed;
        const airSpeed = Number.isFinite(drone.airSpeed) ? drone.airSpeed : groundSpeed;
        const targetSpeed = Number.isFinite(drone.targetGroundSpeed)
            ? drone.targetGroundSpeed
            : (Number.isFinite(drone.commandedGroundSpeed) ? drone.commandedGroundSpeed : 0);
        const so3ReferenceSpeed = Number.isFinite(drone.commandedGroundSpeed)
            ? drone.commandedGroundSpeed
            : 0;
        const pilotCommandSpeed = Number.isFinite(drone.pilotGroundSpeedCommand)
            ? drone.pilotGroundSpeedCommand
            : targetSpeed;
        const effectiveMaxSpeed = Number.isFinite(drone.effectiveMaxSpeed) ? drone.effectiveMaxSpeed : 0;

        if (this.gspeedEl) {
            this.gspeedEl.innerHTML = `${groundSpeed.toFixed(1)}<br><span class="hud-kmh">${Math.round(groundSpeed * 3.6)} km/h</span>`;
        }
        if (this.speedCommandEl) {
            if (drone.flightMode === 'drone') {
                this.speedCommandEl.innerHTML =
                    `${targetSpeed.toFixed(1)} / ${effectiveMaxSpeed.toFixed(1)}<br>` +
                    `<span class="hud-kmh">${Math.round(targetSpeed * 3.6)} / ${Math.round(effectiveMaxSpeed * 3.6)} km/h</span>`;
            } else if (drone.flightMode === 'so3') {
                this.speedCommandEl.innerHTML =
                    `${groundSpeed.toFixed(1)} ACT / ${so3ReferenceSpeed.toFixed(1)} REF<br>` +
                    `<span class="hud-kmh">${Math.round(groundSpeed * 3.6)} / ${Math.round(so3ReferenceSpeed * 3.6)} km/h</span>`;
            } else {
                const throttlePct = Number.isFinite(drone.throttlePercent) ? Math.round(drone.throttlePercent * 100) : 0;
                this.speedCommandEl.innerHTML =
                    `${airSpeed.toFixed(1)} AIR / ${throttlePct}% THR<br>` +
                    `<span class="hud-kmh">${Math.round(airSpeed * 3.6)} km/h air</span>`;
            }
        }
        if (this.speedCommandLabelEl) {
            const labels = {
                drone: 'TGT / LIM (m/s)',
                fpv: 'AIR SPD / THR',
                stabilized: 'AIR SPD / THR',
                so3: 'ACT / POLY5 REF (m/s)',
            };
            this.speedCommandLabelEl.textContent = labels[drone.flightMode] || 'FAILSAFE';
        }
        if (this.speedHintEl) {
            let hint = '';
            if (drone.flightMode === 'drone') {
                if (pilotCommandSpeed < 0.5 && groundSpeed < 1.0) {
                    hint = 'Easy forward uses ↑/↓ or pitch stick. Hold Shift for Boost.';
                } else if (effectiveMaxSpeed < 83 && groundSpeed > effectiveMaxSpeed * 0.7) {
                    hint = 'Raise Tab > Easy Max Speed for faster Easy flight.';
                } else if (drone.boostActive) {
                    hint = effectiveMaxSpeed >= 83 ? 'Boost active; speed is capped at 300 km/h.' : 'Boost active';
                }
            } else if (drone.flightMode === 'fpv') {
                if (groundSpeed < 2.0 && (drone.throttlePercent || 0) > 0.65) {
                    hint = 'FPV needs nose-down pitch to turn thrust into forward speed.';
                }
            } else if (drone.flightMode === 'stabilized') {
                hint = 'Level: roll/pitch self-level; W/S is manual thrust—pilot holds altitude.';
            } else if (drone.flightMode === 'so3') {
                const nominalCruise = Number.isFinite(drone.so3CruiseMps)
                    ? drone.so3CruiseMps
                    : 15;
                hint = `SO3 YOPO Auto: Cruise-${nominalCruise.toFixed(0)} is policy context, not a speed setpoint.`;
            } else {
                hint = 'Invalid flight mode: controller is in failsafe.';
            }
            this.speedHintEl.textContent = hint;
        }
        // Controller status
        if (this.controllerEl) {
            if (controller.connected) {
                const name = controller.gamepadName || 'Gamepad';
                this.controllerEl.textContent = name.includes('(HID)') ? 'HID Connected' : 'Gamepad';
                this.controllerEl.style.color = '#4272F5';
                this.controllerEl.title = name;
            } else {
                this.controllerEl.textContent = 'Keyboard';
                this.controllerEl.style.color = '#0f0';
                this.controllerEl.title = 'Keyboard';
            }
        }

        // Armed indicator
        if (this.armedEl) {
            if (controller.armed) {
                this.armedEl.textContent = 'ARMED';
                this.armedEl.className = 'armed';
            } else {
                this.armedEl.textContent = 'DISARMED';
                this.armedEl.className = 'disarmed';
            }
        }

        // Collision warning
        if (drone.isColliding) {
            this._collisionFlashTimer = 150; // ms
            if (this.collisionWarnEl) this.collisionWarnEl.style.display = 'block';
        } else {
            if (this.collisionWarnEl) this.collisionWarnEl.style.display = 'none';
        }

        // Collision flash effect
        if (this._collisionFlashTimer > 0) {
            this._collisionFlashTimer -= dt;
            if (this.collisionFlashEl) {
                this.collisionFlashEl.classList.add('active');
                this.collisionFlashEl.style.opacity = Math.min(1, drone.collisionIntensity * 0.5);
            }
        } else {
            if (this.collisionFlashEl) {
                this.collisionFlashEl.classList.remove('active');
            }
        }

        // F1-style race timing panel on the left side. Hidden unless
        // the gate course is visible AND the path has >= 3 gates. Shows
        // current lap number + live clock, the session best (gold), and
        // the gate-progress counter (chartreuse, pulses on each pass).
        const hasCourse = !!(gateCourse &&
                             typeof gateCourse.isVisible === 'function' &&
                             gateCourse.isVisible() &&
                             gateCourse.gates && gateCourse.gates.length >= 3);

        if (this.racePanelEl) {
            if (!hasCourse) {
                this.racePanelEl.style.display = 'none';
                this._raceLastPassed = 0;
                this._raceFlashTimer = 0;
            } else {
                this.racePanelEl.style.display = 'block';

                const total         = gateCourse.gates.length;
                const passed        = gateCourse.passedCount();
                const lapStartedYet = gateCourse.lapStart != null;
                const lapMs         = lapStartedYet ? gateCourse.currentLapMs : 0;
                const bestMs        = gateCourse.bestLapMs;

                // Current lap time — big central readout. Dimmed when
                // the timer hasn't started yet. Wrapped in fixed-width
                // character slots (see `_monoDigits`) so the ms ticks
                // don't make the line jitter left-right under a
                // proportional font.
                if (this.hrpLapTimeEl) {
                    this.hrpLapTimeEl.innerHTML = _monoDigits(lapStartedYet
                        ? formatLap(lapMs)
                        : '--:--.---');
                    this.hrpLapTimeEl.classList.toggle('idle', !lapStartedYet);
                }

                // Best lap — purple when we have one, dashed placeholder
                // otherwise. Placeholder uses the same `mm:ss.mmm` width
                // as formatLap() so the row geometry doesn't shift when
                // the first lap is set. The .empty class dims the colour
                // and kills the glow so the slots obviously read "no PB
                // on this layout yet".
                if (this.hrpBestTimeEl) {
                    if (bestMs != null) {
                        this.hrpBestTimeEl.innerHTML = _monoDigits(formatLap(bestMs));
                        this.hrpBestTimeEl.classList.remove('empty');
                    } else {
                        this.hrpBestTimeEl.innerHTML = _monoDigits('--:--.---');
                        this.hrpBestTimeEl.classList.add('empty');
                    }
                }

                // Gate progress — briefly enlarged + tinted cyan on each
                // clean pass; fades back to white over ~400 ms.
                if (this.hrpGateProgEl) {
                    if (passed > this._raceLastPassed) this._raceFlashTimer = 400;
                    this._raceLastPassed = passed;

                    // Split into two spans so CSS can keep the passed
                    // gate count bright while dimming the separator
                    // and the total (matching the F1 'LAP 36/52' look).
                    // Each span's inner text is further broken into
                    // fixed-width character slots by `_monoDigits` so a
                    // one-digit → two-digit transition (e.g. gate 9 →
                    // gate 10) doesn't shift the line horizontally.
                    // The passed count is also left-padded with
                    // spaces to the total's digit count so the string
                    // length itself is stable across the whole lap.
                    const totalStr = String(total);
                    const currStr  = String(passed).padStart(totalStr.length, ' ');
                    this.hrpGateProgEl.innerHTML =
                        `<span class="hrp-gate-curr">${_monoDigits(currStr)}</span>` +
                        `<span class="hrp-gate-total">${_monoDigits(' / ' + totalStr)}</span>`;
                    if (this._raceFlashTimer > 0) {
                        this._raceFlashTimer -= dt;
                        const t = Math.max(0, this._raceFlashTimer) / 400;
                        // Blend white (idle) → cyan (#4dfcff) at peak.
                        const r = Math.round(255 + (77  - 255) * t);
                        const g = Math.round(255 + (252 - 255) * t);
                        const b = 255;
                        this.hrpGateProgEl.style.color     = `rgb(${r}, ${g}, ${b})`;
                        this.hrpGateProgEl.style.transform = `scale(${1 + 0.18 * t})`;
                    } else {
                        this.hrpGateProgEl.style.color     = '';
                        this.hrpGateProgEl.style.transform = 'scale(1)';
                    }
                }
            }
        }
    }
}
