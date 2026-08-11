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
 * FPV OSD (On-Screen Display) — canvas-based overlay drawn during flight mode.
 * Renders artificial horizon, pitch ladder, altitude/speed tapes, heading compass,
 * vertical speed indicator, flight mode, and armed status.
 */

const DEG2RAD = Math.PI / 180;

export class OSD {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
        this.enabled = true;

        // Colors
        this.color = '#00ff00';
        this.dimColor = 'rgba(0,255,0,0.4)';
        this.bgColor = 'rgba(0,0,0,0.25)';
        this.warnColor = '#ff4444';
        this.skyColor = 'rgba(50,120,220,0.15)';
        this.groundColor = 'rgba(120,80,30,0.15)';
    }

    setEnabled(val) {
        this.enabled = val;
        if (this.canvas) this.canvas.style.display = val ? 'block' : 'none';
    }

    update(drone, controller) {
        if (!this.enabled || !this.ctx || !this.canvas) return;

        // Resize canvas to match display
        const dpr = window.devicePixelRatio || 1;
        const w = this.canvas.clientWidth;
        const h = this.canvas.clientHeight;
        if (this.canvas.width !== w * dpr || this.canvas.height !== h * dpr) {
            this.canvas.width = w * dpr;
            this.canvas.height = h * dpr;
            this.ctx.scale(dpr, dpr);
        }

        const ctx = this.ctx;
        ctx.clearRect(0, 0, w, h);

        ctx.font = '12px "Courier New", monospace';
        ctx.textBaseline = 'middle';

        const bodyPitch = Number.isFinite(drone.bodyPitch) ? drone.bodyPitch : 0;
        const bodyRoll = Number.isFinite(drone.bodyRoll) ? drone.bodyRoll : 0;
        const groundSpeed = Number.isFinite(drone.groundSpeed) ? drone.groundSpeed : drone.speed;

        this._drawHorizon(ctx, w, h, bodyPitch, bodyRoll);
        this._drawSpeedTape(ctx, w, h, groundSpeed);
        this._drawAltTape(ctx, w, h, drone.y);
        this._drawHeading(ctx, w, h, drone.yaw);
        this._drawVSI(ctx, w, h, drone.verticalSpeed);
        this._drawFlightInfo(ctx, w, h, drone, controller);
    }

    // ---- Artificial Horizon + Pitch Ladder ----
    _drawHorizon(ctx, w, h, pitch, roll) {
        const cx = w / 2;
        const cy = h / 2;
        const horizonW = w * 0.22;
        const pxPerDeg = h / 60; // pixels per degree of pitch

        ctx.save();
        ctx.translate(cx, cy);
        ctx.rotate(roll * DEG2RAD);

        const pitchOffset = pitch * pxPerDeg;

        // Horizon line
        ctx.strokeStyle = this.color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-horizonW, pitchOffset);
        ctx.lineTo(-30, pitchOffset);
        ctx.moveTo(30, pitchOffset);
        ctx.lineTo(horizonW, pitchOffset);
        ctx.stroke();

        // Pitch ladder
        ctx.lineWidth = 1;
        ctx.font = '10px "Courier New", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';

        const ladderAngles = [-30, -20, -10, -5, 5, 10, 20, 30];
        for (const deg of ladderAngles) {
            const y = pitchOffset - deg * pxPerDeg;
            if (Math.abs(y) > h * 0.4) continue; // clip off-screen

            const ladderW = Math.abs(deg) % 10 === 0 ? 50 : 25;
            ctx.strokeStyle = deg > 0 ? this.color : this.dimColor;
            ctx.beginPath();

            if (deg > 0) {
                // Above horizon — solid
                ctx.moveTo(-ladderW, y);
                ctx.lineTo(ladderW, y);
            } else {
                // Below horizon — dashed
                ctx.setLineDash([4, 4]);
                ctx.moveTo(-ladderW, y);
                ctx.lineTo(ladderW, y);
                ctx.setLineDash([]);
            }
            ctx.stroke();

            // Degree labels
            ctx.fillStyle = this.dimColor;
            ctx.fillText(`${deg}`, -ladderW - 18, y);
            ctx.fillText(`${deg}`, ladderW + 18, y);
        }

        ctx.restore();

        // Center reticle (fixed)
        ctx.strokeStyle = this.color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        // Left wing
        ctx.moveTo(cx - 50, cy);
        ctx.lineTo(cx - 20, cy);
        ctx.lineTo(cx - 20, cy + 6);
        // Right wing
        ctx.moveTo(cx + 50, cy);
        ctx.lineTo(cx + 20, cy);
        ctx.lineTo(cx + 20, cy + 6);
        // Center dot
        ctx.moveTo(cx + 3, cy);
        ctx.arc(cx, cy, 3, 0, Math.PI * 2);
        ctx.stroke();
    }

    // ---- Speed Tape (left side) ----
    _drawSpeedTape(ctx, w, h, speed) {
        const tapeX = w * 0.22;
        const tapeH = h * 0.4;
        const tapeW = 55;
        const cy = h / 2;
        const pxPerUnit = tapeH / 20; // 20 m/s visible range

        ctx.save();
        ctx.beginPath();
        ctx.rect(tapeX - tapeW / 2, cy - tapeH / 2, tapeW, tapeH);
        ctx.clip();

        // Background
        ctx.fillStyle = this.bgColor;
        ctx.fillRect(tapeX - tapeW / 2, cy - tapeH / 2, tapeW, tapeH);

        // Tick marks
        ctx.strokeStyle = this.dimColor;
        ctx.fillStyle = this.dimColor;
        ctx.lineWidth = 1;
        ctx.font = '10px "Courier New", monospace';
        ctx.textAlign = 'right';
        ctx.textBaseline = 'middle';

        const startVal = Math.floor(speed - 10);
        const endVal = Math.ceil(speed + 10);
        for (let v = startVal; v <= endVal; v++) {
            if (v < 0) continue;
            const y = cy - (v - speed) * pxPerUnit;
            if (v % 5 === 0) {
                ctx.beginPath();
                ctx.moveTo(tapeX + tapeW / 2, y);
                ctx.lineTo(tapeX + tapeW / 2 - 10, y);
                ctx.stroke();
                ctx.fillText(`${v}`, tapeX + tapeW / 2 - 14, y);
            } else {
                ctx.beginPath();
                ctx.moveTo(tapeX + tapeW / 2, y);
                ctx.lineTo(tapeX + tapeW / 2 - 5, y);
                ctx.stroke();
            }
        }

        ctx.restore();

        // Current value box
        ctx.fillStyle = 'rgba(0,0,0,0.7)';
        ctx.fillRect(tapeX - tapeW / 2, cy - 10, tapeW, 20);
        ctx.strokeStyle = this.color;
        ctx.lineWidth = 1;
        ctx.strokeRect(tapeX - tapeW / 2, cy - 10, tapeW, 20);
        ctx.fillStyle = this.color;
        ctx.font = '13px "Courier New", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(speed.toFixed(1), tapeX, cy);

        // Label
        ctx.fillStyle = this.dimColor;
        ctx.font = '10px "Courier New", monospace';
        ctx.fillText('GSPD', tapeX, cy - tapeH / 2 - 13);
        ctx.fillText('m/s', tapeX, cy - tapeH / 2 - 2);
    }

    // ---- Altitude Tape (right side) ----
    _drawAltTape(ctx, w, h, alt) {
        const tapeX = w * 0.78;
        const tapeH = h * 0.4;
        const tapeW = 55;
        const cy = h / 2;
        const pxPerUnit = tapeH / 40; // 40m visible range

        ctx.save();
        ctx.beginPath();
        ctx.rect(tapeX - tapeW / 2, cy - tapeH / 2, tapeW, tapeH);
        ctx.clip();

        // Background
        ctx.fillStyle = this.bgColor;
        ctx.fillRect(tapeX - tapeW / 2, cy - tapeH / 2, tapeW, tapeH);

        // Tick marks
        ctx.strokeStyle = this.dimColor;
        ctx.fillStyle = this.dimColor;
        ctx.lineWidth = 1;
        ctx.font = '10px "Courier New", monospace';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';

        const startVal = Math.floor(alt - 20);
        const endVal = Math.ceil(alt + 20);
        for (let v = startVal; v <= endVal; v++) {
            const y = cy - (v - alt) * pxPerUnit;
            if (v % 5 === 0) {
                ctx.beginPath();
                ctx.moveTo(tapeX - tapeW / 2, y);
                ctx.lineTo(tapeX - tapeW / 2 + 10, y);
                ctx.stroke();
                ctx.fillText(`${v}`, tapeX - tapeW / 2 + 14, y);
            } else {
                ctx.beginPath();
                ctx.moveTo(tapeX - tapeW / 2, y);
                ctx.lineTo(tapeX - tapeW / 2 + 5, y);
                ctx.stroke();
            }
        }

        ctx.restore();

        // Current value box
        ctx.fillStyle = 'rgba(0,0,0,0.7)';
        ctx.fillRect(tapeX - tapeW / 2, cy - 10, tapeW, 20);
        ctx.strokeStyle = this.color;
        ctx.lineWidth = 1;
        ctx.strokeRect(tapeX - tapeW / 2, cy - 10, tapeW, 20);
        ctx.fillStyle = this.color;
        ctx.font = '13px "Courier New", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(alt.toFixed(1), tapeX, cy);

        // Label
        ctx.fillStyle = this.dimColor;
        ctx.font = '10px "Courier New", monospace';
        ctx.fillText('ALT', tapeX, cy - tapeH / 2 - 10);
    }

    // ---- Heading Compass (top center) ----
    _drawHeading(ctx, w, h, yaw) {
        const cx = w / 2;
        const barY = h * 0.08;
        const barW = w * 0.22;
        const barH = 22;
        const pxPerDeg = barW / 90; // 90° visible range

        // Simulator yaw=0 faces local -Z (south); compass 0 is true north.
        let heading = (((yaw + 180) % 360) + 360) % 360;

        ctx.save();
        ctx.beginPath();
        ctx.rect(cx - barW / 2, barY - barH / 2, barW, barH);
        ctx.clip();

        // Background
        ctx.fillStyle = this.bgColor;
        ctx.fillRect(cx - barW / 2, barY - barH / 2, barW, barH);

        // Tick marks and labels
        ctx.strokeStyle = this.dimColor;
        ctx.fillStyle = this.dimColor;
        ctx.lineWidth = 1;
        ctx.font = '10px "Courier New", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';

        const cardinals = { 0: 'N', 45: 'NE', 90: 'E', 135: 'SE', 180: 'S', 225: 'SW', 270: 'W', 315: 'NW' };

        for (let d = -180; d <= 540; d += 5) {
            const deg = ((d % 360) + 360) % 360;
            let diff = d - heading;
            // Wrap diff
            if (diff > 180) diff -= 360;
            if (diff < -180) diff += 360;
            const x = cx + diff * pxPerDeg;

            if (d % 10 === 0) {
                ctx.beginPath();
                ctx.moveTo(x, barY + barH / 2);
                ctx.lineTo(x, barY + barH / 2 - 6);
                ctx.stroke();
            }

            if (cardinals[deg] !== undefined) {
                ctx.fillStyle = deg === 0 ? '#ff4444' : this.color;
                ctx.font = '11px "Courier New", monospace';
                ctx.fillText(cardinals[deg], x, barY - barH / 2 + 2);
                ctx.fillStyle = this.dimColor;
                ctx.font = '10px "Courier New", monospace';
            } else if (d % 30 === 0) {
                ctx.fillText(`${deg}`, x, barY - barH / 2 + 2);
            }
        }

        ctx.restore();

        // Center indicator triangle
        ctx.fillStyle = this.color;
        ctx.beginPath();
        ctx.moveTo(cx, barY + barH / 2 + 2);
        ctx.lineTo(cx - 5, barY + barH / 2 + 8);
        ctx.lineTo(cx + 5, barY + barH / 2 + 8);
        ctx.closePath();
        ctx.fill();

        // Heading value box
        ctx.fillStyle = 'rgba(0,0,0,0.7)';
        ctx.fillRect(cx - 22, barY + barH / 2 + 8, 44, 16);
        ctx.strokeStyle = this.color;
        ctx.strokeRect(cx - 22, barY + barH / 2 + 8, 44, 16);
        ctx.fillStyle = this.color;
        ctx.font = '12px "Courier New", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(`${Math.round(heading)}°`, cx, barY + barH / 2 + 16);
    }

    // ---- Vertical Speed Indicator (far right) ----
    _drawVSI(ctx, w, h, vs) {
        const x = w * 0.85;
        const cy = h / 2;
        const barH = h * 0.3;
        const barW = 8;

        // Background
        ctx.fillStyle = this.bgColor;
        ctx.fillRect(x - barW / 2, cy - barH / 2, barW, barH);
        ctx.strokeStyle = this.dimColor;
        ctx.strokeRect(x - barW / 2, cy - barH / 2, barW, barH);

        // Center line
        ctx.strokeStyle = this.dimColor;
        ctx.beginPath();
        ctx.moveTo(x - barW / 2 - 3, cy);
        ctx.lineTo(x + barW / 2 + 3, cy);
        ctx.stroke();

        // VS indicator bar
        const maxVS = 10; // m/s max display
        const clampedVS = Math.max(-maxVS, Math.min(maxVS, vs));
        const barLen = (clampedVS / maxVS) * (barH / 2);
        ctx.fillStyle = vs > 0 ? this.color : this.warnColor;
        ctx.fillRect(x - barW / 2 + 1, cy - barLen, barW - 2, barLen);

        // Label + value
        ctx.fillStyle = this.dimColor;
        ctx.font = '10px "Courier New", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('VS', x, cy - barH / 2 - 10);
        ctx.fillStyle = vs < -2 ? this.warnColor : this.color;
        ctx.fillText(vs.toFixed(1), x, cy + barH / 2 + 10);
    }

    // ---- Flight Info (bottom) ----
    _drawFlightInfo(ctx, w, h, drone, controller) {
        const mode = drone && drone.flightMode ? drone.flightMode : 'drone';
        const armed = !!(controller && controller.armed);
        const y = h * 0.92;
        ctx.font = '12px "Courier New", monospace';
        ctx.textBaseline = 'middle';

        // Flight mode (left)
        const modeLabels = {
            drone: 'EASY',
            fpv: 'FPV',
            stabilized: 'LEVEL',
            so3: 'SO3 AUTO',
        };
        ctx.textAlign = 'left';
        ctx.fillStyle = this.color;
        ctx.fillText(`MODE: ${modeLabels[mode] || 'FAILSAFE'}`, w * 0.15, y);

        // Armed status (right)
        ctx.textAlign = 'right';
        ctx.fillStyle = armed ? this.color : this.warnColor;
        ctx.fillText(armed ? 'ARMED' : 'DISARMED', w * 0.85, y);

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
        const maxSpeed = Number.isFinite(drone.effectiveMaxSpeed) ? drone.effectiveMaxSpeed : 0;
        const throttlePct = Number.isFinite(drone.throttlePercent) ? Math.round(drone.throttlePercent * 100) : 0;

        let cue;
        if (mode === 'drone') {
            if (pilotCommandSpeed < 0.5 && groundSpeed < 1.0) {
                cue = 'EASY: UP/DOWN or pitch stick = forward speed | Shift = Boost | W/S = altitude';
            } else {
                cue = `GSPD ${groundSpeed.toFixed(1)}  TGT ${targetSpeed.toFixed(1)}  LIM ${maxSpeed.toFixed(0)}${drone.boostActive ? ' BOOST' : ''}`;
            }
        } else if (mode === 'fpv' && groundSpeed < 2.0 && throttlePct > 65) {
            cue = 'FPV: pitch nose down to convert motor thrust into forward speed';
        } else if (mode === 'fpv') {
            cue = `GSPD ${groundSpeed.toFixed(1)}  AIR ${airSpeed.toFixed(1)}  THR ${throttlePct}%${drone.boostActive ? ' BOOST' : ''}`;
        } else if (mode === 'stabilized') {
            cue = 'LEVEL: roll/pitch = self-level angle | W/S = manual thrust | pilot holds altitude';
        } else if (mode === 'so3') {
            const nominalCruise = Number.isFinite(drone.so3CruiseMps)
                ? drone.so3CruiseMps
                : 15;
            cue = `SO3 AUTO: GSPD ${groundSpeed.toFixed(1)}  POLY5 ${so3ReferenceSpeed.toFixed(1)}  ` +
                `Cruise-${nominalCruise.toFixed(0)} policy context | C cancel`;
        } else {
            cue = 'FAILSAFE: invalid flight mode';
        }

        ctx.textAlign = 'center';
        ctx.fillStyle = this.dimColor;
        ctx.font = '11px "Courier New", monospace';
        if (ctx.measureText(cue).width > w * 0.64) {
            ctx.font = '10px "Courier New", monospace';
        }
        ctx.fillText(cue, w * 0.5, h * 0.86);
    }
}
