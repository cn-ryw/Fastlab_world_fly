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
 * Top-down gate-path editor — modal overlay that lets the user place,
 * drag, delete, and height-adjust a closed-loop chain of gate control
 * points drawn from above on an XZ canvas. Replaces the old
 * region-picker polygon tool.
 *
 * Interaction summary:
 *   - Left-click empty space → append gate at cursor XZ (Y = mid of
 *     yMin/yMax, clamped to range).
 *   - Left-click an existing gate → select it (outlined orange).
 *   - Left-drag selected gate → move its XZ (Y preserved).
 *   - Z / X with selection → lower / raise by 0.1 m (1 m with Shift).
 *   - Delete / Backspace w/ selection → remove gate.
 *   - Backspace w/o selection → undo last-appended gate.
 *   - Enter → Accept (only if ≥ 3 gates).
 *   - Esc → Cancel (discard changes).
 *   - Wheel → zoom around cursor; right-drag → pan.
 *
 * Live clearance: for each gate, query the scene octree with a sphere of
 * radius `gateSize * 0.5` at that gate's CURRENT Y; red fill if any
 * cloud point is inside (frame intersects geometry), green otherwise.
 * The spline turns amber as a whole if any single gate is red.
 *
 * Public API:
 *
 *   const result = await editPath({
 *       octree,              // collision octree for live clearance
 *       bounds,              // { min:[x,y,z], max:[x,y,z] } — filtered scene bounds
 *       spawnPoint,          // { x, y, z } | null
 *       initialPath:         // null OR { closed: true, points: [{x,y,z}, ...], yMin, yMax, gateSize, clearance }
 *       gateSize,            // metres (default size for new gates)
 *       clearance,           // metres (radius used for the live cloud check)
 *   });
 *
 *   result is:
 *       null  — user cancelled
 *       {     — user accepted
 *         closed:    true,
 *         points:    [{x, y, z}, ...],
 *         yMin:      number,
 *         yMax:      number,
 *         gateSize:  number,
 *         clearance: number,
 *       }
 */

import { sampleClosed, tangentAtPoint } from './catmull-rom.js';
import { reportUserError } from './error-report.js';

// ---- Config constants ----
const MAX_CANVAS_PX   = 900;
const POINT_SUBSAMPLE = 500;      // octree backdrop subsampling
const MIN_GATES       = 3;        // need >= 3 for a closed loop
const ZOOM_STEP       = 1.15;
const GATE_PICK_PX    = 10;       // click tolerance for selecting an existing gate
const GATE_DOT_PX     = 8;        // visual half-width of the gate icon

// ---- Main entry ----
export function editPath({ octree, bounds, spawnPoint, initialPath, gateSize, clearance }) {
    return new Promise((resolve) => {
        // --- State ------------------------------------------------

        // Points: always world coordinates {x, y, z}. Tangents computed
        // on-demand via catmull-rom.js — no need to cache per-vertex.
        let points = [];
        if (initialPath && Array.isArray(initialPath.points)) {
            points = initialPath.points.map(p => ({
                x: Number(p.x) || 0, y: Number(p.y) || 0, z: Number(p.z) || 0,
            }));
        }

        // Global editor params (shared across all gates).
        let yMin = initialPath && Number.isFinite(initialPath.yMin) ? initialPath.yMin
                 : (spawnPoint ? spawnPoint.y - 3
                 : (bounds ? ((bounds.min[1] + bounds.max[1]) * 0.5 - 4) : -4));
        let yMax = initialPath && Number.isFinite(initialPath.yMax) ? initialPath.yMax
                 : (spawnPoint ? spawnPoint.y + 5
                 : (bounds ? ((bounds.min[1] + bounds.max[1]) * 0.5 + 4) : 4));
        let curGateSize  = Number.isFinite(gateSize)  ? gateSize  : 1.2;
        let curClearance = Number.isFinite(clearance) ? clearance : 0.8;

        // Viewport state (pixels-per-metre + world-space pan centre).
        let viewX, viewZ, zoom;

        // Interaction state.
        let selectedIdx = -1;
        let dragging    = false;
        let dragOffsetX = 0, dragOffsetZ = 0;
        let panning     = false;
        let panFromX = 0, panFromZ = 0;
        let panMouseX = 0, panMouseY = 0;
        let hoverWorld  = null;          // cursor position in world space (for ghost append preview)

        // Per-gate clearance cache. Recomputed only when points change or
        // yMin/yMax/clearance change — not per mouse-move (redraw uses it
        // but doesn't mutate). Maps idx → true/false (true = clear/green).
        const clearanceCache = [];

        // --- DOM -----------------------------------------------------
        const overlay = document.createElement('div');
        overlay.id = 'path-editor-overlay';
        overlay.style.cssText = `
            position: fixed; inset: 0; z-index: 2000;
            background: rgba(0, 0, 0, 0.88);
            display: flex; flex-direction: column; align-items: center; justify-content: center;
            gap: 10px; font-family: system-ui, sans-serif; color: #ddd;
        `;

        const header = document.createElement('div');
        header.style.cssText = 'max-width: 92%; text-align: center; font-size: 13px; line-height: 1.5; color: #aac;';
        header.innerHTML = `
            <div style="font-size: 16px; font-weight: bold; color: #4df; margin-bottom: 4px;">Edit Gate Path</div>
            <div>
                <b>L-click</b> add / select &nbsp;·&nbsp;
                <b>drag</b> move XZ &nbsp;·&nbsp;
                <b>Z / X</b> lower / raise gate &nbsp;·&nbsp;
                <b>Del</b> remove &nbsp;·&nbsp;
                <b>Backspace</b> undo &nbsp;·&nbsp;
                <b>Enter</b> accept &nbsp;·&nbsp;
                <b>Esc</b> cancel
            </div>
        `;
        overlay.appendChild(header);

        const canvasSize = Math.min(MAX_CANVAS_PX, Math.floor(Math.min(window.innerWidth, window.innerHeight) * 0.72));
        const canvas = document.createElement('canvas');
        canvas.width = canvasSize;
        canvas.height = canvasSize;
        canvas.style.cssText = 'background: #111; border: 1px solid #446; cursor: crosshair;';
        overlay.appendChild(canvas);
        const ctx = canvas.getContext('2d');

        // Y-range + gate-size + clearance sliders row.
        const controlsRow = document.createElement('div');
        controlsRow.style.cssText = 'display: flex; flex-wrap: wrap; gap: 18px; align-items: center; font-size: 13px;';
        overlay.appendChild(controlsRow);

        // Bottom row: status + buttons.
        const bottom = document.createElement('div');
        bottom.style.cssText = 'display: flex; gap: 12px; align-items: center;';
        const statusEl = document.createElement('span');
        statusEl.style.cssText = 'font-size: 12px; color: #888; min-width: 300px;';
        bottom.appendChild(statusEl);

        const mkBtn = (text, color, onClick) => {
            const b = document.createElement('button');
            b.textContent = text;
            b.style.cssText = `
                background: ${color}; color: #fff; border: none; border-radius: 4px;
                padding: 6px 18px; font-size: 13px; cursor: pointer; font-weight: 600;
            `;
            b.addEventListener('click', onClick);
            return b;
        };
        const cancelBtn = mkBtn('Cancel', '#444', () => finish(null));
        const acceptBtn = mkBtn('Accept', '#2a7a3a', () => {
            if (points.length < MIN_GATES) return;
            finish({
                closed:    true,
                points:    points.map(p => ({ x: p.x, y: p.y, z: p.z })),
                yMin, yMax,
                gateSize:  curGateSize,
                clearance: curClearance,
            });
        });
        bottom.append(cancelBtn, acceptBtn);
        overlay.appendChild(bottom);
        document.body.appendChild(overlay);

        // --- Slider factory ------------------------------------------
        const mkSlider = (label, unit, initial, step, sliderMin, sliderMax, onChange) => {
            const wrap = document.createElement('div');
            wrap.style.cssText = 'display: flex; align-items: center; gap: 6px;';
            const lbl = document.createElement('span');
            lbl.textContent = label;
            const slider = document.createElement('input');
            slider.type = 'range';
            slider.min = String(sliderMin);
            slider.max = String(sliderMax);
            slider.step = String(step);
            slider.value = String(initial);
            slider.style.width = '140px';
            const num = document.createElement('input');
            num.type = 'number';
            num.step = String(step);
            num.value = Number(initial).toFixed(step < 1 ? 1 : 0);
            num.style.cssText = 'width: 60px; background: #223; color: #ddd; border: 1px solid #446; border-radius: 3px; padding: 2px 4px;';
            const unitLbl = document.createElement('span');
            unitLbl.textContent = unit;
            unitLbl.style.color = '#888';
            const sync = (v) => {
                slider.value = String(v);
                num.value = Number(v).toFixed(step < 1 ? 1 : 0);
                onChange(v);
            };
            slider.addEventListener('input', () => {
                const v = parseFloat(slider.value);
                if (Number.isFinite(v)) sync(v);
            });
            num.addEventListener('change', () => {
                const v = parseFloat(num.value);
                if (Number.isFinite(v)) sync(v);
            });
            wrap.append(lbl, slider, num, unitLbl);
            return wrap;
        };

        const yCenter = (yMin + yMax) * 0.5;
        const yLo = yCenter - 50;
        const yHi = yCenter + 50;
        controlsRow.appendChild(mkSlider('y min', 'm', yMin, 0.1, yLo, yHi, (v) => {
            yMin = Math.min(v, yMax);
            clampAllGateY();
            recomputeClearance();
            render();
        }));
        controlsRow.appendChild(mkSlider('y max', 'm', yMax, 0.1, yLo, yHi, (v) => {
            yMax = Math.max(v, yMin);
            clampAllGateY();
            recomputeClearance();
            render();
        }));
        controlsRow.appendChild(mkSlider('size', 'm', curGateSize, 0.1, 0.4, 5.0, (v) => {
            curGateSize = v;
            recomputeClearance();
            render();
        }));
        controlsRow.appendChild(mkSlider('clearance', 'm', curClearance, 0.1, 0.1, 3.0, (v) => {
            curClearance = v;
            recomputeClearance();
            render();
        }));

        // --- Coordinate conversions ----------------------------------
        const centerPx = canvasSize / 2;
        const worldToCanvas = (x, z) => ({
            x: centerPx + (x - viewX) * zoom,
            y: centerPx + (z - viewZ) * zoom,
        });
        const canvasToWorld = (px, py) => ({
            x: viewX + (px - centerPx) / zoom,
            z: viewZ + (py - centerPx) / zoom,
        });
        const eventToCanvas = (e) => {
            const rect = canvas.getBoundingClientRect();
            return { x: e.clientX - rect.left, y: e.clientY - rect.top };
        };

        // --- Initial viewport fit -----------------------------------
        if (bounds) {
            const bw = Math.max(1, bounds.max[0] - bounds.min[0]);
            const bh = Math.max(1, bounds.max[2] - bounds.min[2]);
            viewX = (bounds.max[0] + bounds.min[0]) * 0.5;
            viewZ = (bounds.max[2] + bounds.min[2]) * 0.5;
            zoom = Math.min(canvasSize / bw, canvasSize / bh) * 0.9;
        } else {
            viewX = 0; viewZ = 0; zoom = 10;
        }
        if (spawnPoint) {
            viewX = spawnPoint.x;
            viewZ = spawnPoint.z;
            zoom = Math.max(zoom, canvasSize / 160);
        }
        if (points.length > 0) {
            // Fit to path with a bit of padding.
            let mnX = points[0].x, mxX = points[0].x, mnZ = points[0].z, mxZ = points[0].z;
            for (const p of points) {
                if (p.x < mnX) mnX = p.x; if (p.x > mxX) mxX = p.x;
                if (p.z < mnZ) mnZ = p.z; if (p.z > mxZ) mxZ = p.z;
            }
            const cw = Math.max(1, mxX - mnX);
            const ch = Math.max(1, mxZ - mnZ);
            viewX = (mxX + mnX) * 0.5;
            viewZ = (mxZ + mnZ) * 0.5;
            zoom = Math.min(canvasSize / (cw * 1.4), canvasSize / (ch * 1.4));
            zoom = Math.max(zoom, canvasSize / 160);
        }

        // --- Clamp every gate's Y into current [yMin, yMax] --------
        function clampAllGateY() {
            for (const p of points) {
                if (p.y < yMin) p.y = yMin;
                if (p.y > yMax) p.y = yMax;
            }
        }

        // --- Live clearance check (cache) ---------------------------
        function recomputeClearance() {
            clearanceCache.length = 0;
            if (!octree) return;
            const r = Math.max(curGateSize * 0.5, curClearance);
            for (let i = 0; i < points.length; i++) {
                const p = points[i];
                // Clear (green) iff no cloud point sits inside the sphere.
                let hitCount = 0;
                try {
                    hitCount = typeof octree.querySphereCount === 'function'
                        ? octree.querySphereCount(p.x, p.y, p.z, r)
                        : octree.querySphere(p.x, p.y, p.z, r).length;
                } catch (error) {
                    reportUserError('Path editor clearance query failed', error, {
                        key: 'path-editor-clearance',
                        intervalMs: 10000,
                    });
                    hitCount = 0;
                }
                clearanceCache[i] = hitCount === 0;
            }
        }

        // --- Rendering ----------------------------------------------
        function render() {
            ctx.fillStyle = '#0a0a12';
            ctx.fillRect(0, 0, canvasSize, canvasSize);

            // Faint grid (10 m / 50 m / 200 m depending on zoom)
            if (zoom > 0.5) {
                ctx.strokeStyle = '#1a1a28';
                ctx.lineWidth = 1;
                const step = zoom > 8 ? 10 : (zoom > 2 ? 50 : 200);
                const x0 = Math.floor((viewX - centerPx / zoom) / step) * step;
                const x1 = viewX + centerPx / zoom;
                const z0 = Math.floor((viewZ - centerPx / zoom) / step) * step;
                const z1 = viewZ + centerPx / zoom;
                ctx.beginPath();
                for (let gx = x0; gx <= x1; gx += step) {
                    const cp = worldToCanvas(gx, 0);
                    ctx.moveTo(cp.x, 0); ctx.lineTo(cp.x, canvasSize);
                }
                for (let gz = z0; gz <= z1; gz += step) {
                    const cp = worldToCanvas(0, gz);
                    ctx.moveTo(0, cp.y); ctx.lineTo(canvasSize, cp.y);
                }
                ctx.stroke();
            }

            // Cloud points (only those within [yMin, yMax]).
            ctx.fillStyle = 'rgba(140, 180, 220, 0.55)';
            if (octree && octree.positions) {
                const pos = octree.positions;
                const n = octree.pointCount;
                for (let i = 0; i < n; i += POINT_SUBSAMPLE) {
                    const py = pos[i * 3 + 1];
                    if (py < yMin || py > yMax) continue;
                    const cp = worldToCanvas(pos[i * 3], pos[i * 3 + 2]);
                    if (cp.x < 0 || cp.x >= canvasSize || cp.y < 0 || cp.y >= canvasSize) continue;
                    ctx.fillRect(cp.x, cp.y, 1, 1);
                }
            }

            // Filtered bounds outline.
            if (bounds) {
                const a = worldToCanvas(bounds.min[0], bounds.min[2]);
                const b = worldToCanvas(bounds.max[0], bounds.max[2]);
                ctx.strokeStyle = '#335';
                ctx.lineWidth = 1;
                ctx.strokeRect(a.x, a.y, b.x - a.x, b.y - a.y);
            }

            // Spawn marker.
            if (spawnPoint) {
                const s = worldToCanvas(spawnPoint.x, spawnPoint.z);
                ctx.fillStyle = '#4af';
                ctx.beginPath();
                ctx.arc(s.x, s.y, 5, 0, Math.PI * 2);
                ctx.fill();
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 1;
                ctx.stroke();
                ctx.font = 'bold 11px system-ui';
                ctx.fillStyle = '#4af';
                ctx.textAlign = 'left';
                ctx.fillText('spawn', s.x + 7, s.y - 7);
            }

            // Draw smooth Catmull-Rom preview curve (only if >= 3 pts).
            if (points.length >= MIN_GATES) {
                const samples = sampleClosed(points, 20);
                const anyRed = clearanceCache.some(v => v === false);
                ctx.strokeStyle = anyRed ? 'rgba(255, 170, 60, 0.9)' : 'rgba(80, 220, 120, 0.9)';
                ctx.lineWidth = 2;
                ctx.beginPath();
                for (let i = 0; i < samples.length; i++) {
                    const cp = worldToCanvas(samples[i].pos.x, samples[i].pos.z);
                    if (i === 0) ctx.moveTo(cp.x, cp.y);
                    else ctx.lineTo(cp.x, cp.y);
                }
                ctx.closePath();
                ctx.stroke();
            } else if (points.length === 2) {
                // Dashed line for the degenerate case.
                const a = worldToCanvas(points[0].x, points[0].z);
                const b = worldToCanvas(points[1].x, points[1].z);
                ctx.strokeStyle = 'rgba(255, 200, 60, 0.6)';
                ctx.setLineDash([5, 5]);
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
                ctx.stroke();
                ctx.setLineDash([]);
            }

            // Gate markers (squares + index + tangent arrow).
            for (let i = 0; i < points.length; i++) {
                const p = points[i];
                const cp = worldToCanvas(p.x, p.z);
                const isSel = i === selectedIdx;
                const clear = clearanceCache[i] !== false;  // undefined → treat as clear
                const fill   = clear ? 'rgba(80, 220, 120, 0.85)' : 'rgba(230, 70, 70, 0.85)';
                const border = isSel ? '#ffd000' : '#fff';

                ctx.fillStyle = fill;
                ctx.strokeStyle = border;
                ctx.lineWidth = isSel ? 2.5 : 1.2;
                ctx.fillRect(cp.x - GATE_DOT_PX / 2, cp.y - GATE_DOT_PX / 2, GATE_DOT_PX, GATE_DOT_PX);
                ctx.strokeRect(cp.x - GATE_DOT_PX / 2, cp.y - GATE_DOT_PX / 2, GATE_DOT_PX, GATE_DOT_PX);

                // Index label.
                ctx.font = 'bold 11px system-ui';
                ctx.fillStyle = '#fff';
                ctx.textAlign = 'left';
                ctx.fillText(String(i), cp.x + GATE_DOT_PX, cp.y - GATE_DOT_PX);

                // Tangent arrow: show the direction the drone passes through
                // this gate (computed from closed-loop Catmull-Rom tangent).
                if (points.length >= MIN_GATES) {
                    const td = tangentAtPoint(points, i);
                    // Project tangent onto XZ plane for the top-down arrow.
                    const arrowLen = 16;
                    const mag = Math.hypot(td.x, td.z) || 1;
                    const ax = cp.x + (td.x / mag) * arrowLen;
                    const ay = cp.y + (td.z / mag) * arrowLen;
                    ctx.strokeStyle = isSel ? '#ffd000' : 'rgba(200, 200, 255, 0.8)';
                    ctx.lineWidth = 1.5;
                    ctx.beginPath();
                    ctx.moveTo(cp.x, cp.y); ctx.lineTo(ax, ay);
                    ctx.stroke();
                    // Small arrowhead.
                    const ang = Math.atan2(ay - cp.y, ax - cp.x);
                    const head = 5;
                    ctx.beginPath();
                    ctx.moveTo(ax, ay);
                    ctx.lineTo(ax - head * Math.cos(ang - 0.5), ay - head * Math.sin(ang - 0.5));
                    ctx.moveTo(ax, ay);
                    ctx.lineTo(ax - head * Math.cos(ang + 0.5), ay - head * Math.sin(ang + 0.5));
                    ctx.stroke();
                }

                // Selected: show "y = x.xx m" label under the gate.
                if (isSel) {
                    ctx.font = '11px system-ui';
                    ctx.fillStyle = '#ffd000';
                    ctx.fillText(`y = ${p.y.toFixed(2)} m`, cp.x + GATE_DOT_PX, cp.y + GATE_DOT_PX + 10);
                }
            }

            // Ghost preview of append position (when no gate is selected
            // and cursor is over empty space).
            if (!dragging && hoverWorld && selectedIdx === -1) {
                const cp = worldToCanvas(hoverWorld.x, hoverWorld.z);
                ctx.strokeStyle = 'rgba(120, 200, 255, 0.5)';
                ctx.setLineDash([3, 3]);
                ctx.strokeRect(cp.x - GATE_DOT_PX / 2, cp.y - GATE_DOT_PX / 2, GATE_DOT_PX, GATE_DOT_PX);
                ctx.setLineDash([]);
            }

            // Scale indicator (bottom-right).
            const scaleM = 50 / zoom;
            const scaleText = scaleM < 1 ? scaleM.toFixed(2) + ' m'
                            : scaleM < 10 ? scaleM.toFixed(1) + ' m'
                            : Math.round(scaleM) + ' m';
            ctx.fillStyle = '#888';
            ctx.font = '11px system-ui';
            ctx.textAlign = 'left';
            ctx.fillRect(canvasSize - 70, canvasSize - 18, 50, 2);
            ctx.fillText(scaleText, canvasSize - 70, canvasSize - 22);

            // Status line.
            const n = points.length;
            const redCount = clearanceCache.filter(v => v === false).length;
            let st;
            if (n === 0) st = 'click to drop the first gate';
            else if (n < MIN_GATES) st = `${n} / ${MIN_GATES} gates — need ${MIN_GATES - n} more to close the loop`;
            else if (redCount > 0) st = `${n} gates · ${redCount} red (cloud intersects gate frame)`;
            else st = `${n} gates · all clear`;
            statusEl.textContent = st;

            acceptBtn.style.opacity = (n >= MIN_GATES) ? '1' : '0.4';
            acceptBtn.style.cursor  = (n >= MIN_GATES) ? 'pointer' : 'not-allowed';
        }

        // --- Hit-testing --------------------------------------------
        function hitTestGate(px, py) {
            // Reverse-iterate so later gates (drawn on top) win ties.
            for (let i = points.length - 1; i >= 0; i--) {
                const cp = worldToCanvas(points[i].x, points[i].z);
                if (Math.abs(px - cp.x) <= GATE_PICK_PX && Math.abs(py - cp.y) <= GATE_PICK_PX) return i;
            }
            return -1;
        }

        // --- Mouse handlers -----------------------------------------
        canvas.addEventListener('mousedown', (e) => {
            e.preventDefault();
            if (e.button === 2 || e.button === 1) {
                panning = true;
                panFromX = viewX; panFromZ = viewZ;
                panMouseX = e.clientX; panMouseY = e.clientY;
                canvas.style.cursor = 'grabbing';
                return;
            }
            if (e.button !== 0) return;

            const { x, y } = eventToCanvas(e);
            const idx = hitTestGate(x, y);
            if (idx >= 0) {
                // Select + begin drag.
                selectedIdx = idx;
                dragging = true;
                const w = canvasToWorld(x, y);
                dragOffsetX = points[idx].x - w.x;
                dragOffsetZ = points[idx].z - w.z;
            } else {
                // Empty space: deselect, then append a new gate at cursor.
                selectedIdx = -1;
                const w = canvasToWorld(x, y);
                const yNew = Math.max(yMin, Math.min(yMax, (yMin + yMax) * 0.5));
                points.push({ x: w.x, y: yNew, z: w.z });
                selectedIdx = points.length - 1;
                recomputeClearance();
            }
            render();
        });
        canvas.addEventListener('mousemove', (e) => {
            const { x, y } = eventToCanvas(e);
            if (panning) {
                viewX = panFromX - (e.clientX - panMouseX) / zoom;
                viewZ = panFromZ - (e.clientY - panMouseY) / zoom;
                render();
                return;
            }
            if (dragging && selectedIdx >= 0) {
                const w = canvasToWorld(x, y);
                points[selectedIdx].x = w.x + dragOffsetX;
                points[selectedIdx].z = w.z + dragOffsetZ;
                recomputeClearance();  // Y unchanged, XZ changed — but re-run so curve-neighbour gates redraw tangents
                render();
                return;
            }
            hoverWorld = canvasToWorld(x, y);
            // Only redraw the ghost cursor when no gate is selected (cheap).
            if (selectedIdx === -1) render();
        });
        canvas.addEventListener('mouseup', (e) => {
            if (e.button === 2 || e.button === 1) {
                panning = false;
                canvas.style.cursor = 'crosshair';
            }
            if (e.button === 0 && dragging) {
                dragging = false;
            }
        });
        canvas.addEventListener('contextmenu', (e) => e.preventDefault());
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const { x, y } = eventToCanvas(e);
            const pre = canvasToWorld(x, y);
            zoom *= (e.deltaY < 0) ? ZOOM_STEP : (1 / ZOOM_STEP);
            zoom = Math.max(0.01, Math.min(zoom, 500));
            const post = canvasToWorld(x, y);
            viewX += pre.x - post.x;
            viewZ += pre.z - post.z;
            render();
        }, { passive: false });

        // --- Keyboard handler ----------------------------------------
        function onKey(e) {
            // Block keystrokes from leaking to the page behind.
            // Use an allow-list so typing into inputs still works.
            const isTyping = e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA');
            if (e.key === 'Escape') {
                e.preventDefault();
                finish(null);
                return;
            }
            if (e.key === 'Enter') {
                e.preventDefault();
                if (points.length >= MIN_GATES) acceptBtn.click();
                return;
            }
            if (isTyping) return;  // leave the slider number inputs alone

            if (e.key === 'Backspace' || e.key === 'Delete') {
                e.preventDefault();
                if (selectedIdx >= 0) {
                    points.splice(selectedIdx, 1);
                    selectedIdx = -1;
                } else if (points.length > 0 && e.key === 'Backspace') {
                    points.pop();
                }
                recomputeClearance();
                render();
                return;
            }
            if (selectedIdx >= 0 && (e.key === 'z' || e.key === 'Z' || e.key === 'x' || e.key === 'X')) {
                e.preventDefault();
                const stepMag = e.shiftKey ? 1.0 : 0.1;
                const dir = (e.key === 'x' || e.key === 'X') ? +1 : -1;  // x raises, z lowers
                const p = points[selectedIdx];
                p.y = Math.max(yMin, Math.min(yMax, p.y + dir * stepMag));
                recomputeClearance();
                render();
                return;
            }
        }
        window.addEventListener('keydown', onKey, true);

        // --- Teardown -----------------------------------------------
        function finish(result) {
            window.removeEventListener('keydown', onKey, true);
            try {
                document.body.removeChild(overlay);
            } catch (error) {
                reportUserError('Path editor close failed', error, {
                    key: 'path-editor-close',
                    intervalMs: 10000,
                });
            }
            resolve(result);
        }

        // First paint.
        recomputeClearance();
        render();
    });
}
