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
 * Race-course subsystem — user-drawn, closed-loop gate track with
 * centripetal Catmull-Rom spline smoothing.
 *
 * Design summary:
 *
 *   - Deterministic, non-random: the course is EXACTLY the ordered list
 *     of points the user placed in the path editor. No retries, no seed,
 *     no RNG. Per-map JSON persistence is handled outside this module
 *     (see `src/path-store.js`).
 *   - Tangent-oriented: each gate's plane normal equals the Catmull-Rom
 *     tangent at that control point, so the drone naturally flies through
 *     each gate along the smooth curve direction.
 *   - Closed loop: the first gate doubles as start/finish line for lap
 *     timing (Phase B). Segments connect N-1 → 0 naturally.
 *   - Sensor only: gates don't collide with the drone. They exist purely
 *     as visual landmarks and pass-through triggers.
 *   - G-key gated: entities exist as soon as `rebuild()` runs but are
 *     hidden until `setVisible(true)` is called (typically by main.js
 *     on a `G` keypress).
 *
 * Lap timing (Phase B):
 *
 *   - A lap starts the first time the drone crosses gate 0 in the
 *     positive tangent direction.
 *   - Gates 1..N-1 must be crossed in order. Crossing the wrong gate, or
 *     missing one, flags the current lap as "missed" — the drone can
 *     keep flying but no lap time will be recorded this loop.
 *   - Crossing gate 0 again ends the lap. If not missed, we record
 *     `lapMs = now - lapStart`, update `bestLapMs`, and fire the
 *     `onBestLap(newBestMs)` callback so main.js can persist it.
 *   - `R` reset (via `resetLap()`): clears current-lap progress and
 *     timer, but keeps bestLap and the path itself.
 *
 * Lifecycle (from main.js):
 *
 *   gateCourse = new GateCourse();
 *   gateCourse.configure({ gateSize, clearance });
 *   gateCourse.rebuild({ app, octree, points });   // on path-editor Accept / scene load
 *   gateCourse.setVisible(true);                   // on G key
 *   gateCourse.update(dt, drone, nowMs);           // every flight frame
 *   gateCourse.resetLap();                         // on R reset
 *   gateCourse.destroy();                          // on ESC → loading
 */

import { evaluateClosed, tangentAtPoint, sampleClosed } from './catmull-rom.js';
import { reportUserError } from './error-report.js';

const DEFAULT_OPTS = {
    gateSize:  1.2,    // metres — square gate, edge length
    clearance: 0.8,    // metres — used only by the editor's red/green preview
};

// Gate colors — emissive-only (no lighting contribution), chosen for
// high contrast against bright city/tile backgrounds.
//   next       → the gate you must fly next (yellow, pulses)
//   upcoming   → any other gate within the horizon window (cyan)
// Gate 0 (start) carries a checker-flag texture instead of a solid
// colour — see `getCheckerTexture` below.
const COL_NEXT     = { r: 1.00, g: 0.84, b: 0.00 }; // yellow
const COL_UPCOMING = { r: 0.00, g: 0.75, b: 1.00 }; // cyan

// How many gates ahead of `nextGateIdx` remain visible (inclusive of
// the next gate itself). Gates behind the drone naturally fall outside
// this window via the wrap-around distance, and so do gates far ahead,
// keeping the cockpit view clean while still previewing the first few
// gates of the next lap as the current lap winds down.
const HORIZON = 5;

// Emissive strength used by every gate material. Anything below ~3 gets hard
// to read on bright textured scenes.
const EMISSIVE_INTENSITY = 1.5;

// ------------------------------------------------------------------
// Procedural canvas textures (start-gate checker & gate-0 START sign)
// ------------------------------------------------------------------
//
// Both textures are built from HTMLCanvasElements on first use so no
// binary assets ship with the module. Each is cached at module scope
// so the same pc.Texture is reused across rebuilds within a session.

/**
 * 2×2 black/white checker. One pixel per square so the pattern stays
 * razor-sharp under NEAREST filtering; per-edge materials control how
 * many times the texture repeats along each bar so the squares end up
 * roughly equal-size regardless of the bar's aspect ratio.
 */
let _checkerTex = null;
function getCheckerTexture(app) {
    if (_checkerTex) return _checkerTex;
    if (!app || !app.graphicsDevice) return null;
    /* global pc */
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = 2;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#ffffff'; ctx.fillRect(0, 0, 1, 1); ctx.fillRect(1, 1, 1, 1);
    ctx.fillStyle = '#000000'; ctx.fillRect(1, 0, 1, 1); ctx.fillRect(0, 1, 1, 1);
    const tex = new pc.Texture(app.graphicsDevice, {
        format: pc.PIXELFORMAT_RGBA8,
        width: 2, height: 2,
        mipmaps: false,
        addressU: pc.ADDRESS_REPEAT,
        addressV: pc.ADDRESS_REPEAT,
        magFilter: pc.FILTER_NEAREST,
        minFilter: pc.FILTER_NEAREST,
    });
    tex.setSource(canvas);
    _checkerTex = tex;
    return tex;
}

/**
 * "START" sign panel texture — bold black letters on a white plate with
 * a thin black border so the label reads at a distance when glowing
 * under the high emissive intensity.
 */
let _startLabelTex = null;
function getStartLabelTexture(app) {
    if (_startLabelTex) return _startLabelTex;
    if (!app || !app.graphicsDevice) return null;
    /* global pc */
    const W = 512, H = 128;
    const canvas = document.createElement('canvas');
    canvas.width = W; canvas.height = H;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#ffffff'; ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = '#000000'; ctx.lineWidth = 6; ctx.strokeRect(3, 3, W - 6, H - 6);
    ctx.fillStyle = '#000000';
    ctx.font = 'bold 96px "Arial Black", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('START', W / 2, H / 2 + 4);
    const tex = new pc.Texture(app.graphicsDevice, {
        format: pc.PIXELFORMAT_RGBA8,
        width: W, height: H,
        mipmaps: true,
        addressU: pc.ADDRESS_CLAMP_TO_EDGE,
        addressV: pc.ADDRESS_CLAMP_TO_EDGE,
        magFilter: pc.FILTER_LINEAR,
        minFilter: pc.FILTER_LINEAR_MIPMAP_LINEAR,
    });
    tex.setSource(canvas);
    _startLabelTex = tex;
    return tex;
}

// Pulse animation (next gate only): +/- 8 % scale @ 0.8 Hz — roughly
// one breath every 1.25 s, calmer than the earlier 2 Hz throb.
const PULSE_AMP = 0.08;
const PULSE_HZ  = 0.8;

// Fake-lighting multipliers baked into each face's emissive intensity.
// A gate frame is a square ring, and from any viewing angle the visible
// surfaces fall into exactly four categories — every bar contributes
// one face to each category, and the whole category shares a single
// brightness so the ring reads as "one continuous surface lit from a
// consistent direction" instead of four boxes lit independently.
//
//   front → faces the drone on approach (brightest, catches light)
//   back  → opposite side of the ring (deepest shadow)
//   outer → the outward-pointing rim of the ring (side-lit)
//   inner → the rim facing the ring's opening (AO-darkened by the lip)
//
// Because materials are unlit (`useLighting=false`, needed to punch
// through bright textured backgrounds) these multipliers are the only 3D
// cue we can bake in. End caps on the horizontal bars inherit the
// outer brightness since they visually blend with the outer corners.
const FACE_BRIGHTNESS = {
    front: 1.00,
    back:  1.00,
    outer: 0.40,
    inner: 0.40,
};

// START sign offset above the gate-0 frame (in gate-local units,
// multiplied by the gate's edge length). Pushed well clear of the top
// bar so the label no longer visually crowds the frame; the surrounding
// pulse animation counter-translates this value every frame so the sign
// sits at a constant world-height even as the frame breathes.
const START_SIGN_GAP = 0.50;

export class GateCourse {
    constructor() {
        this.opts = { ...DEFAULT_OPTS };

        /**
         * @type {Array<{
         *   pos: {x:number,y:number,z:number},
         *   travelDir: {x:number,y:number,z:number},
         *   passed: boolean,
         *   entity: any,
         *   material: any,
         *   invWorld: any,
         * }>}
         */
        this.gates = [];
        this.nextGateIdx = 0;

        // Private state
        this._rootEntity = null;
        this._pulseTime = 0;
        this._prevDronePos = null;
        this._visible = false;

        // Lap-timing state (Phase B). Strict-in-order rule (see
        // `_handleCrossing`) means an out-of-order cross is silently
        // ignored, so there is no "missed lap" concept — either you
        // flew the gates in sequence, or the lap simply hasn't
        // completed yet.
        this.lapStart      = null;   // ms timestamp when gate 0 was last crossed cleanly
        this.lapCount      = 0;      // completed laps this session
        this.bestLapMs     = null;   // best lap this session OR loaded from disk
        this.currentLapMs  = 0;      // live for HUD

        // Callbacks wired by main.js.
        this.onGatePassed = null;
        this.onLapComplete = null;   // (lapMs, isBest) → void; main.js persists best
        this.onBestLap    = null;    // (newBestMs) → void; subset of onLapComplete when isBest
    }

    /** Merge new options. Does not rebuild by itself. */
    configure(opts) {
        this.opts = { ...this.opts, ...(opts || {}) };
    }

    /** Restore a previously-persisted best lap (before the first rebuild). */
    setBestLapMs(ms) {
        this.bestLapMs = (Number.isFinite(Number(ms)) && Number(ms) > 0) ? Number(ms) : null;
    }

    /**
     * Rebuild the gate entities from an ordered list of user-placed
     * control points. Destroys any existing gates first.
     *
     * @param {{
     *   app:     any,                           // pc.Application
     *   octree?: any,                           // present, unused — kept for symmetry with old API
     *   points:  Array<{x:number,y:number,z:number}>,   // >= 3 for a valid closed loop
     * }} ctx
     */
    rebuild(ctx) {
        this.destroy();

        const points = Array.isArray(ctx.points) ? ctx.points : [];
        if (points.length < 3) {
            // Not enough points for a closed loop — leave the course empty
            // and let the UI status messaging explain.
            return;
        }

        // Materialise each control point as a gate with orientation taken
        // from the Catmull-Rom tangent at that point.
        const gates = new Array(points.length);
        for (let i = 0; i < points.length; i++) {
            const td = tangentAtPoint(points, i);
            gates[i] = {
                pos:       { x: points[i].x, y: points[i].y, z: points[i].z },
                travelDir: td,
                passed:    false,
                entity:    null,
                // Four category materials for normal gates (front /
                // back / outer / inner, see FACE_BRIGHTNESS) or one
                // per bar for the finish gate. The per-category
                // brightness is baked into `emissiveIntensity` at
                // build time so every subsequent colour repaint
                // preserves the directional-lighting cue.
                // `material` is kept as an alias to `materials[0]` for
                // legacy probes that still expect a single reference.
                materials: null,
                material:  null,
                invWorld:  null,
                // `fixedTint` gates (currently just the start gate) carry
                // a texture that must not be overwritten by the per-frame
                // color-state repaint in `_updateGateAppearance`.
                fixedTint: false,
                // Set on gate 0 only — the floating START sign plate.
                // Kept as a separate reference so the pulse animation can
                // counter-scale it (so the sign holds its natural size)
                // AND counter-translate it along local Y (so the sign's
                // world-height stays constant while the frame breathes,
                // instead of floating up and down with the pulse).
                signEntity:    null,
                signBaseScale: null,
                signBaseLocalY: 0,
            };
        }
        this.gates = gates;
        this.nextGateIdx = 0;

        // Build PlayCanvas entities (optional when unit-testing without an app).
        this._app = ctx.app || null;
        if (ctx.app) {
            /* global pc */
            this._rootEntity = new pc.Entity('race-course');
            ctx.app.root.addChild(this._rootEntity);
            for (let i = 0; i < gates.length; i++) {
                const ent = this._buildGateEntity(gates[i], i);
                this._rootEntity.addChild(ent);
                gates[i].entity = ent;
                // Cache world-to-local BEFORE any pulse scaling so pass
                // detection uses a stable gate plane even while the visual
                // pulse throbs.
                const inv = new pc.Mat4();
                inv.copy(ent.getWorldTransform()).invert();
                gates[i].invWorld = inv;
            }
            // Paint everything once the whole gates[] array is populated
            // so the horizon-based visibility logic sees the final N.
            this._updateAllAppearances();
            // Apply current visibility (entities default to enabled; if the
            // user hasn't pressed G yet we want them hidden).
            this._applyVisibility();
        }

        this._prevDronePos = null;
        this._pulseTime = 0;
        this.resetLap();
    }

    /**
     * Show / hide the whole gate course in the scene. Pass detection still
     * runs only when visible — we treat the course as "not engaged" when
     * the user has toggled it off via G.
     */
    setVisible(on) {
        this._visible = !!on;
        this._applyVisibility();
        if (!on) {
            // Hiding mid-flight: clear current-lap state so a later re-show
            // doesn't resurrect a stale timer.
            this.resetLap();
        }
    }

    isVisible() { return this._visible; }

    _applyVisibility() {
        if (!this._rootEntity) return;
        try {
            this._rootEntity.enabled = this._visible;
        } catch (error) {
            reportUserError('Gate course visibility update failed', error, {
                key: 'gate-course-visibility',
                intervalMs: 10000,
            });
        }
    }

    /**
     * Sampled points along the smooth Catmull-Rom curve, for editor / debug
     * rendering. Returns [] when < 3 gates. Output is a flat list of
     * `{pos, tangent}` entries (see catmull-rom.js#sampleClosed).
     */
    sampleCurve(samplesPerSegment = 24) {
        const points = this.gates.map(g => g.pos);
        return sampleClosed(points, samplesPerSegment);
    }

    /**
     * Build a PlayCanvas entity for one gate — four thin boxes forming a
     * square outline in the gate's local XY plane; local +Z = travel
     * direction. Orientation is derived from the Catmull-Rom tangent so
     * the gate plane is naturally perpendicular to the curve.
     *
     * A per-gate StandardMaterial is created here so colors can change
     * independently without leaking between gates.
     */
    _buildGateEntity(gate, index) {
        /* global pc */
        const size = this.opts.gateSize;
        const half = size * 0.5;
        const thickness = Math.max(0.06, size * 0.10); // 10 % of size, min 6 cm

        const e = new pc.Entity(`gate-${index}`);
        e.setPosition(gate.pos.x, gate.pos.y, gate.pos.z);

        // Build an orthonormal basis with local +Z = travelDir. Runtime is
        // always Y-up in the flight-local coordinate frame
        // to Y-up at parse time), so worldUp is world +Y.
        const fwd = new pc.Vec3(gate.travelDir.x, gate.travelDir.y, gate.travelDir.z);
        const worldUp = new pc.Vec3(0, 1, 0);
        const right = new pc.Vec3();
        right.cross(worldUp, fwd);
        if (right.lengthSq() < 1e-6) {
            // travelDir parallel to worldUp (drone flying straight up/down
            // between two identically-placed gates). Pick any perpendicular.
            if (Math.abs(fwd.x) < 0.9) right.set(1, 0, 0);
            else right.set(0, 1, 0);
            const dot = right.dot(fwd);
            right.sub(new pc.Vec3(fwd.x * dot, fwd.y * dot, fwd.z * dot));
        }
        right.normalize();
        const up = new pc.Vec3();
        up.cross(fwd, right).normalize();

        // Column-major rotation matrix with basis (right, up, fwd).
        const m = new pc.Mat4();
        const d = m.data;
        d[0] = right.x; d[1] = right.y; d[2]  = right.z; d[3]  = 0;
        d[4] = up.x;    d[5] = up.y;    d[6]  = up.z;    d[7]  = 0;
        d[8] = fwd.x;   d[9] = fwd.y;   d[10] = fwd.z;   d[11] = 0;
        d[12] = 0;      d[13] = 0;      d[14] = 0;       d[15] = 1;

        const q = new pc.Quat().setFromMat4(m);
        e.setRotation(q);

        // Start gate (gate 0) wears a checker-flag texture so it is
        // instantly recognisable as the launch line, just like real
        // motorsport circuits. Its shading strategy is intentionally
        // different from the rest: the chequered pattern already provides
        // strong visual signal, so we keep the simpler 4-box construction
        // with uniform brightness on all bars — no per-face shading —
        // which also avoids the material-explosion that per-face checker
        // tiling would force (each bar needs its own tile ratio for the
        // squares to render roughly square).
        const isStart = (index === 0);

        if (isStart) {
            this._buildFinishFrame(e, gate, size, half, thickness);
        } else {
            this._buildFacedFrame(e, gate, size, half, thickness);
        }

        // Gate 0 gets a floating "START" sign above the top bar so it
        // reads unambiguously as the launch position even when the frame
        // itself is partially obscured.
        if (index === 0) {
            const tex = getStartLabelTexture(this._app);
            if (tex) {
                const sign = new pc.Entity('gate-0-sign');
                sign.addComponent('render', { type: 'box' });
                const signMat = new pc.StandardMaterial();
                signMat.diffuseMap        = tex;
                signMat.emissiveMap       = tex;
                signMat.diffuse.set(1, 1, 1);
                signMat.emissive.set(1, 1, 1);
                signMat.emissiveIntensity = EMISSIVE_INTENSITY;
                signMat.useLighting       = false;
                signMat.useFog            = false;
                signMat.opacity           = 1.0;
                signMat.update();
                const signMis = sign.render.meshInstances;
                for (let mi = 0; mi < signMis.length; mi++) signMis[mi].material = signMat;
                // Plate size: full gate width, ~1/4 of gate height, thin.
                const signSX = size * 0.9;
                const signSY = size * 0.28;
                const signSZ = thickness * 0.6;
                const signY  = half + size * START_SIGN_GAP;
                sign.setLocalPosition(0, signY, 0);
                sign.setLocalScale(signSX, signSY, signSZ);
                e.addChild(sign);
                gate.signEntity     = sign;
                gate.signBaseScale  = new pc.Vec3(signSX, signSY, signSZ);
                gate.signBaseLocalY = signY;
            }
        }

        // Force world-transform recompute so invWorld (captured by the
        // caller) reflects the final rotation.
        e.getWorldTransform();
        return e;
    }

    /**
     * Start-gate builder: four checker-textured boxes, uniform brightness.
     * Deliberately skips the per-face shading path used for normal gates —
     * the chequered flag already carries all the visual weight this gate
     * needs, and per-face rebuild would quadruple the material count and
     * force per-face tile-ratio recalculation for the squares to stay
     * roughly square on every face.
     *
     * Sets `gate.fixedTint = true` so `_updateGateAppearance` does not
     * overwrite the white diffuse/emissive that lets the checker colours
     * show through.
     *
     * Note: named `_buildFinishFrame` for historical reasons, but now used
     * for the start gate (gate 0) rather than the finish gate.
     */
    _buildFinishFrame(parent, gate, size, half, thickness) {
        /* global pc */
        const edges = [
            { pos: [0,  half, 0], scale: [size + thickness, thickness, thickness], tileU: 6, tileV: 1 },
            { pos: [0, -half, 0], scale: [size + thickness, thickness, thickness], tileU: 6, tileV: 1 },
            { pos: [-half, 0, 0], scale: [thickness, size - thickness, thickness], tileU: 1, tileV: 5 },
            { pos: [ half, 0, 0], scale: [thickness, size - thickness, thickness], tileU: 1, tileV: 5 },
        ];
        const tex = getCheckerTexture(this._app);
        const mats = edges.map((ed) => {
            const m = new pc.StandardMaterial();
            if (tex) {
                m.diffuseMap  = tex;
                m.emissiveMap = tex;
                m.diffuseMapTiling  = new pc.Vec2(ed.tileU, ed.tileV);
                m.emissiveMapTiling = new pc.Vec2(ed.tileU, ed.tileV);
            }
            m.diffuse.set(1, 1, 1);
            m.emissive.set(1, 1, 1);
            m.emissiveIntensity = EMISSIVE_INTENSITY;
            m.useLighting       = false;
            m.useFog            = false;
            m.opacity           = 1.0;
            m.update();
            return m;
        });
        gate.fixedTint = true;
        gate.materials = mats;
        gate.material  = mats[0];
        for (let ei = 0; ei < edges.length; ei++) {
            const ed  = edges[ei];
            const seg = new pc.Entity();
            seg.addComponent('render', { type: 'box' });
            const mis = seg.render.meshInstances;
            for (let mi = 0; mi < mis.length; mi++) mis[mi].material = mats[ei];
            seg.setLocalPosition(ed.pos[0], ed.pos[1], ed.pos[2]);
            seg.setLocalScale(ed.scale[0], ed.scale[1], ed.scale[2]);
            parent.addChild(seg);
        }
    }

    /**
     * Normal-gate builder: four category materials (front / back / outer
     * / inner) and per-face plane meshes so the ring reads as one lit
     * surface. See `FACE_BRIGHTNESS` for the directional-lighting logic.
     *
     * Geometry, all in gate-local coords (gate lies in the XY plane with
     * +Z = travel direction, +Y = up relative to the path):
     *   Top bar:    center (0,  half, 0), long along X, length size+t
     *   Bottom bar: center (0, -half, 0), long along X, length size+t
     *   Left bar:   center (-half, 0, 0), long along Y, length size-t
     *   Right bar:  center ( half, 0, 0), long along Y, length size-t
     *
     * Each bar contributes one plane per face category; top and bottom
     * bars also emit two end-cap planes (±X) to close the corners — the
     * left/right bars' ±Y ends sit inside the top/bottom bars and stay
     * hidden, so we skip them.
     *
     * Materials are shared by category across all four bars, so a single
     * loop in `_updateGateAppearance` recolours the whole ring with four
     * `material.update()` calls regardless of gate count.
     */
    _buildFacedFrame(parent, gate, size, half, thickness) {
        /* global pc */
        const t  = thickness;
        const hT = t / 2;
        const c  = COL_UPCOMING;

        // One StandardMaterial per face category, reused by every bar's
        // face of that category. Only the `emissiveIntensity` differs —
        // the diffuse/emissive RGB gets repainted together on state
        // changes, which is why category-sharing is safe.
        const mkMat = (brightness) => {
            const m = new pc.StandardMaterial();
            m.diffuse.set(c.r, c.g, c.b);
            m.emissive.set(c.r, c.g, c.b);
            m.emissiveIntensity = EMISSIVE_INTENSITY * brightness;
            m.useLighting = false;
            m.useFog      = false;
            m.opacity     = 1.0;
            m.cull        = pc.CULLFACE_BACK;
            m.update();
            return m;
        };
        const matFront = mkMat(FACE_BRIGHTNESS.front);
        const matBack  = mkMat(FACE_BRIGHTNESS.back);
        const matOuter = mkMat(FACE_BRIGHTNESS.outer);
        const matInner = mkMat(FACE_BRIGHTNESS.inner);

        // Order matters only for `gate.material` aliasing; every entry
        // is visited identically by `_updateGateAppearance`.
        gate.materials = [matFront, matBack, matOuter, matInner];
        gate.material  = matFront;

        // Precomputed plane orientations. A PlayCanvas plane primitive
        // lies in the XZ plane with its visible side facing +Y, so we
        // rotate +Y onto each of the six world-space directions we need.
        //   • front / back  → rotate ±90° around +X  (UP → ±Z)
        //   • outer / inner → rotate  0° or 180° around +X for ±Y, or
        //                     ∓90° around +Z for ±X
        const rotPlusY  = new pc.Quat();                                    // UP → +Y (identity)
        const rotMinusY = new pc.Quat().setFromAxisAngle(new pc.Vec3(1, 0, 0), 180);  // UP → -Y
        const rotPlusZ  = new pc.Quat().setFromAxisAngle(new pc.Vec3(1, 0, 0),  90);  // UP → +Z
        const rotMinusZ = new pc.Quat().setFromAxisAngle(new pc.Vec3(1, 0, 0), -90);  // UP → -Z
        const rotPlusX  = new pc.Quat().setFromAxisAngle(new pc.Vec3(0, 0, 1), -90);  // UP → +X
        const rotMinusX = new pc.Quat().setFromAxisAngle(new pc.Vec3(0, 0, 1),  90);  // UP → -X

        // Helper: spawn a single face plane as a child of `parent`.
        // `scale` is the plane's local (sx, 1, sz) — after rotation,
        // sx ends up along the plane's local +X direction in world
        // coords and sz along its local +Z.
        const addFace = (pos, rot, scale, material) => {
            const f = new pc.Entity();
            f.addComponent('render', { type: 'plane' });
            const mis = f.render.meshInstances;
            for (let mi = 0; mi < mis.length; mi++) mis[mi].material = material;
            f.setLocalPosition(pos[0], pos[1], pos[2]);
            f.setLocalRotation(rot);
            f.setLocalScale(scale[0], scale[1], scale[2]);
            parent.addChild(f);
        };

        // ---- Horizontal bars (top + bottom) -------------------------
        // Length along X is (size + t) so the bar laps over the left/
        // right bars' outer edges and closes the ring cleanly.
        const lenH  = size + t;
        const hLenH = lenH / 2;
        const horiz = [
            { cy:  half, outerY: +1 },   // top bar: outer = +Y
            { cy: -half, outerY: -1 },   // bottom bar: outer = -Y
        ];
        for (const bar of horiz) {
            const cy = bar.cy;
            // FRONT (+Z): plane local X maps to world X, local Z to world -Y.
            //             scale (lenH, 1, t) → world X=lenH, world Y=t. ✓
            addFace([0, cy,  hT], rotPlusZ,  [lenH, 1, t], matFront);
            // BACK (-Z): plane local X → world X, local Z → world +Y.
            addFace([0, cy, -hT], rotMinusZ, [lenH, 1, t], matBack);
            // OUTER (±Y based on which bar): plane local X → world X,
            //                                local Z → world Z.
            if (bar.outerY > 0) addFace([0, cy + hT, 0], rotPlusY,  [lenH, 1, t], matOuter);
            else                addFace([0, cy - hT, 0], rotMinusY, [lenH, 1, t], matOuter);
            // INNER: opposite side from OUTER.
            if (bar.outerY > 0) addFace([0, cy - hT, 0], rotMinusY, [lenH, 1, t], matInner);
            else                addFace([0, cy + hT, 0], rotPlusY,  [lenH, 1, t], matInner);
            // END CAPS: small t×t squares at ±X ends. Coloured with the
            // outer brightness since they visually blend with the ring's
            // outer corners when viewed from any angle.
            // LEFT END (-X): plane local X → world Y, local Z → world Z.
            addFace([-hLenH, cy, 0], rotMinusX, [t, 1, t], matOuter);
            // RIGHT END (+X): same dimensions, opposite rotation.
            addFace([ hLenH, cy, 0], rotPlusX,  [t, 1, t], matOuter);
        }

        // ---- Vertical bars (left + right) ---------------------------
        // Length along Y is (size - t) so they sit between the top and
        // bottom bars without overlap.
        const lenV  = size - t;
        const vert  = [
            { cx: -half, outerX: -1 },   // left bar: outer = -X
            { cx:  half, outerX: +1 },   // right bar: outer = +X
        ];
        for (const bar of vert) {
            const cx = bar.cx;
            // FRONT (+Z): plane local X → world X (scaled to t),
            //             local Z → world -Y (scaled to lenV).
            addFace([cx, 0,  hT], rotPlusZ,  [t, 1, lenV], matFront);
            // BACK (-Z):
            addFace([cx, 0, -hT], rotMinusZ, [t, 1, lenV], matBack);
            // OUTER (±X): plane local X → world ±Y (scaled to lenV),
            //             local Z → world Z (scaled to t).
            if (bar.outerX > 0) addFace([cx + hT, 0, 0], rotPlusX,  [lenV, 1, t], matOuter);
            else                addFace([cx - hT, 0, 0], rotMinusX, [lenV, 1, t], matOuter);
            // INNER: opposite side.
            if (bar.outerX > 0) addFace([cx - hT, 0, 0], rotMinusX, [lenV, 1, t], matInner);
            else                addFace([cx + hT, 0, 0], rotPlusX,  [lenV, 1, t], matInner);
            // No end caps here — the top/bottom bars' end caps already
            // close the corners over these ±Y ends.
        }
    }

    /**
     * Repaint and show/hide one gate based on the current state.
     *
     * Visibility: circular-distance horizon only. A gate is shown iff
     * `(index - nextGateIdx + N) % N < HORIZON`. This one rule covers
     * three cases at once:
     *   - the current `next` gate (distance 0) is always visible;
     *   - gates just flown through wrap to very large distances and
     *     vanish behind the drone;
     *   - gates at the start of the NEXT lap slide into view as
     *     `nextGateIdx` nears the end of the current lap, giving a
     *     seamless rollover between laps.
     */
    _updateGateAppearance(gate, index) {
        if (!gate || !gate.materials || !gate.materials.length) return;
        const N = this.gates.length;

        // Horizon-only visibility. Circular distance already handles the
        // two natural hide cases: gates we just flew through end up with
        // very large distance and vanish, while gates at the start of
        // the next lap slide into view as `nextGateIdx` approaches the
        // end of the current lap — giving a seamless rollover.
        const dist = (index - this.nextGateIdx + N) % N;
        const visible = dist < HORIZON;
        if (gate.entity) {
            try {
                gate.entity.enabled = visible;
            } catch (error) {
                reportUserError('Gate visibility update failed', error, {
                    key: 'gate-visibility',
                    intervalMs: 10000,
                });
            }
        }
        if (!visible) return;

        // Gates with a fixed texture (currently just the start gate's
        // chequered flag) must not have their material diffuse/emissive
        // overwritten — doing so would tint the checker pattern.
        if (gate.fixedTint) return;

        // Gate 0 has the checker texture (fixedTint = true), so it returns
        // early above and never reaches this code path. All other gates
        // (including the last gate) are colored based on their state.
        const isNext = (index === this.nextGateIdx);
        let c;
        if (isNext)                  c = COL_NEXT;
        else                         c = COL_UPCOMING;
        // Repaint every material with the same base colour. Per-face
        // brightness lives in each material's `emissiveIntensity` (set
        // once at build time from FACE_BRIGHTNESS) so we don't touch it
        // here — that keeps the ring's directional-lighting cue stable
        // across colour-state changes.
        for (let ei = 0; ei < gate.materials.length; ei++) {
            const m = gate.materials[ei];
            m.diffuse.set(c.r, c.g, c.b);
            m.emissive.set(c.r, c.g, c.b);
            m.update();
        }
    }

    /**
     * Repaint every gate. Cheap (<= 20 gates typically) and needed on
     * any state change that may flip visibility for multiple gates at
     * once (next-index advance, lap reset, path rebuild).
     */
    _updateAllAppearances() {
        for (let i = 0; i < this.gates.length; i++) {
            this._updateGateAppearance(this.gates[i], i);
        }
    }

    /**
     * Per-frame update — pulses the next gate, runs pass detection, and
     * advances the lap timer. Safe to call even when the course is
     * empty, hidden, or the drone is missing.
     *
     * @param {number} dt       seconds since last frame
     * @param {Drone}  drone    drone with .x/.y/.z
     * @param {number} [nowMs]  monotonic timestamp for lap timing; defaults
     *                          to performance.now() so callers rarely need
     *                          to supply it.
     */
    update(dt, drone, nowMs) {
        if (!this.gates.length || !this._visible) {
            // Keep HUD silent when hidden: clear currentLapMs so the lap
            // readout doesn't display stale milliseconds.
            this.currentLapMs = 0;
            return;
        }

        // Pulse the current "next" gate.
        this._pulseTime += dt;
        const s = 1 + PULSE_AMP * Math.sin(this._pulseTime * PULSE_HZ * Math.PI * 2);
        const nextG = this.gates[this.nextGateIdx];
        if (nextG && nextG.entity) {
            nextG.entity.setLocalScale(s, s, s);
            // START sign decoupling: the sign is a child of the gate
            // entity so it inherits the gate's rotation and world
            // position, BUT we do not want it to inherit the pulse
            // scaling. Two corrections, both run every frame:
            //   • counter-scale    → sign holds its natural size,
            //                        instead of breathing in step.
            //   • counter-translate → sign sits at a constant world
            //                        height above the frame centre,
            //                        instead of bobbing up/down with
            //                        the scale (otherwise parent scale
            //                        s multiplies the sign's local Y
            //                        offset, producing visible float).
            if (nextG.signEntity && nextG.signBaseScale) {
                const inv = 1 / s;
                const b = nextG.signBaseScale;
                nextG.signEntity.setLocalScale(b.x * inv, b.y * inv, b.z * inv);
                nextG.signEntity.setLocalPosition(0, nextG.signBaseLocalY * inv, 0);
            }
        }

        if (!drone) return;
        const now = Number.isFinite(nowMs) ? nowMs : (typeof performance !== 'undefined' ? performance.now() : Date.now());
        const curr = { x: drone.x, y: drone.y, z: drone.z };

        // Live lap clock.
        if (this.lapStart != null) this.currentLapMs = now - this.lapStart;

        if (this._prevDronePos) {
            for (let i = 0; i < this.gates.length; i++) {
                const g = this.gates[i];
                if (!g.entity) continue;
                if (!this._checkCross(g, this._prevDronePos, curr)) continue;

                this._handleCrossing(i, now);
            }
        }
        this._prevDronePos = curr;
    }

    /**
     * React to a confirmed drone-through-gate crossing.
     *
     * Strict in-order rule: anything other than the currently-expected
     * `nextGateIdx` is silently ignored — no mark-passed, no colour
     * change, no HUD update. The player must fly the yellow (or
     * chartreuse-pulsing, for gate 0) gate before any other counts.
     *
     * @param {number} i      gate index just crossed
     * @param {number} now    ms timestamp of the cross
     */
    _handleCrossing(i, now) {
        if (i !== this.nextGateIdx) {
            console.log(`[Race] ignored out-of-order cross: expected gate ${this.nextGateIdx + 1}, got ${i + 1}`);
            return;
        }

        if (i === 0) {
            // Gate 0 is the lap boundary. `lapStart != null` means we
            // have been around the loop at least once — record that lap
            // before resetting. First-ever cross just starts the timer.
            if (this.lapStart != null) {
                const lapMs = now - this.lapStart;
                this.lapCount += 1;
                let isBest = false;
                if (this.bestLapMs == null || lapMs < this.bestLapMs) {
                    this.bestLapMs = lapMs;
                    isBest = true;
                }
                this._notifyLap(lapMs, isBest);
            }
            // Fresh lap in both cases: clear pass flags, restart timer.
            this._beginLap(now);
        }

        this._markPassed(i);
    }

    _markPassed(i) {
        const g = this.gates[i];
        if (!g || g.passed) return;
        g.passed = true;
        // Advance nextGateIdx to the first still-unpassed gate (circular).
        // We scan forward from (i + 1) % N, stopping at the first gap.
        const N = this.gates.length;
        let k = (i + 1) % N;
        for (let step = 0; step < N; step++) {
            if (!this.gates[k].passed) break;
            k = (k + 1) % N;
        }
        this.nextGateIdx = k;

        // Reset the pulse scale of the just-passed gate.
        if (g.entity) g.entity.setLocalScale(1, 1, 1);
        if (g.signEntity && g.signBaseScale) {
            const b = g.signBaseScale;
            g.signEntity.setLocalScale(b.x, b.y, b.z);
            g.signEntity.setLocalPosition(0, g.signBaseLocalY, 0);
        }

        // Visibility depends on `nextGateIdx`, so every advance may
        // reveal the newly-in-horizon gate at the far end and hide the
        // old `nextGateIdx - 0` slot — full refresh is cheapest.
        this._updateAllAppearances();

        console.log(`[Race] passed gate ${i + 1} / ${N}`);
        if (typeof this.onGatePassed === 'function') {
            try { this.onGatePassed(i, N); }
            catch (e) {
                reportUserError('Gate passed callback failed', e, {
                    key: 'gate-passed-callback',
                    intervalMs: 10000,
                });
            }
        }
    }

    /**
     * Begin a fresh lap — clear pass flags on all gates and restart
     * the timer. Does NOT mark gate 0 as passed; caller must do that
     * after calling _beginLap (see _handleCrossing).
     */
    _beginLap(now) {
        for (let i = 0; i < this.gates.length; i++) {
            const gi = this.gates[i];
            gi.passed = false;
            if (gi.entity) gi.entity.setLocalScale(1, 1, 1);
            if (gi.signEntity && gi.signBaseScale) {
                const b = gi.signBaseScale;
                gi.signEntity.setLocalScale(b.x, b.y, b.z);
                gi.signEntity.setLocalPosition(0, gi.signBaseLocalY, 0);
            }
        }
        this.nextGateIdx  = 0;
        this.lapStart     = now;
        this.currentLapMs = 0;
        this._updateAllAppearances();
    }

    _notifyLap(lapMs, isBest) {
        console.log(`[Race] lap ${this.lapCount}: ${(lapMs / 1000).toFixed(3)} s${isBest ? ' — NEW BEST' : ''}`);
        if (typeof this.onLapComplete === 'function') {
            try { this.onLapComplete(lapMs, isBest); }
            catch (e) {
                reportUserError('Lap complete callback failed', e, {
                    key: 'lap-complete-callback',
                    intervalMs: 10000,
                });
            }
        }
        if (isBest && typeof this.onBestLap === 'function') {
            try { this.onBestLap(lapMs); }
            catch (e) {
                reportUserError('Best lap callback failed', e, {
                    key: 'best-lap-callback',
                    intervalMs: 10000,
                });
            }
        }
    }

    /**
     * Line-vs-plane crossing test in gate-local space. Returns true iff
     * the segment (prev→curr) crosses the gate's XY plane AND the
     * intersection point lies within the gate's rectangle (±half, ±half).
     */
    _checkCross(gate, prev, curr) {
        if (!gate.invWorld) return false;
        /* global pc */
        const p0 = new pc.Vec3(prev.x, prev.y, prev.z);
        const p1 = new pc.Vec3(curr.x, curr.y, curr.z);
        gate.invWorld.transformPoint(p0, p0);
        gate.invWorld.transformPoint(p1, p1);

        if (p0.z * p1.z > 0) return false;
        const denom = p0.z - p1.z;
        if (Math.abs(denom) < 1e-9) return false;

        const t = p0.z / denom;
        const hx = p0.x + (p1.x - p0.x) * t;
        const hy = p0.y + (p1.y - p0.y) * t;
        const half = this.opts.gateSize * 0.5;
        return Math.abs(hx) <= half && Math.abs(hy) <= half;
    }

    /**
     * Clear the current lap's progress without touching the layout or
     * best-lap record. Called on `R` reset and whenever the course is
     * toggled off via G.
     */
    resetLap() {
        for (let i = 0; i < this.gates.length; i++) {
            const gi = this.gates[i];
            gi.passed = false;
            if (gi.entity) gi.entity.setLocalScale(1, 1, 1);
            if (gi.signEntity && gi.signBaseScale) {
                const b = gi.signBaseScale;
                gi.signEntity.setLocalScale(b.x, b.y, b.z);
                gi.signEntity.setLocalPosition(0, gi.signBaseLocalY, 0);
            }
        }
        this.nextGateIdx    = 0;
        this.lapStart       = null;
        this.currentLapMs   = 0;
        this._prevDronePos  = null;
        this._updateAllAppearances();
    }

    /**
     * Back-compat shim: older code calls `resetProgress()` on the drone
     * reset event. Forward to the Phase B implementation.
     */
    resetProgress() { this.resetLap(); }

    /** Tear down all PlayCanvas entities and clear state. */
    destroy() {
        if (this._rootEntity) {
            try {
                this._rootEntity.destroy();
            } catch (error) {
                reportUserError('Gate course destroy failed', error, {
                    key: 'gate-course-destroy',
                    intervalMs: 10000,
                });
            }
            this._rootEntity = null;
        }
        this.gates = [];
        this.nextGateIdx   = 0;
        this._prevDronePos = null;
        this._pulseTime    = 0;
        this.lapStart      = null;
        this.currentLapMs  = 0;
        this.lapCount      = 0;
        // bestLapMs intentionally preserved — survives scene-exit so
        // the persisted record isn't clobbered by a transient destroy().
    }

    /** UI status — e.g. "5 gates · best 00:39.1" or "no path drawn". */
    statusText() {
        if (!this.gates.length) return 'no path drawn';
        const gates = `${this.gates.length} gates`;
        if (this.bestLapMs != null) return `${gates} · best ${formatLap(this.bestLapMs)}`;
        return gates;
    }

    /** Count of passed gates this lap — used by HUD. */
    passedCount() {
        let n = 0;
        for (const g of this.gates) if (g.passed) n++;
        return n;
    }
}

// ---- Exports for HUD formatting ------------------------------------
/**
 * Format a millisecond duration as `MM:SS.mmm`. Exported so hud.js can
 * render lap times in the same style the Race Course module uses
 * internally for logging.
 */
export function formatLap(ms) {
    if (!Number.isFinite(ms) || ms < 0) return '--:--.---';
    const totalMs = Math.round(ms);
    const minutes = Math.floor(totalMs / 60000);
    const seconds = Math.floor((totalMs % 60000) / 1000);
    const millis  = totalMs % 1000;
    return `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}.${String(millis).padStart(3, '0')}`;
}

/**
 * Helper: sample the Catmull-Rom-derived gate tangent at control point i.
 * Exposed for the path editor, which shows tangent-direction arrows in
 * the top-down canvas so the user knows which way each gate will face.
 */
export { evaluateClosed, tangentAtPoint, sampleClosed };
