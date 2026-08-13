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
 * Collision adapter for Cesium / Google Photorealistic 3D Tiles.
 *
 * Cesium does not expose Google tiles as a CPU physics mesh. This provider
 * therefore builds a conservative local proxy from currently loaded render data:
 *
 *   - `scene.sampleHeight()` handles vertical roof / terrain overlap.
 *   - `scene.pickFromRay()` probes the current sphere neighborhood.
 *   - A swept ray from previous position to current position catches direct
 *     wall impacts that would otherwise tunnel through between frames.
 */

function normalize(v, fallback = { x: 0, y: 1, z: 0 }) {
    const len = Math.hypot(v.x, v.y, v.z);
    if (len < 1e-8) return fallback;
    return { x: v.x / len, y: v.y / len, z: v.z / len };
}

function hitNormalFromPoint(cx, cy, cz, p, fallback) {
    return normalize({ x: cx - p.x, y: cy - p.y, z: cz - p.z }, fallback);
}

function horizontalDistance(a, b) {
    return Math.hypot(a.x - b.x, a.z - b.z);
}

export class TilesCollisionProvider {
    constructor(world, options = {}) {
        this.world = world;
        this.enabled = true;
        this.verticalSkin = Number.isFinite(options.verticalSkin) ? options.verticalSkin : 0.03;
        this.horizontalSkin = Number.isFinite(options.horizontalSkin) ? options.horizontalSkin : 0.07;
        this.rayExtra = Number.isFinite(options.rayExtra) ? options.rayExtra : 0.45;
        this.sweptExtra = Number.isFinite(options.sweptExtra) ? options.sweptExtra : 0.35;
        this.sweepRadiusFactor = Number.isFinite(options.sweepRadiusFactor)
            ? Math.max(0.35, Math.min(0.95, options.sweepRadiusFactor))
            : 0.72;
        this.heightProbeWidth = Number.isFinite(options.heightProbeWidth) ? options.heightProbeWidth : 0.35;
        this.minPenetration = Number.isFinite(options.minPenetration) ? options.minPenetration : 0.015;
        this.minRayDistance = Number.isFinite(options.minRayDistance) ? options.minRayDistance : 0.04;
        this.queryIntervalMs = Number.isFinite(options.queryIntervalMs) ? options.queryIntervalMs : 58;
        this.maxNeighborhoodRays = Math.max(3, Math.round(
            Number.isFinite(options.maxNeighborhoodRays) ? options.maxNeighborhoodRays : 6
        ));
        this.lastDebug = null;
        this._lastNoHit = null;
        this._rayCursor = 0;

        this._rayDirs = [
            { x: 1, y: 0, z: 0 },
            { x: -1, y: 0, z: 0 },
            { x: 0, y: 0, z: 1 },
            { x: 0, y: 0, z: -1 },
            { x: 0.7071, y: 0, z: 0.7071 },
            { x: -0.7071, y: 0, z: 0.7071 },
            { x: 0.7071, y: 0, z: -0.7071 },
            { x: -0.7071, y: 0, z: -0.7071 },
        ];
    }

    queryCollisionResponse(x, y, z, radius, state = {}) {
        if (!this.enabled || !this.world || !this.world.ready) return null;

        const now = performance.now();
        const hits = [];
        const r = Math.max(0.05, radius || 0.3);
        const center = { x, y, z };
        const velocity = state && state.velocity ? state.velocity : null;
        const speed = velocity
            ? Math.hypot(velocity.x || 0, velocity.y || 0, velocity.z || 0)
            : 0;

        const reuseStaticNoHit = this._canReuseNoHit(center, r, speed, now);
        // Never cache away continuous collision detection: a segment can cross
        // a thin wall even when both endpoints are inside a cached no-hit area.
        this._querySweptMotion(center, r, state, hits);
        if (!reuseStaticNoHit) {
            this._queryHeight(center, r, hits);
            this._queryNeighborhood(center, r, hits, state);
        }

        if (!hits.length) {
            this.lastDebug = { colliding: false, skippedStatic: reuseStaticNoHit };
            this._lastNoHit = { x, y, z, radius: r, speed, time: now };
            return null;
        }

        this._lastNoHit = null;
        hits.sort((a, b) => b.penetration - a.penetration);
        const hit = hits[0];
        this.lastDebug = {
            colliding: true,
            source: hit.source,
            penetration: hit.penetration,
            normal: hit.normal,
        };
        return hit;
    }

    _queryHeight(center, radius, hits) {
        const surfaceY = this.world.sampleHeightAtLocal(center.x, center.z, this.heightProbeWidth);
        if (!Number.isFinite(surfaceY)) return;

        const penetration = surfaceY + radius + this.verticalSkin - center.y;
        if (penetration <= this.minPenetration) return;

        hits.push({
            normal: { x: 0, y: 1, z: 0 },
            penetration,
            source: 'height',
            pointCount: 1,
        });
    }

    _querySweptMotion(center, radius, state, hits) {
        const prev = state && state.previous;
        if (!prev) return;

        const motion = {
            x: center.x - prev.x,
            y: center.y - prev.y,
            z: center.z - prev.z,
        };
        const motionDistance = Math.hypot(motion.x, motion.y, motion.z);
        if (!Number.isFinite(motionDistance) || motionDistance < 0.03) return;

        const dir = normalize(motion, { x: 0, y: 0, z: -1 });
        const fallback = Math.abs(dir.y) < 0.7
            ? normalize({ x: -dir.x, y: 0, z: -dir.z }, { x: 0, y: 0, z: 1 })
            : normalize({ x: -dir.x, y: -dir.y, z: -dir.z }, { x: 0, y: 1, z: 0 });

        // Approximate a swept sphere rather than ray-casting only its centre.
        // The four offset probes catch wall corners, roof edges and thin meshes
        // touched by the vehicle radius between two physics frames.
        const reference = Math.abs(dir.y) < 0.9
            ? { x: 0, y: 1, z: 0 }
            : { x: 1, y: 0, z: 0 };
        const side = normalize({
            x: dir.y * reference.z - dir.z * reference.y,
            y: dir.z * reference.x - dir.x * reference.z,
            z: dir.x * reference.y - dir.y * reference.x,
        }, { x: 1, y: 0, z: 0 });
        const up = normalize({
            x: side.y * dir.z - side.z * dir.y,
            y: side.z * dir.x - side.x * dir.z,
            z: side.x * dir.y - side.y * dir.x,
        }, { x: 0, y: 1, z: 0 });
        const offsetRadius = radius * this.sweepRadiusFactor;
        const probes = [
            { x: 0, y: 0, z: 0, center: true },
            { x: side.x * offsetRadius, y: side.y * offsetRadius, z: side.z * offsetRadius },
            { x: -side.x * offsetRadius, y: -side.y * offsetRadius, z: -side.z * offsetRadius },
            { x: up.x * offsetRadius, y: up.y * offsetRadius, z: up.z * offsetRadius },
            { x: -up.x * offsetRadius, y: -up.y * offsetRadius, z: -up.z * offsetRadius },
        ];

        for (let probeIndex = 0; probeIndex < probes.length; probeIndex++) {
            const probe = probes[probeIndex];
            const origin = {
                x: prev.x + probe.x,
                y: prev.y + probe.y,
                z: prev.z + probe.z,
            };
            const contactAllowance = probe.center ? radius : this.horizontalSkin;
            const maxDistance = motionDistance + contactAllowance + this.sweptExtra;
            const hit = this.world.pickLocalRay(origin, dir, maxDistance);
            if (!this._validRayHit(origin, hit, maxDistance)) continue;

            const penetration = motionDistance + contactAllowance - hit.distance;
            if (penetration <= this.minPenetration) continue;
            hits.push({
                normal: Math.abs(dir.y) < 0.7
                    ? fallback
                    : hitNormalFromPoint(center.x, center.y, center.z, hit.position, fallback),
                penetration,
                source: probe.center ? 'swept-center' : 'swept-radius',
                pointCount: probes.length,
                probeIndex,
            });
        }
    }

    _queryNeighborhood(center, radius, hits, state = {}) {
        const maxDistance = radius + this.rayExtra;
        const verticalOffsets = [0];
        const dirs = this._selectRayDirs(state);

        for (const dy of verticalOffsets) {
            const origin = { x: center.x, y: center.y + dy, z: center.z };
            for (const dir of dirs) {
                const hit = this.world.pickLocalRay(origin, dir, maxDistance);
                if (!this._validRayHit(origin, hit, maxDistance)) continue;

                const penetration = radius + this.horizontalSkin - hit.distance;
                if (penetration <= this.minPenetration) continue;

                const fallback = { x: -dir.x, y: 0, z: -dir.z };
                let normal = hitNormalFromPoint(center.x, center.y, center.z, hit.position, fallback);
                if (Math.abs(normal.y) > 0.65 && horizontalDistance(center, hit.position) > radius * 0.35) {
                    normal = normalize({ x: normal.x, y: 0, z: normal.z }, fallback);
                }

                hits.push({
                    normal,
                    penetration,
                    source: 'ray',
                    pointCount: 1,
                });
            }
        }
    }

    _canReuseNoHit(center, radius, speed, now) {
        const last = this._lastNoHit;
        if (!last) return false;

        const elapsed = now - last.time;
        const interval = speed > 28
            ? Math.min(this.queryIntervalMs, 34)
            : speed > 10
                ? this.queryIntervalMs
                : Math.max(this.queryIntervalMs, 84);
        if (elapsed > interval) return false;

        const moved = Math.hypot(center.x - last.x, center.y - last.y, center.z - last.z);
        const allowedMove = Math.max(radius * 1.35, Math.min(1.2, 0.28 + speed * 0.012));
        return moved <= allowedMove && Math.abs(radius - last.radius) < 0.02;
    }

    _selectRayDirs(state = {}) {
        const dirs = [];
        const velocity = state && state.velocity ? state.velocity : null;
        const speedH = velocity ? Math.hypot(velocity.x || 0, velocity.z || 0) : 0;

        if (speedH > 0.15) {
            const fwd = normalize({ x: velocity.x, y: 0, z: velocity.z }, { x: 0, y: 0, z: -1 });
            dirs.push(fwd);
            dirs.push(normalize({ x: fwd.x * 0.866 - fwd.z * 0.5, y: 0, z: fwd.x * 0.5 + fwd.z * 0.866 }, fwd));
            dirs.push(normalize({ x: fwd.x * 0.866 + fwd.z * 0.5, y: 0, z: -fwd.x * 0.5 + fwd.z * 0.866 }, fwd));
        }

        while (dirs.length < this.maxNeighborhoodRays) {
            dirs.push(this._rayDirs[this._rayCursor % this._rayDirs.length]);
            this._rayCursor++;
        }

        return dirs.slice(0, this.maxNeighborhoodRays);
    }

    _validRayHit(origin, hit, maxDistance) {
        if (!hit || !hit.position || !Number.isFinite(hit.distance)) return false;
        if (hit.distance < this.minRayDistance || hit.distance > maxDistance) return false;
        const dx = hit.position.x - origin.x;
        const dy = hit.position.y - origin.y;
        const dz = hit.position.z - origin.z;
        return Number.isFinite(dx) && Number.isFinite(dy) && Number.isFinite(dz);
    }
}
