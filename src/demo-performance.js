const TILE_REQUEST_OPTIONS = Object.freeze([6, 8, 12, 18]);

function clamp(value, minimum, maximum) {
    return Math.max(minimum, Math.min(maximum, value));
}

function percentile(values, quantile) {
    const finite = values.filter(Number.isFinite).sort((a, b) => a - b);
    if (!finite.length) return null;
    const index = Math.min(finite.length - 1, Math.ceil(finite.length * quantile) - 1);
    return finite[Math.max(0, index)];
}

function rounded(value, digits = 1) {
    if (!Number.isFinite(value)) return null;
    const scale = 10 ** digits;
    return Math.round(value * scale) / scale;
}

function currentSearch() {
    return typeof globalThis.location?.search === 'string' ? globalThis.location.search : '';
}

export function resolvePerformanceConfig(search = currentSearch()) {
    const params = new URLSearchParams(search || '');
    const profile = params.get('perfProfile') === 'baseline' ? 'baseline' : 'demo30';
    const requestedTileLimit = Number(params.get('tileRequestsPerServer'));
    const defaultTileLimit = profile === 'demo30' ? 12 : 18;
    const tileRequestsPerServer = TILE_REQUEST_OPTIONS.includes(requestedTileLimit)
        ? requestedTileLimit
        : defaultTileLimit;
    const dynamicSse = params.get('dynamicSse') === 'off' ? 'off' : 'current';
    const requestedCollisionCadence = params.get('collisionCadence');
    const collisionCadence = requestedCollisionCadence === 'frame'
        || requestedCollisionCadence === 'substep'
        ? requestedCollisionCadence
        : profile === 'demo30' ? 'frame' : 'substep';

    if (profile === 'baseline') {
        return Object.freeze({
            profile,
            targetFps: 30,
            initialResolutionScale: 0.7,
            minimumResolutionScale: 0.7,
            maximumResolutionScale: 0.7,
            normalFacesPerSlice: null,
            constrainedFacesPerSlice: null,
            planningCaptureIntervalMs: null,
            idlePreviewIntervalMs: null,
            movingPreviewIntervalMs: null,
            planningPreviewIntervalMs: 2000,
            collisionSweepMode: 'sphere',
            collisionCadence,
            tileRequestsPerServer,
            preloadRadiusMeters: 500,
            preloadTimeoutMs: 20000,
            preloadViewAttempts: 2,
            preloadFrameDelayMs: null,
            preloadFaceTileTimeoutMs: null,
            preloadFaceTileQuietMs: null,
            preloadTileRequestsPerServer: 18,
            preloadRequired: false,
            dynamicSse,
        });
    }

    return Object.freeze({
        profile,
        targetFps: 30,
        initialResolutionScale: 0.7,
        minimumResolutionScale: 0.55,
        maximumResolutionScale: 0.8,
        normalFacesPerSlice: 3,
        constrainedFacesPerSlice: 2,
        planningCaptureIntervalMs: 1000 / 15,
        idlePreviewIntervalMs: 1000 / 8,
        movingPreviewIntervalMs: 1000 / 2,
        planningPreviewIntervalMs: 500,
        collisionSweepMode: 'cross',
        collisionCadence,
        tileRequestsPerServer,
        preloadRadiusMeters: 400,
        preloadTimeoutMs: 60000,
        preloadViewAttempts: 3,
        preloadFrameDelayMs: 32,
        preloadFaceTileTimeoutMs: 4000,
        preloadFaceTileQuietMs: 150,
        preloadTileRequestsPerServer: 18,
        preloadRequired: false,
        dynamicSse,
    });
}

function velocityMagnitude(state) {
    const velocity = state?.velocity
        || state?.actualState?.velocity
        || state?.referenceState?.velocity
        || state;
    const vx = Number(velocity?.x ?? velocity?.vx);
    const vy = Number(velocity?.y ?? velocity?.vy);
    const vz = Number(velocity?.z ?? velocity?.vz);
    if (![vx, vy, vz].every(Number.isFinite)) return 0;
    return Math.hypot(vx, vy, vz);
}

export class DemoPerformanceController {
    constructor(config = resolvePerformanceConfig()) {
        this.config = config;
        this._viewer = null;
        this._removePostRenderListener = null;
        this._lastFrameAt = null;
        this._frames = [];
        this._captures = [];
        this._latestSlotDrops = [];
        this._resolutionEvents = [];
        this._faceSliceEvents = [];
        this._collisionChecks = [];
        this._resolutionScale = config.initialResolutionScale;
        this._facesPerSlice = config.normalFacesPerSlice;
        this._moving = false;
        this._overBudgetSince = null;
        this._underBudgetSince = null;
        this._lastAdjustmentAt = -Infinity;
        this._tileStatusProvider = null;
        this._preload = {};
        this._yopoStrategy = null;
        this._identityPromise = null;
        this._recordedCaptureKeys = new Map();
    }

    configureCesium(Cesium) {
        const scheduler = Cesium?.RequestScheduler;
        if (scheduler && 'maximumRequestsPerServer' in scheduler) {
            scheduler.maximumRequestsPerServer = this.config.tileRequestsPerServer;
        }
    }

    attachViewer(viewer, tileStatusProvider = null) {
        if (viewer && this._viewer !== viewer) {
            if (typeof this._removePostRenderListener === 'function') {
                this._removePostRenderListener();
            }
            this._viewer = viewer;
            if (this.config.profile === 'demo30' && 'targetFrameRate' in viewer) {
                viewer.targetFrameRate = this.config.targetFps;
            }
            const postRender = viewer.scene?.postRender;
            this._removePostRenderListener = postRender?.addEventListener
                ? postRender.addEventListener(() => this.recordFrame(performance.now()))
                : null;
            this._applyResolutionScale();
            this._resolutionEvents.push({
                recordedAtMs: performance.now(),
                scale: this._resolutionScale,
            });
        }
        if (typeof tileStatusProvider === 'function') {
            this._tileStatusProvider = tileStatusProvider;
        }
    }

    _applyResolutionScale() {
        if (!this._viewer || !('resolutionScale' in this._viewer)) return;
        this._viewer.resolutionScale = this._resolutionScale;
        this._viewer.scene?.requestRender?.();
    }

    _setResolutionScale(value, now) {
        const next = rounded(clamp(
            value,
            this.config.minimumResolutionScale,
            this.config.maximumResolutionScale,
        ), 3);
        if (next === this._resolutionScale) return false;
        this._resolutionScale = next;
        this._resolutionEvents.push({ recordedAtMs: now, scale: next });
        this._applyResolutionScale();
        return true;
    }

    recordFrame(now = performance.now()) {
        if (!Number.isFinite(now)) return;
        if (Number.isFinite(this._lastFrameAt)) {
            const intervalMs = now - this._lastFrameAt;
            if (intervalMs > 0 && intervalMs < 5000) {
                this._frames.push({ recordedAtMs: now, intervalMs });
            }
        }
        this._lastFrameAt = now;
        const retentionBoundary = now - 10 * 60 * 1000;
        while (this._frames[0]?.recordedAtMs < retentionBoundary) this._frames.shift();
        while (this._captures[0]?.recordedAtMs < retentionBoundary) this._captures.shift();
        while (this._latestSlotDrops[0] < retentionBoundary) this._latestSlotDrops.shift();
        while (this._resolutionEvents[1]?.recordedAtMs < retentionBoundary) {
            this._resolutionEvents.shift();
        }
        while (this._faceSliceEvents[0]?.recordedAtMs < retentionBoundary) {
            this._faceSliceEvents.shift();
        }
        while (this._collisionChecks[0]?.recordedAtMs < retentionBoundary) {
            this._collisionChecks.shift();
        }
        this._adjust(now);
    }

    _setFacesPerSlice(value, now, reason) {
        const next = Number.isFinite(value) ? Math.max(1, Math.round(value)) : value;
        if (next === this._facesPerSlice) return false;
        const previous = this._facesPerSlice;
        this._facesPerSlice = next;
        this._faceSliceEvents.push({
            recordedAtMs: now,
            from: previous,
            to: next,
            reason,
        });
        console.info(`[pano-adaptive] facesPerSlice ${previous} -> ${next} reason=${reason}`);
        return true;
    }

    _adjust(now) {
        if (this.config.profile !== 'demo30') return;
        const recent = this._frames
            .filter(frame => frame.recordedAtMs >= now - 2000)
            .map(frame => frame.intervalMs);
        if (recent.length < 20) return;
        const p95 = percentile(recent, 0.95);

        if (p95 > 40) {
            this._underBudgetSince = null;
            if (this._overBudgetSince === null) this._overBudgetSince = now;
            if (now - this._overBudgetSince >= 2000 && now - this._lastAdjustmentAt >= 3000) {
                const constrained = this._setFacesPerSlice(
                    this.config.constrainedFacesPerSlice,
                    now,
                    'main-p95-over-40ms',
                );
                if (!constrained) {
                    this._setResolutionScale(this._resolutionScale - 0.05, now);
                }
                this._lastAdjustmentAt = now;
                this._overBudgetSince = now;
            }
            return;
        }

        this._overBudgetSince = null;
        if (p95 < 35) {
            if (this._underBudgetSince === null) this._underBudgetSince = now;
            if (now - this._underBudgetSince >= 5000 && now - this._lastAdjustmentAt >= 5000) {
                if (this._facesPerSlice !== this.config.normalFacesPerSlice) {
                    this._setFacesPerSlice(
                        this.config.normalFacesPerSlice,
                        now,
                        'main-p95-under-35ms',
                    );
                } else {
                    this._setResolutionScale(this._resolutionScale + 0.025, now);
                }
                this._lastAdjustmentAt = now;
                this._underBudgetSince = now;
            }
        } else {
            this._underBudgetSince = null;
        }
    }

    captureIntervalMs(mode, planningState, baselineMs) {
        if (this.config.profile !== 'demo30') return baselineMs;
        if (mode === 'planning') return this.config.planningCaptureIntervalMs;
        const speed = velocityMagnitude(planningState);
        if (this._moving) {
            if (speed < 0.3) this._moving = false;
        } else if (speed > 0.7) {
            this._moving = true;
        }
        return this._moving
            ? this.config.movingPreviewIntervalMs
            : this.config.idlePreviewIntervalMs;
    }

    facesPerSlice(isChromium, baselineFacesPerSlice) {
        if (this.config.profile !== 'demo30') {
            return isChromium ? 2 : baselineFacesPerSlice;
        }
        return this._facesPerSlice;
    }

    planningPreviewIntervalMs(baselineMs) {
        return this.config.profile === 'demo30'
            ? this.config.planningPreviewIntervalMs
            : baselineMs;
    }

    recordCapture(mode, recordedAtMs = performance.now(), captureMs = null) {
        this._captures.push({ mode, recordedAtMs, captureMs: Number(captureMs) || 0 });
    }

    recordLatestSlotDrop(recordedAtMs = performance.now()) {
        this._latestSlotDrops.push(recordedAtMs);
    }

    recordCollisionCheck(metrics = {}, recordedAtMs = performance.now()) {
        this._collisionChecks.push({
            recordedAtMs,
            durationMs: Math.max(0, Number(metrics.durationMs) || 0),
            rayCount: Math.max(0, Math.round(Number(metrics.rayCount) || 0)),
            queryCount: Math.max(0, Math.round(Number(metrics.queryCount) || 0)),
            sweepDurationMs: Math.max(0, Number(metrics.sweepDurationMs) || 0),
            neighborhoodDurationMs: Math.max(0, Number(metrics.neighborhoodDurationMs) || 0),
            sweepRayCount: Math.max(0, Math.round(Number(metrics.sweepRayCount) || 0)),
            neighborhoodRayCount: Math.max(0, Math.round(Number(metrics.neighborhoodRayCount) || 0)),
            cadence: metrics.cadence || this.config.collisionCadence,
        });
    }

    recordPerceptionCapture(metrics = {}) {
        const mode = metrics.mode;
        if (mode !== 'planning' && mode !== 'preview') return;
        const frameId = Number(metrics.frameId ?? metrics.rgbFrameId);
        if (!Number.isFinite(frameId)) return;

        const recordedAtMs = performance.now();
        const key = `${mode}:${frameId}`;
        if (this._recordedCaptureKeys.has(key)) return;
        this._recordedCaptureKeys.set(key, recordedAtMs);
        this._captures.push({ mode, frameId, recordedAtMs });

        const cutoff = recordedAtMs - 120000;
        for (const [captureKey, capturedAt] of this._recordedCaptureKeys) {
            if (capturedAt >= cutoff) break;
            this._recordedCaptureKeys.delete(captureKey);
        }
    }

    recordPreload(kind, status) {
        this._preload[kind] = {
            ...(status || {}),
            recordedAtMs: performance.now(),
        };
    }

    loadYopoStrategy(endpoint) {
        if (this._identityPromise || typeof fetch !== 'function') return this._identityPromise;
        try {
            const url = new URL(endpoint, globalThis.location?.href || 'http://127.0.0.1/');
            url.pathname = '/yopo/health';
            url.search = '';
            this._identityPromise = fetch(url, { cache: 'no-store' })
                .then(response => response.ok ? response.json() : null)
                .then(payload => {
                    this._yopoStrategy = payload?.strategy || null;
                    return this._yopoStrategy;
                })
                .catch(() => null);
        } catch {
            this._identityPromise = Promise.resolve(null);
        }
        return this._identityPromise;
    }

    snapshotSince(startedAtMs = -Infinity) {
        const frames = this._frames.filter(frame => frame.recordedAtMs >= startedAtMs);
        const intervals = frames.map(frame => frame.intervalMs);
        const captures = this._captures.filter(item => item.recordedAtMs >= startedAtMs);
        const elapsedMs = Math.max(1, (frames.at(-1)?.recordedAtMs || performance.now()) - startedAtMs);
        const captureHz = mode => rounded(
            captures.filter(item => item.mode === mode).length / elapsedMs * 1000,
            1,
        );
        let tileStatus = null;
        try {
            tileStatus = this._tileStatusProvider?.() || null;
        } catch {
            tileStatus = null;
        }
        const scales = this._resolutionEvents
            .filter(event => event.recordedAtMs >= startedAtMs)
            .map(event => event.scale);
        if (!scales.length) scales.push(this._resolutionScale);
        const p50 = percentile(intervals, 0.5);
        const collisionChecks = this._collisionChecks
            .filter(event => event.recordedAtMs >= startedAtMs);
        const collisionDurations = collisionChecks.map(event => event.durationMs);
        const collisionRayCounts = collisionChecks.map(event => event.rayCount);
        const collisionSweepDurations = collisionChecks.map(event => event.sweepDurationMs);
        const collisionNeighborhoodDurations = collisionChecks
            .map(event => event.neighborhoodDurationMs);
        const collisionDurationS = Math.max(
            0.001,
            (performance.now() - startedAtMs) / 1000,
        );
        const collisionQueryCount = collisionChecks.reduce(
            (total, event) => total + event.queryCount,
            0,
        );
        const collisionRayCount = collisionChecks.reduce(
            (total, event) => total + event.rayCount,
            0,
        );
        const collisionSweepRayCount = collisionChecks.reduce(
            (total, event) => total + event.sweepRayCount,
            0,
        );
        const collisionNeighborhoodRayCount = collisionChecks.reduce(
            (total, event) => total + event.neighborhoodRayCount,
            0,
        );
        return Object.freeze({
            performanceProfile: this.config.profile,
            tileRequestsPerServer: this.config.tileRequestsPerServer,
            dynamicSse: this.config.dynamicSse,
            collisionCadence: this.config.collisionCadence,
            collisionCheckHz: rounded(collisionQueryCount / collisionDurationS, 1),
            collisionQueryP50Ms: rounded(percentile(collisionDurations, 0.5), 2),
            collisionQueryP95Ms: rounded(percentile(collisionDurations, 0.95), 2),
            collisionRaysPerFrameP95: rounded(percentile(collisionRayCounts, 0.95), 1),
            collisionRaysPerSecond: rounded(collisionRayCount / collisionDurationS, 1),
            collisionSweepP95Ms: rounded(percentile(collisionSweepDurations, 0.95), 2),
            collisionNeighborhoodP95Ms: rounded(
                percentile(collisionNeighborhoodDurations, 0.95),
                2,
            ),
            collisionSweepRaysPerSecond: rounded(
                collisionSweepRayCount / collisionDurationS,
                1,
            ),
            collisionNeighborhoodRaysPerSecond: rounded(
                collisionNeighborhoodRayCount / collisionDurationS,
                1,
            ),
            yopoStrategy: this._yopoStrategy,
            mainMedianFps: p50 ? rounded(1000 / p50, 1) : null,
            mainFrameIntervalP50Ms: rounded(p50, 1),
            mainFrameIntervalP95Ms: rounded(percentile(intervals, 0.95), 1),
            mainFrameIntervalP99Ms: rounded(percentile(intervals, 0.99), 1),
            mainLongFrame100MsCount: intervals.filter(value => value > 100).length,
            mainLongFrame250MsCount: intervals.filter(value => value > 250).length,
            planningCaptureHz: captureHz('planning'),
            previewCaptureHz: captureHz('preview'),
            latestSlotDroppedFrames: this._latestSlotDrops.filter(time => time >= startedAtMs).length,
            adaptiveResolutionScaleMin: Math.min(...scales),
            adaptiveResolutionScaleMax: Math.max(...scales),
            adaptiveResolutionScaleFinal: this._resolutionScale,
            panoramaFacesPerSliceFinal: this._facesPerSlice,
            panoramaFacesPerSliceEvents: this._faceSliceEvents
                .filter(event => event.recordedAtMs >= startedAtMs)
                .map(event => ({ ...event })),
            tileLoad: tileStatus,
            preload: { ...this._preload },
        });
    }
}

export const demoPerformance = new DemoPerformanceController();
