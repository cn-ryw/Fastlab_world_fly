/**
 * Flight data logger — records pose, velocity, goal, and controller state
 * during goal-based autonomous navigation.  Auto-starts on goal set,
 * auto-stops on arrival or cancel, then downloads a JSON log.
 */

const SENSITIVE_URL_KEY_MARKERS = Object.freeze([
    'token',
    'secret',
    'password',
    'passwd',
    'apikey',
    'auth',
    'authorization',
    'credential',
]);

// Flight logs need the projection/performance configuration, not arbitrary
// browser query state.  A deny-list is insufficient because an endpoint
// override may itself contain userinfo or nested token parameters.  Keep only
// known non-secret tuning keys; newly introduced keys remain private until
// deliberately reviewed and added here.
const SAFE_URL_QUERY_KEYS = new Set([
    'panoProfile',
    'panoCaptureProfile',
    'panoCaptureAnyway',
    'panoPreloadRequired',
    'panoWidth',
    'panoHeight',
    'panoFace',
    'panoVfov',
    'panoJpeg',
    'panoMs',
    'panoFaceFov',
    'panoTopPoleGuard',
    'panoBottomPoleGuard',
    'panoFrameDelayMs',
    'panoFaceTileTimeoutMs',
    'panoFaceTileQuietMs',
    'panoFacesPerSlice',
    'panoPreloadFrameDelayMs',
    'panoPreloadFaceTileTimeoutMs',
    'panoPreloadFaceTileQuietMs',
    'panoPreloadTimeoutMs',
    'da360TimeoutMs',
    'da360UploadScale',
    'da360UploadWidth',
    'da360UploadHeight',
    'depthMs',
    'yopoMaxFrameAgeMs',
    'panoramaTileSse',
    'flightTileSse',
    'placementTileSse',
    'resolutionScale',
    'placementResolutionScale',
    'tileCacheMb',
    'droneScale',
    'flightPreloadRadius',
    'flightPreloadMinCoverage',
    'flightPreloadViewTimeoutMs',
    'flightPreloadViewAttempts',
    'flightPreloadStrict',
]);

const SAFE_URL_ENUM_VALUES = Object.freeze({
    panoProfile: new Set(['flight', 'calibration']),
    panoCaptureProfile: new Set(['flight', 'calibration']),
});

function isSafeResolvedUrlParam(key, values) {
    if (!SAFE_URL_QUERY_KEYS.has(key) || values.length === 0) return false;
    const enumValues = SAFE_URL_ENUM_VALUES[key];
    if (enumValues) return values.every(value => enumValues.has(value));
    return values.every(value => value.trim() !== '' && Number.isFinite(Number(value)));
}

function sanitizeResolvedUrl(value) {
    if (typeof value !== 'string' || !value.trim()) return null;
    try {
        const resolved = new URL(value);
        const localHost = resolved.hostname === '127.0.0.1'
            || resolved.hostname === 'localhost';
        const entryPath = resolved.pathname === '/' || resolved.pathname === '/index.html';
        if (!['http:', 'https:'].includes(resolved.protocol) || !localHost || !entryPath) {
            return null;
        }
        resolved.username = '';
        resolved.password = '';
        resolved.hash = '';

        // Normalize separators so api-key, api_key, APIKey, etc. are all caught.
        const keys = [...new Set(resolved.searchParams.keys())];
        for (const key of keys) {
            const normalized = key.toLowerCase().replace(/[^a-z0-9]/g, '');
            const sensitive = SENSITIVE_URL_KEY_MARKERS.some(
                marker => normalized.includes(marker)
            );
            const values = resolved.searchParams.getAll(key);
            if (sensitive || !isSafeResolvedUrlParam(key, values)) {
                resolved.searchParams.delete(key);
            }
        }
        return resolved.href;
    } catch {
        // A malformed URL must never fall back to the original potentially secret value.
        return null;
    }
}

export class FlightLogger {
    constructor() {
        this._recording = false;
        this._frames = [];
        this._startTime = null;
        this._startedAtIso = null;
        this._goal = null;
        this._spawnAlt = null;
        // 性能统计
        this._depthCount = 0;
        this._depthLatencies = [];
        this._yopoCount = 0;
        this._yopoLatencies = [];
        this._yopoTrackerCount = 0;  // 帧数：有活跃 YOPO 轨迹
        this._perceptionMetrics = [];
        this._planningApplyTimes = [];
        this._appliedPlanningFrames = new Set();
        this._dropReasons = {};
        this._physicsUpdateIntervals = [];
        this._lastPhysicsUpdateAt = null;
        this._navigationIdentity = null;
        this._crossSessionPerceptionDropped = 0;
    }

    /** Start a new recording session. Called when a goal is set. */
    start(goal, spawnAltitude, navigation = null) {
        this._recording = true;
        this._frames = [];
        this._startTime = performance.now();
        this._startedAtIso = new Date().toISOString();
        this._goal = { x: goal.x, y: goal.y, z: goal.z };
        this._spawnAlt = spawnAltitude;
        this._depthCount = 0;
        this._depthLatencies = [];
        this._yopoCount = 0;
        this._yopoLatencies = [];
        this._yopoTrackerCount = 0;
        this._perceptionMetrics = [];
        this._planningApplyTimes = [];
        this._appliedPlanningFrames = new Set();
        this._dropReasons = {};
        this._physicsUpdateIntervals = [];
        this._lastPhysicsUpdateAt = null;
        this._navigationIdentity = navigation?.goalId != null
            && navigation?.generation != null
            ? Object.freeze({
                goalId: String(navigation.goalId),
                generation: String(navigation.generation),
            })
            : null;
        this._crossSessionPerceptionDropped = 0;
        console.log('[FlightLog] recording started, goal:', goal);
    }

    /** 记录一次 DA360 深度推理的性能指标 */
    recordDepth(latencyMs) {
        if (!this._recording) return;
        this._depthCount++;
        if (Number.isFinite(latencyMs)) this._depthLatencies.push(latencyMs);
    }

    /** 记录一次 YOPO 规划的性能指标 */
    recordYopo(latencyMs) {
        if (!this._recording) return;
        this._yopoCount++;
        if (Number.isFinite(latencyMs)) this._yopoLatencies.push(latencyMs);
    }

    /** Record one immutable perception/planning outcome with segmented timing. */
    recordPerception(metrics) {
        if (!this._recording || !metrics) return;
        const item = { ...metrics, recordedAtMs: performance.now() };
        if (
            (item.mode === 'planning' || item.mode === 'depth-preview')
            && this._navigationIdentity
        ) {
            const belongsToSession = String(item.goalId ?? '')
                    === this._navigationIdentity.goalId
                && String(item.generation ?? '')
                    === this._navigationIdentity.generation;
            if (!belongsToSession) {
                this._crossSessionPerceptionDropped++;
                return;
            }
        }
        this._perceptionMetrics.push(item);
        const unauthorizedPlanning = item.mode === 'planning'
            && item.planningAuthorized !== true;
        const trajectoryRejected = item.mode === 'planning'
            && item.trajectoryApplied !== true;
        if (item.outcome !== 'applied' || unauthorizedPlanning || trajectoryRejected) {
            const reason = item.dropReason
                || (unauthorizedPlanning ? 'planning-not-authorized' : null)
                || (trajectoryRejected ? 'trajectory-not-applied' : null)
                || item.outcome
                || 'unknown';
            this._dropReasons[reason] = (this._dropReasons[reason] || 0) + 1;
            return;
        }
        if (item.mode === 'planning') {
            const identity = `${item.goalId ?? ''}:${item.generation ?? ''}:${item.frameId ?? ''}`;
            const appliedAt = Number(item.trajectoryAppliedAtMs);
            if (!this._appliedPlanningFrames.has(identity) && Number.isFinite(appliedAt)) {
                this._appliedPlanningFrames.add(identity);
                this._planningApplyTimes.push(appliedAt);
            }
        }
    }

    /** Record one frame. Call from updateFlight(). */
    record(drone, refX, refY, refZ) {
        if (!this._recording || !drone) return;
        const now = performance.now();
        const t = (now - this._startTime) / 1000;
        if (this._lastPhysicsUpdateAt != null) {
            this._physicsUpdateIntervals.push(now - this._lastPhysicsUpdateAt);
        }
        this._lastPhysicsUpdateAt = now;
        this._frames.push({
            recordedAtMs: now,
            t: Math.round(t * 1000) / 1000,
            x:  Math.round(drone.x  * 100) / 100,
            y:  Math.round(drone.y  * 100) / 100,
            z:  Math.round(drone.z  * 100) / 100,
            vx: Math.round(drone.vx * 100) / 100,
            vy: Math.round(drone.vy * 100) / 100,
            vz: Math.round(drone.vz * 100) / 100,
            yaw:   Math.round(drone.yaw   * 10) / 10,
            pitch: Math.round(drone.pitch * 10) / 10,
            roll:  Math.round(drone.roll  * 10) / 10,
            pitchRate: Math.round((drone.pitchRate || 0) * 10) / 10,
            rollRate:  Math.round((drone.rollRate  || 0) * 10) / 10,
            yawRate:   Math.round((drone.yawRate   || 0) * 10) / 10,
            refX: refX != null ? Math.round(refX * 100) / 100 : null,
            refY: refY != null ? Math.round(refY * 100) / 100 : null,
            refZ: refZ != null ? Math.round(refZ * 100) / 100 : null,
            thrust:     Math.round(drone.thrustOutput || 0),
            groundSpeed: Math.round((drone.groundSpeed || 0) * 100) / 100,
            mode: drone.flightMode || '',
        });
        // YOPO 轨迹跟踪帧计数
        if (drone._yopoPolyX) this._yopoTrackerCount++;
    }

    /** Stop recording and download the log. @param arrived true if reached goal */
    stop(arrived = false) {
        if (!this._recording) return;
        this._recording = false;
        const duration = (performance.now() - this._startTime) / 1000;

        // 性能摘要
        const avgDepthLatency = this._depthLatencies.length > 0
            ? Math.round(this._depthLatencies.reduce((a,b) => a+b, 0) / this._depthLatencies.length) : null;
        const avgYopoLatency = this._yopoLatencies.length > 0
            ? Math.round(this._yopoLatencies.reduce((a,b) => a+b, 0) / this._yopoLatencies.length) : null;
        const percentile = (values, quantile) => {
            const finite = values.filter(Number.isFinite).sort((a, b) => a - b);
            if (!finite.length) return null;
            const index = Math.min(finite.length - 1, Math.ceil(quantile * finite.length) - 1);
            return Math.round(finite[Math.max(0, index)] * 10) / 10;
        };
        const authorizedPlanningMetrics = this._perceptionMetrics.filter(item => (
            item.mode === 'planning'
            && item.outcome === 'applied'
            && item.planningAuthorized === true
            && item.trajectoryApplied === true
        ));
        const metricValues = key => authorizedPlanningMetrics
            .map(item => Number(item[key]))
            .filter(Number.isFinite);
        const planningIntervals = this._planningApplyTimes.slice(1).map(
            (time, index) => time - this._planningApplyTimes[index]
        );
        const calibrationIds = [...new Set(
            this._perceptionMetrics.map(item => item.calibrationId).filter(Boolean)
        )];

        const log = {
            schemaVersion: 2,
            startTime: this._startedAtIso,
            endTime: new Date().toISOString(),
            monotonicStartMs: this._startTime,
            resolvedUrl: sanitizeResolvedUrl(globalThis.location?.href),
            runtime: {
                userAgent: globalThis.navigator?.userAgent || null,
                hardwareConcurrency: Number(globalThis.navigator?.hardwareConcurrency) || null,
            },
            spawnAltitude: this._spawnAlt,
            goal: this._goal,
            navigationSession: this._navigationIdentity,
            frameCount: this._frames.length,
            duration_s: Math.round(duration * 100) / 100,
            arrived,
            perf: {
                depthHz: Math.round(this._depthCount / duration * 10) / 10,
                depthCount: this._depthCount,
                depthLatencyAvgMs: avgDepthLatency,
                yopoHz: Math.round(this._yopoCount / duration * 10) / 10,
                yopoCount: this._yopoCount,
                yopoLatencyAvgMs: avgYopoLatency,
                yopoTrackerFraction: this._frames.length > 0
                    ? Math.round(this._yopoTrackerCount / this._frames.length * 100) : 0,
                uniquePlanningHz: Math.round(this._appliedPlanningFrames.size / duration * 10) / 10,
                uniquePlanningFrames: this._appliedPlanningFrames.size,
                planningIntervalP95Ms: percentile(planningIntervals, 0.95),
                captureToApplyP95Ms: percentile(metricValues('captureToApplyMs'), 0.95),
                frameAgeP95Ms: percentile(metricValues('frameAgeMs'), 0.95),
                captureP95Ms: percentile(metricValues('captureMs'), 0.95),
                renderP95Ms: percentile(metricValues('renderMs'), 0.95),
                sceneRenderP95Ms: percentile(metricValues('sceneRenderMs'), 0.95),
                tileWaitP95Ms: percentile(metricValues('tileWaitMs'), 0.95),
                waitRerenderP95Ms: percentile(metricValues('waitRerenderMs'), 0.95),
                faceUploadP95Ms: percentile(metricValues('faceUploadMs'), 0.95),
                projectP95Ms: percentile(metricValues('projectMs'), 0.95),
                jpegP95Ms: percentile(metricValues('jpegMs'), 0.95),
                networkP95Ms: percentile(metricValues('networkMs'), 0.95),
                serverP95Ms: percentile(metricValues('serverMs'), 0.95),
                da360P95Ms: percentile(metricValues('da360Ms'), 0.95),
                yopoP95Ms: percentile(metricValues('yopoMs'), 0.95),
                applyP95Ms: percentile(metricValues('applyMs'), 0.95),
                physicsUpdateIntervalP95Ms: percentile(this._physicsUpdateIntervals, 0.95),
                flightLoopIntervalP95Ms: percentile(this._physicsUpdateIntervals, 0.95),
                droppedByReason: this._dropReasons,
                crossSessionPerceptionDropped: this._crossSessionPerceptionDropped,
                calibrationIds,
            },
            perception: this._perceptionMetrics,
            frames: this._frames,
        };
        const blob = new Blob([JSON.stringify(log, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const fname = `flight-log-${ts}.json`;
        a.download = fname;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        console.log(`[FlightLog] ${fname}: ${this._frames.length} frames, ${duration.toFixed(1)}s, arrived=${arrived}`);
    }

    /** True while a recording session is active. */
    get recording() { return this._recording; }
}
