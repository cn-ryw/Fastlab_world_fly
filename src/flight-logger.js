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
    'panoramaFarMeters',
    'panoramaLeanStreaming',
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
    'perfProfile',
    'tileRequestsPerServer',
    'dynamicSse',
]);

const SAFE_URL_ENUM_VALUES = Object.freeze({
    panoProfile: new Set(['flight', 'calibration']),
    panoCaptureProfile: new Set(['flight', 'calibration']),
    perfProfile: new Set(['demo30', 'baseline']),
    dynamicSse: new Set(['current', 'off']),
    collisionCadence: new Set(['frame', 'substep']),
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

function roundedFinite(value, digits = 4) {
    if (
        value === null
        || value === undefined
        || (typeof value === 'string' && value.trim() === '')
    ) return null;
    const number = Number(value);
    if (!Number.isFinite(number)) return null;
    const scale = 10 ** digits;
    return Math.round(number * scale) / scale;
}

function nonNegativeSafeIntegerOrNull(value) {
    if (
        value === null
        || value === undefined
        || (typeof value === 'string' && value.trim() === '')
    ) return null;
    const number = Number(value);
    return Number.isSafeInteger(number) && number >= 0 ? number : null;
}

function diagnosticVector(vector) {
    if (!vector || typeof vector !== 'object') return null;
    return {
        x: roundedFinite(vector.x),
        y: roundedFinite(vector.y),
        z: roundedFinite(vector.z),
    };
}

function compactControlDiagnostics(drone) {
    if (!drone || typeof drone.getControlDiagnostics !== 'function') return null;
    let diagnostics;
    try {
        diagnostics = drone.getControlDiagnostics();
    } catch {
        return null;
    }
    if (!diagnostics || typeof diagnostics !== 'object') return null;
    const reference = diagnostics.referenceState;
    return {
        commandType: diagnostics.commandType ?? null,
        source: diagnostics.source ?? null,
        frame: diagnostics.frame ?? 'sim-world-y-up',
        generation: diagnostics.generation ?? null,
        planningFrameId: diagnostics.planningFrameId ?? null,
        planningRequestId: diagnostics.planningRequestId ?? null,
        selectedCandidateId: diagnostics.selectedCandidateId ?? null,
        terminalPhase: diagnostics.terminalPhase ?? null,
        goalReached: diagnostics.goalReached === true,
        goalReachedSimTimeS: roundedFinite(diagnostics.goalReachedSimTimeS),
        fallbackReason: diagnostics.fallbackReason ?? null,
        referencePosition: reference ? diagnosticVector(reference.position || reference) : null,
        referenceVelocity: reference ? diagnosticVector({
            x: reference.velocity?.x ?? reference.vx,
            y: reference.velocity?.y ?? reference.vy,
            z: reference.velocity?.z ?? reference.vz,
        }) : null,
        referenceAcceleration: reference ? diagnosticVector({
            x: reference.acceleration?.x ?? reference.ax,
            y: reference.acceleration?.y ?? reference.ay,
            z: reference.acceleration?.z ?? reference.az,
        }) : null,
        rawAcceleration: diagnosticVector(diagnostics.rawAcceleration),
        limitedAcceleration: diagnosticVector(diagnostics.limitedAcceleration),
        requestedForce: diagnosticVector(diagnostics.requestedForce),
        allocatedForce: diagnosticVector(diagnostics.allocatedForce),
        projectionRatio: roundedFinite(diagnostics.projectionRatio),
        tiltDeg: roundedFinite(diagnostics.tiltDeg, 3),
        thrustGf: roundedFinite(diagnostics.thrustGf, 2),
        saturation: diagnostics.saturation ? {
            horizontal: diagnostics.saturation.horizontal === true,
            vertical: diagnostics.saturation.vertical === true,
            direct: diagnostics.saturation.direct === true,
        } : null,
        antiWindup: diagnostics.antiWindup ? {
            horizontal: diagnostics.antiWindup.horizontal === true,
            vertical: diagnostics.antiWindup.vertical === true,
        } : null,
        trajectoryAgeS: roundedFinite(diagnostics.trajectoryAgeS),
        trajectoryOriginalAgeS: roundedFinite(diagnostics.trajectoryOriginalAgeS),
        trajectoryRemainingS: roundedFinite(diagnostics.trajectoryRemainingS),
        trajectoryApplyPositionErrorM: roundedFinite(
            diagnostics.trajectoryApplyPositionErrorM,
        ),
        trajectoryApplyVelocityErrorMps: roundedFinite(
            diagnostics.trajectoryApplyVelocityErrorMps,
        ),
        poly5PeakSpeedMps: roundedFinite(diagnostics.poly5PeakSpeedMps),
        poly5PeakAccelerationMps2: roundedFinite(
            diagnostics.poly5PeakAccelerationMps2,
        ),
        trajectoryEndpointGoalDistanceM: roundedFinite(
            diagnostics.trajectoryEndpointGoalDistanceM,
        ),
        terminalTrajectoryEligible: diagnostics.terminalTrajectoryEligible === true,
        terminalSettledTimeS: roundedFinite(diagnostics.terminalSettledTimeS),
        overrunCount: nonNegativeSafeIntegerOrNull(diagnostics.overrunCount),
        overrunDroppedSeconds: roundedFinite(diagnostics.overrunDroppedSeconds, 6),
    };
}

export class FlightLogger {
    constructor(options = {}) {
        this._performanceProvider = typeof options.performanceProvider === 'function'
            ? options.performanceProvider
            : null;
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
        this._collisionEvents = [];
        if (!Number.isSafeInteger(this._lastCollisionSequence)) {
            this._lastCollisionSequence = 0;
        }
        this._planningApplyTimes = [];
        this._appliedPlanningFrames = new Set();
        this._dropReasons = {};
        this._dropReasonsByMode = {};
        this._physicsUpdateIntervals = [];
        this._lastPhysicsUpdateAt = null;
        this._navigationIdentity = null;
        this._navigationKind = null;
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
        this._collisionEvents = [];
        this._planningApplyTimes = [];
        this._appliedPlanningFrames = new Set();
        this._dropReasons = {};
        this._dropReasonsByMode = {};
        this._physicsUpdateIntervals = [];
        this._lastPhysicsUpdateAt = null;
        this._navigationIdentity = navigation?.goalId != null
            && navigation?.generation != null
            ? Object.freeze({
                goalId: String(navigation.goalId),
                generation: String(navigation.generation),
            })
            : null;
        this._navigationKind = navigation?.kind === 't8l-rolling' ? 't8l-rolling' : 'fixed';
        this._crossSessionPerceptionDropped = 0;
        console.log('[FlightLog] recording started, goal:', goal);
    }

    updateGoal(goal) {
        if (!this._recording || !goal) return;
        const values = [goal.x, goal.y, goal.z].map(Number);
        if (values.every(Number.isFinite)) this._goal = { x: values[0], y: values[1], z: values[2] };
    }

    /** 记录一次已返回给 UI 的深度预览延迟；规划吞吐以 uniquePlanningHz 为准。 */
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
        const trajectoryIgnored = item.mode === 'planning'
            && (item.outcome === 'ignored' || item.trajectoryIgnored === true);
        const trajectoryRejected = item.mode === 'planning'
            && item.trajectoryApplied !== true
            && !trajectoryIgnored;
        if (item.outcome !== 'applied' || unauthorizedPlanning || trajectoryRejected) {
            const reason = item.dropReason
                || (unauthorizedPlanning ? 'planning-not-authorized' : null)
                || (trajectoryRejected ? 'trajectory-not-applied' : null)
                || item.outcome
                || 'unknown';
            this._dropReasons[reason] = (this._dropReasons[reason] || 0) + 1;
            const mode = String(item.mode || 'unknown');
            if (!this._dropReasonsByMode[mode]) this._dropReasonsByMode[mode] = {};
            this._dropReasonsByMode[mode][reason] = (
                this._dropReasonsByMode[mode][reason] || 0
            ) + 1;
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
    record(drone, refX, refY, refZ, schedule = null) {
        if (!this._recording || !drone) return;
        const now = performance.now();
        const t = (now - this._startTime) / 1000;
        if (this._lastPhysicsUpdateAt != null) {
            this._physicsUpdateIntervals.push(now - this._lastPhysicsUpdateAt);
        }
        this._lastPhysicsUpdateAt = now;
        const control = compactControlDiagnostics(drone);
        const collisionEvent = drone.lastCollisionEvent;
        let collision = null;
        if (collisionEvent && collisionEvent.sequence > this._lastCollisionSequence) {
            this._lastCollisionSequence = collisionEvent.sequence;
            collision = {
                sequence: collisionEvent.sequence,
                simTimeS: roundedFinite(collisionEvent.simTimeS, 6),
                position: diagnosticVector(collisionEvent.position),
                contactPoint: diagnosticVector(collisionEvent.contactPoint),
                normal: diagnosticVector(collisionEvent.normal),
                velocity: diagnosticVector(collisionEvent.velocity),
                speedMps: roundedFinite(collisionEvent.speedMps),
                penetrationM: roundedFinite(collisionEvent.penetrationM, 4),
                source: collisionEvent.source || null,
                probeIndex: collisionEvent.probeIndex ?? null,
                pointCount: collisionEvent.pointCount ?? null,
                recordedAtMs: now,
            };
            this._collisionEvents.push(collision);
        }
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
            navigationKind: this._navigationKind,
            activeGoal: drone._idealGoal ? {
                x: roundedFinite(drone._idealGoal.x),
                y: roundedFinite(drone._idealGoal.y),
                z: roundedFinite(drone._idealGoal.z),
            } : null,
            collision,
            control,
            scheduler: schedule ? {
                steps: Number(schedule.steps) || 0,
                frameSeconds: roundedFinite(schedule.frameSeconds, 6),
                simulatedThisFrameSeconds: roundedFinite(
                    schedule.simulatedThisFrameSeconds,
                    6,
                ),
                droppedSeconds: roundedFinite(schedule.droppedSeconds, 6),
                totalDroppedSeconds: roundedFinite(schedule.totalDroppedSeconds, 6),
            } : null,
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
        const planningIgnoredFrames = this._perceptionMetrics.filter(item => (
            item.mode === 'planning'
            && (item.outcome === 'ignored' || item.trajectoryIgnored === true)
        )).length;
        const planningRejectedFrames = this._perceptionMetrics.filter(item => (
            item.mode === 'planning'
            && item.outcome === 'rejected'
            && item.trajectoryIgnored !== true
        )).length;
        const uniqueAuthorizedPlanningByIdentity = new Map();
        for (const item of authorizedPlanningMetrics) {
            const identity = `${item.goalId ?? ''}:${item.generation ?? ''}:${item.frameId ?? ''}`;
            if (!uniqueAuthorizedPlanningByIdentity.has(identity)) {
                uniqueAuthorizedPlanningByIdentity.set(identity, item);
            }
        }
        const uniqueAuthorizedPlanningMetrics = [
            ...uniqueAuthorizedPlanningByIdentity.values(),
        ];
        const rgbTilesReadyPlanningFrames = uniqueAuthorizedPlanningMetrics.filter(
            item => item.rgbTilesReady === true
        ).length;
        const rgbTilesPartialPlanningFrames = uniqueAuthorizedPlanningMetrics.filter(
            item => item.rgbTilesReady === false
        ).length;
        const rgbTilesUnknownPlanningFrames = uniqueAuthorizedPlanningMetrics.length
            - rgbTilesReadyPlanningFrames
            - rgbTilesPartialPlanningFrames;
        const rgbTileErrorPlanningFrames = uniqueAuthorizedPlanningMetrics.filter(
            item => item.rgbTileError === true || item.rgbReadinessReason === 'tile-error'
        ).length;
        const finiteValues = (items, key) => items
            .map(item => item?.[key])
            .filter(value => value !== null && value !== undefined && value !== '')
            .map(Number)
            .filter(Number.isFinite);
        const metricValues = key => finiteValues(authorizedPlanningMetrics, key);
        const committedDepthPreviewMetrics = this._perceptionMetrics.filter(item => (
            item.mode === 'depth-preview'
            && item.outcome === 'applied'
            && item.depthPreviewCommitted === true
        ));
        const previewMetricValues = key => finiteValues(committedDepthPreviewMetrics, key);
        const controlFrames = this._frames
            .map(frame => frame.control)
            .filter(control => control && typeof control === 'object');
        const controlMetricValues = key => finiteValues(controlFrames, key);
        const maxOrNull = values => values.length > 0
            ? values.reduce((maximum, value) => Math.max(maximum, value), -Infinity)
            : null;
        const terminalPhaseCounts = {};
        for (const control of controlFrames) {
            const phase = String(control.terminalPhase || 'unknown');
            terminalPhaseCounts[phase] = (terminalPhaseCounts[phase] || 0) + 1;
        }
        const directSaturationFrames = controlFrames.filter(
            control => control.saturation?.direct === true
        ).length;
        const horizontalSaturationFrames = controlFrames.filter(
            control => control.saturation?.horizontal === true
        ).length;
        const verticalSaturationFrames = controlFrames.filter(
            control => control.saturation?.vertical === true
        ).length;
        const horizontalArwFrames = controlFrames.filter(
            control => control.antiWindup?.horizontal === true
        ).length;
        const verticalArwFrames = controlFrames.filter(
            control => control.antiWindup?.vertical === true
        ).length;
        const schedulerFrames = this._frames
            .map(frame => frame.scheduler)
            .filter(schedule => schedule && typeof schedule === 'object');
        const planningIntervals = this._planningApplyTimes.slice(1).map(
            (time, index) => time - this._planningApplyTimes[index]
        );
        const calibrationIds = [...new Set(
            this._perceptionMetrics.map(item => item.calibrationId).filter(Boolean)
        )];
        let demoMetrics = {};
        try {
            demoMetrics = this._performanceProvider?.(this._startTime) || {};
        } catch (error) {
            console.warn('[FlightLog] performance snapshot unavailable', error);
        }

        const log = {
            schemaVersion: 2,
            startTime: this._startedAtIso,
            endTime: new Date().toISOString(),
            monotonicStartMs: this._startTime,
            resolvedUrl: sanitizeResolvedUrl(globalThis.location?.href),
            runtime: {
                userAgent: globalThis.navigator?.userAgent || null,
                hardwareConcurrency: Number(globalThis.navigator?.hardwareConcurrency) || null,
                performanceProfile: demoMetrics.performanceProfile,
                tileRequestsPerServer: demoMetrics.tileRequestsPerServer,
                dynamicSse: demoMetrics.dynamicSse,
                yopoStrategy: demoMetrics.yopoStrategy,
            },
            spawnAltitude: this._spawnAlt,
            goal: this._goal,
            navigationSession: this._navigationIdentity,
            navigationKind: this._navigationKind,
            frameCount: this._frames.length,
            duration_s: Math.round(duration * 100) / 100,
            arrived,
            perf: {
                depthPreviewHz: Math.round(this._depthCount / duration * 10) / 10,
                depthPreviewCount: this._depthCount,
                depthPreviewLatencyAvgMs: avgDepthLatency,
                previewDisplayHz: Math.round(this._depthCount / duration * 10) / 10,
                // Compatibility aliases retained for existing log readers.
                depthHz: Math.round(this._depthCount / duration * 10) / 10,
                depthCount: this._depthCount,
                depthLatencyAvgMs: avgDepthLatency,
                yopoHz: Math.round(this._yopoCount / duration * 10) / 10,
                yopoCount: this._yopoCount,
                yopoLatencyAvgMs: avgYopoLatency,
                planningResponseHz: Math.round(this._yopoCount / duration * 10) / 10,
                yopoTrackerFraction: this._frames.length > 0
                    ? Math.round(this._yopoTrackerCount / this._frames.length * 100) : 0,
                uniquePlanningHz: Math.round(this._appliedPlanningFrames.size / duration * 10) / 10,
                trajectoryInstallHz: Math.round(this._appliedPlanningFrames.size / duration * 10) / 10,
                uniquePlanningFrames: this._appliedPlanningFrames.size,
                planningIgnoredFrames,
                planningRejectedFrames,
                rgbTilesReadyPlanningFrames,
                rgbTilesPartialPlanningFrames,
                rgbTilesUnknownPlanningFrames,
                rgbTileErrorPlanningFrames,
                rgbTilesReadyPlanningPercent: uniqueAuthorizedPlanningMetrics.length > 0
                    ? Math.round(rgbTilesReadyPlanningFrames / uniqueAuthorizedPlanningMetrics.length * 1000) / 10
                    : null,
                planningIntervalP95Ms: percentile(planningIntervals, 0.95),
                captureToApplyP95Ms: percentile(metricValues('captureToApplyMs'), 0.95),
                captureToApplyDisplacementP95M: percentile(
                    metricValues('captureToApplyDisplacementM'),
                    0.95,
                ),
                frameAgeP95Ms: percentile(metricValues('frameAgeMs'), 0.95),
                ageAtFetchStartP95Ms: percentile(metricValues('ageAtFetchStartMs'), 0.95),
                ageAtResponseHeadersP95Ms: percentile(
                    metricValues('ageAtResponseHeadersMs'),
                    0.95,
                ),
                ageAtJsonParsedP95Ms: percentile(metricValues('ageAtJsonParsedMs'), 0.95),
                captureP95Ms: percentile(metricValues('captureMs'), 0.95),
                renderP95Ms: percentile(metricValues('renderMs'), 0.95),
                sceneRenderP95Ms: percentile(metricValues('sceneRenderMs'), 0.95),
                tileWaitP95Ms: percentile(metricValues('tileWaitMs'), 0.95),
                waitRerenderP95Ms: percentile(metricValues('waitRerenderMs'), 0.95),
                faceUploadP95Ms: percentile(metricValues('faceUploadMs'), 0.95),
                projectP95Ms: percentile(metricValues('projectMs'), 0.95),
                jpegP95Ms: percentile(metricValues('jpegMs'), 0.95),
                networkP95Ms: percentile(metricValues('networkMs'), 0.95),
                responseBytesP95: percentile(metricValues('responseBytes'), 0.95),
                requestGateHoldP95Ms: percentile(metricValues('gateWaitMs'), 0.95),
                serverP95Ms: percentile(metricValues('serverMs'), 0.95),
                da360P95Ms: percentile(metricValues('da360Ms'), 0.95),
                yopoP95Ms: percentile(metricValues('yopoMs'), 0.95),
                applyP95Ms: percentile(metricValues('applyMs'), 0.95),
                depthDecodeP95Ms: percentile(previewMetricValues('depthDecodeMs'), 0.95),
                depthDrawP95Ms: percentile(previewMetricValues('depthDrawMs'), 0.95),
                selectedTerminalSpeedP95Mps: percentile(
                    metricValues('terminalSpeedMps'),
                    0.95,
                ),
                selectedTerminalAccelerationP95Mps2: percentile(
                    metricValues('terminalAccelerationMps2'),
                    0.95,
                ),
                selectedEndpointDisplacementP95M: percentile(
                    metricValues('endpointDisplacementM'),
                    0.95,
                ),
                controlDirectSaturationPercent: controlFrames.length > 0
                    ? Math.round(directSaturationFrames / controlFrames.length * 1000) / 10
                    : null,
                controlHorizontalSaturationPercent: controlFrames.length > 0
                    ? Math.round(horizontalSaturationFrames / controlFrames.length * 1000) / 10
                    : null,
                controlVerticalSaturationPercent: controlFrames.length > 0
                    ? Math.round(verticalSaturationFrames / controlFrames.length * 1000) / 10
                    : null,
                controlHorizontalArwPercent: controlFrames.length > 0
                    ? Math.round(horizontalArwFrames / controlFrames.length * 1000) / 10
                    : null,
                controlVerticalArwPercent: controlFrames.length > 0
                    ? Math.round(verticalArwFrames / controlFrames.length * 1000) / 10
                    : null,
                controlProjectionRatioP05: percentile(
                    controlMetricValues('projectionRatio'),
                    0.05,
                ),
                poly5PeakAccelerationMaxMps2: maxOrNull(controlMetricValues(
                    'poly5PeakAccelerationMps2'
                )),
                poly5PeakSpeedMaxMps: maxOrNull(controlMetricValues('poly5PeakSpeedMps')),
                trajectoryApplyPositionErrorP95M: percentile(
                    controlMetricValues('trajectoryApplyPositionErrorM'),
                    0.95,
                ),
                trajectoryApplyVelocityErrorP95Mps: percentile(
                    controlMetricValues('trajectoryApplyVelocityErrorMps'),
                    0.95,
                ),
                controlOverrunCountMax: maxOrNull(controlMetricValues('overrunCount')),
                controlOverrunDroppedSecondsMax: maxOrNull(controlMetricValues(
                    'overrunDroppedSeconds'
                )),
                schedulerTotalDroppedSecondsMax: maxOrNull(finiteValues(
                    schedulerFrames,
                    'totalDroppedSeconds',
                )),
                terminalPhaseCounts,
                physicsUpdateIntervalP95Ms: percentile(this._physicsUpdateIntervals, 0.95),
                flightLoopIntervalP95Ms: demoMetrics.mainFrameIntervalP95Ms,
                mainMedianFps: demoMetrics.mainMedianFps,
                mainFrameIntervalP50Ms: demoMetrics.mainFrameIntervalP50Ms,
                mainFrameIntervalP95Ms: demoMetrics.mainFrameIntervalP95Ms,
                mainFrameIntervalP99Ms: demoMetrics.mainFrameIntervalP99Ms,
                mainLongFrame100MsCount: demoMetrics.mainLongFrame100MsCount,
                mainLongFrame250MsCount: demoMetrics.mainLongFrame250MsCount,
                collisionCadence: demoMetrics.collisionCadence,
                collisionCheckHz: demoMetrics.collisionCheckHz,
                collisionQueryP50Ms: demoMetrics.collisionQueryP50Ms,
                collisionQueryP95Ms: demoMetrics.collisionQueryP95Ms,
                collisionRaysPerFrameP95: demoMetrics.collisionRaysPerFrameP95,
                collisionRaysPerSecond: demoMetrics.collisionRaysPerSecond,
                collisionSweepP95Ms: demoMetrics.collisionSweepP95Ms,
                collisionNeighborhoodP95Ms: demoMetrics.collisionNeighborhoodP95Ms,
                collisionSweepRaysPerSecond: demoMetrics.collisionSweepRaysPerSecond,
                collisionNeighborhoodRaysPerSecond:
                    demoMetrics.collisionNeighborhoodRaysPerSecond,
                planningCaptureHz: demoMetrics.planningCaptureHz,
                previewCaptureHz: demoMetrics.previewCaptureHz,
                latestSlotDroppedFrames: demoMetrics.latestSlotDroppedFrames,
                adaptiveResolutionScaleMin: demoMetrics.adaptiveResolutionScaleMin,
                adaptiveResolutionScaleMax: demoMetrics.adaptiveResolutionScaleMax,
                adaptiveResolutionScaleFinal: demoMetrics.adaptiveResolutionScaleFinal,
                panoramaFacesPerSliceFinal: demoMetrics.panoramaFacesPerSliceFinal,
                tileLoad: demoMetrics.tileLoad,
                preload: demoMetrics.preload,
                droppedByReason: this._dropReasons,
                droppedByModeReason: this._dropReasonsByMode,
                crossSessionPerceptionDropped: this._crossSessionPerceptionDropped,
                calibrationIds,
            },
            perception: this._perceptionMetrics,
            collisions: this._collisionEvents,
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
