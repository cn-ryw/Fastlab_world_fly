/**
 * Flight data logger — records pose, velocity, goal, and controller state
 * during goal-based autonomous navigation.  Auto-starts on goal set,
 * auto-stops on arrival or cancel, then downloads a JSON log.
 */

export class FlightLogger {
    constructor() {
        this._recording = false;
        this._frames = [];
        this._startTime = null;
        this._goal = null;
        this._spawnAlt = null;
        // 性能统计
        this._depthCount = 0;
        this._depthLatencies = [];
        this._yopoCount = 0;
        this._yopoLatencies = [];
        this._yopoTrackerCount = 0;  // 帧数：有活跃 YOPO 轨迹
    }

    /** Start a new recording session. Called when a goal is set. */
    start(goal, spawnAltitude) {
        this._recording = true;
        this._frames = [];
        this._startTime = performance.now();
        this._goal = { x: goal.x, y: goal.y, z: goal.z };
        this._spawnAlt = spawnAltitude;
        this._depthCount = 0;
        this._depthLatencies = [];
        this._yopoCount = 0;
        this._yopoLatencies = [];
        this._yopoTrackerCount = 0;
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

    /** Record one frame. Call from updateFlight(). */
    record(drone, refX, refY, refZ) {
        if (!this._recording || !drone) return;
        const t = (performance.now() - this._startTime) / 1000;
        this._frames.push({
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

        const log = {
            startTime: new Date().toISOString(),
            spawnAltitude: this._spawnAlt,
            goal: this._goal,
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
            },
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
