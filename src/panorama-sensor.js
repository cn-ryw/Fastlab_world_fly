import { reportUserError } from './error-report.js';

function urlNumber(name, fallback, min, max) {
    const value = new URLSearchParams(window.location.search).get(name);
    if (value == null || value === '') return fallback;
    const n = Number(value);
    if (!Number.isFinite(n)) return fallback;
    return Math.max(min, Math.min(max, n));
}

function evenNumber(value) {
    const n = Math.max(2, Math.round(value));
    return n % 2 === 0 ? n : n + 1;
}

// 全景采集间隔：plan_full 已降至 ~35ms，采集对齐即可达 20+ Hz
const CAPTURE_INTERVAL_MS = urlNumber('panoMs', 20, 16, 10000);
// 深度请求间隔：50Hz 名义，由 _depthGate 非阻塞漏桶调节实际吞吐
const DEPTH_INTERVAL_MS = urlNumber('depthMs', 20, 16, 10000);
// DA360 超时：含冷启动首次推理裕度
const DA360_TIMEOUT_MS = urlNumber('da360TimeoutMs', 20000, 2000, 60000);
const DA360_UPLOAD_SCALE = urlNumber('da360UploadScale', 0.35, 0.05, 1);
const DA360_UPLOAD_WIDTH = Math.round(urlNumber('da360UploadWidth', 0, 0, 5760));
const DA360_UPLOAD_HEIGHT = Math.round(urlNumber('da360UploadHeight', 0, 0, 2880));
const PANORAMA_WIDTH = evenNumber(urlNumber('panoWidth', 384, 280, 5760));
const PANORAMA_HEIGHT = evenNumber(urlNumber('panoHeight', Math.round(PANORAMA_WIDTH / 2), 140, 2880));
// 每面渲染分辨率：96 匹配 384px ERP 输出需求（384/360×130≈139），
// 优先释放主线程以提升深度帧率，DA360 对此分辨率差异不敏感。
const PANORAMA_FACE_SIZE = Math.round(urlNumber('panoFace', 96, 64, 2048));
const PANORAMA_VERTICAL_FOV = urlNumber('panoVfov', 180, 30, 180);
const PANORAMA_JPEG_QUALITY = urlNumber('panoJpeg', 0.74, 0.35, 0.95);
const PANORAMA_FACE_FOV = urlNumber('panoFaceFov', 130, 90, 170);
const PANORAMA_TOP_POLE_GUARD = urlNumber('panoTopPoleGuard', 10, 0, 45);
const PANORAMA_BOTTOM_POLE_GUARD = urlNumber('panoBottomPoleGuard', 2, 0, 45);
const PANORAMA_FRAME_DELAY_MS = urlNumber('panoFrameDelayMs', 0, 0, 1000);
const PANORAMA_FACE_TILE_TIMEOUT_MS = urlNumber('panoFaceTileTimeoutMs', 400, 0, 10000);
const PANORAMA_FACE_TILE_QUIET_MS = urlNumber('panoFaceTileQuietMs', 0, 0, 5000);
const PANORAMA_CAPTURE_ANYWAY = urlNumber('panoCaptureAnyway', 1, 0, 1) >= 0.5;
const PANORAMA_PRELOAD_FRAME_DELAY_MS = urlNumber(
    'panoPreloadFrameDelayMs',
    Math.max(96, PANORAMA_FRAME_DELAY_MS),
    0,
    1000
);
const PANORAMA_PRELOAD_FACE_TILE_TIMEOUT_MS = urlNumber('panoPreloadFaceTileTimeoutMs', 6000, 500, 30000);
const PANORAMA_PRELOAD_FACE_TILE_QUIET_MS = urlNumber('panoPreloadFaceTileQuietMs', 650, 0, 5000);
const PANORAMA_PRELOAD_TIMEOUT_MS = urlNumber('panoPreloadTimeoutMs', 60000, 500, 120000);

function getDA360Endpoint() {
    const params = new URLSearchParams(window.location.search);
    const explicit = params.get('da360Url');
    if (explicit) return explicit;

    const host = params.get('da360Host') || window.location.hostname || '127.0.0.1';
    const port = params.get('da360Port') || '5688';
    return `http://${host}:${port}/depth`;
}

function getDA360RawEndpoint() {
    const params = new URLSearchParams(window.location.search);
    const host = params.get('da360Host') || window.location.hostname || '127.0.0.1';
    const port = params.get('da360Port') || '5688';
    return `http://${host}:${port}/depth/raw`;
}

function getYopoPlanEndpoint() {
    const params = new URLSearchParams(window.location.search);
    const host = params.get('da360Host') || window.location.hostname || '127.0.0.1';
    const port = params.get('da360Port') || '5688';
    return `http://${host}:${port}/yopo/plan`;
}

function getYopoEndpoint() {
    const params = new URLSearchParams(window.location.search);
    const host = params.get('da360Host') || window.location.hostname || '127.0.0.1';
    const port = params.get('da360Port') || '5688';
    return `http://${host}:${port}/yopo/plan_full`;
}

function shortError(error) {
    const message = error && error.message ? error.message : String(error || 'error');
    return message.length > 52 ? `${message.slice(0, 49)}...` : message;
}

function isDrawableImageSource(value) {
    if (!value || !Number.isFinite(value.width) || !Number.isFinite(value.height)) return false;
    if (value.width <= 0 || value.height <= 0) return false;
    if (typeof HTMLCanvasElement !== 'undefined' && value instanceof HTMLCanvasElement) return true;
    if (typeof OffscreenCanvas !== 'undefined' && value instanceof OffscreenCanvas) return true;
    if (typeof ImageBitmap !== 'undefined' && value instanceof ImageBitmap) return true;
    if (typeof HTMLImageElement !== 'undefined' && value instanceof HTMLImageElement) return true;
    if (typeof HTMLVideoElement !== 'undefined' && value instanceof HTMLVideoElement) return true;
    return false;
}

function captureProgressStatus(result, hasRgb) {
    const faceIndex = result && Number.isFinite(result.faceIndex) ? result.faceIndex : 0;
    const faceCount = result && Number.isFinite(result.faces) ? result.faces : 6;
    if (result && result.loadingTiles) return `tiles ${faceIndex + 1}/${faceCount}`;
    if (hasRgb) return 'ready';
    return `scanning ${faceIndex}/${faceCount}`;
}

export class PanoramaSensor {
    constructor() {
        this.panel = document.getElementById('panorama-sensor-panel');
        this.rgbCanvas = document.getElementById('panorama-rgb-canvas');
        this.depthCanvas = document.getElementById('panorama-depth-canvas');
        if (this.depthCanvas) {
            this.depthCanvas.width = PANORAMA_WIDTH;
            this.depthCanvas.height = PANORAMA_HEIGHT;
        }
        this.depthImg = document.getElementById('panorama-depth-image');  // plan_full JPEG 显示
        this.rgbStatusEl = document.getElementById('panorama-rgb-status');
        this.depthStatusEl = document.getElementById('panorama-depth-status');
        this.depthNearLabelEl = document.getElementById('panorama-depth-near-label');
        this.depthFarLabelEl = document.getElementById('panorama-depth-far-label');
        this.depthUnitEl = document.getElementById('panorama-depth-unit');
        this.endpoint = getDA360Endpoint();
        this.active = false;
        this.capturing = false;
        this.depthPending = false;
        this.lastCaptureStartTime = 0;
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this.hasRgb = false;
        this.hasDepth = false;
        this._depthLatency = '';
        // YOPO planning
        // YOPO planning — 缓存 DA360 推理结果，直通 /yopo/plan 避免二次推理
        this._lastDepthArray = null;  // float32[H][W] 嵌套二维数组，来自 /depth 响应
        this._depthGate = false;       // RuntimeRateGate: 防止深度请求堆积
        this._depthFpsTimer = 0;       // 深度帧率打点
        this._depthFpsCount = 0;
        this._depthCycleSum = 0;         // 限速诊断：累计周期间隔
        this._depthReqStart = 0;         // 当前请求发起时间
        this._yopoGeneration = 0;    // 取消/到达时递增，丢弃过期响应
        this._yopoPending = false;
        this._yopoGoal = null;
        this._yopoPose = null;
        this._yopoYaw = 0;
        this.onYopoResult = null;    // main.js: YOPO endstate → drone trajectory
        this.onDepthResult = null;   // main.js: depth latency → flight logger perf
        this.onYopoLatency = null;   // main.js: YOPO latency → flight logger perf

        if (this.rgbCanvas) {
            this.rgbCanvas.width = PANORAMA_WIDTH;
            this.rgbCanvas.height = PANORAMA_HEIGHT;
            this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        }
        this._drawDepthPlaceholder('DA360 offline');
        this._setStatus('idle', 'offline');
    }

    setActive(active) {
        this.active = !!active;
        this._applyVisibility();
    }

    reset() {
        this.capturing = false;
        this.depthPending = false;
        this.lastCaptureStartTime = 0;
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this.hasRgb = false;
        this.hasDepth = false;
        this._lastDepthArray = null;
        this._yopoGeneration++;   // 取消/到达时递增，使在途响应过期
        this._yopoGoal = null;
        this._yopoPose = null;
        this._yopoPending = false;
        if (this.rgbCanvas) this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        this._drawDepthPlaceholder('DA360 offline');
        this._setStatus('idle', 'offline');
        this._setDepthLegend(null);
    }

    hasRgbFrame() {
        return this.hasRgb;
    }

    getCaptureOptions(options = {}) {
        const preload = !!options.preload;
        return {
            width: PANORAMA_WIDTH,
            height: PANORAMA_HEIGHT,
            faceSize: PANORAMA_FACE_SIZE,
            verticalFovDeg: PANORAMA_VERTICAL_FOV,
            faceFovDeg: PANORAMA_FACE_FOV,
            topPoleGuardDeg: PANORAMA_TOP_POLE_GUARD,
            bottomPoleGuardDeg: PANORAMA_BOTTOM_POLE_GUARD,
            frameDelayMs: preload ? PANORAMA_PRELOAD_FRAME_DELAY_MS : PANORAMA_FRAME_DELAY_MS,
            tileTimeoutMs: preload ? PANORAMA_PRELOAD_FACE_TILE_TIMEOUT_MS : PANORAMA_FACE_TILE_TIMEOUT_MS,
            tileQuietMs: preload ? PANORAMA_PRELOAD_FACE_TILE_QUIET_MS : PANORAMA_FACE_TILE_QUIET_MS,
            captureAnyway: PANORAMA_CAPTURE_ANYWAY,  // Always captureAnyway: skip tile-wait for preload too, else face 1/6 times out
            timeoutMs: preload ? PANORAMA_PRELOAD_TIMEOUT_MS : 0,
        };
    }

    primeFromCaptureResult(result, captureMs = 0) {
        if (!this.rgbCanvas) return false;
        const structuredResult = result && typeof result === 'object' && 'complete' in result;
        const panoCanvas = structuredResult ? result.canvas : result;
        const complete = structuredResult ? result.complete !== false : true;
        if (!complete || !isDrawableImageSource(panoCanvas)) return false;

        const ctx = this.rgbCanvas.getContext('2d');
        ctx.clearRect(0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
        ctx.drawImage(panoCanvas, 0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
        const now = performance.now();
        this.lastCaptureStartTime = now;
        this.lastCaptureTime = now;
        this.hasRgb = true;
        this._setStatus(`preloaded ${Math.round(captureMs)}ms`, this.hasDepth ? 'ready' : 'offline');
        return true;
    }

    update(world, transform, now = performance.now()) {
        if (!this.panel || !this.rgbCanvas || !world || !transform) return;
        this._applyVisibility();
        if (!this._shouldRun()) return;

        // 深度请求与全景采集解耦——用自己的定时器，不依赖采集状态。
        // 每帧检查，服务器 35ms 下理论可达 ~28Hz，gate 防止堆积。
        if (this.hasRgb && !this.depthPending && now - this.lastDepthTime >= DEPTH_INTERVAL_MS) {
            this._requestDepth(this.rgbCanvas);
        }

        // 全景采集：catch stuck captures, skip if busy
        if (this.capturing) {
            const stuckMs = PANORAMA_FACE_TILE_TIMEOUT_MS * 6 + 500;
            if (PANORAMA_CAPTURE_ANYWAY && now - this.lastCaptureStartTime > stuckMs) {
                this.capturing = false;
            } else {
                return;
            }
        }
        if (now - this.lastCaptureStartTime < CAPTURE_INTERVAL_MS) return;
        this._capture(world, transform);
    }

    _enabledByUi() {
        const toggle = document.getElementById('panorama-toggle');
        return toggle ? toggle.checked : true;
    }

    _shouldRun() {
        const cleanMode = document.getElementById('clean-mode-toggle')?.checked ? true : false;
        return this.active && this._enabledByUi() && !cleanMode;
    }

    _applyVisibility() {
        if (!this.panel) return;
        this.panel.classList.toggle('visible', this._shouldRun());
    }

    _drawPlaceholder(canvas, label) {
        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        const gradient = ctx.createLinearGradient(0, 0, canvas.width, canvas.height);
        gradient.addColorStop(0, '#030712');
        gradient.addColorStop(0.55, '#111827');
        gradient.addColorStop(1, '#020617');
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.strokeStyle = 'rgba(125, 211, 252, 0.28)';
        ctx.lineWidth = 2;
        for (let x = 0; x <= canvas.width; x += 64) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, canvas.height);
            ctx.stroke();
        }
        for (let y = 0; y <= canvas.height; y += 64) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(canvas.width, y);
            ctx.stroke();
        }
        ctx.fillStyle = 'rgba(226, 232, 240, 0.78)';
        ctx.font = '24px Courier New, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(label, canvas.width * 0.5, canvas.height * 0.52);
    }

    _drawDepthPlaceholder(label) {
        if (!this.depthCanvas) return;
        this._drawPlaceholder(this.depthCanvas, label);
    }

    /** 将 float32 深度数组直接渲染到 Canvas，零编解码延迟。 */
    _renderDepthToCanvas(depthArray) {
        if (!this.depthCanvas || !depthArray || !depthArray.length) return;
        const H = depthArray.length;
        const W = depthArray[0] ? depthArray[0].length : 0;
        if (W === 0) return;

        // 兼容尺寸：服务器可能返回不同于 PANORAMA_WIDTH/PANORAMA_HEIGHT 的尺寸
        if (this.depthCanvas.width !== W) this.depthCanvas.width = W;
        if (this.depthCanvas.height !== H) this.depthCanvas.height = H;

        const ctx = this.depthCanvas.getContext('2d', { willReadFrequently: false });
        const imgData = ctx.createImageData(W, H);
        const buf = imgData.data;

        // Inferno 伪彩色：t=0 暗(黑) → t=1 亮(黄)。
        // depth_array 是 1/pred_disp min-归一化：近处≈1、远处≈很大。
        // 需要翻转：近→t=1（亮黄）、远→t=0（暗黑），与服务端 depth_to_color 一致。
        const inferno = (t) => {
            t = Math.max(0, Math.min(1, t));
            let r, g, b;
            if (t < 0.333) {
                const s = t / 0.333;
                r = Math.round(s * 100);
                g = 0;
                b = Math.round(s * 100 + 30 * (1 - s));
            } else if (t < 0.666) {
                const s = (t - 0.333) / 0.333;
                r = Math.round(100 + s * 155);
                g = Math.round(s * 50);
                b = Math.round(100 * (1 - s));
            } else {
                const s = (t - 0.666) / 0.334;
                r = 255;
                g = Math.round(50 + s * 205);
                b = Math.round(s * 40);
            }
            return [r, g, b, 255];
        };

        // 一趟扫描：min/max + 映射（t 翻转：近=1→黄，远=0→黑）
        let vmin = Infinity, vmax = -Infinity;
        for (let y = 0; y < H; y++) {
            for (let x = 0; x < W; x++) {
                const v = depthArray[y][x];
                if (Number.isFinite(v)) {
                    if (v < vmin) vmin = v;
                    if (v > vmax) vmax = v;
                }
            }
        }
        const range = vmax > vmin ? vmax - vmin : 1;
        for (let y = 0; y < H; y++) {
            for (let x = 0; x < W; x++) {
                const v = depthArray[y][x];
                const t = Number.isFinite(v) ? 1.0 - (v - vmin) / range : 0;
                const [r, g, b, a] = inferno(t);
                const idx = (y * W + x) * 4;
                buf[idx] = r; buf[idx + 1] = g; buf[idx + 2] = b; buf[idx + 3] = a;
            }
        }
        ctx.putImageData(imgData, 0, 0);
    }

    _setStatus(rgbStatus, depthStatus) {
        if (this.rgbStatusEl) this.rgbStatusEl.textContent = rgbStatus;
        if (this.depthStatusEl) this.depthStatusEl.textContent = depthStatus;
    }

    _updateDepthDisplay() {
        this._setStatus(this.hasRgb ? 'ready' : 'idle', this._depthLatency || 'ready');
    }

    _formatRelativeDepth(value) {
        const n = Number(value);
        if (!Number.isFinite(n) || n <= 0) return '--';
        if (n < 10) return `${n.toFixed(1)}x`;
        if (n < 100) return `${Math.round(n)}x`;
        return `${n.toExponential(1)}x`;
    }

    _setDepthLegend(scale) {
        const valid = scale && scale.valid;
        if (this.depthUnitEl) {
            this.depthUnitEl.textContent = valid ? 'x nearest' : 'relative';
        }
        if (this.depthNearLabelEl) {
            const near = valid ? this._formatRelativeDepth(scale.near) : '1x';
            this.depthNearLabelEl.textContent = `near ${near}`;
        }
        if (this.depthFarLabelEl) {
            const far = valid ? this._formatRelativeDepth(scale.far) : '--';
            this.depthFarLabelEl.textContent = `far ${far}`;
        }
    }

    async _capture(world, transform) {
        this.capturing = true;
        this.lastCaptureStartTime = performance.now();
        this._setStatus('capturing', this.depthPending ? 'inferring' : (this.hasRgb ? 'ready' : 'offline'));

        try {
            const capture = typeof world.capturePanoramaIncrementalAsync === 'function'
                ? world.capturePanoramaIncrementalAsync.bind(world)
                : typeof world.capturePanoramaAsync === 'function'
                ? world.capturePanoramaAsync.bind(world)
                : world.capturePanorama.bind(world);
            const result = await capture(transform, this.getCaptureOptions());
            const structuredResult = result && typeof result === 'object' && 'complete' in result;
            const panoCanvas = structuredResult ? result.canvas : result;
            const complete = structuredResult ? result.complete !== false : true;
            if (!isDrawableImageSource(panoCanvas)) {
                if (!complete || structuredResult) {
                    const rgbStatus = captureProgressStatus(result, this.hasRgb);
                    this._setStatus(rgbStatus, this.depthPending ? 'inferring' : (this.hasDepth ? 'ready' : 'offline'));
                    return;
                }
                throw new Error('panorama capture returned non-drawable frame');
            }
            if (!complete) {
                const rgbStatus = captureProgressStatus(result, this.hasRgb);
                this._setStatus(rgbStatus, this.depthPending ? 'inferring' : (this.hasDepth ? 'ready' : 'offline'));
                return;
            }

            const ctx = this.rgbCanvas.getContext('2d');
            ctx.clearRect(0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            ctx.drawImage(panoCanvas, 0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            this.lastCaptureTime = performance.now();
            const captureMs = this.lastCaptureTime - this.lastCaptureStartTime;
            this.hasRgb = true;
            const rgbStatus = `${Math.round(captureMs)}ms`;
            this._setStatus(rgbStatus, this.depthPending ? 'inferring' : (this.hasDepth ? 'ready' : 'offline'));

            if (!this.depthPending && this.lastCaptureTime - this.lastDepthTime >= DEPTH_INTERVAL_MS) {
                this._requestDepth(this.rgbCanvas);
            }
        } catch (error) {
            reportUserError('Panorama capture failed', error, {
                key: 'panorama-capture',
                intervalMs: 3000,
            });
            this._setStatus(shortError(error), this.depthPending ? 'inferring' : 'offline');
        } finally {
            this.capturing = false;
        }
    }

    _depthUploadCanvas(canvas) {
        if (!canvas || !canvas.width || !canvas.height) return canvas;
        const explicitWidth = DA360_UPLOAD_WIDTH > 0 ? DA360_UPLOAD_WIDTH : 0;
        const explicitHeight = DA360_UPLOAD_HEIGHT > 0 ? DA360_UPLOAD_HEIGHT : 0;
        let width = explicitWidth;
        let height = explicitHeight;

        if (width && !height) {
            height = Math.max(2, Math.round(width * canvas.height / canvas.width));
        } else if (!width && height) {
            width = Math.max(2, Math.round(height * canvas.width / canvas.height));
        } else if (!width && !height) {
            width = Math.max(2, Math.round(canvas.width * DA360_UPLOAD_SCALE));
            height = Math.max(2, Math.round(canvas.height * DA360_UPLOAD_SCALE));
        }

        width = Math.min(canvas.width, Math.max(2, width));
        height = Math.min(canvas.height, Math.max(2, height));
        if (width === canvas.width && height === canvas.height) return canvas;

        if (!this.depthUploadCanvas) this.depthUploadCanvas = document.createElement('canvas');
        this.depthUploadCanvas.width = width;
        this.depthUploadCanvas.height = height;
        const ctx = this.depthUploadCanvas.getContext('2d', { alpha: false });
        ctx.imageSmoothingEnabled = true;
        ctx.imageSmoothingQuality = 'medium';
        ctx.clearRect(0, 0, width, height);
        ctx.drawImage(canvas, 0, 0, width, height);
        return this.depthUploadCanvas;
    }

    _canvasToJpegBlob(canvas) {
        return new Promise(resolve => {
            if (!canvas || typeof canvas.toBlob !== 'function') {
                resolve(null);
                return;
            }
            canvas.toBlob(resolve, 'image/jpeg', PANORAMA_JPEG_QUALITY);
        });
    }

    /** 统一请求入口：有 YOPO 目标时走 /yopo/plan_full（一次 JPEG → DA360+YOPO+深度），
     *  无目标时走 /depth（仅 DA360+深度显示）。消除两段式双 HTTP 往返。 */
    async _requestDepth(canvas) {
        if (this._depthGate) return;
        if (!canvas) return;
        this._depthReqStart = performance.now();
        this._depthGate = true;
        this.depthPending = true;
        this._setStatus('ready', 'inferring');
        // snapshot 必须在 await 之前——await 期间 _yopoGoal/_yopoPose 可能被 resetYopoGoal 清掉
        const yopoGoal = this._yopoGoal, yopoPose = this._yopoPose, yopoYaw = this._yopoYaw;
        const usePlanFull = !!(yopoGoal && yopoPose);
        const started = performance.now();

        const tA = performance.now();
        const uploadCanvas = this._depthUploadCanvas(canvas);
        const blob = await this._canvasToJpegBlob(uploadCanvas);
        const tB = performance.now();
        if (!blob) { this._depthGate = false; this.depthPending = false; return; }

        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), DA360_TIMEOUT_MS);

        try {
            let url, body, headers;
            if (usePlanFull) {
                // 一次调用：DA360 → YOPO → 返回 endstate + depth_image
                const pose = yopoPose, goal = yopoGoal, yaw = yopoYaw;
                const qs = [`px=${pose.x}`,`py=${pose.y}`,`pz=${pose.z}`,
                           `gx=${goal.x}`,`gy=${goal.y}`,`gz=${goal.z}`,
                           `vx=${pose.vx||0}`,`vy=${pose.vy||0}`,`vz=${pose.vz||0}`,
                           `yaw=${yaw}`].join('&');
                url = `${getYopoEndpoint()}?${qs}`;  // /yopo/plan_full
                headers = { 'Content-Type': 'image/jpeg' };
                body = blob;
            } else {
                url = this.endpoint;  // /depth
                headers = { 'Content-Type': blob.type || 'image/jpeg' };
                body = blob;
            }

            const response = await fetch(url, { method: 'POST', headers, body, signal: controller.signal });
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            const payload = await response.json();

            if (usePlanFull) {
                // plan_full 响应：endstate + depth_image JPEG（~6KB，原项目做法）
                if (payload.depth_image && this.depthImg) {
                    this.depthImg.src = payload.depth_image;
                    this.hasDepth = true;
                    this._updateDepthDisplay();
                }
                if (payload.endstate && this.onYopoResult) {
                    this.onYopoResult(payload.endstate, payload.traj_time);
                    if (this.onYopoLatency) this.onYopoLatency(payload.latency_ms);
                }
            } else {
                // /depth 响应：depth_array → Canvas 直绘
                if (payload.depth_array) {
                    this._lastDepthArray = payload.depth_array;
                    this._renderDepthToCanvas(payload.depth_array);
                    this.hasDepth = true;
                    this._updateDepthDisplay();
                }
            }

            // 帧率打点 + 限速诊断
            const now = performance.now();
            this._depthFpsCount++;
            this._depthCycleSum += now - this._depthReqStart;
            if (now - this._depthFpsTimer > 2000) {
                const fps = this._depthFpsCount / ((now - this._depthFpsTimer) / 1000);
                const avgCycle = this._depthCycleSum / Math.max(1, this._depthFpsCount);
                console.log(`[depth] ${fps.toFixed(1)}Hz plan_full=${usePlanFull} srvLat=${Math.round(payload.latency_ms||0)}ms avgCycle=${avgCycle.toFixed(0)}ms prepare=${Math.round(tB-tA)}ms`);
                this._depthFpsTimer = now; this._depthFpsCount = 0; this._depthCycleSum = 0;
            }
            this.lastDepthTime = performance.now();
            if (this.onDepthResult) this.onDepthResult(payload.latency_ms);
        } catch (error) {
            reportUserError('DA360/YOPO request failed', error, {
                key: 'da360-depth-request', intervalMs: 3000,
            });
            this.lastDepthTime = performance.now();
            this._setStatus('ready', shortError(error));
        } finally {
            window.clearTimeout(timeout);
            this.depthPending = false;
            this._depthGate = false;
        }
    }

    setYopoGoal(goal) { this._yopoGoal = goal; }
    resetYopoGoal() { this._yopoGeneration++; this._yopoGoal = null; this._yopoPending = false; }
    setYopoPose(pose, yaw) { this._yopoPose = pose; this._yopoYaw = yaw; }
}
