import { reportUserError } from './error-report.js';
import { PerceptionFrame, normalizePlanningState } from './perception-frame.js';

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

function cloneCaptureTransform(transform, fallbackPosition = { x: 0, y: 0, z: 0 }) {
    const source = transform || {};
    const position = source.position || fallbackPosition;
    const clone = {
        position: { x: Number(position.x || 0), y: Number(position.y || 0), z: Number(position.z || 0) },
    };
    if (source.rotation) {
        clone.rotation = {
            x: Number(source.rotation.x || 0),
            y: Number(source.rotation.y || 0),
            z: Number(source.rotation.z || 0),
        };
    }
    if (source.orientation) {
        clone.orientation = {
            x: Number(source.orientation.x || 0),
            y: Number(source.orientation.y || 0),
            z: Number(source.orientation.z || 0),
            w: Number(source.orientation.w ?? 1),
        };
    }
    return clone;
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

function dataUrlToBlob(source, scope = globalThis) {
    if (typeof source !== 'string' || !source.startsWith('data:')) return null;
    const comma = source.indexOf(',');
    if (comma < 0) throw new Error('invalid depth image data URL');
    const metadata = source.slice(5, comma);
    const encoded = source.slice(comma + 1);
    const mimeType = metadata.split(';')[0] || 'image/jpeg';
    let bytes;
    if (metadata.split(';').includes('base64')) {
        if (typeof scope.atob !== 'function') throw new Error('base64 decoder unavailable');
        const binary = scope.atob(encoded);
        bytes = new Uint8Array(binary.length);
        for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
    } else {
        bytes = new TextEncoder().encode(decodeURIComponent(encoded));
    }
    const BlobCtor = scope.Blob || globalThis.Blob;
    if (typeof BlobCtor !== 'function') throw new Error('Blob unavailable');
    return new BlobCtor([bytes], { type: mimeType });
}

/** Decode a server-returned JPEG without relying on a hidden DOM image element. */
export async function decodeDepthImageSource(source, scope = globalThis) {
    if (typeof source !== 'string' || !source) throw new Error('depth image is empty');

    if (typeof scope.createImageBitmap === 'function') {
        const blob = dataUrlToBlob(source, scope);
        if (blob) return scope.createImageBitmap(blob);
    }

    const ImageCtor = scope.Image;
    if (typeof ImageCtor !== 'function') throw new Error('no image decoder available');
    return new Promise((resolve, reject) => {
        const image = new ImageCtor();
        let settled = false;
        const finish = (callback, value) => {
            if (settled) return;
            settled = true;
            image.onload = null;
            image.onerror = null;
            callback(value);
        };
        image.onload = () => {
            const width = Number(image.naturalWidth || image.width || 0);
            const height = Number(image.naturalHeight || image.height || 0);
            if (width <= 0 || height <= 0) {
                finish(reject, new Error('decoded depth image has no pixels'));
                return;
            }
            finish(resolve, image);
        };
        image.onerror = () => finish(reject, new Error('depth JPEG decode failed'));
        image.decoding = 'async';
        image.src = source;
    });
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
        this._depthState = 'offline';
        this._depthOutcome = 'idle';
        this._depthAbortController = null;
        this._activeDepthRequest = null;
        this._depthRequestSequence = 0;
        this._lastRenderedRequestId = 0;
        this._rgbFrameId = 0;
        this._lastRequestedFrameId = -1;
        this._minimumRequestFrameId = 0;
        this._forceNextDepthRequest = false;
        this._yopoGeneration = 0;    // 目标会话变化时递增，丢弃过期响应
        this._goalSequence = 0;
        this._goalId = null;
        this._navigationMode = 'idle';
        this._navigationTransitionReason = 'initialized';
        this._yopoPending = false;
        this._yopoGoal = null;
        this._yopoPose = null;
        this._yopoYaw = 0;
        this._nextPlanningState = null;
        this._rgbFrameContext = null;
        this._perceptionFrame = null;
        this.onYopoResult = null;    // main.js: YOPO endstate → drone trajectory
        this.onDepthResult = null;   // main.js: depth latency → flight logger perf
        this.onYopoLatency = null;   // main.js: YOPO latency → flight logger perf

        if (this.rgbCanvas) {
            this.rgbCanvas.width = PANORAMA_WIDTH;
            this.rgbCanvas.height = PANORAMA_HEIGHT;
            this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        }
        this._drawDepthPlaceholder('DA360 offline');
        this._setRgbStatus('idle');
        this._setDepthState('offline', 'not-connected');
    }

    setActive(active) {
        this.active = !!active;
        this._applyVisibility();
    }

    reset() {
        const retainedDepth = this.hasDepth;
        this._abortActiveDepthRequest('sensor-reset');
        this.capturing = false;
        this.lastCaptureStartTime = 0;
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this.hasRgb = false;
        this._lastDepthArray = null;
        this._yopoGeneration++;   // 取消/到达时递增，使在途响应过期
        this._goalId = null;
        this._navigationMode = 'idle';
        this._navigationTransitionReason = 'sensor-reset';
        this._yopoGoal = null;
        this._yopoPose = null;
        this._nextPlanningState = null;
        this._rgbFrameContext = null;
        this._perceptionFrame = null;
        this._yopoPending = false;
        this._rgbFrameId = 0;
        this._lastRequestedFrameId = -1;
        this._minimumRequestFrameId = 1;
        this._forceNextDepthRequest = true;
        if (this.rgbCanvas) this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        if (!retainedDepth) {
            this.hasDepth = false;
            this._drawDepthPlaceholder('DA360 offline');
            this._setDepthLegend(null);
        }
        this._setRgbStatus('idle');
        this._setDepthState(retainedDepth ? 'preview' : 'offline', 'sensor-reset');
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

    _projectionConfig() {
        return {
            width: PANORAMA_WIDTH,
            height: PANORAMA_HEIGHT,
            faceSize: PANORAMA_FACE_SIZE,
            verticalFovDeg: PANORAMA_VERTICAL_FOV,
            faceFovDeg: PANORAMA_FACE_FOV,
            topPoleGuardDeg: PANORAMA_TOP_POLE_GUARD,
            bottomPoleGuardDeg: PANORAMA_BOTTOM_POLE_GUARD,
        };
    }

    _snapshotRgbBlob(canvas) {
        const upload = this._depthUploadCanvas(canvas);
        const snapshot = document.createElement('canvas');
        snapshot.width = upload.width;
        snapshot.height = upload.height;
        const ctx = snapshot.getContext('2d', { alpha: false });
        ctx.drawImage(upload, 0, 0, snapshot.width, snapshot.height);
        return this._canvasToJpegBlob(snapshot);
    }

    _makeRgbFrameContext(frameId, capturedAt, transform, planningState, canvas) {
        const fallbackPosition = planningState?.actualState?.position || { x: 0, y: 0, z: 0 };
        const fallbackState = planningState || normalizePlanningState({
            x: fallbackPosition.x,
            y: fallbackPosition.y,
            z: fallbackPosition.z,
            vx: 0,
            vy: 0,
            vz: 0,
        }, 0);
        return Object.freeze({
            frameId,
            capturedAt,
            transform: cloneCaptureTransform(transform, fallbackPosition),
            planningState: fallbackState,
            projectionConfig: Object.freeze(this._projectionConfig()),
            rgbPromise: this._snapshotRgbBlob(canvas),
        });
    }

    async _materializePerceptionFrame(context) {
        if (!context) throw new Error('RGB frame context unavailable');
        const rgb = await context.rgbPromise;
        if (!rgb) throw new Error('RGB JPEG encoding failed');
        const planningState = context.planningState;
        return new PerceptionFrame({
            frameId: context.frameId,
            capturedAt: context.capturedAt,
            transform: context.transform,
            rgb,
            actualState: planningState.actualState,
            referenceState: planningState.referenceState,
            yaw: planningState.yaw,
            projectionConfig: context.projectionConfig,
        });
    }

    primeFromCaptureResult(result, captureMs = 0, context = {}) {
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
        this._rgbFrameId++;
        const planningState = context.planningState || this._nextPlanningState;
        this._rgbFrameContext = this._makeRgbFrameContext(
            this._rgbFrameId,
            Number(context.capturedAt ?? now),
            context.transform,
            planningState,
            this.rgbCanvas,
        );
        this.hasRgb = true;
        this._setRgbStatus(`preloaded ${Math.round(captureMs)}ms`);
        return true;
    }

    update(world, transform, now = performance.now()) {
        if (!this.panel || !this.rgbCanvas || !world || !transform) return;
        this._applyVisibility();
        if (!this._shouldRun()) return;

        // 深度请求与全景采集解耦——用自己的定时器，不依赖采集状态。
        // 每帧检查，服务器 35ms 下理论可达 ~28Hz，gate 防止堆积。
        if (this._shouldRequestDepth(now)) {
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

    _setRgbStatus(status) {
        if (this.rgbStatusEl) this.rgbStatusEl.textContent = status;
    }

    _setDepthState(state, reason = '', options = {}) {
        const allowed = new Set(['offline', 'preview', 'planning', 'error']);
        const normalized = allowed.has(state) ? state : 'error';
        const outcome = options.outcome || (normalized === 'error' ? 'error' : 'ok');
        const latencyMs = Number(options.latencyMs);
        this._depthState = normalized;
        this._depthOutcome = outcome;

        let label = normalized;
        if (outcome === 'stale') label = `${normalized} · stale`;
        else if (normalized === 'error' && reason) label = `error · ${shortError(reason)}`;
        else if (Number.isFinite(latencyMs)) label = `${normalized} ${Math.round(latencyMs)}ms`;

        if (this.depthStatusEl) {
            this.depthStatusEl.textContent = label;
            this.depthStatusEl.dataset.state = normalized;
            this.depthStatusEl.dataset.outcome = outcome;
            this.depthStatusEl.title = reason || normalized;
        }

        const statusKey = `${normalized}|${outcome}|${reason}`;
        if (statusKey !== this._lastDepthStatusKey) {
            console.log(
                `[depth-state] mode=${normalized} goalId=${this._goalId || '-'} ` +
                `frameId=${this._rgbFrameId} generation=${this._yopoGeneration} reason=${reason || outcome}`
            );
            this._lastDepthStatusKey = statusKey;
        }
    }

    getDepthState() {
        return Object.freeze({
            mode: this._depthState,
            outcome: this._depthOutcome,
            reason: this.depthStatusEl?.title || '',
            goalId: this._goalId,
            frameId: this._rgbFrameId,
            generation: this._yopoGeneration,
            hasDepth: this.hasDepth,
        });
    }

    _desiredDepthMode() {
        return this._yopoGoal ? 'planning' : 'preview';
    }

    _shouldRequestDepth(now = performance.now()) {
        if (!this.hasRgb || this._depthGate || this.depthPending) return false;
        if (this._rgbFrameId <= this._lastRequestedFrameId) return false;
        if (this._rgbFrameId < this._minimumRequestFrameId) return false;
        if (this._yopoGoal && !this._yopoPose) {
            this._setDepthState('planning', 'awaiting-pose');
            return false;
        }
        return this._forceNextDepthRequest || now - this.lastDepthTime >= DEPTH_INTERVAL_MS;
    }

    _isRequestSessionCurrent(request) {
        if (!request) return false;
        if (request.generation !== this._yopoGeneration) return false;
        if (request.goalId !== this._goalId) return false;
        if (request.mode !== this._desiredDepthMode()) return false;
        if (request.frameId !== this._rgbFrameId) return false;
        return request.requestId >= this._lastRenderedRequestId;
    }

    _planningResponseMatchesRequest(payload, request) {
        if (request.mode !== 'planning') return true;
        return String(payload?.frame_id ?? '') === String(request.frameId)
            && String(payload?.goal_id ?? '') === String(request.goalId)
            && String(payload?.generation ?? '') === String(request.generation);
    }

    _abortActiveDepthRequest(reason) {
        const request = this._activeDepthRequest;
        if (!request) return;
        request.abortReason = reason;
        const controller = request.controller || this._depthAbortController;
        if (controller && !controller.signal.aborted) controller.abort(reason);
    }

    _markStale(request, reason) {
        const desiredMode = this._desiredDepthMode();
        this._setDepthState(desiredMode, `stale:${reason}`, { outcome: 'stale' });
        console.log(
            `[depth] mode=${request?.mode || desiredMode} goalId=${request?.goalId || '-'} ` +
            `frameId=${request?.frameId ?? this._rgbFrameId} generation=${request?.generation ?? this._yopoGeneration} ` +
            `reason=stale:${reason}`
        );
    }

    _queueLatestDepthRequest() {
        if (this._depthCatchupQueued || !this._shouldRequestDepth()) return;
        this._depthCatchupQueued = true;
        queueMicrotask(() => {
            this._depthCatchupQueued = false;
            if (this._shouldRequestDepth()) this._requestDepth(this.rgbCanvas);
        });
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
        const capturedAt = this.lastCaptureStartTime;
        const capturePlanningState = this._nextPlanningState;
        const captureTransform = cloneCaptureTransform(
            transform,
            capturePlanningState?.actualState?.position,
        );
        this._setRgbStatus('capturing');

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
                    this._setRgbStatus(rgbStatus);
                    return;
                }
                throw new Error('panorama capture returned non-drawable frame');
            }
            if (!complete) {
                const rgbStatus = captureProgressStatus(result, this.hasRgb);
                this._setRgbStatus(rgbStatus);
                return;
            }

            const ctx = this.rgbCanvas.getContext('2d');
            ctx.clearRect(0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            ctx.drawImage(panoCanvas, 0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            this.lastCaptureTime = performance.now();
            const captureMs = this.lastCaptureTime - this.lastCaptureStartTime;
            this._rgbFrameId++;
            this._rgbFrameContext = this._makeRgbFrameContext(
                this._rgbFrameId,
                capturedAt,
                captureTransform,
                capturePlanningState,
                this.rgbCanvas,
            );
            this.hasRgb = true;
            const rgbStatus = `${Math.round(captureMs)}ms`;
            this._setRgbStatus(rgbStatus);

            if (this._shouldRequestDepth(this.lastCaptureTime)) {
                this._requestDepth(this.rgbCanvas);
            }
        } catch (error) {
            reportUserError('Panorama capture failed', error, {
                key: 'panorama-capture',
                intervalMs: 3000,
            });
            this._setRgbStatus(shortError(error));
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

    async _decodeAndCommitDepthImage(source, request) {
        const decoded = await decodeDepthImageSource(source, globalThis);
        try {
            if (!this._isRequestSessionCurrent(request)) {
                this._markStale(request, 'image-decoded-after-session-change');
                return false;
            }
            if (!this.depthCanvas) throw new Error('depth canvas unavailable');
            const ctx = this.depthCanvas.getContext('2d', { alpha: false });
            if (!ctx || typeof ctx.drawImage !== 'function') throw new Error('depth canvas context unavailable');
            ctx.drawImage(decoded, 0, 0, this.depthCanvas.width, this.depthCanvas.height);
            this._lastRenderedRequestId = request.requestId;
            this.hasDepth = true;
            return true;
        } finally {
            if (decoded && typeof decoded.close === 'function') decoded.close();
        }
    }

    /** 统一请求入口：有 YOPO 目标时走 /yopo/plan_full（一次 JPEG → DA360+YOPO+深度），
     *  无目标时走 /depth（仅 DA360+深度显示）。消除两段式双 HTTP 往返。 */
    async _requestDepth(canvas) {
        if (this._depthGate) return;
        if (!canvas) return;
        const mode = this._desiredDepthMode();
        if (mode === 'planning' && !this._yopoPose) {
            this._setDepthState('planning', 'awaiting-pose');
            return;
        }

        const frameContext = this._rgbFrameContext;
        if (!frameContext) {
            this._setDepthState(mode, 'awaiting-complete-rgb-frame');
            return false;
        }
        const request = {
            requestId: ++this._depthRequestSequence,
            frameId: frameContext.frameId,
            generation: this._yopoGeneration,
            goalId: this._goalId,
            mode,
            goal: this._yopoGoal ? { ...this._yopoGoal } : null,
            frameContext,
            perceptionFrame: null,
            controller: null,
            abortReason: null,
        };
        this._depthReqStart = performance.now();
        this._lastRequestedFrameId = request.frameId;
        this._forceNextDepthRequest = false;
        this._depthGate = true;
        this.depthPending = true;
        this._activeDepthRequest = request;
        this._setDepthState(mode, 'request-started');

        let timeout = null;
        let tA = performance.now();
        let tB = tA;

        try {
            const frame = await this._materializePerceptionFrame(frameContext);
            request.perceptionFrame = frame;
            this._perceptionFrame = frame;
            const blob = frame.rgb;
            tB = performance.now();
            if (!this._isRequestSessionCurrent(request)) {
                this._markStale(request, 'session-changed-before-fetch');
                return false;
            }

            const controller = new AbortController();
            request.controller = controller;
            this._depthAbortController = controller;
            timeout = window.setTimeout(() => {
                request.abortReason = 'timeout';
                controller.abort('timeout');
            }, DA360_TIMEOUT_MS);

            let url, body, headers;
            if (request.mode === 'planning') {
                // 一次调用：DA360 → YOPO → 返回 endstate + depth_image
                const observation = frame.planningObservation(request.goal);
                const pose = observation.position;
                const velocity = observation.velocity;
                const acceleration = observation.acceleration;
                const qs = new URLSearchParams({
                    px: String(pose.x), py: String(pose.y), pz: String(pose.z),
                    gx: String(observation.goal.x), gy: String(observation.goal.y), gz: String(observation.goal.z),
                    vx: String(velocity.x), vy: String(velocity.y), vz: String(velocity.z),
                    ax: String(acceleration.x), ay: String(acceleration.y), az: String(acceleration.z),
                    yaw: String(observation.yaw),
                    frame_id: String(request.frameId),
                    goal_id: String(request.goalId),
                    generation: String(request.generation),
                }).toString();
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
            if (!this._isRequestSessionCurrent(request)) {
                this._markStale(request, 'response-after-session-change');
                return false;
            }
            if (!this._planningResponseMatchesRequest(payload, request)) {
                this._markStale(request, 'response-identity-mismatch');
                return false;
            }
            if (!payload.depth_image) throw new Error('response missing depth_image');
            if (!await this._decodeAndCommitDepthImage(payload.depth_image, request)) return false;

            this._depthLatency = Number.isFinite(Number(payload.latency_ms))
                ? `${Math.round(Number(payload.latency_ms))}ms`
                : '';
            this._setDepthState(request.mode, 'depth-ready', { latencyMs: Number(payload.latency_ms) });

            if (request.mode === 'planning') {
                if (payload.endstate && this.onYopoResult) {
                    const context = Object.freeze({
                        mode: request.mode,
                        goalId: request.goalId,
                        frameId: request.frameId,
                        generation: request.generation,
                        requestId: request.requestId,
                        depthMode: payload.depth_mode || null,
                        calibrationId: payload.calibration_id || null,
                        timings: payload.timings_ms || null,
                    });
                    this.onYopoResult(payload.endstate, payload.traj_time, context);
                    if (this.onYopoLatency) this.onYopoLatency(payload.latency_ms);
                }
            }

            // 帧率打点 + 限速诊断
            const now = performance.now();
            this._depthFpsCount++;
            this._depthCycleSum += now - this._depthReqStart;
            if (now - this._depthFpsTimer > 2000) {
                const fps = this._depthFpsCount / ((now - this._depthFpsTimer) / 1000);
                const avgCycle = this._depthCycleSum / Math.max(1, this._depthFpsCount);
                console.log(
                    `[depth] ${fps.toFixed(1)}Hz mode=${request.mode} goalId=${request.goalId || '-'} ` +
                    `frameId=${request.frameId} generation=${request.generation} reason=ok ` +
                    `srvLat=${Math.round(payload.latency_ms || 0)}ms avgCycle=${avgCycle.toFixed(0)}ms ` +
                    `prepare=${Math.round(tB - tA)}ms`
                );
                this._depthFpsTimer = now; this._depthFpsCount = 0; this._depthCycleSum = 0;
            }
            this.lastDepthTime = performance.now();
            if (this.onDepthResult) this.onDepthResult(payload.latency_ms);
            return true;
        } catch (error) {
            const sessionChanged = !this._isRequestSessionCurrent(request);
            const transitionAbort = request.abortReason && request.abortReason !== 'timeout';
            if (sessionChanged || transitionAbort) {
                this._markStale(request, request.abortReason || 'session-changed');
                return false;
            }
            reportUserError('DA360/YOPO request failed', error, {
                key: 'da360-depth-request', intervalMs: 3000,
            });
            this.lastDepthTime = performance.now();
            const offline = request.abortReason === 'timeout' || error?.name === 'TypeError';
            this._setDepthState(offline ? 'offline' : 'error', shortError(error));
            return false;
        } finally {
            if (timeout !== null) window.clearTimeout(timeout);
            if (this._activeDepthRequest === request) {
                this._activeDepthRequest = null;
                this._depthAbortController = null;
                this.depthPending = false;
                this._depthGate = false;
                this._queueLatestDepthRequest();
            }
        }
    }

    setYopoGoal(goal) {
        if (!goal) {
            this.resetYopoGoal('empty-goal');
            return null;
        }
        this._abortActiveDepthRequest('goal-changed');
        this._yopoGeneration++;
        this._goalId = `goal-${++this._goalSequence}`;
        this._yopoGoal = Object.freeze({ ...goal });
        this._navigationMode = 'active';
        this._navigationTransitionReason = 'goal-set';
        this._minimumRequestFrameId = this._rgbFrameId + 1;
        this._forceNextDepthRequest = true;
        this._setDepthState('planning', 'awaiting-new-rgb-frame');
        return this._goalId;
    }

    resetYopoGoal(reason = 'goal-reset') {
        this._abortActiveDepthRequest(reason);
        this._yopoGeneration++;
        this._goalId = null;
        this._yopoGoal = null;
        this._navigationMode = reason.includes('arriv') ? 'arrived'
            : reason.includes('cancel') || reason.includes('mode') || reason.includes('reset') ? 'cancelled'
            : 'idle';
        this._navigationTransitionReason = reason;
        this._yopoPending = false;
        this._minimumRequestFrameId = this._rgbFrameId + 1;
        this._forceNextDepthRequest = true;
        // Keep the last successfully decoded canvas frame while returning to preview.
        this._setDepthState('preview', reason);
    }

    setYopoPose(pose, yaw) {
        if (!pose) {
            this._yopoPose = null;
            this._nextPlanningState = null;
            return;
        }
        const planningState = normalizePlanningState(pose, yaw);
        this._nextPlanningState = planningState;
        this._yopoPose = planningState.actualState;
        this._yopoYaw = planningState.yaw;
    }

    getLatestPerceptionFrame() {
        return this._perceptionFrame;
    }

    getNavigationSession() {
        return Object.freeze({
            mode: this._navigationMode,
            goalId: this._goalId,
            generation: this._yopoGeneration,
            transitionReason: this._navigationTransitionReason,
        });
    }
}
