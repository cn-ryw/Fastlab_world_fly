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
const PANORAMA_FACES_PER_SLICE = Math.round(urlNumber('panoFacesPerSlice', 2, 1, 6));
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

async function sha256Blob(blob) {
    if (!globalThis.crypto?.subtle || !blob?.arrayBuffer) return null;
    const digest = await globalThis.crypto.subtle.digest('SHA-256', await blob.arrayBuffer());
    return Array.from(new Uint8Array(digest), byte => byte.toString(16).padStart(2, '0')).join('');
}

function safeArtifactId(value, fallback) {
    const normalized = String(value || fallback).replace(/[^A-Za-z0-9_-]+/g, '-').replace(/^-+|-+$/g, '');
    return normalized.slice(0, 80) || fallback;
}

function createCalibrationSessionId() {
    if (typeof globalThis.crypto?.randomUUID === 'function') {
        return safeArtifactId(globalThis.crypto.randomUUID(), 'calibration-session');
    }
    const random = Math.random().toString(36).slice(2, 12);
    return safeArtifactId(`session-${Date.now().toString(36)}-${random}`, 'calibration-session');
}

function downloadBlob(blob, filename) {
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement('a');
    anchor.href = url;
    anchor.download = filename;
    document.body.appendChild(anchor);
    anchor.click();
    document.body.removeChild(anchor);
    window.setTimeout(() => URL.revokeObjectURL(url), 0);
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
        this._captureAbortController = null;
        this._captureIdlePromise = null;
        this._captureSequence = 0;
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
        this._calibrationSessionId = createCalibrationSessionId();
        this._calibrationCapturePending = false;
        this._calibrationAbortController = null;
        this._calibrationGeneration = 0;
        this._consumedCalibrationFrames = new Set();
        this._consumedCalibrationCaptureIds = new Set();
        this.onYopoResult = null;    // main.js: YOPO endstate → drone trajectory
        this.onDepthResult = null;   // main.js: depth latency → flight logger perf
        this.onYopoLatency = null;   // main.js: YOPO latency → flight logger perf
        this.onPerceptionMetrics = null; // segmented frame timings/drop reasons

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
        if (!this.active) {
            this._abortActiveCapture('sensor-inactive');
            this._abortCalibrationCapture('sensor-inactive');
        }
        this._applyVisibility();
    }

    _abortActiveCapture(reason) {
        const controller = this._captureAbortController;
        if (controller && !controller.signal.aborted) controller.abort(reason);
    }

    _abortCalibrationCapture(reason) {
        const controller = this._calibrationAbortController;
        if (controller && !controller.signal.aborted) controller.abort(reason);
    }

    reset() {
        const retainedDepth = this.hasDepth;
        this._abortActiveCapture('sensor-reset');
        this._abortActiveDepthRequest('sensor-reset');
        this._calibrationGeneration++;
        this._abortCalibrationCapture('sensor-reset');
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
        this._calibrationSessionId = createCalibrationSessionId();
        this._consumedCalibrationFrames.clear();
        this._consumedCalibrationCaptureIds.clear();
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
            jpegQuality: PANORAMA_JPEG_QUALITY,
            uploadScale: DA360_UPLOAD_SCALE,
            frameDelayMs: preload ? PANORAMA_PRELOAD_FRAME_DELAY_MS : PANORAMA_FRAME_DELAY_MS,
            tileTimeoutMs: preload ? PANORAMA_PRELOAD_FACE_TILE_TIMEOUT_MS : PANORAMA_FACE_TILE_TIMEOUT_MS,
            tileQuietMs: preload ? PANORAMA_PRELOAD_FACE_TILE_QUIET_MS : PANORAMA_FACE_TILE_QUIET_MS,
            captureAnyway: PANORAMA_CAPTURE_ANYWAY,  // Always captureAnyway: skip tile-wait for preload too, else face 1/6 times out
            facesPerSlice: PANORAMA_FACES_PER_SLICE,
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

    async _snapshotRgbBlob(canvas) {
        const startedAt = performance.now();
        const upload = this._depthUploadCanvas(canvas);
        const snapshot = document.createElement('canvas');
        snapshot.width = upload.width;
        snapshot.height = upload.height;
        const ctx = snapshot.getContext('2d', { alpha: false });
        ctx.drawImage(upload, 0, 0, snapshot.width, snapshot.height);
        const blob = await this._canvasToJpegBlob(snapshot);
        return {
            blob,
            width: snapshot.width,
            height: snapshot.height,
            jpegMs: performance.now() - startedAt,
        };
    }

    _makeRgbFrameContext(frameId, capturedAt, transform, planningState, canvas, captureTimings = null) {
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
            captureTimings: Object.freeze({ ...(captureTimings || {}) }),
            rgbPromise: this._snapshotRgbBlob(canvas),
        });
    }

    async _materializePerceptionFrame(context) {
        if (!context) throw new Error('RGB frame context unavailable');
        const encoded = await context.rgbPromise;
        if (!encoded?.blob) throw new Error('RGB JPEG encoding failed');
        const planningState = context.planningState;
        const frame = new PerceptionFrame({
            frameId: context.frameId,
            capturedAt: context.capturedAt,
            transform: context.transform,
            rgb: encoded.blob,
            actualState: planningState.actualState,
            referenceState: planningState.referenceState,
            yaw: planningState.yaw,
            projectionConfig: {
                ...context.projectionConfig,
                rgbWidth: encoded.width,
                rgbHeight: encoded.height,
                jpegQuality: PANORAMA_JPEG_QUALITY,
                uploadScale: encoded.width / context.projectionConfig.width,
            },
        });
        return { frame, jpegMs: encoded.jpegMs };
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
                this._abortActiveCapture('capture-timeout');
            }
            return;
        }
        // A calibration bundle must see the exact viewer/tileset state that
        // produced its frozen RGB. Do not begin the next six-face capture until
        // anchors and raw output have both been bound to that frame.
        if (this._calibrationCapturePending) return;
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
        else if (outcome === 'blocked') label = `${normalized} · blocked`;
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
        this.onPerceptionMetrics?.({
            frameId: request?.frameId ?? null,
            goalId: request?.goalId ?? null,
            generation: request?.generation ?? this._yopoGeneration,
            mode: request?.mode || desiredMode,
            outcome: 'stale',
            dropReason: reason,
            frameAgeMs: request?.frameContext
                ? performance.now() - request.frameContext.capturedAt
                : null,
        });
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
        let resolveCaptureIdle;
        const captureIdlePromise = new Promise(resolve => { resolveCaptureIdle = resolve; });
        this._captureIdlePromise = captureIdlePromise;
        const captureId = ++this._captureSequence;
        const captureController = new AbortController();
        this._captureAbortController = captureController;
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
            const result = await capture(transform, {
                ...this.getCaptureOptions(),
                signal: captureController.signal,
            });
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
                result?.timings_ms || { total: captureMs },
            );
            this.hasRgb = true;
            const rgbStatus = `${Math.round(captureMs)}ms`;
            this._setRgbStatus(rgbStatus);

            if (this._shouldRequestDepth(this.lastCaptureTime)) {
                this._requestDepth(this.rgbCanvas);
            }
        } catch (error) {
            if (error?.name === 'AbortError' || captureController.signal.aborted) {
                this._setRgbStatus(`stale · ${captureController.signal.reason || 'capture-aborted'}`);
                this.onPerceptionMetrics?.({
                    captureId,
                    outcome: 'stale',
                    dropReason: String(captureController.signal.reason || 'capture-aborted'),
                    mode: this._desiredDepthMode(),
                });
                return;
            }
            reportUserError('Panorama capture failed', error, {
                key: 'panorama-capture',
                intervalMs: 3000,
            });
            this._setRgbStatus(shortError(error));
        } finally {
            if (this._captureAbortController === captureController) {
                this._captureAbortController = null;
                this.capturing = false;
            }
            if (this._captureIdlePromise === captureIdlePromise) {
                this._captureIdlePromise = null;
            }
            resolveCaptureIdle();
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
            const materialized = await this._materializePerceptionFrame(frameContext);
            const frame = materialized.frame;
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
            const projectionHeader = JSON.stringify(frame.projectionConfig);
            if (request.mode === 'planning') {
                // 一次调用：DA360 → YOPO → 返回 endstate + depth_image
                const observation = frame.planningObservation(request.goal);
                const pose = observation.actualPosition;
                const reference = observation.referencePosition;
                const velocity = observation.velocity;
                const acceleration = observation.acceleration;
                const qs = new URLSearchParams({
                    px: String(pose.x), py: String(pose.y), pz: String(pose.z),
                    rpx: String(reference.x), rpy: String(reference.y), rpz: String(reference.z),
                    gx: String(observation.goal.x), gy: String(observation.goal.y), gz: String(observation.goal.z),
                    vx: String(velocity.x), vy: String(velocity.y), vz: String(velocity.z),
                    ax: String(acceleration.x), ay: String(acceleration.y), az: String(acceleration.z),
                    yaw: String(observation.yaw),
                    frame_id: String(request.frameId),
                    goal_id: String(request.goalId),
                    generation: String(request.generation),
                }).toString();
                url = `${getYopoEndpoint()}?${qs}`;  // /yopo/plan_full
                headers = {
                    'Content-Type': 'image/jpeg',
                    'X-Projection-Config': projectionHeader,
                };
                body = blob;
            } else {
                url = this.endpoint;  // /depth
                headers = {
                    'Content-Type': blob.type || 'image/jpeg',
                    'X-Projection-Config': projectionHeader,
                };
                body = blob;
            }

            const fetchStartedAt = performance.now();
            const response = await fetch(url, { method: 'POST', headers, body, signal: controller.signal });
            const responseAt = performance.now();
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
            const applyStartedAt = performance.now();
            if (!await this._decodeAndCommitDepthImage(payload.depth_image, request)) return false;
            const appliedAt = performance.now();

            this._depthLatency = Number.isFinite(Number(payload.latency_ms))
                ? `${Math.round(Number(payload.latency_ms))}ms`
                : '';
            const planningAuthorized = request.mode !== 'planning'
                || payload.planning_authorized === true;
            const planningReason = request.mode === 'planning' && !planningAuthorized
                ? String(payload.planning_reason || 'planning-not-authorized')
                : null;
            const outcome = planningAuthorized ? 'applied' : 'blocked';
            this._setDepthState(
                request.mode,
                planningReason ? `blocked:${planningReason}` : 'depth-ready',
                { outcome, latencyMs: Number(payload.latency_ms) },
            );

            if (request.mode === 'planning') {
                if (planningAuthorized && !payload.endstate) {
                    throw new Error('authorized planning response missing endstate');
                }
                if (planningAuthorized && payload.endstate && this.onYopoResult) {
                    const context = Object.freeze({
                        mode: request.mode,
                        goalId: request.goalId,
                        frameId: request.frameId,
                        generation: request.generation,
                        requestId: request.requestId,
                        depthMode: payload.depth_mode || null,
                        calibrationId: payload.calibration_id || null,
                        planningAuthorized: true,
                        planningReason: payload.planning_reason || null,
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
                    `frameId=${request.frameId} generation=${request.generation} ` +
                    `reason=${planningReason ? `blocked:${planningReason}` : 'ok'} ` +
                    `capture=${Math.round(frameContext.captureTimings?.total || 0)}ms ` +
                    `render=${Math.round(frameContext.captureTimings?.render || 0)}ms ` +
                    `project=${Math.round(frameContext.captureTimings?.project || 0)}ms ` +
                    `jpeg=${Math.round(materialized.jpegMs || tB - tA)}ms ` +
                    `network=${Math.round(responseAt - fetchStartedAt)}ms ` +
                    `da360=${Math.round(payload.timings_ms?.da360_ms ?? payload.timings_ms?.infer_ms ?? 0)}ms ` +
                    `yopo=${Math.round(payload.timings_ms?.yopo_ms || 0)}ms ` +
                    `apply=${Math.round(appliedAt - applyStartedAt)}ms age=${Math.round(appliedAt - frame.capturedAt)}ms ` +
                    `avgCycle=${avgCycle.toFixed(0)}ms`
                );
                this._depthFpsTimer = now; this._depthFpsCount = 0; this._depthCycleSum = 0;
            }
            this.lastDepthTime = performance.now();
            if (this.onDepthResult) this.onDepthResult(payload.latency_ms);
            const metrics = Object.freeze({
                frameId: request.frameId,
                goalId: request.goalId,
                generation: request.generation,
                mode: request.mode,
                outcome,
                planningAuthorized: request.mode === 'planning' ? planningAuthorized : null,
                dropReason: planningReason,
                calibrationId: payload.calibration_id || null,
                captureMs: Number(frameContext.captureTimings?.total || 0),
                renderMs: Number(frameContext.captureTimings?.render || 0),
                projectMs: Number(frameContext.captureTimings?.project || 0),
                schedulerYieldMs: Number(frameContext.captureTimings?.scheduler_yield || 0),
                jpegMs: Number(materialized.jpegMs || 0),
                networkMs: responseAt - fetchStartedAt,
                serverMs: Number(payload.latency_ms || 0),
                da360Ms: Number(payload.timings_ms?.da360_ms ?? payload.timings_ms?.infer_ms ?? 0),
                yopoMs: Number(payload.timings_ms?.yopo_ms || 0),
                applyMs: appliedAt - applyStartedAt,
                captureToApplyMs: appliedAt - frame.capturedAt,
                frameAgeMs: appliedAt - frame.capturedAt,
                serverTimings: payload.timings_ms || null,
            });
            this.onPerceptionMetrics?.(metrics);
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
            this.onPerceptionMetrics?.({
                frameId: request.frameId,
                goalId: request.goalId,
                generation: request.generation,
                mode: request.mode,
                outcome: offline ? 'offline' : 'error',
                dropReason: shortError(error),
                frameAgeMs: request.frameContext
                    ? performance.now() - request.frameContext.capturedAt
                    : null,
            });
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
        this._abortActiveCapture('goal-changed');
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
        this._abortActiveCapture(reason);
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

    /**
     * Export RGB, Cesium anchors, manifest and DA360 raw NPZ from exactly one
     * frozen PerceptionFrame. This is deliberately unavailable while moving or
     * navigating: calibration captures are defined as static poses.
     */
    async captureCalibrationSample(world, options = {}) {
        if (this._calibrationCapturePending) {
            throw new Error('a calibration capture is already in progress');
        }
        // Acquire the lock before the first await so update() cannot schedule a
        // new capture between selecting a frame and ray-casting its anchors.
        this._calibrationCapturePending = true;
        const calibrationGeneration = this._calibrationGeneration;
        const calibrationController = new AbortController();
        this._calibrationAbortController = calibrationController;
        const assertCalibrationCurrent = () => {
            if (
                calibrationController.signal.aborted
                || calibrationGeneration !== this._calibrationGeneration
            ) {
                const error = new Error(String(
                    calibrationController.signal.reason || 'calibration session changed'
                ));
                error.name = 'AbortError';
                throw error;
            }
        };
        try {
            const activeCapture = this._captureIdlePromise;
            if (activeCapture) await activeCapture;
            assertCalibrationCurrent();
            if (this.capturing) {
                throw new Error('panorama capture did not reach an idle state');
            }
            const latestContext = this._rgbFrameContext;
            const currentFrameMatches = latestContext && this._perceptionFrame
                && String(this._perceptionFrame.frameId) === String(latestContext.frameId);
            const materialized = latestContext && !currentFrameMatches
                ? await this._materializePerceptionFrame(latestContext)
                : null;
            assertCalibrationCurrent();
            const frame = currentFrameMatches ? this._perceptionFrame
                : materialized?.frame || this._perceptionFrame;
            if (!frame) {
                throw new Error('wait for a complete perception frame before calibration capture');
            }
            this._perceptionFrame = frame;
            const maxFrameAgeMs = Number(options.maxFrameAgeMs ?? 1000);
            const frameAgeMs = performance.now() - frame.capturedAt;
            if (!Number.isFinite(maxFrameAgeMs) || maxFrameAgeMs <= 0) {
                throw new TypeError('maxFrameAgeMs must be positive and finite');
            }
            if (!Number.isFinite(frameAgeMs) || frameAgeMs < 0 || frameAgeMs > maxFrameAgeMs) {
                throw new Error(
                    `calibration frame is stale (${Math.max(0, Math.round(frameAgeMs))}ms > ${maxFrameAgeMs}ms)`,
                );
            }
            if (this._yopoGoal) throw new Error('cancel the active navigation goal before calibration capture');
            const velocity = frame.actualState.velocity;
            const speed = Math.hypot(velocity.x, velocity.y, velocity.z);
            const maxStaticSpeed = Number.isFinite(options.maxStaticSpeedMps)
                ? Math.max(0, Number(options.maxStaticSpeedMps))
                : 0.2;
            if (speed > maxStaticSpeed) {
                throw new Error(`vehicle must be static for calibration (${speed.toFixed(2)}m/s > ${maxStaticSpeed}m/s)`);
            }
            if (!world || typeof world.sampleMetricDepthAnchors !== 'function') {
                throw new Error('metric anchor sampler unavailable');
            }

            const locationId = safeArtifactId(options.locationId, '');
            if (!locationId) throw new Error('locationId is required for leave-one-location-out validation');
            const sessionId = this._calibrationSessionId;
            const frameId = String(frame.frameId);
            const frameIdentity = `${sessionId}:${frameId}`;
            if (this._consumedCalibrationFrames.has(frameIdentity)) {
                throw new Error(`perception frame ${frameId} was already exported for calibration`);
            }
            const captureId = safeArtifactId(
                options.captureId,
                `${sessionId}-frame-${frameId}`,
            );
            if (this._consumedCalibrationCaptureIds.has(captureId)) {
                throw new Error(`captureId ${captureId} was already exported in this calibration session`);
            }
            const anchors = world.sampleMetricDepthAnchors(frame.transform, {
                ...options.anchorOptions,
                imageWidth: frame.projectionConfig.rgbWidth,
                imageHeight: frame.projectionConfig.rgbHeight,
                verticalFovDeg: frame.projectionConfig.verticalFovDeg,
                sessionId,
                locationId,
                captureId,
                frameId,
            });
            assertCalibrationCurrent();

            const rawUrl = new URL(getDA360RawEndpoint());
            rawUrl.searchParams.set('frame_id', frameId);
            rawUrl.searchParams.set('session_id', sessionId);
            rawUrl.searchParams.set('capture_id', captureId);
            rawUrl.searchParams.set('location_id', locationId);
            const rawResponse = await fetch(rawUrl.toString(), {
                method: 'POST',
                headers: {
                    'Content-Type': frame.rgb.type || 'image/jpeg',
                    'X-Frame-ID': frameId,
                    'X-Projection-Config': JSON.stringify(frame.projectionConfig),
                },
                body: frame.rgb,
                signal: calibrationController.signal,
            });
            assertCalibrationCurrent();
            if (!rawResponse.ok) throw new Error(`DA360 raw capture failed: HTTP ${rawResponse.status}`);
            const echoedFrameId = rawResponse.headers.get('X-Frame-ID');
            if (echoedFrameId !== frameId) {
                throw new Error(`DA360 raw frame mismatch: ${echoedFrameId || 'missing'} != ${frameId}`);
            }
            for (const [header, expected] of [
                ['X-Session-ID', sessionId],
                ['X-Capture-ID', captureId],
                ['X-Location-ID', locationId],
            ]) {
                const actual = rawResponse.headers.get(header);
                if (actual !== expected) {
                    throw new Error(`DA360 raw ${header} mismatch: ${actual || 'missing'} != ${expected}`);
                }
            }
            const rawBytes = await rawResponse.arrayBuffer();
            assertCalibrationCurrent();
            const rawBlob = new Blob([rawBytes], { type: 'application/x-npz' });

            const anchorsBlob = new Blob([JSON.stringify(anchors, null, 2)], { type: 'application/json' });
            const [rgbSha256, rawSha256, anchorsSha256] = await Promise.all([
                sha256Blob(frame.rgb),
                sha256Blob(rawBlob),
                sha256Blob(anchorsBlob),
            ]);
            assertCalibrationCurrent();
            if (!rgbSha256 || !rawSha256 || !anchorsSha256) {
                throw new Error('SHA-256 is required for calibration bundle export');
            }

            const filenames = Object.freeze({
                rgb: `${captureId}-rgb.jpg`,
                anchors: `${captureId}-anchors.json`,
                manifest: `${captureId}-manifest.json`,
                raw: `${captureId}-raw.npz`,
            });
            const manifest = Object.freeze({
                schemaVersion: 2,
                sessionId,
                captureId,
                locationId,
                frameId,
                capturedAt: frame.capturedAt,
                exportedAt: new Date().toISOString(),
                rgbWidth: frame.projectionConfig.rgbWidth,
                rgbHeight: frame.projectionConfig.rgbHeight,
                rgbSha256,
                rawSha256,
                anchorsSha256,
                files: Object.freeze({
                    rgb: Object.freeze({ name: filenames.rgb, sha256: rgbSha256 }),
                    anchors: Object.freeze({ name: filenames.anchors, sha256: anchorsSha256 }),
                    raw: Object.freeze({ name: filenames.raw, sha256: rawSha256 }),
                }),
                rawModel: rawResponse.headers.get('X-DA360-Model'),
                rawWidth: Number(rawResponse.headers.get('X-DA360-Width')) || null,
                rawHeight: Number(rawResponse.headers.get('X-DA360-Height')) || null,
                rawLatencyMs: Number(rawResponse.headers.get('X-DA360-Latency-Ms')) || null,
                transform: frame.transform,
                actualState: frame.actualState,
                referenceState: frame.referenceState,
                yaw: frame.yaw,
                projectionConfig: frame.projectionConfig,
                validAnchors: anchors.metadata?.validAnchors ?? anchors.anchors?.length ?? 0,
                failedAnchors: anchors.metadata?.failureCount ?? anchors.failures?.length ?? 0,
            });
            const manifestBlob = new Blob([JSON.stringify(manifest, null, 2)], { type: 'application/json' });
            const artifacts = Object.freeze({
                sessionId,
                captureId,
                locationId,
                frame,
                anchors,
                manifest,
                files: Object.freeze({
                    [filenames.rgb]: frame.rgb,
                    [filenames.anchors]: anchorsBlob,
                    [filenames.manifest]: manifestBlob,
                    [filenames.raw]: rawBlob,
                }),
            });
            this._consumedCalibrationFrames.add(frameIdentity);
            this._consumedCalibrationCaptureIds.add(captureId);
            if (options.download !== false) {
                for (const [filename, blob] of Object.entries(artifacts.files)) downloadBlob(blob, filename);
            }
            return artifacts;
        } finally {
            if (this._calibrationAbortController === calibrationController) {
                this._calibrationAbortController = null;
            }
            this._calibrationCapturePending = false;
        }
    }
}
