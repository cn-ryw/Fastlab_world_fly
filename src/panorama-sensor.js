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

const CAPTURE_INTERVAL_MS = urlNumber('panoMs', 16, 8, 10000);
const DEPTH_INTERVAL_MS = urlNumber('depthMs', 100, 50, 10000);
const DA360_TIMEOUT_MS = urlNumber('da360TimeoutMs', 12000, 1000, 60000);
const DA360_UPLOAD_SCALE = urlNumber('da360UploadScale', 0.2, 0.05, 1);
const DA360_UPLOAD_WIDTH = Math.round(urlNumber('da360UploadWidth', 0, 0, 5760));
const DA360_UPLOAD_HEIGHT = Math.round(urlNumber('da360UploadHeight', 0, 0, 2880));
const PANORAMA_WIDTH = evenNumber(urlNumber('panoWidth', 672, 280, 5760));
const PANORAMA_HEIGHT = evenNumber(urlNumber('panoHeight', Math.round(PANORAMA_WIDTH / 2), 140, 2880));
const PANORAMA_FACE_SIZE = Math.round(urlNumber('panoFace', 192, 128, 2048));
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
        this.depthImg = document.getElementById('panorama-depth-image');
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

        if (this.rgbCanvas) {
            this.rgbCanvas.width = PANORAMA_WIDTH;
            this.rgbCanvas.height = PANORAMA_HEIGHT;
            this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        }
        if (this.depthImg) {
            this.depthImg.onload = () => {
                this.hasDepth = true;
                this._updateDepthDisplay();
            };
            this.depthImg.onerror = () => {
                this.hasDepth = false;
                this._setStatus(this.hasRgb ? 'ready' : 'idle', 'decode error');
                this._setDepthPlaceholder('decode error');
            };
        }
        this._setDepthPlaceholder('DA360 offline');
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
        if (this.rgbCanvas) this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        this._setDepthPlaceholder('DA360 offline');
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
            captureAnyway: preload ? false : PANORAMA_CAPTURE_ANYWAY,
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

        // B6 fix: stuck capture watchdog — force-reset if captureAnyway is on
        if (this.capturing) {
            const stuckMs = PANORAMA_FACE_TILE_TIMEOUT_MS * 6 + 500;
            if (PANORAMA_CAPTURE_ANYWAY && now - this.lastCaptureStartTime > stuckMs) {
                this.capturing = false;
            } else {
                // B7: depth still triggers independently even while capture is stuck
                if (this.hasRgb && !this.depthPending && now - this.lastDepthTime >= DEPTH_INTERVAL_MS) {
                    this._requestDepth(this.rgbCanvas);
                }
                return;
            }
        }
        if (this.capturing || now - this.lastCaptureStartTime < CAPTURE_INTERVAL_MS) return;

        // B7: also trigger depth at normal update rate when no capture is active
        if (this.hasRgb && !this.depthPending && now - this.lastDepthTime >= DEPTH_INTERVAL_MS) {
            this._requestDepth(this.rgbCanvas);
        }
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

    _setDepthPlaceholder(label) {
        if (!this.depthImg) return;
        const canvas = document.createElement('canvas');
        canvas.width = PANORAMA_WIDTH;
        canvas.height = PANORAMA_HEIGHT;
        this._drawPlaceholder(canvas, label);
        try {
            this.depthImg.src = canvas.toDataURL('image/png');
        } catch (error) {
            reportUserError('Depth placeholder render failed', error, {
                key: 'depth-placeholder',
                intervalMs: 10000,
            });
        }
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

    async _requestDepth(canvas) {
        this.depthPending = true;
        this._setStatus('ready', 'inferring');
        const controller = new AbortController();
        const timeout = window.setTimeout(() => controller.abort(), DA360_TIMEOUT_MS);
        const started = performance.now();

        try {
            const uploadCanvas = this._depthUploadCanvas(canvas);
            const blob = await this._canvasToJpegBlob(uploadCanvas);
            const headers = {};
            let body;
            if (blob) {
                headers['Content-Type'] = blob.type || 'image/jpeg';
                body = blob;
            } else {
                headers['Content-Type'] = 'application/json';
                body = JSON.stringify({ image: uploadCanvas.toDataURL('image/jpeg', PANORAMA_JPEG_QUALITY) });
            }
            const response = await fetch(this.endpoint, {
                method: 'POST',
                headers,
                body,
                signal: controller.signal,
            });
            if (!response.ok) {
                throw new Error(`DA360 HTTP ${response.status}`);
            }
            const payload = await response.json();
            if (!payload || !payload.depth_image) {
                throw new Error('DA360 response missing depth_image');
            }
            this.depthImg.src = payload.depth_image;
            this._setDepthLegend(payload.depth_scale);
            this.lastDepthTime = performance.now();
            this._depthLatency = Number.isFinite(payload.latency_ms)
                ? `${Math.round(payload.latency_ms)}ms`
                : `${Math.round(this.lastDepthTime - started)}ms`;
            // hasDepth and status set in depthImg.onload
        } catch (error) {
            reportUserError('DA360 depth request failed', error, {
                key: 'da360-depth-request',
                intervalMs: 3000,
            });
            this.lastDepthTime = performance.now();
            this._setStatus('ready', shortError(error));
        } finally {
            window.clearTimeout(timeout);
            this.depthPending = false;
        }
    }
}
