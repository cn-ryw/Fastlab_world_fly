import { reportUserError } from './error-report.js';
import { demoPerformance } from './demo-performance.js?v=20260814-adaptive-a1';
import { PerceptionFrame, normalizePlanningState } from './perception-frame.js';
import { normalizeDepthPolarScan } from './depth-topdown.js';

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

const PANORAMA_CAPTURE_PROFILES = new Set(['flight', 'calibration']);

function normalizeCaptureProfile(value, fallback = null) {
    const normalized = String(value || '').trim().toLowerCase();
    if (PANORAMA_CAPTURE_PROFILES.has(normalized)) return normalized;
    if (fallback !== null) return fallback;
    throw new RangeError('panorama capture profile must be "flight" or "calibration"');
}

function initialCaptureProfile() {
    const params = new URLSearchParams(window.location.search);
    const explicit = params.get('panoProfile') || params.get('panoCaptureProfile');
    if (explicit) return normalizeCaptureProfile(explicit, 'flight');

    // Backward compatibility for the strict static-calibration URL documented
    // before profiles existed. Flight is the fail-safe default for every other
    // URL so a tile quiet period cannot silently throttle the live loop.
    const legacyCaptureAnyway = params.get('panoCaptureAnyway');
    const legacyCaptureAnywayNumber = Number(legacyCaptureAnyway);
    if (legacyCaptureAnyway !== null
        && legacyCaptureAnyway !== ''
        && Number.isFinite(legacyCaptureAnywayNumber)
        && legacyCaptureAnywayNumber < 0.5) {
        return 'calibration';
    }
    return 'flight';
}

// 名义调度间隔；真实吞吐由完整六面 capture、单个在途请求及验收门禁决定。
const CAPTURE_INTERVAL_MS = urlNumber('panoMs', 20, 16, 10000);
// 深度请求间隔：50Hz 名义，由 _depthGate 非阻塞漏桶调节实际吞吐
const DEPTH_INTERVAL_MS = urlNumber('depthMs', 20, 16, 10000);
// A response may legitimately arrive after the next RGB capture completes,
// but never apply a trajectory from an arbitrarily old frozen observation.
const YOPO_MAX_FRAME_AGE_MS = urlNumber('yopoMaxFrameAgeMs', 250, 50, 250);
// Do not spend GPU time on a frame whose remaining hard-age envelope is too
// small for the accepted 50 ms service budget plus JSON/apply overhead.  This
// is deliberately not URL-configurable: it may reject earlier, but it can
// never relax the 250 ms trajectory-install limit.
const YOPO_MIN_FETCH_REMAINING_MS = 55;
// Reserve callback/accounting headroom so a trajectory accepted just under the
// hard observation limit cannot be logged/applied just over it.
const YOPO_APPLY_DEADLINE_RESERVE_MS = 2;
// Planning needs the compact control response at full rate. Preview rendering
// is requested at no more than 2 Hz and is prepared by the server's latest-only
// CPU worker; the control response never carries preview bytes.
const PLANNING_PREVIEW_INTERVAL_MS = demoPerformance.planningPreviewIntervalMs(2000);
const STALE_LOG_SUMMARY_INTERVAL_MS = 2000;
const IS_CHROMIUM = typeof navigator !== 'undefined'
    && /\bChrom(?:e|ium)\//.test(String(navigator.userAgent || ''));
// A live six-face capture should normally finish in under 75 ms. Recover from
// an asynchronously stalled capture before YOPO's 1.125 s trajectory expires.
const CAPTURE_STALL_TIMEOUT_MS = 500;
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
const RAW_RGBA8_CONTENT_TYPE = 'application/x-mindcloud-rgba8';
const PANORAMA_FACE_FOV = urlNumber('panoFaceFov', 130, 90, 170);
const PANORAMA_TOP_POLE_GUARD = urlNumber('panoTopPoleGuard', 10, 0, 45);
const PANORAMA_BOTTOM_POLE_GUARD = urlNumber('panoBottomPoleGuard', 2, 0, 45);
const PANORAMA_FRAME_DELAY_MS = urlNumber('panoFrameDelayMs', 0, 0, 1000);
const PANORAMA_FACE_TILE_TIMEOUT_MS = urlNumber('panoFaceTileTimeoutMs', 6000, 0, 10000);
const PANORAMA_FACE_TILE_QUIET_MS = urlNumber('panoFaceTileQuietMs', 650, 0, 5000);
// Baseline fallback stays at two faces. demo30 starts at three faces (six faces
// in two 30 FPS slices) and falls back to two when main-view p95 is over budget.
const PANORAMA_FACES_PER_SLICE = Math.round(urlNumber('panoFacesPerSlice', 2, 1, 6));
const PANORAMA_PRELOAD_FRAME_DELAY_MS = urlNumber(
    'panoPreloadFrameDelayMs',
    Math.max(96, PANORAMA_FRAME_DELAY_MS),
    0,
    1000
);
const PANORAMA_PRELOAD_FACE_TILE_TIMEOUT_MS = urlNumber(
    'panoPreloadFaceTileTimeoutMs',
    PANORAMA_FACE_TILE_TIMEOUT_MS,
    0,
    30000
);
const PANORAMA_PRELOAD_FACE_TILE_QUIET_MS = urlNumber(
    'panoPreloadFaceTileQuietMs',
    PANORAMA_FACE_TILE_QUIET_MS,
    0,
    5000
);
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

function getYopoPreviewEndpoint() {
    const params = new URLSearchParams(window.location.search);
    const host = params.get('da360Host') || window.location.hostname || '127.0.0.1';
    const port = params.get('da360Port') || '5688';
    return `http://${host}:${port}/yopo/preview`;
}

function shortError(error) {
    const message = error && error.message ? error.message : String(error || 'error');
    return message.length > 52 ? `${message.slice(0, 49)}...` : message;
}

function normalizeYopoObserverResult(observer) {
    if (!observer?.available || !observer.succeeded) {
        return Object.freeze({ committed: false, outcome: 'rejected', reason: null });
    }
    if (observer.value === true) {
        return Object.freeze({ committed: true, outcome: 'applied', reason: null });
    }
    if (observer.value && typeof observer.value === 'object') {
        const outcome = String(observer.value.outcome || '').trim().toLowerCase();
        if (outcome === 'applied' || outcome === 'ignored') {
            const reason = typeof observer.value.reason === 'string'
                && observer.value.reason.trim().length > 0
                ? observer.value.reason.trim()
                : outcome === 'ignored' ? 'consumer-ignored' : null;
            return Object.freeze({ committed: true, outcome, reason });
        }
    }
    return Object.freeze({ committed: false, outcome: 'rejected', reason: null });
}

function abortErrorFromSignal(signal, fallback = 'request aborted') {
    const error = new Error(String(signal?.reason || fallback));
    error.name = 'AbortError';
    return error;
}

/**
 * Await work that cannot necessarily consume an AbortSignal itself (notably
 * canvas.toBlob()).  The losing promise remains safely observed, while the
 * request owner can release its gate as soon as the session/deadline aborts.
 */
function awaitWithAbort(value, signal) {
    if (!signal) return Promise.resolve(value);
    if (signal.aborted) return Promise.reject(abortErrorFromSignal(signal));
    return new Promise((resolve, reject) => {
        let settled = false;
        const finish = (callback, result) => {
            if (settled) return;
            settled = true;
            signal.removeEventListener('abort', onAbort);
            callback(result);
        };
        const onAbort = () => finish(reject, abortErrorFromSignal(signal));
        signal.addEventListener('abort', onAbort, { once: true });
        Promise.resolve(value).then(
            result => finish(resolve, result),
            error => finish(reject, error),
        );
    });
}

function finiteNumberOrNull(value) {
    if (value === null || value === undefined || value === '') return null;
    const number = Number(value);
    return Number.isFinite(number) ? number : null;
}

function safeIntegerOrNull(value) {
    if (value === null || value === undefined || value === '') return null;
    const number = Number(value);
    return Number.isSafeInteger(number) ? number : null;
}

function frozenNumberArray(value, expectedLength = null) {
    if (!Array.isArray(value)) return null;
    if (expectedLength !== null && value.length !== expectedLength) return null;
    const numbers = value.map(Number);
    if (!numbers.every(Number.isFinite)) return null;
    return Object.freeze(numbers);
}

const PLANNING_DIAGNOSTICS_SCHEMA_VERSION = 1;
const NORMALIZED_ENDSTATE_TOLERANCE = 1e-5;

function emptyPlanningDiagnostics(selectedEndstate = null) {
    return Object.freeze({
        planningDiagnostics: null,
        planningDiagnosticsSchemaVersion: null,
        selectedCandidateId: null,
        selectedActionId: null,
        selectedLatticeId: null,
        selectedScore: null,
        selectedEndstate,
        selectedRawEndstate: null,
        terminalSpeedMps: null,
        terminalAccelerationMps2: null,
        endpointDisplacementM: null,
        trajectoryTimeS: null,
        candidateCount: null,
        velocityScaleMps: null,
        accelerationScaleMps2: null,
    });
}

function normalizePlanningDiagnostics(payload) {
    const raw = payload?.planning_diagnostics;
    const selectedEndstate = frozenNumberArray(payload?.endstate, 9);
    const planningAuthorized = payload?.planning_authorized === true;
    if (raw === null || raw === undefined) {
        if (planningAuthorized) {
            throw new Error('authorized planning response missing planning_diagnostics v1');
        }
        return emptyPlanningDiagnostics(selectedEndstate);
    }
    if (!raw || typeof raw !== 'object' || Array.isArray(raw)) {
        throw new Error('planning_diagnostics must be an object');
    }
    const source = raw;
    const planningDiagnosticsSchemaVersion = source.schema_version;
    if (planningDiagnosticsSchemaVersion !== PLANNING_DIAGNOSTICS_SCHEMA_VERSION) {
        throw new Error(
            `unsupported planning_diagnostics schema_version=${source.schema_version ?? 'missing'}`
        );
    }
    const selectedRawEndstate = frozenNumberArray(
        source.selected_endstate_raw ?? source.selected_raw_endstate,
        9,
    );
    const selectedCandidateId = safeIntegerOrNull(source.selected_candidate_id);
    const selectedActionId = safeIntegerOrNull(source.selected_action_id);
    const selectedLatticeId = safeIntegerOrNull(source.selected_lattice_id);
    const selectedScore = finiteNumberOrNull(source.selected_score);
    const terminalSpeedMps = finiteNumberOrNull(source.terminal_speed_mps);
    const terminalAccelerationMps2 = finiteNumberOrNull(
        source.terminal_acceleration_mps2,
    );
    const endpointDisplacementM = finiteNumberOrNull(source.endpoint_displacement_m);
    const trajectoryTimeS = finiteNumberOrNull(source.trajectory_time_s);
    const candidateCount = safeIntegerOrNull(source.candidate_count);
    const velocityScaleMps = finiteNumberOrNull(source.velocity_scale_mps);
    const accelerationScaleMps2 = finiteNumberOrNull(
        source.acceleration_scale_mps2,
    );
    if (!selectedRawEndstate) {
        throw new Error('planning_diagnostics selected_endstate_raw must contain 9 finite values');
    }
    if (selectedRawEndstate.some(
        value => Math.abs(value) > 1 + NORMALIZED_ENDSTATE_TOLERANCE
    )) {
        throw new Error('planning_diagnostics selected_endstate_raw is outside normalized [-1,1]');
    }
    if ([selectedCandidateId, selectedActionId, selectedLatticeId, candidateCount]
        .some(value => value === null)) {
        throw new Error('planning_diagnostics candidate/action/lattice/count must be safe integers');
    }
    if ([
        selectedScore,
        terminalSpeedMps,
        terminalAccelerationMps2,
        endpointDisplacementM,
        trajectoryTimeS,
        velocityScaleMps,
        accelerationScaleMps2,
    ].some(value => value === null)) {
        throw new Error('planning_diagnostics missing/non-finite number');
    }
    if (selectedScore < 0
        || terminalSpeedMps < 0
        || terminalAccelerationMps2 < 0
        || endpointDisplacementM < 0) {
        throw new Error('planning_diagnostics negative score/terminal metric');
    }
    if (trajectoryTimeS <= 0 || velocityScaleMps <= 0 || accelerationScaleMps2 <= 0) {
        throw new Error('planning_diagnostics time/scale must be positive');
    }
    if (candidateCount <= 0
        || selectedCandidateId < 0
        || selectedActionId < 0
        || selectedLatticeId < 0
        || selectedCandidateId >= candidateCount
        || selectedActionId >= candidateCount
        || selectedLatticeId >= candidateCount) {
        throw new Error('planning_diagnostics candidate/action/lattice IDs are out of range');
    }
    if (selectedCandidateId !== selectedActionId
        || selectedLatticeId !== candidateCount - 1 - selectedActionId) {
        throw new Error('planning_diagnostics candidate/action/lattice/count are inconsistent');
    }
    const planningDiagnostics = Object.freeze({
        schema_version: planningDiagnosticsSchemaVersion,
        selected_endstate_raw: selectedRawEndstate,
        selected_candidate_id: selectedCandidateId,
        selected_action_id: selectedActionId,
        selected_lattice_id: selectedLatticeId,
        selected_score: selectedScore,
        terminal_speed_mps: terminalSpeedMps,
        terminal_acceleration_mps2: terminalAccelerationMps2,
        endpoint_displacement_m: endpointDisplacementM,
        trajectory_time_s: trajectoryTimeS,
        candidate_count: candidateCount,
        velocity_scale_mps: velocityScaleMps,
        acceleration_scale_mps2: accelerationScaleMps2,
    });
    return Object.freeze({
        planningDiagnostics,
        planningDiagnosticsSchemaVersion,
        selectedCandidateId,
        selectedActionId,
        selectedLatticeId,
        selectedScore,
        selectedEndstate,
        selectedRawEndstate,
        terminalSpeedMps,
        terminalAccelerationMps2,
        endpointDisplacementM,
        trajectoryTimeS,
        candidateCount,
        velocityScaleMps,
        accelerationScaleMps2,
    });
}

function responseContentLength(response) {
    const headerValue = response?.headers?.get?.('Content-Length');
    const headerLength = Number(headerValue);
    if (Number.isSafeInteger(headerLength) && headerLength >= 0) return headerLength;
    // Never stringify the parsed depth preview again on the control-critical
    // path merely to estimate bytes. Same-origin production responses expose
    // Content-Length; mocks/proxies that omit it are reported as unknown.
    return null;
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

const PANORAMA_TOTAL_FACES = 6;

/**
 * Freeze the 3D-tile readiness provenance that belongs to one projected RGB
 * canvas. Older/custom capture implementations did not return readiness; they
 * remain compatible and are treated as ready after a complete projection.
 */
function normalizeCaptureReadiness(result, structuredResult, complete) {
    const rawFaces = structuredResult && Array.isArray(result?.faceTileReadiness)
        ? result.faceTileReadiness
        : [];
    const faceTileReadiness = Object.freeze(rawFaces.map((entry, index) => Object.freeze({
        face: String(entry?.face ?? index),
        readyWhenCopied: entry?.readyWhenCopied === true,
    })));
    const requestedTotal = Number(result?.faces);
    const rgbTotalFaces = Number.isSafeInteger(requestedTotal) && requestedTotal > 0
        ? requestedTotal
        : Math.max(PANORAMA_TOTAL_FACES, faceTileReadiness.length);
    const inferredReadyFaces = faceTileReadiness.reduce(
        (count, entry) => count + (entry.readyWhenCopied ? 1 : 0),
        0,
    );
    const requestedReadyFaces = Number(result?.readyFaces);
    const hasExplicitReadyFaces = structuredResult
        && Number.isSafeInteger(requestedReadyFaces)
        && requestedReadyFaces >= 0;
    let rgbReadyFaces = hasExplicitReadyFaces ? requestedReadyFaces : inferredReadyFaces;
    const hasExplicitReady = structuredResult && typeof result?.ready === 'boolean';
    const hasExplicitAllReady = structuredResult
        && typeof result?.allFacesTileReady === 'boolean';
    const hasFaceEvidence = faceTileReadiness.length > 0;
    // Prefer the explicit per-face aggregate when both legacy `ready`
    // (historically canvas-ready) and `allFacesTileReady` are present.
    const rgbTilesReady = hasExplicitAllReady
        ? result.allFacesTileReady === true
        : hasExplicitReady
        ? result.ready === true
        : hasFaceEvidence
        ? complete && inferredReadyFaces === rgbTotalFaces
        : complete;
    if (rgbTilesReady && !hasExplicitReadyFaces && !hasFaceEvidence) {
        rgbReadyFaces = rgbTotalFaces;
    }
    rgbReadyFaces = Math.max(0, Math.min(rgbTotalFaces, rgbReadyFaces));
    const rgbTileError = structuredResult && result?.tileError === true;
    const rgbReadinessReason = String(
        structuredResult && result?.readinessReason
        || (rgbTilesReady
            ? 'tiles-ready'
            : rgbTileError
            ? 'tile-error'
            : complete
            ? 'tiles-partial'
            : 'capture-incomplete'),
    );
    return Object.freeze({
        rgbFrameComplete: complete === true,
        rgbTilesReady,
        rgbReadyFaces,
        rgbTotalFaces,
        rgbReadinessReason,
        rgbTileError,
        faceTileReadiness,
    });
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
        demoPerformance.loadYopoStrategy(this.endpoint);
        this.active = false;
        this.capturing = false;
        this._captureProfile = initialCaptureProfile();
        this._captureAbortController = null;
        this._captureIdlePromise = null;
        this._captureSequence = 0;
        this.depthPending = false;
        this.lastCaptureStartTime = 0;
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this._lastDepthRequestStartedAt = -Infinity;
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
        this._lastDepthLogKey = null;
        this._depthAbortController = null;
        this._activeDepthRequest = null;
        this._depthRequestSequence = 0;
        this._depthCatchupQueued = false;
        this._depthCatchupTimer = null;
        this._depthCatchupDeadlineAt = null;
        this._lastRenderedRequestId = 0;
        this._latestDepthPreviewRequestId = 0;
        this._pendingDepthPreviewJob = null;
        this._depthPreviewWorkerRunning = false;
        this._depthPreviewWorkerPromise = Promise.resolve();
        this._lastPlanningPreviewFetchAt = -Infinity;
        this._staleLogBuckets = new Map();
        this._depthPolarFrame = null;
        this._rgbFrameId = 0;
        this._rgbReadiness = Object.freeze({
            frameId: 0,
            rgbFrameComplete: false,
            rgbTilesReady: false,
            rgbReadyFaces: 0,
            rgbTotalFaces: PANORAMA_TOTAL_FACES,
            rgbReadinessReason: 'no-rgb-frame',
            rgbTileError: false,
            faceTileReadiness: Object.freeze([]),
        });
        this._lastRequestedFrameId = -1;
        this._minimumRequestFrameId = 0;
        this._forceNextDepthRequest = false;
        this._yopoGeneration = 0;    // 目标会话变化时递增，丢弃过期响应
        this._planningEpoch = 0;     // 同一目标内重规划时递增，隔离故障前的帧/响应
        this._planningPaused = false;
        this._planningPauseReason = null;
        this._goalSequence = 0;
        this._goalId = null;
        this._navigationMode = 'idle';
        this._navigationKind = null;
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
            this._cancelDepthCatchup();
        }
        this._applyVisibility();
    }

    setCaptureProfile(profile, reason = 'profile-changed') {
        const normalized = normalizeCaptureProfile(profile);
        if (normalized === 'calibration' && this._yopoGoal) {
            throw new Error('cannot enter calibration capture profile while navigation is active');
        }
        if (normalized === this._captureProfile) return normalized;
        const transitionReason = `${reason}:${this._captureProfile}->${normalized}`;
        this._abortActiveCapture(transitionReason);
        if (this._captureProfile === 'calibration' && normalized !== 'calibration') {
            // A raw/anchor bundle is valid only while the sensor remains in the
            // calibration profile. Generation invalidates non-abort-aware
            // awaits; the signal cancels the in-flight /depth/raw request.
            this._calibrationGeneration++;
            this._abortCalibrationCapture(transitionReason);
        }
        this._captureProfile = normalized;
        return normalized;
    }

    getCaptureProfile() {
        return this._captureProfile;
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
        this._invalidateQueuedDepthPreviews();
        this._staleLogBuckets.clear();
        this._calibrationGeneration++;
        this._abortCalibrationCapture('sensor-reset');
        this.capturing = false;
        this.lastCaptureStartTime = 0;
        this.lastCaptureTime = 0;
        this.lastDepthTime = 0;
        this._lastDepthRequestStartedAt = -Infinity;
        this._cancelDepthCatchup();
        this.hasRgb = false;
        this._lastDepthArray = null;
        this._yopoGeneration++;   // 取消/到达时递增，使在途响应过期
        this._planningEpoch++;
        this._planningPaused = false;
        this._planningPauseReason = null;
        this._goalId = null;
        this._navigationMode = 'idle';
        this._navigationKind = null;
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
        this._rgbReadiness = Object.freeze({
            frameId: 0,
            rgbFrameComplete: false,
            rgbTilesReady: false,
            rgbReadyFaces: 0,
            rgbTotalFaces: PANORAMA_TOTAL_FACES,
            rgbReadinessReason: 'no-rgb-frame',
            rgbTileError: false,
            faceTileReadiness: Object.freeze([]),
        });
        this._lastRequestedFrameId = -1;
        this._minimumRequestFrameId = 1;
        this._forceNextDepthRequest = true;
        this._lastPlanningPreviewFetchAt = -Infinity;
        if (this.rgbCanvas) this._drawPlaceholder(this.rgbCanvas, 'RGB PANORAMA');
        // reset() may teleport the vehicle back to spawn. A previously drawn
        // JPEG can remain as a visual placeholder, but an ego-centred obstacle
        // outline must never be presented as belonging to the new pose.
        this._depthPolarFrame = null;
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

    /** Schedule the next planning capture/request at the earliest safe frame. */
    requestImmediatePlanningFrame(reason = 'controller-replan') {
        if (this._navigationMode !== 'active' || !this._yopoGoal) return false;
        // A controller failure changes the valid planning observation even
        // though goalId/generation stay constant. Invalidate both asynchronous
        // stages and require a panorama captured after this boundary.
        this._planningEpoch++;
        this._abortActiveDepthRequest(reason);
        this._abortActiveCapture(reason);
        this._invalidateQueuedDepthPreviews();
        this._minimumRequestFrameId = this._rgbFrameId + 1;
        this.lastCaptureStartTime = -Infinity;
        this._forceNextDepthRequest = true;
        this._navigationTransitionReason = reason;
        this._setDepthState('planning', 'awaiting-new-rgb-frame');
        return true;
    }

    getCaptureOptions(options = {}) {
        const preload = !!options.preload;
        const profile = normalizeCaptureProfile(options.profile, this._captureProfile);
        const calibration = profile === 'calibration';
        const demoPreload = preload && demoPerformance.config.profile === 'demo30';
        return {
            profile,
            width: PANORAMA_WIDTH,
            height: PANORAMA_HEIGHT,
            faceSize: PANORAMA_FACE_SIZE,
            verticalFovDeg: PANORAMA_VERTICAL_FOV,
            faceFovDeg: PANORAMA_FACE_FOV,
            topPoleGuardDeg: PANORAMA_TOP_POLE_GUARD,
            bottomPoleGuardDeg: PANORAMA_BOTTOM_POLE_GUARD,
            jpegQuality: PANORAMA_JPEG_QUALITY,
            uploadScale: DA360_UPLOAD_SCALE,
            // Preload is a one-shot, pre-flight operation even when the live
            // capture profile is `flight`. It must dwell on each hidden-viewer
            // face and actually let Cesium settle its tile queue. Live flight
            // remains the separate zero-wait/capture-anyway path below.
            frameDelayMs: preload
                ? (demoPreload
                    ? demoPerformance.config.preloadFrameDelayMs
                    : PANORAMA_PRELOAD_FRAME_DELAY_MS)
                : calibration
                ? PANORAMA_FRAME_DELAY_MS
                : 0,
            tileTimeoutMs: preload
                ? (demoPreload
                    ? demoPerformance.config.preloadFaceTileTimeoutMs
                    : PANORAMA_PRELOAD_FACE_TILE_TIMEOUT_MS)
                : calibration
                ? PANORAMA_FACE_TILE_TIMEOUT_MS
                : 0,
            tileQuietMs: preload
                ? (demoPreload
                    ? demoPerformance.config.preloadFaceTileQuietMs
                    : PANORAMA_PRELOAD_FACE_TILE_QUIET_MS)
                : calibration
                ? PANORAMA_FACE_TILE_QUIET_MS
                : 0,
            captureAnyway: !preload && !calibration,
            // A slow first direction must not prevent the other five views
            // from issuing their tile requests during the one-shot warm-up.
            // Calibration remains fail-closed and live flight remains the
            // separate zero-wait capture-anyway path.
            continueOnTileTimeout: preload,
            // Six synchronous Cesium renders blocked Chrome's main/physics
            // loop for a complete panorama. Keep JPEG/inference pipelined,
            // but yield after two faces so the flight view can present frames.
            facesPerSlice: demoPerformance.facesPerSlice(
                IS_CHROMIUM,
                PANORAMA_FACES_PER_SLICE,
            ),
            timeoutMs: preload
                ? (demoPerformance.config.profile === 'demo30'
                    ? Math.min(PANORAMA_PRELOAD_TIMEOUT_MS, demoPerformance.config.preloadTimeoutMs)
                    : PANORAMA_PRELOAD_TIMEOUT_MS)
                : 0,
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
        const snapshot = typeof OffscreenCanvas !== 'undefined'
            ? new OffscreenCanvas(upload.width, upload.height)
            : document.createElement('canvas');
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
            uploadEncoding: 'jpeg',
        };
    }

    _snapshotRgbRgba(canvas) {
        const startedAt = performance.now();
        const upload = this._depthUploadCanvas(canvas);
        const ctx = upload.getContext('2d', { alpha: false, willReadFrequently: true });
        const image = ctx.getImageData(0, 0, upload.width, upload.height);
        return {
            // Blob construction freezes this exact typed-array view before the
            // shared upload canvas can be overwritten by a newer capture.
            blob: new Blob([image.data], { type: RAW_RGBA8_CONTENT_TYPE }),
            width: upload.width,
            height: upload.height,
            jpegMs: performance.now() - startedAt,
            uploadEncoding: 'rgba8',
        };
    }

    _makeRgbFrameContext(
        frameId,
        capturedAt,
        transform,
        planningState,
        canvas,
        captureTimings = null,
        captureProfile = this._captureProfile,
        planningEpoch = this._planningEpoch,
        readiness = normalizeCaptureReadiness(null, false, true),
    ) {
        const fallbackPosition = planningState?.actualState?.position || { x: 0, y: 0, z: 0 };
        const fallbackState = planningState || normalizePlanningState({
            x: fallbackPosition.x,
            y: fallbackPosition.y,
            z: fallbackPosition.z,
            vx: 0,
            vy: 0,
            vz: 0,
        }, 0);
        const context = {
            frameId,
            capturedAt,
            transform: cloneCaptureTransform(transform, fallbackPosition),
            planningState: fallbackState,
            captureProfile: normalizeCaptureProfile(captureProfile),
            planningEpoch,
            rgbFrameComplete: readiness.rgbFrameComplete === true,
            rgbTilesReady: readiness.rgbTilesReady === true,
            rgbReadyFaces: Number(readiness.rgbReadyFaces) || 0,
            rgbTotalFaces: Number(readiness.rgbTotalFaces) || PANORAMA_TOTAL_FACES,
            rgbReadinessReason: String(readiness.rgbReadinessReason || 'tiles-partial'),
            rgbTileError: readiness.rgbTileError === true,
            faceTileReadiness: Object.freeze([...(readiness.faceTileReadiness || [])]),
            projectionConfig: Object.freeze(this._projectionConfig()),
            captureTimings: Object.freeze({ ...(captureTimings || {}) }),
        };
        // JPEG is a latest-only control product, not a by-product of every
        // panorama. Starting convertToBlob here queued unbounded encodes while
        // the single DA360/YOPO request gate was busy. The selected request
        // reads this getter synchronously before yielding, which freezes the
        // matching canvas once and memoizes that one encode.
        let rgbPromise = null;
        let rawRgbPayload = null;
        Object.defineProperty(context, 'rgbPromise', {
            enumerable: true,
            get: () => {
                if (!rgbPromise) rgbPromise = this._snapshotRgbBlob(canvas);
                return rgbPromise;
            },
        });
        Object.defineProperty(context, 'rawRgbPayload', {
            enumerable: true,
            get: () => {
                if (!rawRgbPayload) rawRgbPayload = this._snapshotRgbRgba(canvas);
                return rawRgbPayload;
            },
        });
        return Object.freeze(context);
    }

    async _materializePerceptionFrame(context, requestedEncoding = 'jpeg') {
        if (!context) throw new Error('RGB frame context unavailable');
        let encoded;
        if (requestedEncoding === 'rgba8') {
            try {
                encoded = context.rawRgbPayload;
            } catch (error) {
                reportUserError('Raw planning upload unavailable; using JPEG', error, {
                    key: 'raw-planning-upload', intervalMs: 3000,
                });
                encoded = await context.rgbPromise;
            }
        } else {
            encoded = await context.rgbPromise;
        }
        if (!encoded?.blob) throw new Error('RGB JPEG encoding failed');
        const planningState = context.planningState;
        const frame = new PerceptionFrame({
            frameId: context.frameId,
            capturedAt: context.capturedAt,
            captureSimTimeS: planningState.simTimeS,
            transform: context.transform,
            rgb: encoded.blob,
            actualState: planningState.actualState,
            referenceState: planningState.referenceState,
            yaw: planningState.yaw,
            captureProfile: context.captureProfile,
            rgbFrameComplete: context.rgbFrameComplete,
            rgbTilesReady: context.rgbTilesReady,
            rgbReadyFaces: context.rgbReadyFaces,
            rgbTotalFaces: context.rgbTotalFaces,
            rgbReadinessReason: context.rgbReadinessReason,
            rgbTileError: context.rgbTileError,
            faceTileReadiness: context.faceTileReadiness,
            projectionConfig: {
                ...context.projectionConfig,
                rgbWidth: encoded.width,
                rgbHeight: encoded.height,
                jpegQuality: PANORAMA_JPEG_QUALITY,
                uploadScale: encoded.width / context.projectionConfig.width,
            },
        });
        return {
            frame,
            jpegMs: encoded.jpegMs,
            uploadEncoding: encoded.uploadEncoding || 'jpeg',
        };
    }

    primeFromCaptureResult(result, captureMs = 0, context = {}) {
        if (!this.rgbCanvas) return false;
        const structuredResult = result && typeof result === 'object' && 'complete' in result;
        const panoCanvas = structuredResult ? result.canvas : result;
        const complete = structuredResult ? result.complete !== false : true;
        if (!complete || !isDrawableImageSource(panoCanvas)) return false;
        const readiness = normalizeCaptureReadiness(result, structuredResult, complete);

        const ctx = this.rgbCanvas.getContext('2d');
        ctx.clearRect(0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
        ctx.drawImage(panoCanvas, 0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
        const now = performance.now();
        if (!Number.isFinite(this.lastCaptureStartTime) || this.lastCaptureStartTime <= 0) {
            this.lastCaptureStartTime = now;
        }
        this.lastCaptureTime = now;
        const replacedUnrequestedFrame = this._rgbFrameId > this._lastRequestedFrameId;
        this._rgbFrameId++;
        if (replacedUnrequestedFrame) demoPerformance.recordLatestSlotDrop(now);
        demoPerformance.recordCapture(this._desiredDepthMode(), now, captureMs);
        this._rgbReadiness = Object.freeze({ frameId: this._rgbFrameId, ...readiness });
        const planningState = context.planningState || this._nextPlanningState;
        this._rgbFrameContext = this._makeRgbFrameContext(
            this._rgbFrameId,
            Number(context.capturedAt ?? now),
            context.transform,
            planningState,
            this.rgbCanvas,
            result?.timings_ms || { total: captureMs },
            context.captureProfile ?? this._captureProfile,
            context.planningEpoch ?? this._planningEpoch,
            readiness,
        );
        this.hasRgb = true;
        this._setRgbStatus(`preloaded ${Math.round(captureMs)}ms`);
        return true;
    }

    update(world, transform, now = performance.now()) {
        if (!this.panel || !this.rgbCanvas || !world || !transform) return;
        this._applyVisibility();
        if (!this._shouldRun()) return;

        // 深度请求与下一次全景采集可流水化，但只保留一个在途请求，
        // 并且每个完整 RGB frame 最多请求一次，避免排队和时空拼接。
        if (this._shouldRequestDepth(now)) {
            this._requestDepth(this.rgbCanvas);
        }

        // 全景采集：catch stuck captures, skip if busy
        if (this.capturing) {
            if (this._captureProfile === 'flight'
                && now - this.lastCaptureStartTime > CAPTURE_STALL_TIMEOUT_MS) {
                this._abortActiveCapture('capture-timeout');
            }
            return;
        }
        // A calibration bundle must see the exact viewer/tileset state that
        // produced its frozen RGB. Do not begin the next six-face capture until
        // anchors and raw output have both been bound to that frame.
        if (this._calibrationCapturePending) return;
        const captureIntervalMs = demoPerformance.captureIntervalMs(
            this._desiredDepthMode(),
            this._nextPlanningState || this._yopoPose,
            CAPTURE_INTERVAL_MS,
        );
        if (now - this.lastCaptureStartTime < captureIntervalMs) return;
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
        else if (outcome === 'rejected') label = `${normalized} · rejected`;
        else if (outcome === 'ignored') label = `${normalized} · paused`;
        else if (normalized === 'error' && reason) label = `error · ${shortError(reason)}`;
        else if (Number.isFinite(latencyMs)) label = `${normalized} ${Math.round(latencyMs)}ms`;

        if (this.depthStatusEl) {
            this.depthStatusEl.textContent = label;
            this.depthStatusEl.dataset.state = normalized;
            this.depthStatusEl.dataset.outcome = outcome;
            this.depthStatusEl.title = reason || normalized;
        }

        // request-started alternates with the terminal result for every frame.
        // Logging it would defeat terminal-state de-duplication and flood
        // Firefox DevTools at perception rate. The UI is still updated above;
        // console output records each distinct terminal/wait/error state once
        // per goal generation, plus the periodic segmented [depth] summary.
        if (outcome === 'ok' && reason === 'request-started') return;
        // High-rate planning-age drops are summarized by `_markStale` on a
        // time window.  Suppressing this state line prevents alternating
        // ready/stale transitions from bypassing ordinary de-duplication; the
        // UI fields above still update for every event.
        if (outcome === 'stale' && reason === 'stale:planning-frame-too-old') return;
        const rawLogReason = reason || outcome;
        const logReason = /^trajectory-ready:depth-lag-\d+-frames$/.test(rawLogReason)
            ? 'trajectory-ready:depth-lag'
            : rawLogReason;
        const logKey =
            `${normalized}|${outcome}|${logReason}|${this._goalId || '-'}|${this._yopoGeneration}`;
        if (logKey !== this._lastDepthLogKey) {
            console.log(
                `[depth-state] mode=${normalized} goalId=${this._goalId || '-'} ` +
                `frameId=${this._rgbFrameId} generation=${this._yopoGeneration} reason=${logReason}`
            );
            this._lastDepthLogKey = logKey;
        }
    }

    getDepthState() {
        const readiness = this._rgbReadiness;
        return Object.freeze({
            mode: this._depthState,
            outcome: this._depthOutcome,
            reason: this.depthStatusEl?.title || '',
            goalId: this._goalId,
            frameId: this._rgbFrameId,
            generation: this._yopoGeneration,
            planningEpoch: this._planningEpoch,
            planningPaused: this._planningPaused,
            planningPauseReason: this._planningPauseReason,
            hasDepth: this.hasDepth,
            rgbFrameId: readiness.frameId,
            rgbFrameComplete: readiness.rgbFrameComplete,
            rgbTilesReady: readiness.rgbTilesReady,
            rgbReadyFaces: readiness.rgbReadyFaces,
            rgbTotalFaces: readiness.rgbTotalFaces,
            rgbReadinessReason: readiness.rgbReadinessReason,
            rgbTileError: readiness.rgbTileError,
            faceTileReadiness: readiness.faceTileReadiness,
        });
    }

    _rgbMetrics(frameContext = null) {
        const source = frameContext || this._rgbReadiness;
        return Object.freeze({
            rgbFrameId: Number.isSafeInteger(source?.frameId) ? source.frameId : null,
            rgbFrameComplete: source?.rgbFrameComplete === true,
            rgbCapturedFaces: source?.rgbFrameComplete === true
                ? Math.max(0, Number(source?.rgbTotalFaces) || PANORAMA_TOTAL_FACES)
                : Math.min(
                    Math.max(0, Number(source?.rgbTotalFaces) || PANORAMA_TOTAL_FACES),
                    source?.faceTileReadiness?.length || 0,
                ),
            rgbTilesReady: source?.rgbTilesReady === true,
            rgbReadyFaces: Math.max(0, Number(source?.rgbReadyFaces) || 0),
            rgbTotalFaces: Math.max(0, Number(source?.rgbTotalFaces) || PANORAMA_TOTAL_FACES),
            rgbReadinessReason: String(source?.rgbReadinessReason || 'no-rgb-frame'),
            rgbTileError: source?.rgbTileError === true,
            faceTileReadiness: Object.freeze([...(source?.faceTileReadiness || [])]),
        });
    }

    /** Latest numerical 360-depth scan, tied to the frozen RGB capture yaw. */
    getDepthPolarScan() {
        return this._depthPolarFrame;
    }

    _desiredDepthMode() {
        return this._yopoGoal ? 'planning' : 'preview';
    }

    _shouldRequestDepth(now = performance.now()) {
        if (!this.hasRgb) return false;
        if (this._depthGate || this.depthPending) return false;
        if (this._planningPaused && this._yopoGoal) return false;
        if (this._rgbFrameId <= this._lastRequestedFrameId) return false;
        if (this._rgbFrameId < this._minimumRequestFrameId) return false;
        if (this._yopoGoal && !this._yopoPose) {
            this._setDepthState('planning', 'awaiting-pose');
            return false;
        }
        return this._forceNextDepthRequest
            || now - this._lastDepthRequestStartedAt >= DEPTH_INTERVAL_MS;
    }

    _isRequestNavigationCurrent(request) {
        if (!request) return false;
        if (this._activeDepthRequest !== request) return false;
        if (request.generation !== this._yopoGeneration) return false;
        if (request.planningEpoch !== this._planningEpoch) return false;
        if (request.goalId !== this._goalId) return false;
        if (request.mode !== this._desiredDepthMode()) return false;
        return true;
    }

    _isRequestSessionCurrent(request) {
        if (!this._isRequestNavigationCurrent(request)) return false;
        return request.requestId >= this._lastRenderedRequestId;
    }

    // Depth-preview decoding intentionally outlives the network request gate.
    // Its ownership check therefore uses the immutable navigation identity,
    // not `_activeDepthRequest`, while still preventing an old goal/epoch from
    // drawing into the current session.
    _isDepthPreviewSessionCurrent(request) {
        if (!request) return false;
        if (request.generation !== this._yopoGeneration) return false;
        if (request.planningEpoch !== this._planningEpoch) return false;
        if (request.goalId !== this._goalId) return false;
        if (request.mode !== this._desiredDepthMode()) return false;
        return true;
    }

    _isRequestSourceCurrent(request) {
        return this._isRequestSessionCurrent(request)
            // `_materializePerceptionFrame()` synchronously copies the selected
            // RGB canvas into an independent snapshot before JPEG encoding can
            // yield. A newer panorama may therefore complete while that frozen
            // image is encoding without invalidating the selected request.
            && request.frameId === request.frameContext?.frameId
            && request.frameId >= this._minimumRequestFrameId;
    }

    _canCommitDepthPreview(request) {
        return this._isDepthPreviewSessionCurrent(request)
            && (request.mode !== 'planning'
                || request.requestId === this._latestDepthPreviewRequestId)
            && request.requestId > this._lastRenderedRequestId;
    }

    _planningResponseMatchesRequest(payload, request) {
        if (request.mode !== 'planning') return true;
        return String(payload?.frame_id ?? '') === String(request.frameId)
            && String(payload?.goal_id ?? '') === String(request.goalId)
            && String(payload?.generation ?? '') === String(request.generation);
    }

    _invokeObserver(label, callback, args = []) {
        if (typeof callback !== 'function') {
            return Object.freeze({ available: false, succeeded: false, value: undefined });
        }
        try {
            return Object.freeze({
                available: true,
                succeeded: true,
                value: callback.apply(this, args),
            });
        } catch (error) {
            reportUserError(`${label} observer failed`, error, {
                key: `panorama-observer-${String(label).toLowerCase().replace(/[^a-z0-9]+/g, '-')}`,
                intervalMs: 3000,
            });
            return Object.freeze({ available: true, succeeded: false, value: undefined });
        }
    }

    _emitPerceptionMetrics(metrics) {
        demoPerformance.recordPerceptionCapture(metrics);
        return this._invokeObserver(
            'Perception metrics',
            this.onPerceptionMetrics,
            [metrics],
        ).succeeded;
    }

    _requestStageMetrics(request, now = performance.now()) {
        const captureTimings = request?.frameContext?.captureTimings || null;
        const timing = (...keys) => {
            for (const key of keys) {
                const value = finiteNumberOrNull(captureTimings?.[key]);
                if (value !== null) return value;
            }
            return null;
        };
        const fetchStartedAtMs = finiteNumberOrNull(request?.fetchStartedAtMs);
        const responseHeadersAtMs = finiteNumberOrNull(request?.responseHeadersAtMs);
        const payloadParsedAtMs = finiteNumberOrNull(request?.payloadParsedAtMs);
        const elapsed = (end, start) => end === null || start === null
            ? null
            : Math.max(0, end - start);
        const planningMetadata = request?.planningMetadata
            || emptyPlanningDiagnostics(
                frozenNumberArray(request?.parsedEndstate, 9)
            );
        const observedSchemaVersion = safeIntegerOrNull(
            request?.planningDiagnosticsSchemaVersionObserved
        );
        const gateAcquiredAt = finiteNumberOrNull(request?.gateAcquiredAt);
        const gateWaitMs = finiteNumberOrNull(request?.gateWaitMs)
            ?? (gateAcquiredAt === null ? null : Math.max(0, now - gateAcquiredAt));
        return Object.freeze({
            captureProfile: request?.frameContext?.captureProfile ?? null,
            captureMs: timing('total'),
            renderMs: timing('render'),
            sceneRenderMs: timing('scene_render'),
            tileWaitMs: timing('tile_wait'),
            waitRerenderMs: timing('wait_rerender'),
            faceUploadMs: timing('face_upload'),
            projectMs: timing('project'),
            schedulerYieldMs: timing('scheduler', 'scheduler_yield'),
            jpegMs: finiteNumberOrNull(request?.jpegMs),
            ageAtRequestStartMs: finiteNumberOrNull(request?.ageAtRequestStartMs),
            ageAtFetchStartMs: finiteNumberOrNull(request?.ageAtFetchStartMs),
            ageAtResponseHeadersMs: finiteNumberOrNull(request?.ageAtResponseHeadersMs),
            ageAtJsonParsedMs: finiteNumberOrNull(request?.ageAtJsonParsedMs),
            ageBudgetRemainingMs: finiteNumberOrNull(request?.ageBudgetRemainingMs),
            planningFetchReserveMs: request?.mode === 'planning'
                ? YOPO_MIN_FETCH_REMAINING_MS
                : null,
            planningApplyReserveMs: request?.mode === 'planning'
                ? YOPO_APPLY_DEADLINE_RESERVE_MS
                : null,
            applyDeadlineWallTimeMs: finiteNumberOrNull(
                request?.hardApplyDeadlineWallTimeMs
            ),
            fetchStarted: request?.fetchStarted === true,
            fetchStartedAtMs,
            responseHeadersAtMs,
            payloadParsedAtMs,
            responseHeadersMs: elapsed(responseHeadersAtMs, fetchStartedAtMs),
            fetchToHeadersMs: elapsed(responseHeadersAtMs, fetchStartedAtMs),
            httpBodyMs: elapsed(payloadParsedAtMs, responseHeadersAtMs),
            responseJsonMs: elapsed(payloadParsedAtMs, responseHeadersAtMs),
            networkMs: elapsed(payloadParsedAtMs, fetchStartedAtMs),
            responseBytes: Number.isSafeInteger(request?.responseBytes)
                ? request.responseBytes
                : null,
            gateWaitMs,
            serverMs: finiteNumberOrNull(request?.serverMs),
            da360Ms: finiteNumberOrNull(request?.da360Ms),
            yopoMs: finiteNumberOrNull(request?.yopoMs),
            serverTimings: request?.serverTimings || null,
            uploadEncoding: request?.uploadEncoding || null,
            ...planningMetadata,
            planningDiagnosticsSchemaVersion: observedSchemaVersion
                ?? planningMetadata.planningDiagnosticsSchemaVersion,
        });
    }

    _abortActiveDepthRequest(reason) {
        const request = this._activeDepthRequest;
        if (!request) return;
        request.abortReason = reason;
        const controller = request.controller || this._depthAbortController;
        if (controller && !controller.signal.aborted) controller.abort(reason);
    }

    _armDepthRequestDeadline(request) {
        const now = performance.now();
        const serviceDeadlineAt = now + DA360_TIMEOUT_MS;
        let deadlineAt = serviceDeadlineAt;
        let reason = 'timeout';
        if (request.mode === 'planning'
            && Number.isFinite(request.networkDeadlineWallTimeMs)
            && request.networkDeadlineWallTimeMs <= deadlineAt) {
            deadlineAt = request.networkDeadlineWallTimeMs;
            reason = 'planning-deadline';
        }
        request.timeoutDeadlineAt = deadlineAt;
        request.timeoutReason = reason;
        request.timeoutHandle = window.setTimeout(() => {
            if (this._activeDepthRequest !== request) return;
            request.abortReason = reason;
            if (!request.controller.signal.aborted) request.controller.abort(reason);
        }, Math.max(0, deadlineAt - now));
    }

    _releaseDepthRequest(request) {
        if (this._activeDepthRequest !== request) return false;
        if (request.timeoutHandle !== null && request.timeoutHandle !== undefined) {
            window.clearTimeout(request.timeoutHandle);
            request.timeoutHandle = null;
        }
        request.gateReleasedAt = performance.now();
        request.gateWaitMs = request.gateReleasedAt - request.gateAcquiredAt;
        this._activeDepthRequest = null;
        this._depthAbortController = null;
        this.depthPending = false;
        this._depthGate = false;
        this._queueLatestDepthRequest();
        return true;
    }

    _enqueueDepthPreview(request, run) {
        this._latestDepthPreviewRequestId = request.requestId;
        this._pendingDepthPreviewJob = Object.freeze({ request, run });
        if (this._depthPreviewWorkerRunning) return this._depthPreviewWorkerPromise;

        this._depthPreviewWorkerRunning = true;
        const worker = (async () => {
            try {
                while (this._pendingDepthPreviewJob) {
                    const job = this._pendingDepthPreviewJob;
                    this._pendingDepthPreviewJob = null;
                    // Coalesce queued previews before decode.  A newer job that
                    // arrives during decode is also checked by the commit gate.
                    if (job.request.requestId !== this._latestDepthPreviewRequestId) continue;
                    try {
                        await job.run();
                    } catch (error) {
                        // Preview/UI observers are outside the control path. A
                        // faulty decoder or observer must not reject the worker
                        // promise or prevent the newest queued preview running.
                        reportUserError('Planning depth preview worker failed', error, {
                            key: 'planning-depth-preview-worker', intervalMs: 3000,
                        });
                    }
                }
            } finally {
                this._depthPreviewWorkerRunning = false;
                if (this._pendingDepthPreviewJob) {
                    this._enqueueDepthPreview(
                        this._pendingDepthPreviewJob.request,
                        this._pendingDepthPreviewJob.run,
                    );
                }
            }
        })();
        this._depthPreviewWorkerPromise = worker;
        return worker;
    }

    async _waitForDepthPreviewIdle() {
        while (this._depthPreviewWorkerRunning || this._pendingDepthPreviewJob) {
            await this._depthPreviewWorkerPromise;
        }
    }

    _invalidateQueuedDepthPreviews() {
        this._pendingDepthPreviewJob = null;
    }

    _logStaleSummary(request, reason) {
        const key = `${request?.goalId || '-'}|${request?.generation ?? '-'}|`
            + `${request?.planningEpoch ?? '-'}|${reason}`;
        const now = performance.now();
        const prior = this._staleLogBuckets.get(key) || {
            count: 0,
            lastLogAt: -Infinity,
            firstAt: now,
        };
        prior.count++;
        const shouldLog = now - prior.lastLogAt >= STALE_LOG_SUMMARY_INTERVAL_MS;
        if (shouldLog) {
            console.log(
                `[depth-stale] mode=${request?.mode || this._desiredDepthMode()} `
                + `goalId=${request?.goalId || '-'} frameId=${request?.frameId ?? this._rgbFrameId} `
                + `generation=${request?.generation ?? this._yopoGeneration} reason=${reason} `
                + `count=${prior.count}`
            );
            prior.count = 0;
            prior.firstAt = now;
            prior.lastLogAt = now;
        }
        this._staleLogBuckets.set(key, prior);
    }

    _markStale(request, reason) {
        const completedAt = performance.now();
        const desiredMode = this._desiredDepthMode();
        // An old request may finish after setYopoGoal/resetYopoGoal advanced the
        // navigation identity.  Preserve its diagnostic with the frozen request
        // identity, but never let it overwrite the new session's UI state.
        const ownsCurrentUi = !request || this._isDepthPreviewSessionCurrent(request);
        if (ownsCurrentUi) {
            this._setDepthState(desiredMode, `stale:${reason}`, { outcome: 'stale' });
        }
        if (reason === 'planning-frame-too-old') {
            this._logStaleSummary(request, reason);
        } else {
            console.log(
                `[depth] mode=${request?.mode || desiredMode} goalId=${request?.goalId || '-'} ` +
                `frameId=${request?.frameId ?? this._rgbFrameId} generation=${request?.generation ?? this._yopoGeneration} ` +
                `reason=stale:${reason}`
            );
        }
        this._emitPerceptionMetrics(Object.freeze({
            frameId: request?.frameId ?? null,
            goalId: request?.goalId ?? null,
            generation: request?.generation ?? this._yopoGeneration,
            mode: request?.mode || desiredMode,
            outcome: 'stale',
            dropReason: reason,
            frameAgeMs: request?.frameContext
                ? completedAt - request.frameContext.capturedAt
                : null,
            ...this._requestStageMetrics(request, completedAt),
            ...this._rgbMetrics(request?.frameContext),
        }));
    }

    _cancelDepthCatchup() {
        if (this._depthCatchupTimer !== null) {
            window.clearTimeout(this._depthCatchupTimer);
            this._depthCatchupTimer = null;
        }
        this._depthCatchupDeadlineAt = null;
    }

    _queueLatestDepthRequest(now = performance.now()) {
        if (!this.hasRgb) return;
        if (this._depthGate || this.depthPending) return;
        if (this._planningPaused && this._yopoGoal) return;
        if (this._rgbFrameId <= this._lastRequestedFrameId) return;
        if (this._rgbFrameId < this._minimumRequestFrameId) return;
        if (this._yopoGoal && !this._yopoPose) return;

        const earliestStartAt = this._forceNextDepthRequest
            ? now
            : this._lastDepthRequestStartedAt + DEPTH_INTERVAL_MS;
        if (now < earliestStartAt) {
            if (this._depthCatchupTimer !== null
                && this._depthCatchupDeadlineAt <= earliestStartAt) {
                return;
            }
            this._cancelDepthCatchup();
            this._depthCatchupDeadlineAt = earliestStartAt;
            this._depthCatchupTimer = window.setTimeout(() => {
                this._depthCatchupTimer = null;
                this._depthCatchupDeadlineAt = null;
                this._queueLatestDepthRequest();
            }, Math.max(1, Math.ceil(earliestStartAt - now)));
            return;
        }

        this._cancelDepthCatchup();
        if (this._depthCatchupQueued) return;
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
        const captureProfile = this._captureProfile;
        const captureController = new AbortController();
        this._captureAbortController = captureController;
        this.lastCaptureStartTime = performance.now();
        const capturedAt = this.lastCaptureStartTime;
        const capturePlanningState = this._nextPlanningState;
        const capturePlanningEpoch = this._planningEpoch;
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
                ...this.getCaptureOptions({ profile: captureProfile }),
                signal: captureController.signal,
            });
            if (captureController.signal.aborted) {
                const abortError = new Error('panorama capture aborted');
                abortError.name = 'AbortError';
                throw abortError;
            }
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
            const readiness = normalizeCaptureReadiness(result, structuredResult, complete);

            const ctx = this.rgbCanvas.getContext('2d');
            ctx.clearRect(0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            ctx.drawImage(panoCanvas, 0, 0, this.rgbCanvas.width, this.rgbCanvas.height);
            this.lastCaptureTime = performance.now();
            const captureMs = this.lastCaptureTime - this.lastCaptureStartTime;
            this._rgbFrameId++;
            this._rgbReadiness = Object.freeze({ frameId: this._rgbFrameId, ...readiness });
            const frameContext = this._makeRgbFrameContext(
                this._rgbFrameId,
                capturedAt,
                captureTransform,
                capturePlanningState,
                this.rgbCanvas,
                result?.timings_ms || { total: captureMs },
                captureProfile,
                capturePlanningEpoch,
                readiness,
            );
            this._rgbFrameContext = frameContext;
            this.hasRgb = true;
            this._setRgbStatus(`${Math.round(captureMs)}ms`);

            if (this._shouldRequestDepth(this.lastCaptureTime)) {
                this._requestDepth(this.rgbCanvas);
            } else {
                this._queueLatestDepthRequest(this.lastCaptureTime);
            }
            // Release capture immediately. The request gate selects the latest
            // context and only then snapshots/encodes it, so intermediate
            // captures can never create an image-encoder backlog.
        } catch (error) {
            if (error?.name === 'AbortError' || captureController.signal.aborted) {
                this._setRgbStatus(`stale · ${captureController.signal.reason || 'capture-aborted'}`);
                this._emitPerceptionMetrics(Object.freeze({
                    captureId,
                    outcome: 'stale',
                    dropReason: String(captureController.signal.reason || 'capture-aborted'),
                    mode: 'capture',
                    intendedDepthMode: this._desiredDepthMode(),
                    goalId: this._goalId,
                    generation: this._yopoGeneration,
                    ...this._rgbMetrics(),
                }));
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

    async _canvasToJpegBlob(canvas) {
        if (!canvas || !canvas.width || !canvas.height) return null;

        if (typeof canvas.convertToBlob === 'function') {
            try {
                return await canvas.convertToBlob({
                    type: 'image/jpeg',
                    quality: PANORAMA_JPEG_QUALITY,
                });
            } catch (error) {
                reportUserError('Offscreen JPEG encoding failed', error, {
                    key: 'offscreen-jpeg', intervalMs: 3000,
                });
            }
        }

        if (typeof canvas.toBlob === 'function') {
            return new Promise(resolve => {
                canvas.toBlob(resolve, 'image/jpeg', PANORAMA_JPEG_QUALITY);
            });
        }
        return null;
    }

    async _decodeAndCommitDepthImage(
        source,
        request,
        {
            markStale = true,
            timings = null,
            commitPolarFrame = false,
            polarFrame = null,
        } = {},
    ) {
        const decodeStartedAt = performance.now();
        const decoded = await decodeDepthImageSource(source, globalThis);
        const decodedAt = performance.now();
        if (timings) timings.depthDecodeMs = decodedAt - decodeStartedAt;
        try {
            if (!this._canCommitDepthPreview(request)) {
                if (markStale) {
                    const reason = this._isRequestNavigationCurrent(request)
                        ? 'depth-preview-superseded'
                        : 'image-decoded-after-session-change';
                    this._markStale(request, reason);
                }
                return false;
            }
            if (!this.depthCanvas) throw new Error('depth canvas unavailable');
            const ctx = this.depthCanvas.getContext('2d', { alpha: false });
            if (!ctx || typeof ctx.drawImage !== 'function') throw new Error('depth canvas context unavailable');
            const drawStartedAt = performance.now();
            ctx.drawImage(decoded, 0, 0, this.depthCanvas.width, this.depthCanvas.height);
            if (timings) timings.depthDrawMs = performance.now() - drawStartedAt;
            // Canvas pixels and their numerical polar projection share the same
            // latest/session gate and become visible in one synchronous commit.
            if (commitPolarFrame) this._depthPolarFrame = polarFrame;
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
        if (mode === 'planning' && this._planningPaused) {
            this._setDepthState(
                'planning',
                `paused:${this._planningPauseReason || 'controller-owned-motion'}`,
                { outcome: 'ignored' },
            );
            return false;
        }
        if (mode === 'planning' && !this._yopoPose) {
            this._setDepthState('planning', 'awaiting-pose');
            return;
        }

        const frameContext = this._rgbFrameContext;
        if (!frameContext) {
            this._setDepthState(mode, 'awaiting-complete-rgb-frame');
            return false;
        }
        const gateAcquiredAt = performance.now();
        this._cancelDepthCatchup();
        this._lastDepthRequestStartedAt = gateAcquiredAt;
        const hardApplyDeadlineWallTimeMs = mode === 'planning'
            ? frameContext.capturedAt + YOPO_MAX_FRAME_AGE_MS
            : null;
        const request = {
            requestId: ++this._depthRequestSequence,
            frameId: frameContext.frameId,
            generation: this._yopoGeneration,
            planningEpoch: frameContext.planningEpoch,
            goalId: this._goalId,
            mode,
            goal: this._yopoGoal ? { ...this._yopoGoal } : null,
            frameContext,
            perceptionFrame: null,
            controller: new AbortController(),
            abortReason: null,
            timeoutHandle: null,
            timeoutDeadlineAt: null,
            timeoutReason: null,
            hardApplyDeadlineWallTimeMs,
            networkDeadlineWallTimeMs: mode === 'planning'
                ? hardApplyDeadlineWallTimeMs - YOPO_APPLY_DEADLINE_RESERVE_MS
                : null,
            gateAcquiredAt,
            gateReleasedAt: null,
            gateWaitMs: null,
            fetchStarted: false,
            includePreview: mode !== 'planning',
            previewRequested: mode !== 'planning',
            ageAtRequestStartMs: gateAcquiredAt - frameContext.capturedAt,
            ageAtFetchStartMs: null,
            ageAtResponseHeadersMs: null,
            ageAtJsonParsedMs: null,
            ageBudgetRemainingMs: null,
            jpegMs: null,
            fetchStartedAtMs: null,
            responseHeadersAtMs: null,
            payloadParsedAtMs: null,
            responseBytes: null,
            serverMs: null,
            da360Ms: null,
            yopoMs: null,
            serverTimings: null,
            parsedEndstate: null,
            planningDiagnosticsSchemaVersionObserved: null,
            planningMetadata: null,
            uploadEncoding: mode === 'planning' ? 'rgba8' : 'jpeg',
        };
        this._depthReqStart = request.gateAcquiredAt;
        this._lastRequestedFrameId = request.frameId;
        this._forceNextDepthRequest = false;
        this._depthGate = true;
        this.depthPending = true;
        this._activeDepthRequest = request;
        this._depthAbortController = request.controller;
        this._armDepthRequestDeadline(request);
        this._setDepthState(mode, 'request-started');

        let tA = performance.now();
        let tB = tA;

        try {
            request.ageBudgetRemainingMs = YOPO_MAX_FRAME_AGE_MS
                - request.ageAtRequestStartMs;
            if (request.mode === 'planning'
                && request.ageBudgetRemainingMs <= YOPO_MIN_FETCH_REMAINING_MS) {
                this._markStale(request, 'planning-frame-too-old');
                return false;
            }
            const materialized = await awaitWithAbort(
                this._materializePerceptionFrame(frameContext, request.uploadEncoding),
                request.controller.signal,
            );
            const frame = materialized.frame;
            request.perceptionFrame = frame;
            this._perceptionFrame = frame;
            request.jpegMs = finiteNumberOrNull(materialized.jpegMs);
            request.uploadEncoding = materialized.uploadEncoding;
            const blob = frame.rgb;
            tB = performance.now();
            if (!this._isRequestSourceCurrent(request)) {
                this._markStale(request, 'session-changed-before-fetch');
                return false;
            }

            const fetchBudgetCheckedAt = performance.now();
            request.ageAtFetchStartMs = fetchBudgetCheckedAt - frame.capturedAt;
            request.ageBudgetRemainingMs = YOPO_MAX_FRAME_AGE_MS - request.ageAtFetchStartMs;
            if (request.mode === 'planning'
                && request.ageBudgetRemainingMs <= YOPO_MIN_FETCH_REMAINING_MS) {
                this._markStale(request, 'planning-frame-too-old');
                return false;
            }

            if (request.mode === 'planning') {
                request.includePreview = false;
                request.previewRequested = fetchBudgetCheckedAt - this._lastPlanningPreviewFetchAt
                    >= PLANNING_PREVIEW_INTERVAL_MS;
                if (request.previewRequested) {
                    this._lastPlanningPreviewFetchAt = fetchBudgetCheckedAt;
                }
            }

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
                    // Control responses are always compact. Operator preview
                    // is fetched later from the exact cached planning depth on
                    // an independent latest-only worker.
                    include_preview: '0',
                    prepare_preview: request.previewRequested ? '1' : '0',
                }).toString();
                url = `${getYopoEndpoint()}?${qs}`;  // /yopo/plan_full
                headers = {
                    'Content-Type': blob.type || 'image/jpeg',
                    'X-Projection-Config': projectionHeader,
                };
                if (request.uploadEncoding === 'rgba8') {
                    headers['X-Image-Width'] = String(frame.projectionConfig.rgbWidth);
                    headers['X-Image-Height'] = String(frame.projectionConfig.rgbHeight);
                }
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
            request.fetchStarted = true;
            request.fetchStartedAtMs = fetchStartedAt;
            request.ageAtFetchStartMs = fetchStartedAt - frame.capturedAt;
            request.ageBudgetRemainingMs = YOPO_MAX_FRAME_AGE_MS - request.ageAtFetchStartMs;
            if (request.mode === 'planning'
                && request.ageBudgetRemainingMs <= YOPO_MIN_FETCH_REMAINING_MS) {
                request.fetchStarted = false;
                this._markStale(request, 'planning-frame-too-old');
                return false;
            }
            const response = await awaitWithAbort(fetch(url, {
                method: 'POST',
                headers,
                body,
                signal: request.controller.signal,
            }), request.controller.signal);
            const responseHeadersAt = performance.now();
            request.responseHeadersAtMs = responseHeadersAt;
            request.ageAtResponseHeadersMs = responseHeadersAt - frame.capturedAt;
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            const payload = await awaitWithAbort(
                response.json(),
                request.controller.signal,
            );
            const payloadParsedAt = performance.now();
            request.payloadParsedAtMs = payloadParsedAt;
            request.ageAtJsonParsedMs = payloadParsedAt - frame.capturedAt;
            request.responseBytes = responseContentLength(response);
            request.serverMs = finiteNumberOrNull(payload.latency_ms);
            request.da360Ms = finiteNumberOrNull(
                payload.timings_ms?.da360_ms ?? payload.timings_ms?.infer_ms
            );
            request.yopoMs = finiteNumberOrNull(payload.timings_ms?.yopo_ms);
            request.serverTimings = payload.timings_ms
                && typeof payload.timings_ms === 'object'
                ? Object.freeze({ ...payload.timings_ms })
                : null;
            request.parsedEndstate = Array.isArray(payload.endstate)
                ? [...payload.endstate]
                : null;
            request.planningDiagnosticsSchemaVersionObserved = safeIntegerOrNull(
                payload.planning_diagnostics?.schema_version
            );
            const responseSessionCurrent = request.mode === 'planning'
                ? this._isRequestNavigationCurrent(request)
                : this._isRequestSessionCurrent(request);
            if (!responseSessionCurrent) {
                this._markStale(request, 'response-after-session-change');
                return false;
            }
            if (!this._planningResponseMatchesRequest(payload, request)) {
                this._markStale(request, 'response-identity-mismatch');
                return false;
            }
            // Normalize diagnostics as soon as an identity-matched response is
            // available. Later provenance/apply failures must retain the
            // parsed planner evidence instead of collapsing it to null.
            const planningMetadata = normalizePlanningDiagnostics(payload);
            request.planningMetadata = planningMetadata;
            const previewIncluded = typeof payload.depth_image === 'string'
                && payload.depth_image.length > 0;
            if (request.mode !== 'planning' && !previewIncluded) {
                throw new Error('response missing depth_image');
            }
            const captureRotation = frame.transform?.rotation;
            const capturePosition = frame.transform?.position;
            const captureIsLeveled = captureRotation
                && [captureRotation.x, captureRotation.y, captureRotation.z].every(Number.isFinite)
                && Math.abs(captureRotation.x) <= 1e-3
                && Math.abs(captureRotation.z) <= 1e-3;

            this._depthLatency = Number.isFinite(Number(payload.latency_ms))
                ? `${Math.round(Number(payload.latency_ms))}ms`
                : '';
            const planningAuthorized = request.mode !== 'planning'
                || payload.planning_authorized === true;
            if (request.mode === 'planning' && planningAuthorized) {
                const trustedDepthMode = payload.depth_mode === 'da360-metric'
                    || payload.depth_mode === 'cesium-truth';
                if (!trustedDepthMode) {
                    throw new Error(`authorized planning used untrusted depth mode: ${payload.depth_mode || 'missing'}`);
                }
                if (!String(payload.calibration_id || '').trim()) {
                    throw new Error('authorized planning response missing calibration_id');
                }
                if (!String(payload.service_session_id || '').trim()) {
                    throw new Error('authorized planning response missing service_session_id');
                }
            }
            const planningReason = request.mode === 'planning' && !planningAuthorized
                ? String(payload.planning_reason || 'planning-not-authorized')
                : null;
            let finalReason = planningReason;
            let trajectoryApplied = request.mode === 'planning' ? false : null;
            let trajectoryIgnored = request.mode === 'planning' ? false : null;
            let trajectoryAppliedAt = null;
            let trajectoryApplyMs = 0;
            let applyWallTimeMs = null;
            let applyActualState = null;
            let captureToApplyDelta = null;
            let captureToApplyDisplacementM = null;
            let applyDeadlineExceeded = false;

            if (request.mode === 'planning') {
                if (planningAuthorized && !payload.endstate) {
                    throw new Error('authorized planning response missing endstate');
                }
                if (planningAuthorized && payload.endstate) {
                    const candidateApplyWallTimeMs = performance.now();
                    request.ageBudgetRemainingMs = YOPO_MAX_FRAME_AGE_MS
                        - (candidateApplyWallTimeMs - frame.capturedAt);
                    if (candidateApplyWallTimeMs - frame.capturedAt
                        >= YOPO_MAX_FRAME_AGE_MS - YOPO_APPLY_DEADLINE_RESERVE_MS) {
                        this._markStale(request, 'planning-frame-too-old');
                        return false;
                    }
                    applyWallTimeMs = candidateApplyWallTimeMs;
                    applyActualState = this._nextPlanningState?.actualState || null;
                    const capturePositionState = frame.actualState?.position;
                    const applyPositionState = applyActualState?.position;
                    if (capturePositionState && applyPositionState) {
                        const dx = Number(applyPositionState.x) - Number(capturePositionState.x);
                        const dy = Number(applyPositionState.y) - Number(capturePositionState.y);
                        const dz = Number(applyPositionState.z) - Number(capturePositionState.z);
                        if ([dx, dy, dz].every(Number.isFinite)) {
                            captureToApplyDelta = Object.freeze({ x: dx, y: dy, z: dz });
                            captureToApplyDisplacementM = Math.hypot(dx, dy, dz);
                        }
                    }
                    let commitFailureReason = null;
                    let commitDepth = 0;
                    let commitTransactionFailed = false;
                    const pendingCommitRollbacks = [];
                    const rollbackPendingCommits = () => {
                        while (pendingCommitRollbacks.length > 0) {
                            const rollback = pendingCommitRollbacks.pop();
                            try {
                                rollback();
                            } catch (error) {
                                reportUserError('YOPO trajectory rollback failed', error, {
                                    key: 'yopo-trajectory-rollback',
                                    intervalMs: 3000,
                                });
                            }
                        }
                    };
                    const commitIfFresh = (mutation, rollback = null) => {
                        const rootCommit = commitDepth === 0;
                        if (rootCommit) {
                            pendingCommitRollbacks.length = 0;
                            commitTransactionFailed = false;
                        }
                        let localSucceeded = false;
                        commitDepth++;
                        try {
                            if (typeof mutation !== 'function') {
                                commitFailureReason = 'trajectory-commit-callback-invalid';
                                commitTransactionFailed = true;
                                return false;
                            }
                            if (rollback !== null && typeof rollback !== 'function') {
                                commitFailureReason = 'trajectory-rollback-callback-invalid';
                                commitTransactionFailed = true;
                                return false;
                            }
                            if (performance.now() >= request.hardApplyDeadlineWallTimeMs) {
                                commitFailureReason = 'trajectory-apply-deadline-exceeded';
                                commitTransactionFailed = true;
                                return false;
                            }
                            // Arm recovery before entering user/controller code:
                            // a callback that mutates and then throws is still a
                            // failed transaction that must be compare-and-cleared.
                            if (rollback) pendingCommitRollbacks.push(rollback);
                            const committed = mutation() === true;
                            if (performance.now() >= request.hardApplyDeadlineWallTimeMs) {
                                // A cooperative consumer supplies a compare-and-
                                // clear rollback so a mutation that itself crosses
                                // the hard boundary cannot remain published. The
                                // rollback must be synchronous and scoped to this
                                // immutable request identity; a newer N+1 command
                                // must never be cleared by late N accounting.
                                commitFailureReason = 'trajectory-apply-deadline-exceeded';
                                commitTransactionFailed = true;
                                rollbackPendingCommits();
                                return false;
                            }
                            if (!committed) commitTransactionFailed = true;
                            if (commitTransactionFailed) return false;
                            localSucceeded = true;
                            return true;
                        } finally {
                            commitDepth--;
                            if (rootCommit) {
                                if (!localSucceeded || commitTransactionFailed) {
                                    rollbackPendingCommits();
                                } else {
                                    // The outer post-check is the transaction's
                                    // commit point. Inner rollbacks remain armed
                                    // until this exact point.
                                    pendingCommitRollbacks.length = 0;
                                }
                            }
                        }
                    };
                    const context = Object.freeze({
                        mode: request.mode,
                        goalId: request.goalId,
                        frameId: request.frameId,
                        generation: request.generation,
                        planningEpoch: request.planningEpoch,
                        requestId: request.requestId,
                        depthMode: payload.depth_mode || null,
                        calibrationId: payload.calibration_id || null,
                        calibrationAccuracyAccepted:
                            typeof payload.calibration_accuracy_accepted === 'boolean'
                                ? payload.calibration_accuracy_accepted
                                : null,
                        serviceSessionId: payload.service_session_id || null,
                        planningAuthorized: true,
                        planningReason: payload.planning_reason || null,
                        timings: payload.timings_ms || null,
                        captureSimTimeS: finiteNumberOrNull(frame.captureSimTimeS),
                        captureActualState: frame.actualState || null,
                        captureReferenceState: frame.referenceState || null,
                        captureWallTimeMs: frame.capturedAt,
                        applyWallTimeMs,
                        applyDeadlineWallTimeMs: request.hardApplyDeadlineWallTimeMs,
                        commitIfFresh,
                        planningAgeS: (applyWallTimeMs - frame.capturedAt) / 1000,
                        applyActualState,
                        captureToApplyDelta,
                        captureToApplyDisplacementM,
                        planningOrigin: payload.planning_origin || null,
                        ...planningMetadata,
                        ...this._rgbMetrics(frameContext),
                    });
                    const trajectoryApplyStartedAt = performance.now();
                    let applyObserver = null;
                    let observerDisposition = null;
                    const observerCommitted = commitIfFresh(() => {
                        applyObserver = this._invokeObserver(
                            'YOPO trajectory',
                            this.onYopoResult,
                            [payload.endstate, payload.traj_time, context],
                        );
                        observerDisposition = normalizeYopoObserverResult(applyObserver);
                        return observerDisposition.committed;
                    });
                    trajectoryApplied = observerCommitted
                        && observerDisposition?.outcome === 'applied';
                    trajectoryIgnored = observerCommitted
                        && observerDisposition?.outcome === 'ignored';
                    trajectoryAppliedAt = performance.now();
                    trajectoryApplyMs = trajectoryAppliedAt - trajectoryApplyStartedAt;
                    applyDeadlineExceeded = commitFailureReason
                        === 'trajectory-apply-deadline-exceeded';
                    if (trajectoryApplied) {
                        this._invokeObserver(
                            'YOPO latency',
                            this.onYopoLatency,
                            [payload.latency_ms],
                        );
                    } else if (trajectoryIgnored) {
                        finalReason = observerDisposition.reason || 'consumer-ignored';
                    } else {
                        finalReason = commitFailureReason
                            || (!applyObserver?.available
                                ? 'trajectory-handler-unavailable'
                                : !applyObserver.succeeded
                                ? 'trajectory-handler-error'
                                : 'trajectory-apply-rejected');
                    }
                }
            }

            const makeMetrics = ({
                outcome,
                depthPreviewError = null,
                depthPreviewCommitted = null,
                depthCommittedAt = null,
                depthCommitStartedAt = null,
                depthDecodeMs = null,
                depthDrawMs = null,
            }) => {
                const metricAppliedAt = trajectoryAppliedAt
                    || depthCommittedAt
                    || performance.now();
                const depthCommitMs = depthCommittedAt != null
                    && depthCommitStartedAt != null
                    ? depthCommittedAt - depthCommitStartedAt
                    : null;
                return Object.freeze({
                    frameId: request.frameId,
                    goalId: request.goalId,
                    generation: request.generation,
                    mode: request.mode,
                    outcome,
                    planningAuthorized: request.mode === 'planning' ? planningAuthorized : null,
                    trajectoryApplied,
                    trajectoryIgnored,
                    trajectoryAppliedAtMs: trajectoryApplied ? trajectoryAppliedAt : null,
                    dropReason: finalReason,
                    depthMode: payload.depth_mode || null,
                    calibrationId: payload.calibration_id || null,
                    calibrationAccuracyAccepted:
                        typeof payload.calibration_accuracy_accepted === 'boolean'
                            ? payload.calibration_accuracy_accepted
                            : null,
                    serviceSessionId: payload.service_session_id || null,
                    previewRequested: request.previewRequested,
                    previewIncluded,
                    depthPreviewError,
                    depthPreviewCommitted,
                    depthPreviewLagFrames: Math.max(
                        0,
                        this._rgbFrameId - request.frameId,
                    ),
                    depthPreviewAgeMs: metricAppliedAt - frame.capturedAt,
                    ...this._requestStageMetrics(request, metricAppliedAt),
                    applyDeadlineExceeded,
                    depthCommitMs,
                    depthDecodeMs,
                    depthDrawMs,
                    trajectoryApplyMs,
                    applyMs: trajectoryApplied ? trajectoryApplyMs : depthCommitMs,
                    captureToApplyMs: metricAppliedAt - frame.capturedAt,
                    frameAgeMs: metricAppliedAt - frame.capturedAt,
                    captureSimTimeS: finiteNumberOrNull(frame.captureSimTimeS),
                    captureWallTimeMs: frame.capturedAt,
                    applyWallTimeMs,
                    planningAgeS: applyWallTimeMs == null
                        ? null
                        : (applyWallTimeMs - frame.capturedAt) / 1000,
                    captureActualState: frame.actualState || null,
                    captureReferenceState: frame.referenceState || null,
                    applyActualState,
                    captureToApplyDelta,
                    captureToApplyDisplacementM,
                    planningOrigin: payload.planning_origin || null,
                    ...this._rgbMetrics(frameContext),
                });
            };
            const outcome = request.mode !== 'planning'
                ? 'applied'
                : !planningAuthorized
                ? 'blocked'
                : applyDeadlineExceeded
                ? 'stale'
                : trajectoryApplied
                ? 'applied'
                : trajectoryIgnored
                ? 'ignored'
                : 'rejected';
            const makePolarFrame = (previewPayload = payload) => {
                let polarScan = normalizeDepthPolarScan(previewPayload.polar_scan);
                if (polarScan && polarScan.depthMode !== previewPayload.depth_mode) {
                    throw new Error('polar scan depth mode does not match response depth mode');
                }
                if (!captureIsLeveled || !capturePosition) polarScan = null;
                return polarScan
                    ? Object.freeze({
                        scan: polarScan,
                        frameId: request.frameId,
                        requestId: request.requestId,
                        capturedAt: frame.capturedAt,
                        captureYawDeg: captureRotation.y,
                        capturePosition: Object.freeze({
                            x: capturePosition.x,
                            y: capturePosition.y,
                            z: capturePosition.z,
                        }),
                    })
                    : null;
            };
            const recordCycle = (completedAt, applyDurationMs) => {
                const depthPreviewLagFrames = Math.max(0, this._rgbFrameId - request.frameId);
                this._depthFpsCount++;
                this._depthCycleSum += completedAt - this._depthReqStart;
                if (completedAt - this._depthFpsTimer <= 2000) return;
                const fps = this._depthFpsCount
                    / ((completedAt - this._depthFpsTimer) / 1000);
                const avgCycle = this._depthCycleSum / Math.max(1, this._depthFpsCount);
                console.log(
                    `[depth] ${fps.toFixed(1)}Hz mode=${request.mode} goalId=${request.goalId || '-'} `
                    + `frameId=${request.frameId} generation=${request.generation} `
                    + `rgbCaptured=${frameContext.rgbFrameComplete ? frameContext.rgbTotalFaces : 0}/${frameContext.rgbTotalFaces} `
                    + `depthLag=${depthPreviewLagFrames}f `
                    + `reason=${finalReason ? `${outcome}:${finalReason}` : 'ok'} `
                    + `capture=${Math.round(frameContext.captureTimings?.total || 0)}ms `
                    + `render=${Math.round(frameContext.captureTimings?.render || 0)}ms `
                    + `scene=${Math.round(frameContext.captureTimings?.scene_render || 0)}ms `
                    + `tileWait=${Math.round(frameContext.captureTimings?.tile_wait || 0)}ms `
                    + `waitRender=${Math.round(frameContext.captureTimings?.wait_rerender || 0)}ms `
                    + `upload=${Math.round(frameContext.captureTimings?.face_upload || 0)}ms `
                    + `project=${Math.round(frameContext.captureTimings?.project || 0)}ms `
                    + `jpeg=${Math.round(materialized.jpegMs || tB - tA)}ms `
                    + `format=${request.uploadEncoding} `
                    + `headers=${Math.round(responseHeadersAt - fetchStartedAt)}ms `
                    + `body=${Math.round(payloadParsedAt - responseHeadersAt)}ms `
                    + `bytes=${request.responseBytes ?? '-'} `
                    + `gate=${Math.round(request.gateWaitMs ?? completedAt - request.gateAcquiredAt)}ms `
                    + `da360=${Math.round(payload.timings_ms?.da360_ms ?? payload.timings_ms?.infer_ms ?? 0)}ms `
                    + `yopo=${Math.round(payload.timings_ms?.yopo_ms || 0)}ms `
                    + `apply=${Math.round(applyDurationMs || 0)}ms `
                    + `age=${Math.round((trajectoryAppliedAt || completedAt) - frame.capturedAt)}ms `
                    + `avgCycle=${avgCycle.toFixed(0)}ms`
                );
                this._depthFpsTimer = completedAt;
                this._depthFpsCount = 0;
                this._depthCycleSum = 0;
            };

            if (request.mode === 'planning') {
                // The compact trajectory response is the control product.  Set
                // state and release the single-request gate before any base64
                // decode or canvas draw belonging only to the operator preview.
                if (this._isRequestSessionCurrent(request)) {
                    this._setDepthState(
                        request.mode,
                        finalReason
                            ? `${outcome}:${finalReason}`
                            : trajectoryApplied ? 'trajectory-ready' : 'depth-ready',
                        { outcome, latencyMs: Number(payload.latency_ms) },
                    );
                }
                this.lastDepthTime = performance.now();
                this._releaseDepthRequest(request);
                this._emitPerceptionMetrics(makeMetrics({ outcome }));
                recordCycle(performance.now(), trajectoryApplyMs);

                const emitDisplayMetrics = ({
                    depthPreviewCommitted,
                    depthPreviewError,
                    depthCommitStartedAt,
                    depthCommittedAt,
                    depthDecodeMs = null,
                    depthDrawMs = null,
                    previewWasIncluded = false,
                    previewFetchMs = null,
                    previewResponseBytes = null,
                }) => {
                    const displayOutcome = depthPreviewCommitted
                        ? 'applied'
                        : depthPreviewError?.startsWith('stale:') ? 'stale' : 'error';
                    const displayMetrics = makeMetrics({
                        outcome: displayOutcome,
                        depthPreviewError,
                        depthPreviewCommitted,
                        depthCommittedAt,
                        depthCommitStartedAt,
                        depthDecodeMs,
                        depthDrawMs,
                    });
                    this._emitPerceptionMetrics(Object.freeze({
                        ...displayMetrics,
                        mode: 'depth-preview',
                        outcome: displayOutcome,
                        planningAuthorized: null,
                        trajectoryApplied: null,
                        trajectoryAppliedAtMs: null,
                        dropReason: depthPreviewError,
                        previewIncluded: previewWasIncluded,
                        previewFetchMs,
                        previewResponseBytes,
                        applyMs: displayMetrics.depthCommitMs,
                        captureToApplyMs: depthCommittedAt - frame.capturedAt,
                        frameAgeMs: depthCommittedAt - frame.capturedAt,
                        depthPreviewAgeMs: depthCommittedAt - frame.capturedAt,
                    }));
                };

                if (!applyDeadlineExceeded && !trajectoryIgnored && request.previewRequested) {
                    // `_releaseDepthRequest()` queued the next compact planning
                    // request first. Queue this UI-only worker afterwards so
                    // preview fetch/colorization can never extend or reacquire
                    // the single control gate.
                    queueMicrotask(() => this._enqueueDepthPreview(request, async () => {
                        const depthCommitStartedAt = performance.now();
                        const previewTimings = {};
                        let depthPreviewCommitted = false;
                        let depthPreviewError = null;
                        let previewPayload = previewIncluded ? payload : null;
                        let previewFetchMs = null;
                        let previewResponseBytes = null;
                        try {
                            if (!this._isDepthPreviewSessionCurrent(request)) {
                                throw new Error('stale:preview-session-changed-before-fetch');
                            }
                            if (!previewPayload && payload.preview_available === true) {
                                const previewFetchStartedAt = performance.now();
                                const previewController = new AbortController();
                                const previewTimeout = window.setTimeout(
                                    () => previewController.abort('preview-timeout'),
                                    Math.min(2000, DA360_TIMEOUT_MS),
                                );
                                try {
                                    const previewQuery = new URLSearchParams({
                                        frame_id: String(request.frameId),
                                        goal_id: String(request.goalId),
                                        generation: String(request.generation),
                                    }).toString();
                                    const previewResponse = await awaitWithAbort(
                                        fetch(`${getYopoPreviewEndpoint()}?${previewQuery}`, {
                                            method: 'GET',
                                            signal: previewController.signal,
                                        }),
                                        previewController.signal,
                                    );
                                    previewResponseBytes = responseContentLength(previewResponse);
                                    if (!previewResponse.ok) {
                                        throw new Error(`preview HTTP ${previewResponse.status}`);
                                    }
                                    previewPayload = await awaitWithAbort(
                                        previewResponse.json(),
                                        previewController.signal,
                                    );
                                } finally {
                                    window.clearTimeout(previewTimeout);
                                    previewFetchMs = performance.now() - previewFetchStartedAt;
                                }
                            }
                            if (!previewPayload) {
                                throw new Error('response-missing-depth-preview');
                            }
                            if (String(previewPayload.frame_id ?? request.frameId)
                                    !== String(request.frameId)
                                || String(previewPayload.goal_id ?? request.goalId)
                                    !== String(request.goalId)
                                || String(previewPayload.generation ?? request.generation)
                                    !== String(request.generation)) {
                                throw new Error('stale:preview-identity-mismatch');
                            }
                            if (typeof previewPayload.depth_image !== 'string'
                                || previewPayload.depth_image.length === 0) {
                                throw new Error('response-missing-depth-preview');
                            }
                            const polarFrame = makePolarFrame(previewPayload);
                            depthPreviewCommitted = await this._decodeAndCommitDepthImage(
                                previewPayload.depth_image,
                                request,
                                {
                                    markStale: false,
                                    timings: previewTimings,
                                    commitPolarFrame: true,
                                    polarFrame,
                                },
                            );
                            if (!depthPreviewCommitted) {
                                depthPreviewError = 'stale:image-decoded-after-session-change';
                            }
                        } catch (error) {
                            depthPreviewError = shortError(error);
                            if (!depthPreviewError.startsWith('stale:')) {
                                reportUserError('Planning depth preview failed', error, {
                                    key: 'planning-depth-preview', intervalMs: 3000,
                                });
                            }
                        }
                        const depthCommittedAt = performance.now();
                        if (depthPreviewCommitted
                            && this._isDepthPreviewSessionCurrent(request)) {
                            this._invokeObserver(
                                'Depth result',
                                this.onDepthResult,
                                [previewPayload?.latency_ms ?? payload.latency_ms],
                            );
                        }
                        emitDisplayMetrics({
                            depthPreviewCommitted,
                            depthPreviewError,
                            depthCommitStartedAt,
                            depthCommittedAt,
                            previewWasIncluded: !!previewPayload?.depth_image,
                            previewFetchMs,
                            previewResponseBytes,
                            ...previewTimings,
                        });
                    }));
                }
                return !applyDeadlineExceeded;
            }

            // Preview-only mode still awaits its display result because there
            // is no trajectory to release independently.
            const polarFrame = makePolarFrame();
            const depthCommitStartedAt = performance.now();
            const previewTimings = {};
            const depthPreviewCommitted = await this._decodeAndCommitDepthImage(
                payload.depth_image,
                request,
                {
                    timings: previewTimings,
                    commitPolarFrame: true,
                    polarFrame,
                },
            );
            if (!depthPreviewCommitted) return false;
            const depthCommittedAt = performance.now();
            if (this._isRequestSessionCurrent(request)) {
                this._setDepthState('preview', 'depth-ready', {
                    outcome,
                    latencyMs: Number(payload.latency_ms),
                });
                this.lastDepthTime = depthCommittedAt;
                this._invokeObserver(
                    'Depth result',
                    this.onDepthResult,
                    [payload.latency_ms],
                );
            }
            this._emitPerceptionMetrics(makeMetrics({
                outcome,
                depthPreviewCommitted,
                depthCommittedAt,
                depthCommitStartedAt,
                ...previewTimings,
            }));
            recordCycle(depthCommittedAt, depthCommittedAt - depthCommitStartedAt);
            return true;
        } catch (error) {
            if (request.abortReason === 'planning-deadline') {
                request.ageBudgetRemainingMs = YOPO_MAX_FRAME_AGE_MS
                    - (performance.now() - request.frameContext.capturedAt);
                this._markStale(request, 'planning-frame-too-old');
                return false;
            }
            const sessionChanged = request.mode === 'planning'
                ? !this._isRequestNavigationCurrent(request)
                : !this._isRequestSessionCurrent(request);
            const transitionAbort = request.abortReason && request.abortReason !== 'timeout';
            if (sessionChanged || transitionAbort) {
                this._markStale(request, request.abortReason || 'session-changed');
                return false;
            }
            reportUserError('DA360/YOPO request failed', error, {
                key: 'da360-depth-request', intervalMs: 3000,
            });
            this._depthPolarFrame = null;
            const failedAt = performance.now();
            this.lastDepthTime = failedAt;
            const offline = request.abortReason === 'timeout' || error?.name === 'TypeError';
            this._setDepthState(offline ? 'offline' : 'error', shortError(error));
            this._emitPerceptionMetrics(Object.freeze({
                frameId: request.frameId,
                goalId: request.goalId,
                generation: request.generation,
                mode: request.mode,
                outcome: offline ? 'offline' : 'error',
                dropReason: shortError(error),
                frameAgeMs: request.frameContext
                    ? failedAt - request.frameContext.capturedAt
                    : null,
                ...this._requestStageMetrics(request, failedAt),
                ...this._rgbMetrics(request.frameContext),
            }));
            return false;
        } finally {
            this._releaseDepthRequest(request);
        }
    }

    setYopoPlanningPaused(paused, reason = 'controller-owned-motion') {
        const nextPaused = !!paused && this._navigationMode === 'active' && !!this._yopoGoal;
        const nextReason = nextPaused
            ? String(reason || 'controller-owned-motion')
            : null;
        if (nextPaused === this._planningPaused
            && nextReason === this._planningPauseReason) {
            return false;
        }
        const wasPaused = this._planningPaused;
        this._planningPaused = nextPaused;
        this._planningPauseReason = nextReason;
        this._cancelDepthCatchup();
        if (nextPaused) {
            this._forceNextDepthRequest = false;
            this._setDepthState('planning', `paused:${nextReason}`, { outcome: 'ignored' });
        } else if (wasPaused && this._navigationMode === 'active' && this._yopoGoal) {
            // A frame captured while terminal motion owned the controller does
            // not automatically become a post-fault planning observation.
            this._minimumRequestFrameId = this._rgbFrameId + 1;
            this._forceNextDepthRequest = true;
            this.lastCaptureStartTime = -Infinity;
            this._setDepthState('planning', 'awaiting-new-rgb-frame');
        }
        return true;
    }

    setYopoGoal(goal, navigationKind = 'fixed') {
        if (!goal) {
            this.resetYopoGoal('empty-goal');
            return null;
        }
        this.setCaptureProfile('flight', 'yopo-goal');
        this._abortActiveDepthRequest('goal-changed');
        this._abortActiveCapture('goal-changed');
        this._cancelDepthCatchup();
        this._invalidateQueuedDepthPreviews();
        this._staleLogBuckets.clear();
        this._yopoGeneration++;
        this._planningEpoch++;
        this._planningPaused = false;
        this._planningPauseReason = null;
        this._goalId = `goal-${++this._goalSequence}`;
        this._yopoGoal = Object.freeze({ ...goal });
        this._navigationMode = 'active';
        this._navigationKind = navigationKind === 't8l-rolling' ? 't8l-rolling' : 'fixed';
        this._navigationTransitionReason = 'goal-set';
        this._minimumRequestFrameId = this._rgbFrameId + 1;
        this._forceNextDepthRequest = true;
        this._lastPlanningPreviewFetchAt = -Infinity;
        this._setDepthState('planning', 'awaiting-new-rgb-frame');
        return this._goalId;
    }

    /** Update a rolling target without changing the asynchronous session identity. */
    updateYopoGoal(goal, navigationKind = 't8l-rolling') {
        const values = [goal?.x, goal?.y, goal?.z].map(Number);
        if (this._navigationMode !== 'active' || !this._goalId
            || !values.every(Number.isFinite)) return false;
        this._yopoGoal = Object.freeze({ x: values[0], y: values[1], z: values[2] });
        this._navigationKind = navigationKind === 't8l-rolling' ? 't8l-rolling' : 'fixed';
        this._navigationTransitionReason = 'goal-updated';
        return true;
    }

    resetYopoGoal(reason = 'goal-reset') {
        this._abortActiveDepthRequest(reason);
        this._abortActiveCapture(reason);
        this._cancelDepthCatchup();
        this._invalidateQueuedDepthPreviews();
        this._staleLogBuckets.clear();
        this._yopoGeneration++;
        this._planningEpoch++;
        this._planningPaused = false;
        this._planningPauseReason = null;
        this._goalId = null;
        this._yopoGoal = null;
        this._navigationKind = null;
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
            kind: this._navigationKind,
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
        if (this._captureProfile !== 'calibration') {
            throw new Error(
                'metric calibration requires the "calibration" panorama capture profile'
            );
        }
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
            if (latestContext?.captureProfile !== 'calibration') {
                throw new Error(
                    'wait for a complete calibration-profile panorama frame before export'
                );
            }
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
            if (frame.captureProfile !== 'calibration') {
                throw new Error('calibration export rejected a non-calibration panorama frame');
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
            assertCalibrationCurrent();

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
                files: Object.freeze({
                    rgb: Object.freeze({ name: filenames.rgb, bytes: frame.rgb.size }),
                    anchors: Object.freeze({ name: filenames.anchors, bytes: anchorsBlob.size }),
                    raw: Object.freeze({ name: filenames.raw, bytes: rawBlob.size }),
                }),
                rawModel: rawResponse.headers.get('X-DA360-Model'),
                rawWidth: Number(rawResponse.headers.get('X-DA360-Width')) || null,
                rawHeight: Number(rawResponse.headers.get('X-DA360-Height')) || null,
                rawLatencyMs: Number(rawResponse.headers.get('X-DA360-Latency-Ms')) || null,
                transform: frame.transform,
                actualState: frame.actualState,
                referenceState: frame.referenceState,
                yaw: frame.yaw,
                captureProfile: frame.captureProfile,
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
