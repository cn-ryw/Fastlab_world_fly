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
 * Main entry point for the Google 3D Tiles flight mode.
 *
 * Rendering is Cesium + Google Photorealistic 3D Tiles. Flight dynamics,
 * controller mapping, WebHID/Gamepad support, HUD and OSD are retained
 * from the original simulator.
 */

import { CesiumWorld } from './cesium-world.js?v=20260703-panorama-tile-idle';
import { TilesCollisionProvider } from './tiles-collision.js';
import { Controller } from './controller.js';
import { Drone } from './drone.js?v=20260807';
import { HUD } from './hud.js';
import { OSD } from './osd.js';
import { PanoramaSensor } from './panorama-sensor.js?v=20260807';
import { FlightLogger } from './flight-logger.js?v=20260807';
import { reportUserError } from './error-report.js';

let world = null;
let collisionProvider = null;
let drone = null;
let controller = null;
let hud = null;
let osd = null;
let panoramaSensor = null;
let flightLogger = null;

let mode = 'loading'; // loading | placement | view-select | flight
let cameraMode = 'first'; // first | third
let spawnPoint = null;
let spawnAltitudeMeters = 100;
let sceneLoaded = false;
let loopStarted = false;
let lastFrameTime = 0;
let placementKeysDown = new Set();
let placementInitClickUntil = 0;
let screenHandler = null;
let flightGoalHandler = null;
let spawnConfirmInProgress = false;
let startTilesModeInProgress = false;
let panoramaWarmupPromise = null;
let thirdPersonPointer = {
    active: false,
    button: -1,
    x: 0,
    y: 0,
};
let thirdPersonCamera = {
    yaw: 0,
    pitch: 0.28,
    distance: 10,
    height: 0.7,
    lateral: 0,
};

const SPAWN_ALTITUDE_MIN = 0;
const SPAWN_ALTITUDE_MAX = 20000;
const SPAWN_ALTITUDE_SLIDER_DEFAULT_MAX = 1000;
const SPAWN_PRELOAD_RADIUS_METERS = Math.round(urlNumber('flightPreloadRadius', 420, 120, 2000));
const FLIGHT_PRELOAD_MIN_COVERAGE = urlNumber('flightPreloadMinCoverage', 0.95, 0.5, 1);
const FLIGHT_PRELOAD_VIEW_TIMEOUT_MS = Math.round(urlNumber('flightPreloadViewTimeoutMs', 20000, 3000, 60000));
const FLIGHT_PRELOAD_VIEW_ATTEMPTS = Math.round(urlNumber('flightPreloadViewAttempts', 2, 1, 5));
const FLIGHT_PRELOAD_STRICT = urlNumber('flightPreloadStrict', 0, 0, 1) >= 0.5;
const PANORAMA_PRELOAD_REQUIRED = urlNumber('panoPreloadRequired', 0, 0, 1) >= 0.5;
const VIEW_CHOICE_HINT_HTML = '1 / O: First Person &nbsp;|&nbsp; 2: Third Person<br>Easy speed: ↑/↓ forward/back, Shift boost, Tab &gt; Easy Max Speed';
const MAX_PHYSICS_FRAME_DT = 0.25;
const PHYSICS_SUBSTEP_DT = 0.05;
const MAX_PHYSICS_SUBSTEPS = 3;
const SETTINGS_READ_INTERVAL_MS = 100;

let lastSettingsReadTime = 0;
let lastKeyGuideState = '';
let lastDisplaySettingsState = '';
let lastHFovReadTime = 0;
let cachedHFov = 120;
let flightStartWarnings = [];

function urlNumber(name, fallback, min = -Infinity, max = Infinity) {
    const value = new URLSearchParams(window.location.search).get(name);
    if (value == null || value === '') return fallback;
    const n = Number(value);
    if (!Number.isFinite(n)) return fallback;
    return Math.max(min, Math.min(max, n));
}

function normalizeViewMode(value, fallback = 'first') {
    return value === 'third' || value === '3rd' ? 'third' : fallback;
}

function clampSpawnAltitude(value) {
    const n = Number(value);
    if (!Number.isFinite(n)) return spawnAltitudeMeters;
    return Math.max(SPAWN_ALTITUDE_MIN, Math.min(SPAWN_ALTITUDE_MAX, n));
}

function setSpawnAltitude(value, updateMarker = true) {
    spawnAltitudeMeters = clampSpawnAltitude(value);
    if (spawnPoint) {
        spawnPoint.y = spawnAltitudeMeters;
        if (updateMarker) world?.updateSpawnMarker(spawnPoint);
    }
    syncSpawnAltitudeControls();
    updateSpawnUI();
}

function syncSpawnAltitudeControls() {
    const slider = document.getElementById('spawn-altitude-range');
    const input = document.getElementById('spawn-altitude-input');
    const value = Math.round(spawnAltitudeMeters * 10) / 10;

    if (slider) {
        const neededMax = Math.max(SPAWN_ALTITUDE_SLIDER_DEFAULT_MAX, Math.ceil(value / 100) * 100);
        slider.max = String(Math.min(SPAWN_ALTITUDE_MAX, neededMax));
        slider.value = String(Math.min(Number(slider.max), value));
    }
    if (input) input.value = String(value);
}

function setProgress(message, isError = false) {
    const el = document.getElementById('loading-progress');
    if (!el) return;
    el.textContent = message;
    el.style.color = isError ? '#f44' : '#4272F5';
}

function escapeHtml(value) {
    return String(value)
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;')
        .replaceAll("'", '&#039;');
}

function shortStatusMessage(value, maxLength = 96) {
    const message = value && value.message ? value.message : String(value || '');
    if (message.length <= maxLength) return message;
    return `${message.slice(0, maxLength - 3)}...`;
}

function rememberFlightStartWarning(message) {
    const text = String(message || '').trim();
    if (!text || flightStartWarnings.includes(text)) return;
    flightStartWarnings.push(text);
}

function updateViewChoiceHint() {
    const el = document.getElementById('view-choice-hint');
    if (!el) return;
    if (!flightStartWarnings.length) {
        el.innerHTML = VIEW_CHOICE_HINT_HTML;
        return;
    }
    const warnings = flightStartWarnings
        .map(message => escapeHtml(message))
        .join('<br>');
    el.innerHTML = `${VIEW_CHOICE_HINT_HTML}<br><span style="color:#fbbf24">Preload warning: ${warnings}. Tiles may continue loading after takeoff.</span>`;
}

function showError(error) {
    reportUserError('Startup failed', error, { overlay: true, intervalMs: 0 });
}

function withTimeout(promise, timeoutMs, label) {
    if (!Number.isFinite(timeoutMs) || timeoutMs <= 0) return promise;
    let timeout = null;
    return Promise.race([
        promise,
        new Promise((_, reject) => {
            timeout = window.setTimeout(() => reject(new Error(`${label} timed out`)), timeoutMs);
        }),
    ]).finally(() => {
        if (timeout !== null) window.clearTimeout(timeout);
    });
}

async function waitForCesiumReady(timeoutMs = 15000) {
    if (window.Cesium) return;
    if (!window.googleTilesCesiumReady || typeof window.googleTilesCesiumReady.then !== 'function') return;
    let timeout = null;
    try {
        await Promise.race([
            window.googleTilesCesiumReady,
            new Promise((_, reject) => {
                timeout = window.setTimeout(() => reject(new Error('Timed out loading CesiumJS.')), timeoutMs);
            }),
        ]);
    } finally {
        if (timeout !== null) window.clearTimeout(timeout);
    }
}

function initSubsystems() {
    if (controller && drone && hud && osd && panoramaSensor) return;

    if (!window.pc) {
        throw new Error('PlayCanvas math library is not loaded. Check network access to cdn.jsdelivr.net.');
    }

    controller = new Controller();
    drone = new Drone();
    window.__drone = drone;  // debug: inspect mass/thrust from console
    hud = new HUD();
    osd = new OSD('osd-canvas');
    panoramaSensor = new PanoramaSensor();
    window.__getPerceptionFrame = () => panoramaSensor?.getLatestPerceptionFrame?.() || null;
    window.__captureMetricCalibration = (locationId, options = {}) => {
        if (!panoramaSensor || !world) throw new Error('flight world is not ready');
        return panoramaSensor.captureCalibrationSample(world, { ...options, locationId });
    };
    if (!flightLogger) flightLogger = new FlightLogger();

    setupDisplaySettingsListeners();
}

export async function startTilesMode() {
    if (startTilesModeInProgress) return;
    startTilesModeInProgress = true;
    try {
        initSubsystems();
        document.getElementById('drop-zone')?.classList.add('hidden');
        document.getElementById('loading-overlay')?.classList.add('visible');
        setProgress('Starting Google 3D Tiles world...');
        await waitForCesiumReady();

        if (screenHandler) {
            screenHandler.destroy();
            screenHandler = null;
        }
        if (flightGoalHandler) {
            flightGoalHandler.destroy();
            flightGoalHandler = null;
        }
        if (world) world.destroy();
        panoramaWarmupPromise = null;
        world = new CesiumWorld('cesium-container');
        await world.init(setProgress);
        collisionProvider = new TilesCollisionProvider(world);
        sceneLoaded = true;

        setupCesiumPlacementHandler();
        setupThirdPersonPointerControls();
        await enterPlacementMode(true);
        warmPanoramaViewerInBackground();
        document.getElementById('loading-overlay')?.classList.remove('visible');

        if (!loopStarted) {
            loopStarted = true;
            lastFrameTime = performance.now();
            requestAnimationFrame(gameLoop);
        }
    } catch (e) {
        showError(e);
    } finally {
        startTilesModeInProgress = false;
    }
}

function warmPanoramaViewerInBackground() {
    if (!world || !panoramaSensor || panoramaWarmupPromise) return panoramaWarmupPromise;
    if (typeof world.warmPanoramaCaptureViewer !== 'function') return null;

    const options = typeof panoramaSensor.getCaptureOptions === 'function'
        ? panoramaSensor.getCaptureOptions({ preload: true })
        : { faceSize: 256 };
    panoramaWarmupPromise = world.warmPanoramaCaptureViewer(options.faceSize)
        .catch((error) => {
            reportUserError('Panorama viewer warmup failed', error, {
                key: 'panorama-warmup',
                intervalMs: 10000,
            });
            panoramaWarmupPromise = null;
            return false;
        });
    return panoramaWarmupPromise;
}

async function preloadPanoramaBeforeFlight() {
    if (
        !world ||
        !drone ||
        !panoramaSensor ||
        typeof world.preloadPanoramaAtTransform !== 'function' ||
        typeof panoramaSensor.getCaptureOptions !== 'function'
    ) {
        return false;
    }

    const transform = drone.getPanoramaTransform
        ? drone.getPanoramaTransform()
        : (drone.getBodyTransform ? drone.getBodyTransform() : drone.getCameraTransform());
    if (!transform) return false;

    const options = {
        ...panoramaSensor.getCaptureOptions({ preload: true }),
        progressCb: (message) => setProgress(`Preloading 360 panorama sensor (${message})...`),
    };
    const started = performance.now();
    setProgress('Preloading 360 panorama sensor before flight...');

    try {
        const result = await withTimeout(
            (async () => {
                const warmup = warmPanoramaViewerInBackground();
                if (warmup) await warmup;
                return world.preloadPanoramaAtTransform(transform, options);
            })(),
            options.timeoutMs,
            '360 panorama preload'
        );
        const ready = panoramaSensor.primeFromCaptureResult(result, performance.now() - started, {
            capturedAt: started,
            transform,
            planningState: drone.getYopoPlanningState ? drone.getYopoPlanningState() : null,
        });
        if (!ready && (PANORAMA_PRELOAD_REQUIRED || FLIGHT_PRELOAD_STRICT)) {
            throw new Error('360 panorama preload did not produce a complete frame.');
        }
        return ready;
    } catch (error) {
        if (PANORAMA_PRELOAD_REQUIRED || FLIGHT_PRELOAD_STRICT) throw error;
        reportUserError('Panorama preload failed; live capture will retry in flight', error, {
            key: 'panorama-preload',
            intervalMs: 10000,
        });
        return false;
    }
}

async function preloadInitialFlightViewsBeforeControl() {
    if (
        !world ||
        !drone ||
        typeof world.settleCurrentCameraView !== 'function'
    ) {
        return;
    }

    const bodyTransform = drone.getBodyTransform ? drone.getBodyTransform() : drone.getCameraTransform();
    const cameraTransform = drone.getCameraTransform();
    const settleOptions = {
        dwellMs: 180,
        timeoutMs: FLIGHT_PRELOAD_VIEW_TIMEOUT_MS,
        quietMs: 500,
    };

    world.setFlightPerformanceMode(true);

    setProgress('Preloading first-person flight view...');
    const firstReady = await settleFlightView('first-person flight view', () => {
        world.setCameraFromDroneTransform(cameraTransform, getCameraHFov());
    }, settleOptions);
    if (!firstReady && FLIGHT_PRELOAD_STRICT) {
        throw new Error('First-person flight view tiles did not finish loading before control.');
    }

    setProgress('Preloading third-person flight view...');
    initThirdPersonCamera(bodyTransform);
    world.updateAircraftFromDroneTransform(bodyTransform);
    world.showAircraft(true);
    let thirdReady = false;
    try {
        thirdReady = await settleFlightView('third-person flight view', () => {
            world.setThirdPersonCamera(bodyTransform, thirdPersonCamera);
        }, settleOptions);
    } finally {
        world.showAircraft(false);
    }
    if (!thirdReady && FLIGHT_PRELOAD_STRICT) {
        throw new Error('Third-person flight view tiles did not finish loading before control.');
    }
}

async function settleFlightView(label, applyView, settleOptions) {
    let ready = false;
    for (let attempt = 1; attempt <= FLIGHT_PRELOAD_VIEW_ATTEMPTS; attempt++) {
        if (attempt > 1) {
            setProgress(`Waiting for ${label} tiles (${attempt}/${FLIGHT_PRELOAD_VIEW_ATTEMPTS})...`);
        }
        applyView();
        ready = await world.settleCurrentCameraView(settleOptions);
        if (ready) return true;
    }
    return ready;
}

function setupCesiumPlacementHandler() {
    if (!world || !world.viewer || screenHandler) return;
    const Cesium = world.Cesium;
    const canvas = world.viewer.scene.canvas;

    const rememberInitClick = () => {
        if (mode !== 'placement' || !placementKeysDown.has('KeyI')) return;
        placementInitClickUntil = performance.now() + 1500;
    };
    canvas.addEventListener('pointerdown', rememberInitClick, true);
    canvas.addEventListener('click', rememberInitClick, true);

    screenHandler = new Cesium.ScreenSpaceEventHandler(world.viewer.scene.canvas);
    screenHandler.setInputAction(async (movement) => {
        if (mode !== 'placement') return;
        const initClickActive =
            placementKeysDown.has('KeyI') ||
            performance.now() <= placementInitClickUntil;
        if (!initClickActive) return;

        const picked = await world.pickSpawn(movement.position, spawnAltitudeMeters);
        if (picked) {
            spawnPoint = picked;
            setSpawnAltitude(spawnAltitudeMeters);
            updateSpawnUI();
        }
    }, Cesium.ScreenSpaceEventType.LEFT_CLICK);
}

// 目标高度覆盖值：null = 沿用无人机当前高度（等价于参考实现 callback_set_goal
// 里"目标落在当前 fixed_height 平面"的行为）；用户按住 G 滚滚轮后变为显式高度，
// 等价于 callback_set_goal_3d 移动整个高度平面。按 C 取消航点时复位。
let goalAltitudeOverride = null;

function setupFlightGoalClickHandler() {
    if (!world || !world.viewer || flightGoalHandler) return;
    const Cesium = world.Cesium;
    const canvas = world.viewer.scene.canvas;

    let _goalGdown = false;
    const onMouseDown = (e) => {
        if (mode !== 'flight' || !drone) return;
        if (drone.flightMode !== 'so3') return;
        if (!_goalGdown) { console.log('[G+click] G not held, _goalGdown=' + _goalGdown); return; }
        console.log('[G+click] picking...');

        e.stopPropagation();
        e.preventDefault();

        const rect = canvas.getBoundingClientRect();
        const clickPos = new Cesium.Cartesian2(e.clientX - rect.left, e.clientY - rect.top);

        let cartesian = null;
        try {
            if (world.viewer.scene.pickPositionSupported) {
                const p = world.viewer.scene.pickPosition(clickPos);
                if (Cesium.defined(p)) cartesian = p;
            }
        } catch (_) {}
        if (!cartesian) {
            try {
                const p = world.viewer.camera.pickEllipsoid(clickPos, Cesium.Ellipsoid.WGS84);
                if (Cesium.defined(p)) cartesian = p;
            } catch (_) {}
        }
        if (!cartesian) { console.log('[goal] no ground — click on buildings/terrain, not sky'); return; }

        const local = world.cartesianToLocal(cartesian);
        const altY = goalAltitudeOverride != null ? goalAltitudeOverride : drone.y;
        console.log('[goal] SET:', local.x.toFixed(1), altY.toFixed(1), local.z.toFixed(1),
            goalAltitudeOverride != null ? '(高度已手动指定)' : '(沿用当前高度)');
        drone.setIdealGoal({ x: local.x, y: altY, z: local.z });
        world.showGoalMarker({ x: local.x, y: altY, z: local.z });
        // 目标高度即 YOPO 的高度平面，服务端会把轨迹末端拉到该平面
        panoramaSensor?.setYopoGoal({ x: local.x, y: altY, z: local.z });
        flightLogger?.start({ x: local.x, y: altY, z: local.z }, spawnAltitudeMeters);
    };

    // Track G key state
    window.addEventListener('keydown', (e) => {
        if (e.code === 'KeyG' && mode === 'flight' && drone && (drone.flightMode === 'so3')) {
            _goalGdown = true;
            console.log('[G key] DOWN');
        }
    }, true);
    window.addEventListener('keyup', (e) => {
        if (e.code === 'KeyG') { _goalGdown = false; console.log('[G key] UP'); }
    }, true);
    window.addEventListener('blur', () => { _goalGdown = false; });

    canvas.addEventListener('pointerdown', onMouseDown, true);

    // Click on radar minimap also sets goal
    const radarCanvas = document.getElementById('radar-canvas');
    if (radarCanvas) {
        radarCanvas.addEventListener('mousedown', (e) => {
            if (mode !== 'flight' || !drone || drone.flightMode !== 'so3') return;
            e.stopPropagation();
            const rw = radarCanvas.width, rh = radarCanvas.height;
            const range = 200; // must match drawRadar
            const scale = (rw/2 - 10) / range;
            const rx = (e.offsetX - rw/2) / scale;
            const rz = -(e.offsetY - rh/2) / scale; // invert Y
            const goalX = drone.x + rx;
            const goalZ = drone.z + rz;
            const altY = goalAltitudeOverride != null ? goalAltitudeOverride : drone.y;
            console.log('[radar goal] SET:', goalX.toFixed(1), altY.toFixed(1), goalZ.toFixed(1));
            drone.setIdealGoal({ x: goalX, y: altY, z: goalZ });
            world.showGoalMarker({ x: goalX, y: altY, z: goalZ });
            panoramaSensor?.setYopoGoal({ x: goalX, y: altY, z: goalZ });
            flightLogger?.start({ x: goalX, y: altY, z: goalZ }, spawnAltitudeMeters);
        });
    }

    // mouse wheel adjusts goal altitude when G is held
    const onWheel = (e) => {
        if (mode !== 'flight' || !drone || drone.flightMode !== 'so3') return;
        if (!placementKeysDown.has('KeyG')) return;
        e.stopPropagation();
        e.preventDefault();
        const step = placementKeysDown.has('ShiftLeft') || placementKeysDown.has('ShiftRight') ? 25 : 5;
        // 首次滚动从当前高度起步，之后才是纯粹的增量调整
        const base = goalAltitudeOverride != null ? goalAltitudeOverride : drone.y;
        goalAltitudeOverride = Math.max(SPAWN_ALTITUDE_MIN,
            Math.min(SPAWN_ALTITUDE_MAX, base - Math.sign(e.deltaY) * step));
        console.log('[goal] 目标高度:', goalAltitudeOverride.toFixed(1), 'm');
    };
    canvas.addEventListener('wheel', onWheel, { passive: false, capture: true });

    // Store for cleanup
    flightGoalHandler = { destroy: () => {
        canvas.removeEventListener('mousedown', onMouseDown, true);
        canvas.removeEventListener('wheel', onWheel, { capture: true });
    }};
}

async function enterPlacementMode(autoPick = false) {
    flightLogger?.stop(false);  // cancel any active recording

    if (!world) return;
    mode = 'placement';

    world.setFlightPerformanceMode(false);
    world.setNativeCameraControls(true);
    world.showAircraft(false);
    thirdPersonPointer.active = false;
    panoramaSensor?.setActive(false);
    hud?.hide();
    document.getElementById('game-logo')?.classList.remove('visible');
    document.getElementById('key-guide')?.classList.remove('visible');
    document.getElementById('placement-overlay')?.classList.add('visible');
    document.getElementById('view-choice-overlay')?.classList.remove('visible');
    applyDisplaySettings();

    if (autoPick || !spawnPoint) {
        await new Promise(resolve => setTimeout(resolve, 300));
        const canvas = world.viewer.scene.canvas;
        const center = new world.Cesium.Cartesian2(canvas.clientWidth * 0.5, canvas.clientHeight * 0.56);
        spawnPoint = await world.pickSpawn(center, spawnAltitudeMeters);
        if (!spawnPoint) {
            spawnPoint = { x: 0, y: spawnAltitudeMeters, z: 0 };
            world.updateSpawnMarker(spawnPoint);
        }
    } else {
        spawnPoint.y = spawnAltitudeMeters;
        world.updateSpawnMarker(spawnPoint);
    }
    syncSpawnAltitudeControls();
    updateSpawnUI();

}

async function confirmSpawnAndFly() {
    if (!world || !spawnPoint || spawnConfirmInProgress) return;
    spawnConfirmInProgress = true;
    flightStartWarnings = [];
    updateViewChoiceHint();

    try {
        const Cesium = world.Cesium;
        const spawnCarto = world.localToCartographic({ x: spawnPoint.x, y: 0, z: spawnPoint.z });
        const origin = new Cesium.Cartographic(
            spawnCarto.longitude,
            spawnCarto.latitude,
            0
        );
        const spawnAltitude = clampSpawnAltitude(spawnAltitudeMeters);
        world.setOrigin(origin);
        spawnPoint = { x: 0, y: spawnAltitude, z: 0 };

        world.setNativeCameraControls(false);
        world.hideSpawnMarker();
        document.getElementById('placement-overlay')?.classList.remove('visible');
        const coordsEl = document.getElementById('spawn-coords');
        if (coordsEl) coordsEl.style.display = 'none';

        drone.setSpawnPoint(spawnPoint.x, spawnPoint.y, spawnPoint.z);
        drone.reset();
        controller.armed = true;
        panoramaSensor?.reset();

        mode = 'loading';
        applyDisplaySettings();
        document.getElementById('loading-overlay')?.classList.add('visible');
        setProgress(`Preloading ${SPAWN_PRELOAD_RADIUS_METERS} m flight area before control...`);
        try {
            const preload = await world.preloadLocalArea(spawnPoint, {
                radius: SPAWN_PRELOAD_RADIUS_METERS,
                lift: 220,
                gridSpacing: 160,
                viewDistance: 240,
                maxTargets: 22,
                dwellMs: 160,
                perViewTimeoutMs: 2500,
                finalIdleTimeoutMs: 15000,
                verifyCoverage: true,
                coverageSpacing: 160,
                minCoverageRatio: FLIGHT_PRELOAD_MIN_COVERAGE,
                repairPasses: 1,
                repairTargets: 12,
                progressCb: setProgress,
            });
            const coverage = preload && preload.coverage ? preload.coverage.ratio : 0;
            const pct = Math.round(coverage * 100);
            if (preload && preload.coverage && coverage < FLIGHT_PRELOAD_MIN_COVERAGE) {
                reportUserError(
                    'Flight tile preload coverage low',
                    new Error(`coverage ${pct}% below required ${Math.round(FLIGHT_PRELOAD_MIN_COVERAGE * 100)}%`),
                    { key: 'flight-preload-coverage-low', intervalMs: 10000 }
                );
            }
            const coverageReady = preload && preload.coverage
                ? coverage >= FLIGHT_PRELOAD_MIN_COVERAGE
                : preload && preload.finalIdle === true;
            const preloadReady = preload &&
                coverageReady &&
                (!FLIGHT_PRELOAD_STRICT || preload.finalIdle === true);
            if (!preloadReady) {
                const coverageText = preload && preload.coverage ? `${pct}%` : 'unknown';
                const message = `flight tile preload incomplete: idle=${preload ? preload.finalIdle : false}, coverage=${coverageText}`;
                if (FLIGHT_PRELOAD_STRICT) {
                    throw new Error(message);
                }
                rememberFlightStartWarning(message);
            }
        } catch (e) {
            const msg = e && e.message ? e.message : String(e);
            if (FLIGHT_PRELOAD_STRICT) {
                reportUserError('Required flight tile preload failed', e, { intervalMs: 0 });
                throw new Error(`Required flight tile preload failed: ${msg}`);
            }
            reportUserError('Flight tile preload failed; continuing to view selection', e, {
                key: 'flight-tile-preload',
                intervalMs: 10000,
            });
            rememberFlightStartWarning(`flight tile preload skipped: ${shortStatusMessage(msg)}`);
        }

        // Run flight-view preload first to warm main tileset cache, then
        // panorama preload second — serialized because parallel streaming
        // from two independent Google tilesets competes for bandwidth and
        // leaves both caches cold.
        try {
            await preloadInitialFlightViewsBeforeControl();
        } catch (e) {
            if (FLIGHT_PRELOAD_STRICT) throw e;
            reportUserError('Initial flight view preload failed; continuing', e, {
                key: 'initial-flight-view-preload',
                intervalMs: 10000,
            });
        }

        try {
            await preloadPanoramaBeforeFlight();
        } catch (e) {
            if (PANORAMA_PRELOAD_REQUIRED || FLIGHT_PRELOAD_STRICT) throw e;
            reportUserError('Panorama preload failed; continuing', e, {
                key: 'panorama-preload-before-flight',
                intervalMs: 10000,
            });
        }

        mode = 'view-select';
        updateViewChoiceHint();
        document.getElementById('view-choice-overlay')?.classList.add('visible');
        applyDisplaySettings();
    } catch (e) {
        reportUserError('Spawn failed', e, { overlay: true, intervalMs: 0 });
        try {
            await enterPlacementMode(false);
        } catch (restoreError) {
            reportUserError('Failed to restore placement mode', restoreError, {
                key: 'restore-placement',
                intervalMs: 10000,
            });
        }
    } finally {
        document.getElementById('loading-overlay')?.classList.remove('visible');
        spawnConfirmInProgress = false;
    }
}

function startFlight(viewMode = 'first') {
    if (!world || !drone || !controller) return;
    cameraMode = normalizeViewMode(viewMode, 'first');

    mode = 'flight';
    drone.readSettings();
    lastSettingsReadTime = performance.now();
    world.setFlightPerformanceMode(true);
    document.getElementById('view-choice-overlay')?.classList.remove('visible');
    document.getElementById('game-logo')?.classList.add('visible');
    hud?.show();
    if (!panoramaSensor?.hasRgbFrame?.()) panoramaSensor?.reset();
    panoramaSensor?.setActive(true);

    // Wire YOPO: depth → YOPO plan → drone trajectory
    if (panoramaSensor) {
        panoramaSensor.onYopoResult = (endstate, trajTime, context = null) => {
            const session = panoramaSensor.getNavigationSession?.();
            if (
                context && session &&
                (context.goalId !== session.goalId || context.generation !== session.generation)
            ) {
                console.warn(
                    `[YOPO] stale apply rejected goalId=${context.goalId} frameId=${context.frameId} ` +
                    `generation=${context.generation}`
                );
                return;
            }
            if (!drone?.setYopoTrajectory(endstate, trajTime)) {
                console.warn(`[YOPO] invalid response rejected frameId=${context?.frameId ?? '-'}`);
            }
        };
        panoramaSensor.onDepthResult = (latencyMs) => { flightLogger?.recordDepth(latencyMs); };
        panoramaSensor.onYopoLatency = (latencyMs) => { flightLogger?.recordYopo(latencyMs); };
        panoramaSensor.onPerceptionMetrics = (metrics) => { flightLogger?.recordPerception(metrics); };
    }

    // Setup click-to-goal for ideal mode
    setupFlightGoalClickHandler();

    const transform = drone.getBodyTransform ? drone.getBodyTransform() : drone.getCameraTransform();
    if (cameraMode === 'third') {
        initThirdPersonCamera(transform);
        world.updateAircraftFromDroneTransform(transform);
        world.showAircraft(true);
    } else {
        world.showAircraft(false);
    }

    applyDisplaySettings();
}

function initThirdPersonCamera(transform) {
    const forward = world.getForwardLocal(transform);
    thirdPersonCamera.yaw = Math.atan2(-forward.x, -forward.z);
    thirdPersonCamera.pitch = 0.45;
    thirdPersonCamera.distance = 16;
    thirdPersonCamera.height = 1.2;
    thirdPersonCamera.lateral = 0;
}

function updateSpawnUI() {
    const coordsEl = document.getElementById('spawn-coords');
    if (coordsEl && world && spawnPoint) {
        coordsEl.style.display = 'block';
        coordsEl.textContent = `Spawn: ${world.describeSpawn(spawnPoint, spawnAltitudeMeters)}`;
    }
}

function moveSpawn(dt) {
    if (mode !== 'placement' || !spawnPoint || !world) return;
    const fast = placementKeysDown.has('ShiftLeft') || placementKeysDown.has('ShiftRight');
    const speed = (fast ? 25 : 6) * dt;
    const heading = world.viewer.camera.heading || 0;
    const fwd = { x: Math.sin(heading), z: Math.cos(heading) };
    const right = { x: Math.cos(heading), z: -Math.sin(heading) };

    if (placementKeysDown.has('KeyW')) {
        spawnPoint.x += fwd.x * speed;
        spawnPoint.z += fwd.z * speed;
    }
    if (placementKeysDown.has('KeyS')) {
        spawnPoint.x -= fwd.x * speed;
        spawnPoint.z -= fwd.z * speed;
    }
    if (placementKeysDown.has('KeyD')) {
        spawnPoint.x += right.x * speed;
        spawnPoint.z += right.z * speed;
    }
    if (placementKeysDown.has('KeyA')) {
        spawnPoint.x -= right.x * speed;
        spawnPoint.z -= right.z * speed;
    }
    spawnPoint.y = spawnAltitudeMeters;

    world.updateSpawnMarker(spawnPoint);
    updateSpawnUI();
}

function getCameraHFov(now = performance.now()) {
    if (now - lastHFovReadTime < 250) return cachedHFov;
    lastHFovReadTime = now;
    const el = document.getElementById('cam-hfov');
    const v = el ? parseFloat(el.value) : 120;
    cachedHFov = Number.isFinite(v) ? v : 120;
    return cachedHFov;
}

function drawRadar() {
    const panel = document.getElementById('radar-panel');
    const canvas = document.getElementById('radar-canvas');
    if (!panel || !canvas || !drone) return;
    if (mode !== 'flight') { panel.classList.remove('visible'); return; }
    panel.classList.add('visible');

    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height, cx = w/2, cy = h/2;
    const range = 200; // meters
    const scale = (w/2 - 10) / range;

    ctx.clearRect(0, 0, w, h);
    // Background
    ctx.fillStyle = 'rgba(3,7,18,0.95)';
    ctx.fillRect(0, 0, w, h);

    // Range rings
    for (let r = 50; r <= range; r += 50) {
        ctx.beginPath();
        ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(125,211,252,0.15)';
        ctx.stroke();
    }

    // North indicator
    ctx.strokeStyle = '#f44';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy); ctx.lineTo(cx, 8);
    ctx.stroke();

    // Drone (green dot, facing direction)
    const dx = drone.x * scale, dz = -drone.z * scale; // x=east→right, z=north→up
    const droneSx = cx + dx, droneSy = cy + dz;
    ctx.fillStyle = '#0f0';
    ctx.beginPath();
    ctx.arc(droneSx, droneSy, 5, 0, Math.PI * 2);
    ctx.fill();

    // Goal (red dot)
    const goal = drone._idealGoal;
    if (goal) {
        const gx = goal.x * scale, gz = -goal.z * scale;
        const goalSx = cx + gx, goalSy = cy + gz;
        ctx.fillStyle = '#f44';
        ctx.beginPath();
        ctx.arc(goalSx, goalSy, 4, 0, Math.PI * 2);
        ctx.fill();
        // Line from drone to goal
        ctx.strokeStyle = 'rgba(255,64,43,0.5)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(droneSx, droneSy);
        ctx.lineTo(goalSx, goalSy);
        ctx.stroke();
    }

    // Range text
    document.querySelector('.radar-range').textContent = `${range}m`;
    // Drone position text
    ctx.fillStyle = '#7dd3fc';
    ctx.font = '8px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${drone.x.toFixed(0)},${drone.z.toFixed(0)}`, droneSx, droneSy + 16);
}

function gameLoop(now) {
    const frameDt = Math.min(MAX_PHYSICS_FRAME_DT, Math.max(0.001, (now - lastFrameTime) / 1000));
    lastFrameTime = now;

    try {
        if (mode === 'placement') {
            moveSpawn(Math.min(PHYSICS_SUBSTEP_DT, frameDt));
            updateKeyGuide();
        } else if (mode === 'view-select') {
            updateKeyGuide();
        } else if (mode === 'flight') {
            updateFlight(frameDt);
        }
    } catch (e) {
        reportUserError('Frame update failed', e, {
            key: 'game-loop',
            intervalMs: 3000,
        });
    }
    requestAnimationFrame(gameLoop);
}

function updateFlight(dt) {
    if (!drone || !controller || !world) return;

    const now = performance.now();
    const input = controller.update();
    const modeSelect = document.getElementById('flight-mode-select');
    if (
        now - lastSettingsReadTime >= SETTINGS_READ_INTERVAL_MS ||
        (modeSelect && modeSelect.value !== drone.flightMode)
    ) {
        drone.readSettings();
        lastSettingsReadTime = now;
    }
    if (input.resetTriggered) {
        finishNavigationSession('reset', { cancelDrone: true, arrived: false });
        drone.reset();
        controller.armed = true;
    }

    if (drone.flightMode === 'drone') {
        if (Math.abs(input.cameraTiltKeyboard) > 0.05) {
            drone.adjustCameraTilt(input.cameraTiltKeyboard * 60 * dt);
        }
        if (input.cameraTiltAxisChanged) {
            drone.cameraTiltAngle = ((input.cameraTiltAxis + 1) / 2) * -90;
        }
    }

    let remainingDt = Math.max(0, Math.min(dt, PHYSICS_SUBSTEP_DT * MAX_PHYSICS_SUBSTEPS));
    let substeps = 0;
    while (remainingDt > 1e-6 && substeps < MAX_PHYSICS_SUBSTEPS) {
        const stepDt = Math.min(PHYSICS_SUBSTEP_DT, remainingDt);
        drone.update(stepDt, input, collisionProvider);
        remainingDt -= stepDt;
        substeps++;
    }

    // Camera mode only selects visualization; controller and physics stay shared.
    const cameraTransform = drone.getCameraTransform();
    const bodyTransform = drone.getBodyTransform ? drone.getBodyTransform() : cameraTransform;
    if (cameraMode === 'third') {
        world.updateAircraftFromDroneTransform(bodyTransform);
        world.showAircraft(true);
        world.setThirdPersonCamera(bodyTransform, thirdPersonCamera);
    } else {
        world.showAircraft(false);
        world.setCameraFromDroneTransform(cameraTransform, getCameraHFov(now));
    }

    // Level panorama horizon if checkbox enabled (default on, like YOPO use_leveled_depth)
    const panoLevelEl = document.getElementById('pano-level-toggle');
    const useLeveled = panoLevelEl ? panoLevelEl.checked : true;
    const panoramaTransform = useLeveled && drone.getLeveledPanoramaTransform
        ? drone.getLeveledPanoramaTransform()
        : (drone.getPanoramaTransform ? drone.getPanoramaTransform() : bodyTransform);
    if (!updateFlight._loggedPanoMode) {
        console.log(`[pano] leveled=${useLeveled} method=${useLeveled && drone.getLeveledPanoramaTransform ? 'getLeveledPanoramaTransform' : 'getPanoramaTransform'}`);
        updateFlight._loggedPanoMode = true;
    }
    // Capture starts only after this immutable planning snapshot is installed.
    // PanoramaSensor binds it to that capture's transform/RGB/frame ID, so a
    // slow six-face render cannot be paired with a newer moving pose.
    if (panoramaSensor && drone) {
        panoramaSensor.setYopoPose(drone.getYopoPlanningState
            ? drone.getYopoPlanningState()
            : { x: drone.x, y: drone.y, z: drone.z, vx: drone.vx, vy: drone.vy, vz: drone.vz },
        drone.getFixedYaw ? drone.getFixedYaw() : drone.yaw);
    }
    panoramaSensor?.update(world, panoramaTransform, now);

    // Flight log recording
    if (flightLogger?.recording) {
        const ref = drone._yopoPolyX ? drone._getYopoReference
            ? drone._getYopoReference(drone._yopoTrackerTime || 0)
            : { x: drone.x, y: drone.y, z: drone.z }
            : (drone._idealGoal ? drone._idealGoal : { x: drone.x, y: drone.y, z: drone.z });
        flightLogger.record(drone, ref?.x ?? drone.x, ref?.y ?? drone.y, ref?.z ?? drone.z);
    }
    const navigationTransition = drone.consumeNavigationTransition?.();
    if (navigationTransition?.state === 'arrived') {
        finishNavigationSession('arrived', { cancelDrone: false, arrived: true });
    } else if (navigationTransition?.reason === 'mode-exit') {
        finishNavigationSession('mode-exit', { cancelDrone: false, arrived: false });
    }

    // Radar minimap
    drawRadar();
    hud?.update(drone, controller, null);
    applyDisplaySettings();
    osd?.update(drone, controller);
    updateKeyGuide();
}

function applyDisplaySettings() {
    const cleanToggle = document.getElementById('clean-mode-toggle');
    const cleanMode = cleanToggle ? cleanToggle.checked : false;
    const osdToggle = document.getElementById('osd-toggle');
    const osdEnabled = !cleanMode && (osdToggle ? osdToggle.checked : true) && mode === 'flight' && cameraMode === 'first';
    const panoToggle = document.getElementById('panorama-toggle');
    const panoEnabled = panoToggle ? panoToggle.checked : true;
    const state = `${mode}|${cameraMode}|${cleanMode ? 1 : 0}|${osdEnabled ? 1 : 0}|${panoEnabled ? 1 : 0}`;
    if (state === lastDisplaySettingsState) return;
    lastDisplaySettingsState = state;

    if (osd) {
        osd.setEnabled(osdEnabled);
    }
    panoramaSensor?.setActive(mode === 'flight');

    const logo = document.getElementById('game-logo');
    const keyGuide = document.getElementById('key-guide');
    const hudEl = document.getElementById('hud');
    if (cleanMode) {
        logo?.classList.remove('visible');
        keyGuide?.classList.remove('visible');
        if (hudEl && mode === 'flight') hudEl.classList.add('hidden');
    } else if (mode === 'flight') {
        logo?.classList.add('visible');
        hudEl?.classList.remove('hidden');
    } else if (mode === 'placement' || mode === 'view-select') {
        logo?.classList.remove('visible');
        hudEl?.classList.add('hidden');
    }
}

function setupDisplaySettingsListeners() {
    for (const id of ['clean-mode-toggle', 'osd-toggle', 'panorama-toggle']) {
        const el = document.getElementById(id);
        if (!el || el._tilesDisplayBound) continue;
        el._tilesDisplayBound = true;
        el.addEventListener('change', applyDisplaySettings);
    }
    // Drone model scale slider
    const modelScaleSlider = document.getElementById('drone-model-scale');
    const modelScaleNum = document.getElementById('drone-model-scale-num');
    if (modelScaleSlider && modelScaleNum && !modelScaleSlider._bound) {
        modelScaleSlider._bound = true;
        const sync = () => {
            const v = parseFloat(modelScaleSlider.value);
            modelScaleNum.value = v;
            // 属性名是 aircraftModelEntity（无下划线）。此前误写成 _aircraftModelEntity，
            // 取到 undefined，滑块一直是空操作。
            if (world && world.aircraftModelEntity && world.aircraftModelEntity.model) {
                world.aircraftModelEntity.model.scale = v;
            }
        };
        modelScaleSlider.addEventListener('input', sync);
        modelScaleNum.addEventListener('change', () => {
            modelScaleSlider.value = modelScaleNum.value;
            sync();
        });
    }
    // Goal font size slider
    const goalFontSlider = document.getElementById('goal-font-size');
    const goalFontNum = document.getElementById('goal-font-size-num');
    if (goalFontSlider && goalFontNum && !goalFontSlider._bound) {
        goalFontSlider._bound = true;
        const sync = () => {
            const v = parseInt(goalFontSlider.value);
            goalFontNum.value = v;
            // stored for showGoalMarker
            window._goalFontSize = v;
        };
        goalFontSlider.addEventListener('input', sync);
        goalFontNum.addEventListener('change', () => {
            goalFontSlider.value = goalFontNum.value;
            sync();
        });
        sync();
    }
}

function setupSpawnAltitudeControls() {
    const slider = document.getElementById('spawn-altitude-range');
    const input = document.getElementById('spawn-altitude-input');
    const panel = document.getElementById('spawn-altitude-panel');
    if (!slider || !input || !panel || panel._spawnAltitudeBound) return;
    panel._spawnAltitudeBound = true;

    const commit = (value) => setSpawnAltitude(value);
    slider.addEventListener('input', () => commit(slider.value));
    input.addEventListener('input', () => {
        if (input.value !== '') commit(input.value);
    });
    input.addEventListener('change', () => commit(input.value));

    panel.addEventListener('wheel', (e) => {
        if (mode !== 'placement') return;
        e.preventDefault();
        e.stopPropagation();
        const step = e.shiftKey ? 25 : 5;
        const direction = e.deltaY < 0 ? 1 : -1;
        commit(spawnAltitudeMeters + direction * step);
    }, { passive: false });

    for (const el of [slider, input]) {
        el.addEventListener('pointerdown', (e) => e.stopPropagation());
        el.addEventListener('keydown', (e) => e.stopPropagation());
    }
    syncSpawnAltitudeControls();
}

function updateKeyGuide() {
    const el = document.getElementById('key-guide');
    if (!el) return;
    const cleanMode = document.getElementById('clean-mode-toggle')?.checked ? 1 : 0;
    const guideState = `${mode}|${cameraMode}|${drone ? drone.flightMode : ''}|${cleanMode}`;
    if (guideState === lastKeyGuideState) return;
    lastKeyGuideState = guideState;

    if (mode !== 'flight') {
        el.classList.remove('visible');
        return;
    }
    const fm = drone ? drone.flightMode : '';
    const isFPV = fm === 'fpv';
    const isStab = fm === 'stabilized';
    const isSO3 = fm === 'so3';
    const title = isFPV ? 'FLIGHT CONTROLS - FPV' : isStab ? 'FLIGHT CONTROLS - STABILIZED' : isSO3 ? 'FLIGHT CONTROLS - SO3' : 'FLIGHT CONTROLS - EASY';

    const rows = isFPV ? [
        '<kbd>↑ ↓</kbd>  Pitch Forward / Back',
        '<kbd>← →</kbd>  Roll Left / Right',
        '<kbd>W S</kbd>  Motor Thrust',
        '<kbd>A D</kbd>  Yaw Left / Right',
        '<span style="color:#8cff8c">Nose down builds forward speed</span>',
    ] : isStab ? [
        '<kbd>↑ ↓</kbd>  Tilt Forward / Back',
        '<kbd>← →</kbd>  Tilt Left / Right (auto-level)',
        '<kbd>W S</kbd>  Throttle Up / Down',
        '<kbd>A D</kbd>  Yaw Left / Right',
    ] : isSO3 ? [
        '<kbd>↑ ↓</kbd>  Forward / Back',
        '<kbd>← →</kbd>  Strafe Left / Right',
        '<kbd>W S</kbd>  Climb / Descend',
        '<kbd>A D</kbd>  Yaw Left / Right',
        '<kbd>G+click</kbd> Set waypoint (YOPO auto-plan)',
        '<kbd>C</kbd>    Cancel waypoint',
    ] : [
        '<kbd>↑ ↓</kbd>  Forward / Back',
        '<kbd>← →</kbd>  Strafe Left / Right',
        '<kbd>W S</kbd>  Climb / Descend',
        '<kbd>A D</kbd>  Yaw Left / Right',
        '<kbd>Q E</kbd>  Camera Tilt',
    ];
    rows.push(
        '<kbd>Space</kbd> Arm / Disarm',
        '<kbd>Shift</kbd> Boost',
        '<kbd>R</kbd>    Reset',
        `<kbd>V</kbd>    View (${cameraMode === 'third' ? 'Third' : 'First'})`,
        '<kbd>M</kbd>    Flight Mode (FPV/Easy)',
        '<kbd>P</kbd>    Placement mode',
        `<kbd>Tab</kbd>  ${isFPV ? 'Settings' : 'Settings / Easy Max Speed'}`,
    );
    if (cameraMode === 'third') {
        rows.push(
            '<kbd>L/R Mouse</kbd> Orbit observer',
            '<kbd>Wheel</kbd> Zoom',
            '<kbd>Middle</kbd> Pan / height',
        );
    }
    const html = `<div class="guide-title">${title}</div>\n${rows.join('\n')}`;
    if (el.innerHTML !== html) el.innerHTML = html;
    if (!cleanMode) {
        el.classList.add('visible');
    }
}

function clampNumber(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function isThirdPersonObserverActive() {
    return mode === 'flight' &&
        cameraMode === 'third' &&
        !(controller && controller.isSettingsOpen && controller.isSettingsOpen());
}

function isTextEntryTarget(target) {
    if (!target || !target.closest) return false;
    return !!target.closest('input, textarea, select, [contenteditable="true"], [contenteditable=""]');
}

function isPointerOverCesiumCanvas() {
    const canvas = world?.viewer?.scene?.canvas;
    return !!(canvas && typeof canvas.matches === 'function' && canvas.matches(':hover'));
}

function setupThirdPersonPointerControls() {
    if (!world || !world.viewer) return;
    const canvas = world.viewer.scene.canvas;
    if (!canvas || canvas._flightThirdPersonBound) return;
    canvas._flightThirdPersonBound = true;

    canvas.addEventListener('contextmenu', (e) => {
        if (isThirdPersonObserverActive()) e.preventDefault();
    });

    canvas.addEventListener('pointerdown', (e) => {
        if (!isThirdPersonObserverActive()) return;
        if (![0, 1, 2].includes(e.button)) return;
        e.preventDefault();
        thirdPersonPointer.active = true;
        thirdPersonPointer.button = e.button;
        thirdPersonPointer.x = e.clientX;
        thirdPersonPointer.y = e.clientY;
        try {
            canvas.setPointerCapture(e.pointerId);
        } catch (error) {
            reportUserError('Pointer capture failed', error, {
                key: 'pointer-capture',
                intervalMs: 10000,
            });
        }
    });

    canvas.addEventListener('pointermove', (e) => {
        if (!thirdPersonPointer.active || !isThirdPersonObserverActive()) return;
        e.preventDefault();
        const dx = e.clientX - thirdPersonPointer.x;
        const dy = e.clientY - thirdPersonPointer.y;
        thirdPersonPointer.x = e.clientX;
        thirdPersonPointer.y = e.clientY;

        if (thirdPersonPointer.button === 1) {
            thirdPersonCamera.lateral = clampNumber(thirdPersonCamera.lateral + dx * 0.025, -25, 25);
            thirdPersonCamera.height = clampNumber(thirdPersonCamera.height - dy * 0.025, -8, 20);
        } else {
            thirdPersonCamera.yaw -= dx * 0.005;
            thirdPersonCamera.pitch = clampNumber(thirdPersonCamera.pitch - dy * 0.004, -0.75, 1.05);
        }
    });

    const stopPointer = () => {
        thirdPersonPointer.active = false;
        thirdPersonPointer.button = -1;
    };
    canvas.addEventListener('pointerup', stopPointer);
    canvas.addEventListener('pointercancel', stopPointer);
    canvas.addEventListener('pointerleave', stopPointer);

    canvas.addEventListener('wheel', (e) => {
        if (!isThirdPersonObserverActive()) return;
        e.preventDefault();
        thirdPersonCamera.distance = clampNumber(
            thirdPersonCamera.distance * Math.exp(e.deltaY * 0.001),
            2.0,
            120.0
        );
    }, { passive: false });
}

function finishNavigationSession(reason, { cancelDrone = false, arrived = false } = {}) {
    flightLogger?.stop(arrived);
    if (cancelDrone) drone?.cancelWaypoint();
    panoramaSensor?.resetYopoGoal(reason);
    goalAltitudeOverride = null;   // 复位高度覆盖，下次设点重新沿用当前高度
    if (world && typeof world.clearGoalMarker === 'function') world.clearGoalMarker();
    console.log(`[navigation] ${reason}`);
}

function cancelWaypoint() {
    if (!drone) return;
    finishNavigationSession('cancelled', { cancelDrone: true, arrived: false });
}

function setupKeyboard() {
    window.addEventListener('keydown', (e) => {
        if (controller && controller.isSettingsOpen && controller.isSettingsOpen()) return;
        if (isTextEntryTarget(e.target)) {
            // G key for goal setting works even when focus is on a text input
            if (e.code === 'KeyG' && mode === 'flight' && drone && (drone.flightMode === 'so3')) {
                placementKeysDown.add(e.code);
                e.preventDefault();
                return;
            }
            if (mode === 'placement' && e.code === 'KeyI' && isPointerOverCesiumCanvas()) {
                placementKeysDown.add(e.code);
                e.preventDefault();
            }
            return;
        }

        if (mode === 'placement') {
            placementKeysDown.add(e.code);
            if (['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyI', 'KeyO'].includes(e.code)) {
                e.preventDefault();
            }
            if (e.code === 'KeyO' && spawnPoint) {
                confirmSpawnAndFly();
            }
        } else if (mode === 'view-select') {
            if (['Digit1', 'Numpad1', 'KeyO'].includes(e.code)) {
                e.preventDefault();
                startFlight('first');
            } else if (['Digit2', 'Numpad2'].includes(e.code)) {
                e.preventDefault();
                startFlight('third');
            } else if (e.code === 'Escape' || e.code === 'KeyP') {
                e.preventDefault();
                enterPlacementMode(false);
            }
        } else if (mode === 'flight') {
            // G key: hold to click goal in ideal mode
            if (e.code === 'KeyG' && drone && (drone.flightMode === 'so3')) {
                placementKeysDown.add(e.code);
                e.preventDefault();
                return;
            }
            if (e.code === 'KeyC') {
                e.preventDefault();
                cancelWaypoint();
                return;
            }
            if (e.code === 'KeyV') {
                e.preventDefault();
                cameraMode = cameraMode === 'third' ? 'first' : 'third';
                if (cameraMode === 'third') initThirdPersonCamera(drone.getBodyTransform ? drone.getBodyTransform() : drone.getCameraTransform());
                applyDisplaySettings();
                return;
            }
            if (e.code === 'KeyP') {
                e.preventDefault();
                enterPlacementMode(false);
            }
            if (e.code === 'Escape' && sceneLoaded) {
                e.preventDefault();
                if (window.confirm('Return to placement mode?')) enterPlacementMode(false);
            }
        }
    }, true);

    window.addEventListener('keyup', (e) => {
        placementKeysDown.delete(e.code);
    }, true);
    window.addEventListener('blur', () => placementKeysDown.clear());
}

function setupStartUI() {
    const startBtn = document.getElementById('file-picker-btn');
    const dropZone = document.getElementById('drop-zone');
    if (startBtn && !startBtn._flightStartBound) {
        startBtn._flightStartBound = true;
        startBtn.textContent = 'Start Google 3D Tiles Flight';
        startBtn.addEventListener('click', () => startTilesMode());
    }
    if (dropZone) {
        dropZone.addEventListener('dragover', (e) => {
            e.preventDefault();
            dropZone.classList.add('dragover');
        });
        dropZone.addEventListener('dragleave', () => dropZone.classList.remove('dragover'));
        dropZone.addEventListener('drop', (e) => {
            e.preventDefault();
            dropZone.classList.remove('dragover');
            startTilesMode();
        });
    }

    for (const btn of document.querySelectorAll('[data-view-choice]')) {
        if (btn._flightViewChoiceBound) continue;
        btn._flightViewChoiceBound = true;
        btn.addEventListener('click', () => startFlight(btn.getAttribute('data-view-choice')));
    }
}

function initializeAppShell() {
    setupStartUI();
    setupKeyboard();
    setupSpawnAltitudeControls();
    setProgress('');
    window.googleTilesFlightStart = startTilesMode;
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeAppShell, { once: true });
} else {
    initializeAppShell();
}
