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
 * CesiumWorld
 *
 * Wraps Cesium/Google Photorealistic 3D Tiles behind the local metre-based
 * coordinate convention already used by the drone physics:
 *
 *   local x = east, local y = up, local z = north
 *
 * Cesium itself renders in ECEF. The conversion is anchored at a user-selected
 * origin so the existing controller, physics and HUD do not need to know about
 * longitude/latitude.
 */

import { formatError, reportUserError } from './error-report.js';
import { erpDirectionToComponent, sampleAnchorDirections } from './erp-geometry.js';
import { demoPerformance } from './demo-performance.js?v=20260814-route-corridor-r15';

const DEFAULT_ION_TOKEN = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJlMTg2MGFhOS02YTdhLTQ1NWMtYjkzMi05YjQ2ODRlZjI5YTgiLCJpZCI6MjUxNzM1LCJpYXQiOjE3MzAyODI0ODN9.prWAxx4RB8teelutQQbVqdxhgRZpZ4zjw8wzM-8k1Ug';
const DEFAULT_ASSET_ID = 2275207;
const DEFAULT_VIEW = {
    longitude: 114.1690321,
    latitude: 22.3246282,
    height: 1800,
};
const CESIUM_DRONE_MODEL_URI = 'asset/models/CesiumDrone.glb';
// CesiumDrone.glb 的原始包围盒为 3.964 × 1.120 × 4.668 单位（水平最大跨度 4.668）。
// 目标是让渲染尺寸匹配物理机体：半径约 0.4 m，即跨度 0.8 m。
//   scale = 0.8 / 4.668 ≈ 0.171
// 旧默认值 1.35 会渲染出 6.3 m 跨度（等效半径 3.15 m），比物理碰撞半径
// (collisionRadius = 0.6 m) 大一个数量级，视觉上完全失真。
const CESIUM_DRONE_MODEL_SCALE = clampNumber(
    urlNumber('droneScale', 0.171),
    0.01, 10.0, 0.171
);
const HEIGHT_CACHE_TTL_MS = 140;
const HEIGHT_CACHE_LIMIT = 256;
// These texture names are legacy cubemap labels.  In this simulator's flight
// convention local +X is body-left (yaw=0 faces -Z and body-right is -X), even
// though the +X texture has historically been named `right`.  ERP azimuth must
// therefore be derived from the explicit NWU contract, never from these names.
const PANORAMA_FACE_DEFS = [
    { name: 'front', dir: { x: 0, y: 0, z: -1 }, up: { x: 0, y: 1, z: 0 } },
    { name: 'right', dir: { x: 1, y: 0, z: 0 }, up: { x: 0, y: 1, z: 0 } },
    { name: 'back', dir: { x: 0, y: 0, z: 1 }, up: { x: 0, y: 1, z: 0 } },
    { name: 'left', dir: { x: -1, y: 0, z: 0 }, up: { x: 0, y: 1, z: 0 } },
    { name: 'up', dir: { x: 0, y: 1, z: 0 }, up: { x: 0, y: 0, z: 1 } },
    { name: 'down', dir: { x: 0, y: -1, z: 0 }, up: { x: 0, y: 0, z: -1 } },
];

function urlNumber(name, fallback) {
    const v = new URLSearchParams(window.location.search).get(name);
    if (v == null || v === '') return fallback;
    const n = Number(v);
    return Number.isFinite(n) ? n : fallback;
}

function urlString(name, fallback) {
    const v = new URLSearchParams(window.location.search).get(name);
    return v == null || v === '' ? fallback : v;
}

function requireCesium() {
    if (!window.Cesium) {
        throw new Error('CesiumJS is not loaded. Run via the Docker image or provide /ThirdParty/Cesium/Cesium.js.');
    }
    return window.Cesium;
}

function rotateVectorByQuat(q, v) {
    // q * v * q^-1; q is expected to rotate body-local vectors into the
    // app-local world frame used by Drone.
    const x = v.x, y = v.y, z = v.z;
    const qx = q.x, qy = q.y, qz = q.z, qw = q.w;

    const ix =  qw * x + qy * z - qz * y;
    const iy =  qw * y + qz * x - qx * z;
    const iz =  qw * z + qx * y - qy * x;
    const iw = -qx * x - qy * y - qz * z;

    return {
        x: ix * qw + iw * -qx + iy * -qz - iz * -qy,
        y: iy * qw + iw * -qy + iz * -qx - ix * -qz,
        z: iz * qw + iw * -qz + ix * -qy - iy * -qx,
    };
}

function normalize3(v) {
    const len = Math.hypot(v.x, v.y, v.z);
    if (len < 1e-9) return { x: 0, y: 0, z: 0 };
    return { x: v.x / len, y: v.y / len, z: v.z / len };
}

function negate3(v) {
    return { x: -v.x, y: -v.y, z: -v.z };
}

function smoothstep(edge0, edge1, x) {
    const t = Math.max(0, Math.min(1, (x - edge0) / Math.max(1e-9, edge1 - edge0)));
    return t * t * (3 - 2 * t);
}

function compilePanoramaShader(gl, type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        const message = gl.getShaderInfoLog(shader) || 'unknown shader compile error';
        gl.deleteShader(shader);
        throw new Error(message);
    }
    return shader;
}

function createPanoramaProgram(gl, vertexSource, fragmentSource) {
    const vertex = compilePanoramaShader(gl, gl.VERTEX_SHADER, vertexSource);
    const fragment = compilePanoramaShader(gl, gl.FRAGMENT_SHADER, fragmentSource);
    const program = gl.createProgram();
    gl.attachShader(program, vertex);
    gl.attachShader(program, fragment);
    gl.linkProgram(program);
    gl.deleteShader(vertex);
    gl.deleteShader(fragment);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
        const message = gl.getProgramInfoLog(program) || 'unknown shader link error';
        gl.deleteProgram(program);
        throw new Error(message);
    }
    return program;
}

class PanoramaEquirectProjector {
    constructor() {
        this.canvas = document.createElement('canvas');
        const gl = this.canvas.getContext('webgl', {
            alpha: false,
            antialias: false,
            depth: false,
            stencil: false,
            preserveDrawingBuffer: true,
            powerPreference: 'high-performance',
        });
        if (!gl) throw new Error('WebGL is unavailable for panorama projection.');
        this.gl = gl;
        this.readyFaces = new Set();
        this.faceNames = ['front', 'right', 'back', 'left', 'up', 'down'];
        this.textures = new Map();
        this.textureSizes = new Map();

        this.program = createPanoramaProgram(gl, `
            attribute vec2 a_position;
            varying vec2 v_uv;
            void main() {
                v_uv = a_position * 0.5 + 0.5;
                gl_Position = vec4(a_position, 0.0, 1.0);
            }
        `, `
            precision mediump float;
            varying vec2 v_uv;
            uniform float u_vertical_fov;
            uniform float u_tan_half_face_fov;
            uniform float u_top_pole_guard;
            uniform float u_bottom_pole_guard;
            uniform sampler2D u_front;
            uniform sampler2D u_right;
            uniform sampler2D u_back;
            uniform sampler2D u_left;
            uniform sampler2D u_up;
            uniform sampler2D u_down;

            const float PI = 3.141592653589793;
            const float TWO_PI = 6.283185307179586;

            vec2 faceCoord(vec3 dir, vec3 faceDir, vec3 faceRight, vec3 faceUp) {
                float denom = max(dot(dir, faceDir), 0.000001);
                float u = dot(dir, faceRight) / (denom * u_tan_half_face_fov);
                float v = dot(dir, faceUp) / (denom * u_tan_half_face_fov);
                return vec2(u, v);
            }

            vec2 coordUv(vec2 coord) {
                return clamp(vec2(coord.x * 0.5 + 0.5, 0.5 - coord.y * 0.5), 0.001, 0.999);
            }

            vec2 faceUv(vec3 dir, vec3 faceDir, vec3 faceRight, vec3 faceUp) {
                return coordUv(faceCoord(dir, faceDir, faceRight, faceUp));
            }

            vec4 sampleXFace(vec3 dir) {
                if (dir.x >= 0.0) {
                    return texture2D(u_right, faceUv(dir, vec3(1.0, 0.0, 0.0), vec3(0.0, 0.0, -1.0), vec3(0.0, 1.0, 0.0)));
                }
                return texture2D(u_left, faceUv(dir, vec3(-1.0, 0.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 1.0, 0.0)));
            }

            vec4 sampleYFace(vec3 dir) {
                if (dir.y >= 0.0) {
                    return texture2D(u_up, faceUv(dir, vec3(0.0, 1.0, 0.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 0.0, 1.0)));
                }
                return texture2D(u_down, faceUv(dir, vec3(0.0, -1.0, 0.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 0.0, -1.0)));
            }

            vec4 sampleZFace(vec3 dir) {
                if (dir.z >= 0.0) {
                    return texture2D(u_back, faceUv(dir, vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0)));
                }
                return texture2D(u_front, faceUv(dir, vec3(0.0, 0.0, -1.0), vec3(-1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0)));
            }

            vec4 sampleSideRing(vec3 dir) {
                vec3 horizontal = normalize(vec3(dir.x, 0.0, dir.z));
                vec3 a = abs(horizontal);
                if (a.x >= a.z) {
                    return sampleXFace(dir);
                }
                return sampleZFace(dir);
            }

            vec4 sampleHybridRing(vec3 dir) {
                vec4 side = sampleSideRing(dir);
                vec4 cap = sampleYFace(dir);
                float capBlend = smoothstep(0.78, 0.90, abs(dir.y));
                return mix(side, cap, capBlend);
            }

            vec3 directionFromPitchYaw(float pitch, float yaw) {
                float cosPitch = cos(pitch);
                float forward = cosPitch * cos(yaw);
                // YOPO training ERP uses body NWU (+x forward, +y left,
                // +z up).  This renderer's component +X is that body-left
                // axis, so yaw=+90 deg maps directly to component +X.
                float left = cosPitch * sin(yaw);
                return normalize(vec3(left, sin(pitch), -forward));
            }

            void main() {
                float halfFov = u_vertical_fov * 0.5;
                float pitch = (v_uv.y - 0.5) * u_vertical_fov;
                float yaw = PI - v_uv.x * TWO_PI;
                float topGuardStart = halfFov - u_top_pole_guard;
                float bottomGuardStart = -halfFov + u_bottom_pole_guard;
                float guardedPitch = clamp(pitch, bottomGuardStart, topGuardStart);
                vec4 color = sampleHybridRing(directionFromPitchYaw(guardedPitch, yaw));

                if (u_top_pole_guard > 0.0001 && pitch > topGuardStart) {
                    float t = smoothstep(topGuardStart, halfFov, pitch);
                    color = mix(color, sampleYFace(vec3(0.0, 1.0, 0.0)), t);
                }
                if (u_bottom_pole_guard > 0.0001 && pitch < bottomGuardStart) {
                    float t = smoothstep(-halfFov, bottomGuardStart, pitch);
                    color = mix(sampleYFace(vec3(0.0, -1.0, 0.0)), color, t);
                }

                gl_FragColor = color;
            }
        `);

        this.positionBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
        gl.bufferData(
            gl.ARRAY_BUFFER,
            new Float32Array([-1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1]),
            gl.STATIC_DRAW
        );

        gl.useProgram(this.program);
        this.locations = {
            position: gl.getAttribLocation(this.program, 'a_position'),
            verticalFov: gl.getUniformLocation(this.program, 'u_vertical_fov'),
            tanHalfFaceFov: gl.getUniformLocation(this.program, 'u_tan_half_face_fov'),
            topPoleGuard: gl.getUniformLocation(this.program, 'u_top_pole_guard'),
            bottomPoleGuard: gl.getUniformLocation(this.program, 'u_bottom_pole_guard'),
        };
        this.faceNames.forEach((name, i) => {
            const texture = gl.createTexture();
            gl.activeTexture(gl.TEXTURE0 + i);
            gl.bindTexture(gl.TEXTURE_2D, texture);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
            gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
            gl.uniform1i(gl.getUniformLocation(this.program, `u_${name}`), i);
            this.textures.set(name, texture);
        });
    }

    updateFace(name, sourceCanvas) {
        const gl = this.gl;
        const texture = this.textures.get(name);
        if (!texture || !sourceCanvas || !sourceCanvas.width || !sourceCanvas.height) return;
        gl.bindTexture(gl.TEXTURE_2D, texture);
        gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
        const allocated = this.textureSizes.get(name);
        if (allocated?.width === sourceCanvas.width && allocated?.height === sourceCanvas.height) {
            gl.texSubImage2D(
                gl.TEXTURE_2D, 0, 0, 0, gl.RGBA, gl.UNSIGNED_BYTE, sourceCanvas
            );
        } else {
            gl.texImage2D(
                gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, sourceCanvas
            );
            this.textureSizes.set(name, {
                width: sourceCanvas.width,
                height: sourceCanvas.height,
            });
        }
        this.readyFaces.add(name);
    }

    render(width, height, verticalFovDeg, faceFovDeg = 130, topPoleGuardDeg = 0, bottomPoleGuardDeg = 0) {
        if (!this.faceNames.every(name => this.readyFaces.has(name))) return null;
        const gl = this.gl;
        if (this.canvas.width !== width) this.canvas.width = width;
        if (this.canvas.height !== height) this.canvas.height = height;

        gl.viewport(0, 0, width, height);
        gl.disable(gl.DEPTH_TEST);
        gl.disable(gl.BLEND);
        gl.useProgram(this.program);
        gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
        gl.enableVertexAttribArray(this.locations.position);
        gl.vertexAttribPointer(this.locations.position, 2, gl.FLOAT, false, 0, 0);
        this.faceNames.forEach((name, i) => {
            gl.activeTexture(gl.TEXTURE0 + i);
            gl.bindTexture(gl.TEXTURE_2D, this.textures.get(name));
        });
        const verticalFov = Math.max(1, Math.min(180, verticalFovDeg || 180)) * Math.PI / 180;
        const faceFov = Math.max(45, Math.min(170, faceFovDeg || 90)) * Math.PI / 180;
        const maxGuard = Math.max(0, verticalFov * 0.5 - (1 * Math.PI / 180));
        const topPoleGuard = Math.min(maxGuard, Math.max(0, Number(topPoleGuardDeg) || 0) * Math.PI / 180);
        const bottomPoleGuard = Math.min(maxGuard, Math.max(0, Number(bottomPoleGuardDeg) || 0) * Math.PI / 180);
        gl.uniform1f(this.locations.verticalFov, verticalFov);
        gl.uniform1f(this.locations.tanHalfFaceFov, Math.tan(faceFov * 0.5));
        gl.uniform1f(this.locations.topPoleGuard, topPoleGuard);
        gl.uniform1f(this.locations.bottomPoleGuard, bottomPoleGuard);
        gl.drawArrays(gl.TRIANGLES, 0, 6);
        gl.flush();
        return this.canvas;
    }
}

function getTransformBasisLocal(transform) {
    if (!transform || !transform.orientation) {
        const right = { x: 1, y: 0, z: 0 };
        const up = { x: 0, y: 1, z: 0 };
        const back = { x: 0, y: 0, z: 1 };
        return {
            right,
            left: negate3(right),
            up,
            down: negate3(up),
            back,
            forward: negate3(back),
        };
    }

    const q = transform.orientation;
    const right = normalize3(rotateVectorByQuat(q, { x: 1, y: 0, z: 0 }));
    const up = normalize3(rotateVectorByQuat(q, { x: 0, y: 1, z: 0 }));
    const back = normalize3(rotateVectorByQuat(q, { x: 0, y: 0, z: 1 }));
    return {
        right,
        left: negate3(right),
        up,
        down: negate3(up),
        back,
        forward: negate3(back),
    };
}

function componentDirectionToLocal(basis, component) {
    return normalize3({
        x: basis.right.x * component.x + basis.up.x * component.y + basis.back.x * component.z,
        y: basis.right.y * component.x + basis.up.y * component.y + basis.back.y * component.z,
        z: basis.right.z * component.x + basis.up.z * component.y + basis.back.z * component.z,
    });
}

function captureTransformsEquivalent(a, b, positionToleranceM = 1e-5, orientationTolerance = 1e-6) {
    if (!a?.position || !a?.orientation || !b?.position || !b?.orientation) return false;
    const positionDelta = Math.hypot(
        a.position.x - b.position.x,
        a.position.y - b.position.y,
        a.position.z - b.position.z,
    );
    const qa = a.orientation;
    const qb = b.orientation;
    const dot = Math.abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w);
    return positionDelta <= positionToleranceM && Math.abs(1 - dot) <= orientationTolerance;
}

function clampNumber(value, min, max, fallback) {
    const n = Number(value);
    if (!Number.isFinite(n)) return fallback;
    return Math.max(min, Math.min(max, n));
}

function rotateXZ(v, radians) {
    const c = Math.cos(radians);
    const s = Math.sin(radians);
    return {
        x: v.x * c - v.z * s,
        z: v.x * s + v.z * c,
    };
}

export class CesiumWorld {
    constructor(containerId, options = {}) {
        this.containerId = containerId;
        this.token = options.token || urlString('ionToken', DEFAULT_ION_TOKEN);
        this.assetId = Number(options.assetId || urlNumber('assetId', DEFAULT_ASSET_ID));
        this.initialView = {
            longitude: urlNumber('lon', options.longitude ?? DEFAULT_VIEW.longitude),
            latitude: urlNumber('lat', options.latitude ?? DEFAULT_VIEW.latitude),
            height: urlNumber('height', options.height ?? DEFAULT_VIEW.height),
        };
        this.flightResolutionScale = clampNumber(
            urlNumber('resolutionScale', options.resolutionScale ?? 0.80),
            0.45,
            1,
            0.80
        );
        this.placementResolutionScale = clampNumber(
            urlNumber('placementResolutionScale', options.placementResolutionScale ?? 0.88),
            0.5,
            1,
            0.88
        );
        this.flightTileSSE = clampNumber(
            urlNumber('flightTileSse', options.flightTileSSE ?? 20),
            8,
            64,
            20
        );
        this.placementTileSSE = clampNumber(
            urlNumber('placementTileSse', options.placementTileSSE ?? 16),
            8,
            64,
            16
        );
        this.tileCacheMb = Math.round(clampNumber(
            urlNumber('tileCacheMb', options.tileCacheMb ?? 2048),
            512,
            8192,
            2048
        ));
        const defaultPanoramaTileSSE = demoPerformance.config.profile === 'demo30' ? 256 : 512;
        this.panoramaTileSSE = clampNumber(
            urlNumber('panoramaTileSse', options.panoramaTileSSE ?? defaultPanoramaTileSSE),
            4,
            1024,
            defaultPanoramaTileSSE
        );
        // The hidden 96 px capture faces are used for local obstacle sensing,
        // not a globe-scale horizon. A finite far plane materially reduces the
        // six repeated tileset traversals without changing ERP dimensions or
        // the accepted DA360 projection/calibration contract.
        this.panoramaFarMeters = clampNumber(
            urlNumber('panoramaFarMeters', options.panoramaFarMeters ?? 1200),
            500,
            15000000,
            1200
        );
        this.panoramaLeanStreaming = urlNumber(
            'panoramaLeanStreaming',
            options.panoramaLeanStreaming === undefined
                ? 1
                : options.panoramaLeanStreaming ? 1 : 0,
        ) >= 0.5;
        this.Cesium = null;
        this.viewer = null;
        this.tileset = null;
        this.ready = false;
        this._panoramaViewer = null;
        this._panoramaContainer = null;
        this._panoramaInitPromise = null;
        this._panoramaFaceSize = 0;
        this._panoramaProjector = null;
        this._panoramaTileset = null;
        this._panoramaTileLoadState = {
            pending: null,
            processing: null,
            errorCount: 0,
            lastErrorAt: null,
            lastErrorMessage: null,
        };
        this._panoramaCaptureActiveCount = 0;
        this._panoramaCaptureRevision = 0;
        this._lastCompletedPanoramaCapture = null;

        this.originCartographic = null;
        this.enuToFixed = null;
        this.fixedToEnu = null;
        this.spawnMarker = null;
        this._goalMarker = null;
        this.aircraftEntities = [];
        this.aircraftModelEntity = null;
        this._aircraftModelPosition = null;
        this._aircraftModelOrientation = null;
        this._tileLoadPending = null;
        this._tileLoadProcessing = null;
        this._lastPickWarning = 0;
        this._heightSampleCache = new Map();
        this._flightPerformanceMode = false;
    }

    async init(progressCb = null) {
        const Cesium = requireCesium();
        this.Cesium = Cesium;
        Cesium.Ion.defaultAccessToken = this.token;

        demoPerformance.configureCesium(Cesium);

        if (progressCb) progressCb('Creating Cesium viewer...');
        this.viewer = new Cesium.Viewer(this.containerId, {
            animation: false,
            timeline: false,
            baseLayerPicker: false,
            geocoder: true,
            homeButton: true,
            infoBox: false,
            navigationHelpButton: true,
            sceneModePicker: false,
            selectionIndicator: false,
            fullscreenButton: false,
            scene3DOnly: true,
            shouldAnimate: true,
            globe: false,
            skyAtmosphere: new Cesium.SkyAtmosphere(),
            requestRenderMode: true,    // Cesium 社区 #1 CPU 优化：空闲时 0% CPU
            // Keep explicit rendering, but do not hard-cap an interactive
            // flight view at 20 fps when the measured Chrome loop sustains 40+.
            targetFrameRate: 30,
            // resolutionScale 降低渲染像素数，同时减少 CPU draw-call 准备开销
            resolutionScale: 0.7,
            useBrowserRecommendedResolution: true,
            orderIndependentTranslucency: false,
            contextOptions: {
                webgl: {
                    alpha: false,
                    antialias: false,
                    preserveDrawingBuffer: false,
                    powerPreference: 'high-performance',
                    failIfMajorPerformanceCaveat: false,
                },
            },
        });

        demoPerformance.attachViewer(this.viewer, () => ({
            main: this.getTileLoadStatus?.() || null,
            panorama: this._panoramaTileLoadState
                ? { ...this._panoramaTileLoadState }
                : null,
        }));

        this.viewer.scene.fog.enabled = false;
        this.viewer.scene.highDynamicRange = false;
        this.viewer.scene.screenSpaceCameraController.enableCollisionDetection = false;
        this._configureScenePerformance(false);

        const origin = Cesium.Cartographic.fromDegrees(
            this.initialView.longitude,
            this.initialView.latitude,
            0
        );
        this.setOrigin(origin);

        if (progressCb) progressCb('Loading Google Photorealistic 3D Tiles...');
        this.tileset = await this._createGoogleTileset(progressCb);
        this._configureTilesetStreaming(false);
        this.viewer.scene.primitives.add(this.tileset);
        this._wireTilesetDiagnostics(progressCb);

        this.viewer.scene.camera.setView({
            destination: Cesium.Cartesian3.fromDegrees(
                this.initialView.longitude,
                this.initialView.latitude,
                this.initialView.height
            ),
            orientation: {
                heading: Cesium.Math.toRadians(0),
                pitch: Cesium.Math.toRadians(-35),
                roll: 0,
            },
        });
        this._configureHomeButton();
        this.viewer.scene.requestRender();
        if (progressCb) progressCb('Waiting for initial Google 3D Tiles...');
        await new Promise(resolve => window.setTimeout(resolve, 150));
        await this.waitForTilesIdle(3000, 250);

        this.ready = true;
        this.viewer.scene.requestRender();
        return this;
    }

    _configureHomeButton() {
        if (!this.viewer || !this.viewer.homeButton) return;
        const Cesium = this.Cesium;
        const command = this.viewer.homeButton.viewModel.command;
        command.beforeExecute.addEventListener((e) => {
            e.cancel = true;
            this.viewer.camera.flyTo({
                destination: Cesium.Cartesian3.fromDegrees(
                    this.initialView.longitude,
                    this.initialView.latitude,
                    this.initialView.height
                ),
                orientation: {
                    heading: Cesium.Math.toRadians(0),
                    pitch: Cesium.Math.toRadians(-35),
                    roll: 0,
                },
                duration: 1.2,
            });
        });
    }

    async _createGoogleTileset(progressCb = null) {
        const Cesium = this.Cesium;
        // Resolve the ion external asset ourselves so the Google API key can
        // travel in its supported X-Goog-Api-Key header instead of every tile
        // URL. Derived Cesium Resources inherit headers, while Firefox's own
        // network errors can no longer print the key in a failed request URL.
        if (Cesium.IonResource?.fromAssetId
            && Cesium.Resource
            && Cesium.Cesium3DTileset?.fromUrl) {
            try {
                if (progressCb) progressCb('Loading Google Photorealistic 3D Tiles...');
                const ionResource = await Cesium.IonResource.fromAssetId(this.assetId);
                const sourceUrl = new URL(ionResource.url);
                const googleApiKey = sourceUrl.searchParams.get('key');
                if (sourceUrl.hostname !== 'tile.googleapis.com' || !googleApiKey) {
                    throw new Error('ion Google Tiles endpoint is missing its expected host/key contract');
                }
                sourceUrl.username = '';
                sourceUrl.password = '';
                sourceUrl.searchParams.delete('key');
                sourceUrl.hash = '';
                const resourceOptions = {
                    url: sourceUrl.toString(),
                    headers: {
                        ...(ionResource.headers || {}),
                        'X-Goog-Api-Key': googleApiKey,
                    },
                    credits: ionResource.credits,
                };
                if (ionResource.proxy) resourceOptions.proxy = ionResource.proxy;
                const resource = new Cesium.Resource(resourceOptions);
                return await Cesium.Cesium3DTileset.fromUrl(resource, {
                    cacheBytes: 1536 * 1024 * 1024,
                    maximumCacheOverflowBytes: 1024 * 1024 * 1024,
                    enableCollision: true,
                });
            } catch (e) {
                reportUserError('Credential-safe Google Photorealistic tileset load failed', e, {
                    key: 'google-photorealistic-tileset-safe-load',
                    intervalMs: 10000,
                });
                // Do not silently fall back to query credentials: that would
                // reintroduce API keys into Firefox-native network errors.
                throw e;
            }
        }

        throw new Error('Cesium build lacks credential-safe Google Tiles Resource APIs');
    }

    _wireTilesetDiagnostics(progressCb = null, tileset = this.tileset, loadState = null, label = 'Google 3D Tiles') {
        if (!tileset) return;
        const keyPrefix = String(label || 'Google 3D Tiles').toLowerCase().replace(/[^a-z0-9]+/g, '-');
        let nextFailureReportAt = -Infinity;
        let failureReportBackoffMs = 10000;
        let lastFailureAt = -Infinity;
        const onFailure = (error) => {
            const message = formatError(error);
            const now = performance.now();
            if (loadState) {
                loadState.errorCount = Math.max(0, Number(loadState.errorCount) || 0) + 1;
                loadState.lastErrorAt = now;
                loadState.lastErrorMessage = message;
            }
            // Cesium can emit one event per failed JSON/GLB. Report one
            // sanitized summary with exponential backoff instead of flooding
            // DevTools (and the Firefox main thread) with full request errors.
            if (now - lastFailureAt > 60000) failureReportBackoffMs = 10000;
            lastFailureAt = now;
            if (now < nextFailureReportAt) return;
            nextFailureReportAt = now + failureReportBackoffMs;
            failureReportBackoffMs = Math.min(60000, failureReportBackoffMs * 2);
            reportUserError(`${label} request failed`, error, {
                key: `${keyPrefix}-request-failed`,
                intervalMs: 0,
            });
            if (progressCb) progressCb(`${label} request failed: ${message}`, true);
        };

        if (tileset.tileFailed && typeof tileset.tileFailed.addEventListener === 'function') {
            tileset.tileFailed.addEventListener(onFailure);
        }
        if (tileset.errorEvent && typeof tileset.errorEvent.addEventListener === 'function') {
            tileset.errorEvent.addEventListener(onFailure);
        }
        if (tileset.loadProgress && typeof tileset.loadProgress.addEventListener === 'function') {
            tileset.loadProgress.addEventListener((pending, processing) => {
                const nextPending = Math.max(0, Number(pending) || 0);
                const nextProcessing = Math.max(0, Number(processing) || 0);
                if (loadState) {
                    loadState.pending = nextPending;
                    loadState.processing = nextProcessing;
                } else {
                    this._tileLoadPending = nextPending;
                    this._tileLoadProcessing = nextProcessing;
                }
            });
        }
    }

    _configureScenePerformance(flightMode = this._flightPerformanceMode) {
        if (!this.viewer || !this.viewer.scene) return;
        const scene = this.viewer.scene;
        const resolutionScale = flightMode ? this.flightResolutionScale : this.placementResolutionScale;

        if ('resolutionScale' in this.viewer) {
            this.viewer.resolutionScale = resolutionScale;
        }
        if ('msaaSamples' in scene) {
            scene.msaaSamples = 1;
        }
        if (scene.postProcessStages && scene.postProcessStages.fxaa) {
            scene.postProcessStages.fxaa.enabled = false;
        }
        scene.highDynamicRange = false;
    }

    _configureTilesetStreaming(flightMode = this._flightPerformanceMode) {
        const tileset = this.tileset;
        if (!tileset) return;

        const setIfPresent = (key, value) => {
            if (key in tileset) tileset[key] = value;
        };

        setIfPresent('maximumScreenSpaceError', flightMode ? this.flightTileSSE : this.placementTileSSE);
        setIfPresent('cullRequestsWhileMoving', true);
        setIfPresent('cullRequestsWhileMovingMultiplier', flightMode ? 90 : 60);
        setIfPresent('preloadWhenHidden', false);
        setIfPresent('preloadFlightDestinations', false);
        setIfPresent('foveatedScreenSpaceError', true);
        setIfPresent('foveatedConeSize', flightMode ? 0.2 : 0.28);
        setIfPresent('foveatedMinimumScreenSpaceErrorRelaxation', flightMode ? 4 : 2);
        const dynamicSseEnabled = demoPerformance.config.dynamicSse !== 'off';
        setIfPresent('dynamicScreenSpaceError', dynamicSseEnabled);  // 远处自动降 LOD
        setIfPresent('dynamicScreenSpaceErrorDensity', 0.2);
        setIfPresent('dynamicScreenSpaceErrorFactor', 4.0);
        setIfPresent('foveatedTimeDelay', flightMode ? 0.08 : 0.15);
        setIfPresent('dynamicScreenSpaceError', dynamicSseEnabled);
        setIfPresent('dynamicScreenSpaceErrorDensity', flightMode ? 0.0035 : 0.0025);
        setIfPresent('dynamicScreenSpaceErrorFactor', flightMode ? 12 : 8);
        setIfPresent('loadSiblings', false);
        setIfPresent('skipLevelOfDetail', true);
        setIfPresent('baseScreenSpaceError', flightMode ? 1536 : 1024);
        setIfPresent('skipScreenSpaceErrorFactor', flightMode ? 18 : 12);
        setIfPresent('skipLevels', flightMode ? 2 : 1);
        setIfPresent('immediatelyLoadDesiredLevelOfDetail', false);
        setIfPresent('preferLeaves', false);

        if ('maximumMemoryUsage' in tileset) {
            tileset.maximumMemoryUsage = Math.max(tileset.maximumMemoryUsage || 0, this.tileCacheMb);
        }
        if ('cacheBytes' in tileset) {
            tileset.cacheBytes = Math.max(tileset.cacheBytes || 0, this.tileCacheMb * 1024 * 1024);
        }
        if ('maximumCacheOverflowBytes' in tileset) {
            const overflowMb = Math.min(768, Math.max(256, Math.round(this.tileCacheMb * 0.35)));
            tileset.maximumCacheOverflowBytes = Math.max(
                tileset.maximumCacheOverflowBytes || 0,
                overflowMb * 1024 * 1024
            );
        }
    }

    setFlightPerformanceMode(enabled) {
        const flightMode = !!enabled;
        if (this._flightPerformanceMode === flightMode) return;
        this._flightPerformanceMode = flightMode;
        this._configureScenePerformance(flightMode);
        this._configureTilesetStreaming(flightMode);
        this.viewer?.scene?.requestRender();
    }

    getTileLoadStatus() {
        return {
            pending: this._tileLoadPending,
            processing: this._tileLoadProcessing,
            tilesLoaded: !!(this.tileset && this.tileset.tilesLoaded === true),
        };
    }

    waitForTilesIdle(
        timeoutMs = 1600,
        quietMs = 180,
        tileset = null,
        loadState = null,
        renderViewer = null,
        renderTimings = null,
        signal = null
    ) {
        const targetTileset = tileset || this.tileset;
        if (!targetTileset) return Promise.resolve(true);

        return new Promise((resolve) => {
            const started = performance.now();
            let idleSince = null;
            let done = false;

            const finish = (idle) => {
                if (done) return;
                done = true;
                resolve(!!idle);
            };

            const tick = () => {
                if (done) return;
                if (signal?.aborted) return finish(false);
                const renderViewerDestroyed = renderViewer
                    && typeof renderViewer.isDestroyed === 'function'
                    && renderViewer.isDestroyed();
                if (renderViewer && !renderViewerDestroyed && renderViewer.scene) {
                    renderViewer.scene.requestRender();
                    const renderStartedAt = performance.now();
                    this._renderViewerNow(renderViewer);
                    if (renderTimings && typeof renderTimings === 'object') {
                        renderTimings.renderMs = (Number(renderTimings.renderMs) || 0)
                            + performance.now() - renderStartedAt;
                        renderTimings.renderCount = (Number(renderTimings.renderCount) || 0) + 1;
                    }
                }
                const now = performance.now();
                const pending = loadState ? loadState.pending : this._tileLoadPending;
                const processing = loadState ? loadState.processing : this._tileLoadProcessing;
                const queueKnown = pending !== null || processing !== null;
                const queueIdle = !queueKnown ||
                    ((pending || 0) <= 0 && (processing || 0) <= 0);
                const loaded = targetTileset.tilesLoaded === true && queueIdle;

                if (loaded) {
                    if (idleSince == null) idleSince = now;
                    if (now - idleSince >= quietMs) return finish(true);
                } else {
                    idleSince = null;
                }

                if (now - started >= timeoutMs) return finish(false);
                window.setTimeout(tick, 80);
            };

            tick();
        });
    }

    _buildPreloadTargets(radius, spacing, maxTargets = 36) {
        const targets = [{ x: 0, z: 0 }];
        const steps = Math.max(1, Math.ceil(radius / spacing));

        for (let iz = -steps; iz <= steps; iz++) {
            for (let ix = -steps; ix <= steps; ix++) {
                const x = ix * spacing;
                const z = iz * spacing;
                const d = Math.hypot(x, z);
                if (d < 1 || d > radius) continue;
                targets.push({ x, z, d });
            }
        }

        targets.sort((a, b) => (a.d || 0) - (b.d || 0));
        return targets.slice(0, Math.max(1, maxTargets));
    }

    _makePreloadView(centerLocal, offset, index, lift, viewDistance) {
        const dist = Math.hypot(offset.x, offset.z);
        const cardinals = [
            { x: 0, z: -1 },
            { x: 1, z: 0 },
            { x: 0, z: 1 },
            { x: -1, z: 0 },
        ];
        const baseDir = dist > 1
            ? { x: -offset.x / dist, z: -offset.z / dist }
            : cardinals[index % cardinals.length];
        const dir = rotateXZ(baseDir, ((index % 3) - 1) * 0.38);
        const target = {
            x: centerLocal.x + offset.x,
            y: centerLocal.y + 8,
            z: centerLocal.z + offset.z,
        };
        return {
            eye: {
                x: target.x - dir.x * viewDistance,
                y: centerLocal.y + lift,
                z: target.z - dir.z * viewDistance,
            },
            target,
        };
    }

    _buildLocalAreaPreloadViews(centerLocal, radius, lift, viewDistance, gridSpacing, maxTargets) {
        const views = [];
        const overviewLift = Math.max(lift * 1.35, 240);
        const overviewDistance = Math.max(viewDistance, Math.min(radius * 0.45, 420));
        const overviewTarget = { x: centerLocal.x, y: centerLocal.y + 20, z: centerLocal.z };
        const overviewDirs = [
            { x: 0, z: 1 },
            { x: 1, z: 0 },
            { x: -1, z: 0 },
            { x: 0, z: -1 },
        ];

        views.push({
            eye: { x: centerLocal.x, y: centerLocal.y + Math.max(overviewLift, radius * 0.35), z: centerLocal.z + Math.min(radius * 0.15, 160) },
            target: overviewTarget,
        });
        for (const dir of overviewDirs) {
            views.push({
                eye: {
                    x: centerLocal.x + dir.x * overviewDistance,
                    y: centerLocal.y + overviewLift,
                    z: centerLocal.z + dir.z * overviewDistance,
                },
                target: overviewTarget,
            });
        }
        for (const dir of overviewDirs) {
            views.push({
                eye: { x: centerLocal.x, y: centerLocal.y + 4, z: centerLocal.z },
                target: {
                    x: centerLocal.x + dir.x * Math.min(radius, 500),
                    y: centerLocal.y + 3,
                    z: centerLocal.z + dir.z * Math.min(radius, 500),
                },
            });
        }

        const targets = this._buildPreloadTargets(radius, gridSpacing, maxTargets);
        for (let i = 0; i < targets.length; i++) {
            views.push(this._makePreloadView(centerLocal, targets[i], i, lift, viewDistance));
        }
        return views;
    }

    _sampleLoadedCoverage(centerLocal, radius, spacing) {
        this._heightSampleCache.clear();
        const samples = this._buildPreloadTargets(radius, spacing, 80);
        let loaded = 0;
        const missing = [];

        for (const sample of samples) {
            const y = this.sampleHeightAtLocal(centerLocal.x + sample.x, centerLocal.z + sample.z, 1.0);
            if (Number.isFinite(y)) {
                loaded++;
            } else {
                missing.push(sample);
            }
        }

        return {
            loaded,
            total: samples.length,
            ratio: samples.length ? loaded / samples.length : 1,
            missing,
        };
    }

    _sampleLoadedCorridor(startLocal, endLocal, options = {}) {
        this._heightSampleCache.clear();
        const spacing = clampNumber(options.spacing, 10, 100, 30);
        const halfWidth = clampNumber(options.halfWidth, 0, 120, 35);
        const dx = endLocal.x - startLocal.x;
        const dz = endLocal.z - startLocal.z;
        const routeLength = Math.hypot(dx, dz);
        const center = {
            x: (startLocal.x + endLocal.x) * 0.5,
            y: (startLocal.y + endLocal.y) * 0.5,
            z: (startLocal.z + endLocal.z) * 0.5,
        };
        const normalX = routeLength > 1e-6 ? -dz / routeLength : 1;
        const normalZ = routeLength > 1e-6 ? dx / routeLength : 0;
        const alongSteps = routeLength > 1e-6 ? Math.max(1, Math.ceil(routeLength / spacing)) : 0;
        const lateralSteps = halfWidth > 0 ? Math.max(1, Math.ceil(halfWidth / spacing)) : 0;
        const lateralOffsets = lateralSteps > 0
            ? Array.from(
                { length: lateralSteps * 2 + 1 },
                (_, index) => ((index - lateralSteps) / lateralSteps) * halfWidth,
            )
            : [0];
        const samples = [];
        const missing = [];
        let loaded = 0;

        for (let alongIndex = 0; alongIndex <= alongSteps; alongIndex++) {
            const t = alongSteps > 0 ? alongIndex / alongSteps : 0;
            const baseX = startLocal.x + dx * t;
            const baseZ = startLocal.z + dz * t;
            for (const lateralOffset of lateralOffsets) {
                const localX = baseX + normalX * lateralOffset;
                const localZ = baseZ + normalZ * lateralOffset;
                const sample = {
                    x: localX - center.x,
                    z: localZ - center.z,
                    localX,
                    localZ,
                    d: Math.hypot(localX - center.x, localZ - center.z),
                    distanceAlong: routeLength * t,
                    lateralOffset,
                };
                samples.push(sample);
                const y = this.sampleHeightAtLocal(localX, localZ, 1.0);
                if (Number.isFinite(y)) loaded++;
                else missing.push(sample);
            }
        }

        return {
            loaded,
            total: samples.length,
            ratio: samples.length ? loaded / samples.length : 1,
            missing,
            center,
            routeLength,
            halfWidth,
            spacing,
        };
    }

    async preloadCollisionCorridor(startLocal, endLocal, options = {}) {
        if (!this.viewer || !this.ready || !startLocal || !endLocal) return null;
        const coordinates = [
            startLocal.x, startLocal.y, startLocal.z,
            endLocal.x, endLocal.y, endLocal.z,
        ].map(Number);
        if (!coordinates.every(Number.isFinite)) return null;

        const start = { x: coordinates[0], y: coordinates[1], z: coordinates[2] };
        const end = { x: coordinates[3], y: coordinates[4], z: coordinates[5] };
        const routeLength = Math.hypot(end.x - start.x, end.z - start.z);
        const halfWidth = clampNumber(options.halfWidth, 10, 120, 35);
        const spacing = clampNumber(options.spacing, 10, 100, 30);
        const maxRadius = clampNumber(options.maxRadius, 120, 1200, 400);
        const radius = Math.min(maxRadius, Math.max(80, routeLength * 0.5 + halfWidth + 40));
        const center = {
            x: (start.x + end.x) * 0.5,
            y: (start.y + end.y) * 0.5,
            z: (start.z + end.z) * 0.5,
        };
        const attempts = Math.round(clampNumber(options.attempts, 1, 4, 3));
        const maxTargets = Math.round(clampNumber(
            options.maxTargets,
            8,
            32,
            Math.min(24, Math.max(10, Math.ceil(routeLength / 50) + 8)),
        ));
        const coverageSampler = () => this._sampleLoadedCorridor(start, end, {
            halfWidth,
            spacing,
        });

        const report = await this.preloadLocalArea(center, {
            radius,
            lift: Number.isFinite(options.lift) ? options.lift : 180,
            gridSpacing: Number.isFinite(options.gridSpacing) ? options.gridSpacing : 100,
            viewDistance: Number.isFinite(options.viewDistance)
                ? options.viewDistance
                : Math.max(160, Math.min(260, radius * 0.9)),
            maxTargets,
            dwellMs: Number.isFinite(options.dwellMs) ? options.dwellMs : 120,
            perViewTimeoutMs: Number.isFinite(options.perViewTimeoutMs)
                ? options.perViewTimeoutMs
                : 2500,
            finalIdleTimeoutMs: Number.isFinite(options.finalIdleTimeoutMs)
                ? options.finalIdleTimeoutMs
                : 10000,
            totalTimeoutMs: options.totalTimeoutMs,
            verifyCoverage: true,
            coverageSampler,
            minCoverageRatio: 1,
            repairPasses: attempts - 1,
            repairTargets: 32,
            progressCb: options.progressCb,
        });
        if (report) {
            report.corridor = { center, routeLength, halfWidth, spacing, attempts };
        }
        return report;
    }

    async preloadLocalArea(centerLocal, options = {}) {
        if (!this.viewer || !this.ready || !centerLocal) return null;
        const Cesium = this.Cesium;
        const requestScheduler = Cesium?.RequestScheduler;
        const savedRequestsPerServer = requestScheduler
            && 'maximumRequestsPerServer' in requestScheduler
            ? requestScheduler.maximumRequestsPerServer
            : null;
        if (savedRequestsPerServer !== null) {
            requestScheduler.maximumRequestsPerServer = Math.max(
                savedRequestsPerServer,
                Number(demoPerformance.config.preloadTileRequestsPerServer) || 18,
            );
        }
        const camera = this.viewer.camera;
        const saved = {
            position: Cesium.Cartesian3.clone(camera.positionWC),
            direction: Cesium.Cartesian3.clone(camera.directionWC),
            up: Cesium.Cartesian3.clone(camera.upWC),
        };

        const radius = Math.max(60, Number.isFinite(options.radius) ? options.radius : 220);
        const lift = Math.max(80, Number.isFinite(options.lift) ? options.lift : (radius >= 800 ? 260 : 150));
        const gridSpacing = clampNumber(options.gridSpacing, 100, 600, radius >= 800 ? 330 : Math.max(180, radius * 0.75));
        const viewDistance = clampNumber(options.viewDistance, 140, 420, radius >= 800 ? 260 : Math.max(160, radius * 0.75));
        const maxTargets = Math.round(clampNumber(options.maxTargets, 4, 60, radius >= 800 ? 34 : 12));
        const dwellMs = Math.max(80, Number.isFinite(options.dwellMs) ? options.dwellMs : 180);
        const perViewTimeoutMs = Math.max(450, Number.isFinite(options.perViewTimeoutMs) ? options.perViewTimeoutMs : 1600);
        const finalIdleTimeoutMs = Math.max(perViewTimeoutMs, Number.isFinite(options.finalIdleTimeoutMs) ? options.finalIdleTimeoutMs : 5000);
        const verifyCoverage = options.verifyCoverage === true
            || (options.verifyCoverage !== false && radius >= 350);
        const coverageSampler = typeof options.coverageSampler === 'function'
            ? options.coverageSampler
            : null;
        const coverageSpacing = clampNumber(options.coverageSpacing, 100, 600, Math.max(240, gridSpacing));
        const minCoverageRatio = clampNumber(options.minCoverageRatio, 0, 1, 0.72);
        const repairPasses = Math.round(clampNumber(options.repairPasses, 0, 3, verifyCoverage ? 1 : 0));
        const repairTargets = Math.round(clampNumber(options.repairTargets, 4, 32, 16));
        const progressCb = typeof options.progressCb === 'function' ? options.progressCb : null;
        const label = radius >= 1000 ? `${(radius / 1000).toFixed(1)} km` : `${Math.round(radius)} m`;
        const delay = (ms) => new Promise(resolve => window.setTimeout(resolve, ms));
        const totalTimeoutMs = Number.isFinite(options.totalTimeoutMs)
            ? Math.max(1000, options.totalTimeoutMs)
            : Infinity;
        const preloadStartedAt = performance.now();
        const remainingBudgetMs = () => totalTimeoutMs - (performance.now() - preloadStartedAt);
        const report = {
            radius,
            views: 0,
            timedOutViews: 0,
            finalIdle: false,
            coverage: null,
            deadlineExceeded: false,
        };

        const runViews = async (views, passLabel) => {
            for (let i = 0; i < views.length; i++) {
                if (remainingBudgetMs() <= 0) {
                    report.deadlineExceeded = true;
                    break;
                }
                const v = views[i];
                const status = this.getTileLoadStatus();
                const queue = status.pending !== null || status.processing !== null
                    ? `; queue ${status.pending || 0}/${status.processing || 0}`
                    : '';
                if (progressCb) progressCb(`Preloading ${label} collision tiles ${passLabel} (${i + 1}/${views.length}${queue})...`);

                const eye = { x: v.eye.x, y: v.eye.y, z: v.eye.z };
                const surfaceY = this.sampleHeightAtLocal(eye.x, eye.z, 1.0);
                if (Number.isFinite(surfaceY)) eye.y = Math.max(eye.y, surfaceY + 18);

                const directionLocal = normalize3({
                    x: v.target.x - eye.x,
                    y: v.target.y - eye.y,
                    z: v.target.z - eye.z,
                });
                camera.setView({
                    destination: this.localToCartesian(eye),
                    orientation: {
                        direction: this.localDirectionToFixed(directionLocal),
                        up: this.localDirectionToFixed({ x: 0, y: 1, z: 0 }),
                    },
                });
                this.viewer.scene.requestRender();
                await delay(Math.min(dwellMs, Math.max(0, remainingBudgetMs())));
                const remaining = remainingBudgetMs();
                if (remaining <= 0) {
                    report.deadlineExceeded = true;
                    report.views++;
                    break;
                }
                const idle = await this.waitForTilesIdle(Math.max(1, Math.min(perViewTimeoutMs, remaining)));
                if (!idle) report.timedOutViews++;
                report.views++;
            }
        };

        const settleAfterViews = async () => {
            const remaining = remainingBudgetMs();
            if (remaining <= 0) {
                report.deadlineExceeded = true;
                return false;
            }
            return this.waitForTilesIdle(Math.max(1, Math.min(finalIdleTimeoutMs, remaining)), 350);
        };

        try {
            const initialViews = this._buildLocalAreaPreloadViews(
                centerLocal,
                radius,
                lift,
                viewDistance,
                gridSpacing,
                maxTargets
            );
            await runViews(initialViews, 'scan');
            report.finalIdle = await settleAfterViews();

            for (let pass = 0; verifyCoverage && pass <= repairPasses; pass++) {
                if (progressCb) progressCb(`Verifying ${label} collision tile coverage...`);
                report.coverage = coverageSampler
                    ? coverageSampler()
                    : this._sampleLoadedCoverage(centerLocal, radius, coverageSpacing);
                const pct = Math.round(report.coverage.ratio * 100);
                if (progressCb) progressCb(`Collision preload coverage ${report.coverage.loaded}/${report.coverage.total} (${pct}%).`);
                if (report.coverage.ratio >= minCoverageRatio || pass === repairPasses || !report.coverage.missing.length) break;
                if (remainingBudgetMs() <= 0) {
                    report.deadlineExceeded = true;
                    break;
                }

                const repairViews = report.coverage.missing
                    .slice(0, repairTargets)
                    .map((offset, i) => this._makePreloadView(centerLocal, offset, i + pass * repairTargets, lift, viewDistance));
                await runViews(repairViews, `repair ${pass + 1}`);
                report.finalIdle = await settleAfterViews();
            }
            if (report.deadlineExceeded && progressCb) {
                progressCb(`Collision preload reached its ${Math.round(totalTimeoutMs / 1000)} s time budget.`);
            }
        } finally {
            if (savedRequestsPerServer !== null) {
                requestScheduler.maximumRequestsPerServer = savedRequestsPerServer;
            }
            camera.setView({
                destination: saved.position,
                orientation: {
                    direction: saved.direction,
                    up: saved.up,
                },
            });
            this.viewer.scene.requestRender();
        }

        return report;
    }

    destroy() {
        this._destroyPanoramaCaptureViewer();
        if (this.viewer && !this.viewer.isDestroyed()) {
            this.viewer.destroy();
        }
        this.viewer = null;
        this.tileset = null;
        this.ready = false;
        this._heightSampleCache.clear();
    }

    setOrigin(cartographic) {
        const Cesium = this.Cesium || requireCesium();
        this._heightSampleCache.clear();
        this.originCartographic = new Cesium.Cartographic(
            cartographic.longitude,
            cartographic.latitude,
            cartographic.height || 0
        );
        const originCartesian = Cesium.Cartesian3.fromRadians(
            this.originCartographic.longitude,
            this.originCartographic.latitude,
            this.originCartographic.height
        );
        this.enuToFixed = Cesium.Transforms.eastNorthUpToFixedFrame(originCartesian);
        this.fixedToEnu = Cesium.Matrix4.inverse(this.enuToFixed, new Cesium.Matrix4());
    }

    localToCartesian(local) {
        const Cesium = this.Cesium;
        const enu = new Cesium.Cartesian3(local.x, local.z, local.y);
        return Cesium.Matrix4.multiplyByPoint(this.enuToFixed, enu, new Cesium.Cartesian3());
    }

    localToCartographic(local) {
        const Cesium = this.Cesium;
        return Cesium.Cartographic.fromCartesian(this.localToCartesian(local));
    }

    cartesianToLocal(cartesian) {
        const Cesium = this.Cesium;
        const enu = Cesium.Matrix4.multiplyByPoint(this.fixedToEnu, cartesian, new Cesium.Cartesian3());
        return { x: enu.x, y: enu.z, z: enu.y };
    }

    localDirectionToFixed(direction) {
        const Cesium = this.Cesium;
        const enu = new Cesium.Cartesian3(direction.x, direction.z, direction.y);
        const fixed = Cesium.Matrix4.multiplyByPointAsVector(this.enuToFixed, enu, new Cesium.Cartesian3());
        return Cesium.Cartesian3.normalize(fixed, fixed);
    }

    setNativeCameraControls(enabled) {
        if (!this.viewer) return;
        const c = this.viewer.scene.screenSpaceCameraController;
        c.enableRotate = enabled;
        c.enableTranslate = enabled;
        c.enableZoom = enabled;
        c.enableTilt = enabled;
        c.enableLook = enabled;
    }

    async pickSpawn(windowPosition, altitudeMeters = 100) {
        const Cesium = this.Cesium;
        const scene = this.viewer.scene;
        let cartesian = null;

        try {
            const picked = scene.pick(windowPosition);
            if (picked && scene.pickPositionSupported) {
                const p = scene.pickPosition(windowPosition);
                if (Cesium.defined(p)) cartesian = p;
            }
        } catch (error) {
            reportUserError('Scene pickPosition failed', error, {
                key: 'scene-pick-position',
                intervalMs: 10000,
            });
            cartesian = null;
        }

        if (!cartesian) {
            try {
                const ray = this.viewer.camera.getPickRay(windowPosition);
                if (ray && typeof scene.pickFromRay === 'function') {
                    const hit = scene.pickFromRay(ray);
                    if (hit && Cesium.defined(hit.position)) cartesian = hit.position;
                }
            } catch (error) {
                reportUserError('Scene pickFromRay failed while picking spawn', error, {
                    key: 'scene-pick-from-ray-spawn',
                    intervalMs: 10000,
                });
                cartesian = null;
            }
        }

        if (!cartesian) {
            try {
                const p = this.viewer.camera.pickEllipsoid(windowPosition, Cesium.Ellipsoid.WGS84);
                if (Cesium.defined(p)) cartesian = p;
            } catch (error) {
                reportUserError('Camera pickEllipsoid failed while picking spawn', error, {
                    key: 'camera-pick-ellipsoid-spawn',
                    intervalMs: 10000,
                });
                cartesian = null;
            }
        }

        if (!cartesian) {
            try {
                const ray = this.viewer.camera.getPickRay(windowPosition);
                const ellipsoidHit = ray
                    ? Cesium.IntersectionTests.rayEllipsoid(ray, Cesium.Ellipsoid.WGS84)
                    : null;
                if (ellipsoidHit) {
                    const distance = ellipsoidHit.start >= 0 ? ellipsoidHit.start : ellipsoidHit.stop;
                    cartesian = Cesium.Ray.getPoint(ray, distance, new Cesium.Cartesian3());
                }
            } catch (error) {
                reportUserError('Ray ellipsoid fallback failed while picking spawn', error, {
                    key: 'ray-ellipsoid-spawn',
                    intervalMs: 10000,
                });
                cartesian = null;
            }
        }

        if (!cartesian) return null;

        const cartographic = Cesium.Cartographic.fromCartesian(cartesian);
        this.setOrigin(new Cesium.Cartographic(
            cartographic.longitude,
            cartographic.latitude,
            0
        ));
        const spawn = { x: 0, y: Math.max(0, altitudeMeters || 0), z: 0 };
        this.updateSpawnMarker(spawn);
        return spawn;
    }

    updateSpawnMarker(local) {
        if (!this.viewer || !local) return;
        const Cesium = this.Cesium;
        const position = this.localToCartesian(local);
        if (!this.spawnMarker) {
            this.spawnMarker = this.viewer.entities.add({
                name: 'spawn-point',
                position,
                point: {
                    pixelSize: 14,
                    color: Cesium.Color.CYAN,
                    outlineColor: Cesium.Color.WHITE,
                    outlineWidth: 2,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY,
                },
                label: {
                    text: 'SPAWN',
                    font: '12px sans-serif',
                    pixelOffset: new Cesium.Cartesian2(0, -24),
                    fillColor: Cesium.Color.CYAN,
                    outlineColor: Cesium.Color.BLACK,
                    outlineWidth: 2,
                    style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                    disableDepthTestDistance: Number.POSITIVE_INFINITY,
                },
            });
        } else {
            this.spawnMarker.position = position;
            this.spawnMarker.show = true;
        }
        this.viewer.scene.requestRender();
    }

    hideSpawnMarker() {
        if (this.spawnMarker) this.spawnMarker.show = false;
    }

    showGoalMarker(local) {
        const Cesium = this.Cesium;
        if (this._goalMarker) this.viewer.entities.remove(this._goalMarker);
        const pos = this.localToCartesian(local);
        const groundPos = this.localToCartesian({ x: local.x, y: 0, z: local.z });
        this._goalMarker = this.viewer.entities.add({
            position: pos,
            point: { pixelSize: 10, color: Cesium.Color.LIME, outlineColor: Cesium.Color.BLACK, outlineWidth: 2 },
            label: {
                text: `${Math.round(local.y)}m`,
                font: `${(typeof window !== 'undefined' && window._goalFontSize) || 18}px Chakra Petch, monospace`,
                fillColor: Cesium.Color.LIME,
                outlineColor: Cesium.Color.BLACK,
                outlineWidth: 3,
                style: Cesium.LabelStyle.FILL_AND_OUTLINE,
                verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
                pixelOffset: new Cesium.Cartesian2(0, -14),
            },
            polyline: {
                positions: [groundPos, pos],
                material: new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.LIME.withAlpha(0.6), dashLength: 8 }),
                width: 2,
            },
        });
    }

    clearGoalMarker() {
        if (this._goalMarker) {
            this.viewer.entities.remove(this._goalMarker);
            this._goalMarker = null;
        }
    }

    _collisionExclusions() {
        const excluded = [];
        if (this.spawnMarker) excluded.push(this.spawnMarker);
        for (const entity of this.aircraftEntities) {
            if (entity) excluded.push(entity);
        }
        return excluded;
    }

    _isExcludedCollisionHit(hit) {
        if (!hit || !hit.object) return false;
        const object = hit.object;
        const entity = object.id || object;
        if (this.spawnMarker && (object === this.spawnMarker || entity === this.spawnMarker)) return true;
        return this.aircraftEntities.some(e => e && (object === e || entity === e));
    }

    _ensureAircraft() {
        if (this.aircraftEntities.length || !this.viewer) return;
        const Cesium = this.Cesium;
        this.aircraftModelEntity = this.viewer.entities.add({
            name: 'cesium-drone-model',
            position: new Cesium.CallbackProperty(() => (
                this._aircraftModelPosition || Cesium.Cartesian3.ZERO
            ), false),
            orientation: new Cesium.CallbackProperty(() => (
                this._aircraftModelOrientation || new Cesium.Quaternion(0, 0, 0, 1)
            ), false),
            model: {
                uri: CESIUM_DRONE_MODEL_URI,
                scale: CESIUM_DRONE_MODEL_SCALE,
                // minimumPixelSize 会在远距离强行把模型撑到给定屏占像素，
                // 从而让实际观感脱离物理尺寸。之前的 44 是视觉过大的第二个原因，
                // 这里降到刚好保证远处可见的程度。
                minimumPixelSize: 8,
                maximumScale: 4,
                runAnimations: true,
                incrementallyLoadTextures: false,
                shadows: Cesium.ShadowMode.DISABLED,
                silhouetteColor: Cesium.Color.fromAlpha(Cesium.Color.CYAN, 0.8),
                silhouetteSize: 1.0,
            },
            show: false,
        });
        this.aircraftEntities.push(this.aircraftModelEntity);
    }

    showAircraft(show) {
        this._ensureAircraft();
        for (const e of this.aircraftEntities) e.show = !!show;
    }

    updateAircraftFromDroneTransform(transform) {
        if (!this.viewer || !transform || !transform.orientation) return;
        this._ensureAircraft();
        const Cesium = this.Cesium;
        this._aircraftModelPosition = this.localToCartesian(transform.position);

        const basis = this.getTransformBasisFixed(transform);
        // Cesium axis-corrects glTF 2.0 models from Y-up/Z-forward into its
        // runtime model frame: +X forward, +Y left, +Z up.
        const xAxis = basis.forward;
        const yAxis = basis.right;
        const zAxis = basis.up;
        const rotation = Cesium.Matrix3.fromColumnMajorArray([
            xAxis.x, xAxis.y, xAxis.z,
            yAxis.x, yAxis.y, yAxis.z,
            zAxis.x, zAxis.y, zAxis.z,
        ], new Cesium.Matrix3());
        this._aircraftModelOrientation = Cesium.Quaternion.fromRotationMatrix(rotation, new Cesium.Quaternion());
    }

    sampleHeightAtLocal(x, z, width = 0.4) {
        if (!this.viewer || !this.ready) return null;
        const Cesium = this.Cesium;
        const scene = this.viewer.scene;
        if (typeof scene.sampleHeight !== 'function') return null;
        const now = performance.now();
        const grid = Math.max(0.75, width * 1.5);
        const key = `${Math.round(x / grid)}:${Math.round(z / grid)}:${Math.round(width * 10)}`;
        const cached = this._heightSampleCache.get(key);
        if (cached && now - cached.time <= HEIGHT_CACHE_TTL_MS) {
            return cached.value;
        }

        const carto = this.localToCartographic({ x, y: 0, z });
        let sampledHeight;
        try {
            sampledHeight = scene.sampleHeight(carto, this._collisionExclusions(), width);
        } catch (error) {
            reportUserError('Scene height sample with exclusions failed', error, {
                key: 'height-sample-exclusions',
                intervalMs: 10000,
            });
            try {
                sampledHeight = scene.sampleHeight(carto, undefined, width);
            } catch (fallbackError) {
                reportUserError('Scene height sample failed', fallbackError, {
                    key: 'height-sample',
                    intervalMs: 10000,
                });
                return null;
            }
        }
        if (!Number.isFinite(sampledHeight)) {
            this._rememberHeightSample(key, null, now);
            return null;
        }

        const surfaceCartesian = Cesium.Cartesian3.fromRadians(
            carto.longitude,
            carto.latitude,
            sampledHeight
        );
        const localY = this.cartesianToLocal(surfaceCartesian).y;
        this._rememberHeightSample(key, localY, now);
        return localY;
    }

    _rememberHeightSample(key, value, time) {
        this._heightSampleCache.set(key, { value, time });
        if (this._heightSampleCache.size <= HEIGHT_CACHE_LIMIT) return;
        const firstKey = this._heightSampleCache.keys().next().value;
        if (firstKey !== undefined) this._heightSampleCache.delete(firstKey);
    }

    pickLocalRay(originLocal, directionLocal, maxDistance) {
        if (!this.viewer || !this.ready) return null;
        const Cesium = this.Cesium;
        const scene = this.viewer.scene;
        if (typeof scene.pickFromRay !== 'function') {
            const now = performance.now();
            if (now - this._lastPickWarning > 5000) {
                reportUserError(
                    'Scene pickFromRay unavailable',
                    new Error('collision uses height sampling only'),
                    { key: 'scene-pick-from-ray-unavailable', intervalMs: 10000 }
                );
                this._lastPickWarning = now;
            }
            return null;
        }

        const dir = normalize3(directionLocal);
        if (Math.hypot(dir.x, dir.y, dir.z) < 1e-6) return null;

        const origin = this.localToCartesian(originLocal);
        const direction = this.localDirectionToFixed(dir);
        const ray = new Cesium.Ray(origin, direction);

        let hit;
        try {
            hit = scene.pickFromRay(ray, this._collisionExclusions());
        } catch (error) {
            reportUserError('Scene pickFromRay failed during collision query', error, {
                key: 'scene-pick-from-ray-collision',
                intervalMs: 10000,
            });
            return null;
        }
        if (!hit || !Cesium.defined(hit.position)) return null;
        if (this._isExcludedCollisionHit(hit)) return null;

        const local = this.cartesianToLocal(hit.position);
        const dx = local.x - originLocal.x;
        const dy = local.y - originLocal.y;
        const dz = local.z - originLocal.z;
        const distance = Math.hypot(dx, dy, dz);
        if (!Number.isFinite(distance) || distance > maxDistance) return null;
        return { position: local, distance };
    }

    setCameraFromDroneTransform(transform, hfovDeg) {
        if (!this.viewer || !this.ready || !transform || !transform.orientation) return;
        const Cesium = this.Cesium;
        const aspect = Math.max(0.1, this.viewer.canvas.clientWidth / Math.max(1, this.viewer.canvas.clientHeight));
        const hfov = Cesium.Math.toRadians(Math.max(30, Math.min(140, hfovDeg || 100)));
        const vfov = 2 * Math.atan(Math.tan(hfov * 0.5) / aspect);
        if (this.viewer.camera.frustum && Number.isFinite(vfov)) {
            this.viewer.camera.frustum.fov = vfov;
            this.viewer.camera.frustum.near = 0.03;
            this.viewer.camera.frustum.far = 15000000;
        }

        const basis = this.getTransformBasisFixed(transform);

        const destination = this.localToCartesian(transform.position);
        const direction = basis.forward;
        const up = basis.up;

        this.viewer.camera.setView({
            destination,
            orientation: { direction, up },
        });
        this.viewer.scene.requestRender();
    }

    getTransformBasisFixed(transform) {
        const basis = getTransformBasisLocal(transform);
        return {
            right: this.localDirectionToFixed(basis.right),
            left: this.localDirectionToFixed(basis.left),
            up: this.localDirectionToFixed(basis.up),
            down: this.localDirectionToFixed(basis.down),
            back: this.localDirectionToFixed(basis.back),
            forward: this.localDirectionToFixed(basis.forward),
        };
    }

    getForwardLocal(transform) {
        if (!transform || !transform.orientation) return { x: 0, y: 0, z: -1 };
        return getTransformBasisLocal(transform).forward;
    }

    setThirdPersonCamera(transform, state = {}) {
        if (!this.viewer || !this.ready || !transform || !transform.position) return;
        const Cesium = this.Cesium;
        const distance = Math.max(2.0, Math.min(120.0, state.distance || 16.0));
        const pitch = Math.max(-1.1, Math.min(1.15, state.pitch ?? 0.28));
        const yaw = Number.isFinite(state.yaw) ? state.yaw : 0;
        const lateral = Number.isFinite(state.lateral) ? state.lateral : 0;
        const height = Number.isFinite(state.height) ? state.height : 0.6;

        const cosPitch = Math.cos(pitch);
        const target = {
            x: transform.position.x,
            y: transform.position.y + height,
            z: transform.position.z,
        };
        const offset = {
            x: Math.sin(yaw) * cosPitch * distance + Math.cos(yaw) * lateral,
            y: Math.sin(pitch) * distance + height,
            z: Math.cos(yaw) * cosPitch * distance - Math.sin(yaw) * lateral,
        };
        const cameraLocal = {
            x: transform.position.x + offset.x,
            y: transform.position.y + offset.y,
            z: transform.position.z + offset.z,
        };
        const cameraSurfaceY = this.sampleHeightAtLocal(cameraLocal.x, cameraLocal.z, 0.8);
        if (Number.isFinite(cameraSurfaceY)) {
            cameraLocal.y = Math.max(cameraLocal.y, cameraSurfaceY + 4.0);
        }
        const directionLocal = normalize3({
            x: target.x - cameraLocal.x,
            y: target.y - cameraLocal.y,
            z: target.z - cameraLocal.z,
        });

        const destination = this.localToCartesian(cameraLocal);
        const direction = this.localDirectionToFixed(directionLocal);
        const up = this.localDirectionToFixed({ x: 0, y: 1, z: 0 });

        if (this.viewer.camera.frustum) {
            this.viewer.camera.frustum.near = 0.03;
            this.viewer.camera.frustum.far = 15000000;
        }
        this.viewer.camera.setView({
            destination,
            orientation: { direction, up },
        });
        this.viewer.scene.requestRender();
    }

    _componentDirectionToFixed(basis, component) {
        const Cesium = this.Cesium;
        const out = new Cesium.Cartesian3();
        const tmp = new Cesium.Cartesian3();

        Cesium.Cartesian3.multiplyByScalar(basis.right, component.x, out);
        Cesium.Cartesian3.multiplyByScalar(basis.up, component.y, tmp);
        Cesium.Cartesian3.add(out, tmp, out);
        Cesium.Cartesian3.multiplyByScalar(basis.back, component.z, tmp);
        Cesium.Cartesian3.add(out, tmp, out);
        return Cesium.Cartesian3.normalize(out, out);
    }

    _renderViewerNow(viewer = this.viewer) {
        if (!viewer || !viewer.scene) return;
        try {
            if (typeof viewer.render === 'function') {
                viewer.render();
                return;
            }
        } catch (error) {
            reportUserError('Viewer render failed', error, {
                key: 'viewer-render',
                intervalMs: 10000,
            });
        }
        try {
            if (typeof viewer.scene.render === 'function') {
                viewer.scene.render(viewer.clock ? viewer.clock.currentTime : undefined);
            }
        } catch (error) {
            reportUserError('Scene render failed', error, {
                key: 'scene-render',
                intervalMs: 10000,
            });
            viewer.scene.requestRender();
        }
    }

    _renderNow() {
        this._renderViewerNow(this.viewer);
    }

    async settleCurrentCameraView(options = {}) {
        if (!this.viewer || !this.ready) return false;
        const dwellMs = Math.max(0, Number.isFinite(options.dwellMs) ? options.dwellMs : 120);
        const timeoutMs = Math.max(500, Number.isFinite(options.timeoutMs) ? options.timeoutMs : 5000);
        const quietMs = Math.max(0, Number.isFinite(options.quietMs) ? options.quietMs : 350);
        const delay = (ms) => new Promise(resolve => window.setTimeout(resolve, ms));

        this.viewer.scene.requestRender();
        this._renderNow();
        if (dwellMs > 0) {
            await delay(dwellMs);
            this.viewer.scene.requestRender();
            this._renderNow();
        }
        return this.waitForTilesIdle(timeoutMs, quietMs);
    }

    _configurePanoramaTileset(tileset) {
        if (!tileset) return;

        const setIfPresent = (key, value) => {
            if (key in tileset) tileset[key] = value;
        };
        const baselineProfile = demoPerformance.config.profile === 'baseline';
        const leanStreaming = baselineProfile && this.panoramaLeanStreaming;

        setIfPresent('maximumScreenSpaceError', this.panoramaTileSSE);
        setIfPresent('cullRequestsWhileMoving', false);
        setIfPresent('preloadWhenHidden', baselineProfile && !leanStreaming);
        setIfPresent('preloadFlightDestinations', baselineProfile && !leanStreaming);
        setIfPresent('foveatedScreenSpaceError', false);
        setIfPresent(
            'dynamicScreenSpaceError',
            demoPerformance.config.dynamicSse !== 'off',
        );
        setIfPresent('dynamicScreenSpaceErrorDensity', 0.004);
        setIfPresent('dynamicScreenSpaceErrorFactor', 12);
        // demo30 uses normal parent-first replacement refinement. A coarse
        // parent remains visible until its children arrive, avoiding blank
        // blocks without expanding requests to siblings or hidden views.
        setIfPresent('loadSiblings', baselineProfile && !leanStreaming);
        setIfPresent('skipLevelOfDetail', baselineProfile && leanStreaming);
        setIfPresent('baseScreenSpaceError', baselineProfile && leanStreaming ? 1536 : 512);
        setIfPresent('skipScreenSpaceErrorFactor', baselineProfile && leanStreaming ? 18 : 8);
        setIfPresent('skipLevels', baselineProfile && leanStreaming ? 2 : 0);
        setIfPresent('immediatelyLoadDesiredLevelOfDetail', baselineProfile && !leanStreaming);
        setIfPresent('preferLeaves', baselineProfile && !leanStreaming);

        // The hidden viewer owns a separate cache from the main view. Keep it
        // large enough to avoid churn without switching policy while capturing.
        if ('maximumMemoryUsage' in tileset) tileset.maximumMemoryUsage = 1536;
        if ('cacheBytes' in tileset) tileset.cacheBytes = 1536 * 1024 * 1024;
        if ('maximumCacheOverflowBytes' in tileset) tileset.maximumCacheOverflowBytes = 512 * 1024 * 1024;
    }

    _destroyPanoramaCaptureViewer() {
        if (this._panoramaViewer && !this._panoramaViewer.isDestroyed()) {
            this._panoramaViewer.destroy();
        }
        if (this._panoramaContainer && this._panoramaContainer.parentNode) {
            this._panoramaContainer.parentNode.removeChild(this._panoramaContainer);
        }
        this._panoramaViewer = null;
        this._panoramaContainer = null;
        this._panoramaInitPromise = null;
        this._panoramaFaceSize = 0;
        this._panoramaTileset = null;
        this._panoramaTileLoadState = {
            pending: null,
            processing: null,
            errorCount: 0,
            lastErrorAt: null,
            lastErrorMessage: null,
        };
        this._panoramaCaptureActiveCount = 0;
        this._lastCompletedPanoramaCapture = null;
    }

    async _createPanoramaCaptureViewer(faceSize) {
        const Cesium = this.Cesium || requireCesium();
        this._destroyPanoramaCaptureViewer();

        const container = document.createElement('div');
        container.className = 'cesium-panorama-capture';
        Object.assign(container.style, {
            position: 'fixed',
            left: '0',
            top: '0',
            width: `${faceSize}px`,
            height: `${faceSize}px`,
            overflow: 'hidden',
            pointerEvents: 'none',
            opacity: '0.001',
            zIndex: '0',
        });
        document.body.appendChild(container);

        const viewer = new Cesium.Viewer(container, {
            animation: false,
            timeline: false,
            baseLayerPicker: false,
            geocoder: false,
            homeButton: false,
            infoBox: false,
            navigationHelpButton: false,
            sceneModePicker: false,
            selectionIndicator: false,
            fullscreenButton: false,
            scene3DOnly: true,
            shouldAnimate: false,
            globe: false,
            skyAtmosphere: new Cesium.SkyAtmosphere(),
            requestRenderMode: true,
            useDefaultRenderLoop: false,
            useBrowserRecommendedResolution: false,
            orderIndependentTranslucency: false,
            contextOptions: {
                webgl: {
                    alpha: false,
                    antialias: false,
                    preserveDrawingBuffer: true,
                    powerPreference: 'high-performance',
                    failIfMajorPerformanceCaveat: false,
                },
            },
        });

        viewer.scene.fog.enabled = false;
        viewer.scene.highDynamicRange = false;
        if ('resolutionScale' in viewer) viewer.resolutionScale = 1;
        if ('msaaSamples' in viewer.scene) viewer.scene.msaaSamples = 1;
        if (viewer.scene.postProcessStages && viewer.scene.postProcessStages.fxaa) {
            viewer.scene.postProcessStages.fxaa.enabled = false;
        }

        const tileset = await this._createGoogleTileset(null);
        this._configurePanoramaTileset(tileset);
        this._panoramaTileset = tileset;
        this._panoramaTileLoadState = {
            pending: null,
            processing: null,
            errorCount: 0,
            lastErrorAt: null,
            lastErrorMessage: null,
        };
        this._wireTilesetDiagnostics(null, tileset, this._panoramaTileLoadState, 'Panorama Google 3D Tiles');
        viewer.scene.primitives.add(tileset);
        viewer.resize();

        this._panoramaViewer = viewer;
        this._panoramaContainer = container;
        this._panoramaFaceSize = faceSize;
        return viewer;
    }

    async _ensurePanoramaCaptureViewer(faceSize) {
        if (
            this._panoramaViewer &&
            !this._panoramaViewer.isDestroyed() &&
            this._panoramaFaceSize === faceSize
        ) {
            return this._panoramaViewer;
        }

        if (!this._panoramaInitPromise) {
            const initTimeoutMs = 20000;
            const initPromise = this._createPanoramaCaptureViewer(faceSize);
            const trackedPromise = initPromise.finally(() => {
                if (this._panoramaInitPromise === trackedPromise) {
                    this._panoramaInitPromise = null;
                }
            });
            // Keep the real initialization promise authoritative after a
            // caller times out. Clearing it on Promise.race timeout allowed a
            // slow Chrome/network startup to create overlapping Cesium/WebGL
            // panorama viewers, multiplying render and tile-streaming cost.
            this._panoramaInitPromise = trackedPromise;
        }

        return Promise.race([
            this._panoramaInitPromise,
            new Promise((_, reject) => setTimeout(
                () => reject(new Error('Panorama capture viewer init timed out')),
                20000,
            )),
        ]);
    }

    _getPanoramaProjector() {
        if (this._panoramaProjector === false) return null;
        if (this._panoramaProjector) return this._panoramaProjector;
        try {
            this._panoramaProjector = new PanoramaEquirectProjector();
            return this._panoramaProjector;
        } catch (error) {
            reportUserError('GPU panorama projection unavailable', error, {
                key: 'gpu-panorama-projection',
                intervalMs: 10000,
            });
            this._panoramaProjector = false;
            return null;
        }
    }

    async warmPanoramaCaptureViewer(faceSize = 256) {
        if (!this.viewer || !this.ready) return false;
        const size = Math.max(64, Math.round(faceSize || 256));
        await this._ensurePanoramaCaptureViewer(size);
        return !!this._getPanoramaProjector();
    }

    async _capturePanoramaHybridWithViewerAsync(viewer, transform, width, height, faceSize, verticalFovDeg = 180, options = {}) {
        const totalStartedAt = performance.now();
        const projector = this._getPanoramaProjector();
        if (!projector) {
            return {
                canvas: null,
                complete: false,
                ready: false,
                faces: PANORAMA_FACE_DEFS.length,
            };
        }
        if (projector.readyFaces) projector.readyFaces.clear();

        const camera = viewer.camera;
        const frustum = camera.frustum;
        const saved = {
            fov: frustum && 'fov' in frustum ? frustum.fov : undefined,
            near: frustum && 'near' in frustum ? frustum.near : undefined,
            far: frustum && 'far' in frustum ? frustum.far : undefined,
        };
        const basis = this.getTransformBasisFixed(transform);
        const destination = this.localToCartesian(transform.position);
        const faceFovDeg = Math.max(90, Math.min(170, Number(options.faceFovDeg) || 130));
        const topPoleGuardDeg = Math.max(0, Math.min(45, Number(options.topPoleGuardDeg) || 0));
        const bottomPoleGuardDeg = Math.max(0, Math.min(45, Number(options.bottomPoleGuardDeg) || 0));
        const frameDelayMs = Math.max(0, Math.min(1000, Number(options.frameDelayMs) || 0));
        const tileTimeoutMs = Math.max(0, Math.min(120000, Number(options.tileTimeoutMs) || 0));
        const tileQuietMs = Math.max(0, Math.min(5000, Number(options.tileQuietMs) || 0));
        const captureAnyway = !!options.captureAnyway;
        const continueOnTileTimeout = options.continueOnTileTimeout === true;
        const signal = options.signal || null;
        const facesPerSlice = Math.max(1, Math.min(
            PANORAMA_FACE_DEFS.length,
            Math.round(Number(options.facesPerSlice) || 2)
        ));
        const progressCb = typeof options.progressCb === 'function' ? options.progressCb : null;
        const sleep = (ms) => new Promise(resolve => window.setTimeout(resolve, ms));
        // Yield to the browser task queue between face batches without waiting
        // for the next visible animation frame. Waiting on requestAnimationFrame
        // coupled perception latency to main-view tile stalls (up to hundreds
        // of milliseconds) even though the six actual renders took ~30 ms.
        const yieldFrame = () => new Promise(resolve => window.setTimeout(resolve, 0));
        const throwIfAborted = () => {
            if (!signal?.aborted) return;
            const error = new Error(String(signal.reason || 'panorama capture aborted'));
            error.name = 'AbortError';
            throw error;
        };
        let sceneRenderMs = 0;
        let tileWaitMs = 0;
        let waitRerenderMs = 0;
        let faceUploadMs = 0;
        let projectMs = 0;
        let schedulerMs = 0;
        const captureTimings = () => ({
            scene_render: sceneRenderMs,
            tile_wait: tileWaitMs,
            wait_rerender: waitRerenderMs,
            face_upload: faceUploadMs,
            project: projectMs,
            scheduler: schedulerMs,
            // Backward-compatible aggregate fields. `render` deliberately
            // excludes tile quiet time, scheduler waits and texture uploads.
            render: sceneRenderMs + waitRerenderMs,
            scheduler_yield: schedulerMs,
            total: performance.now() - totalStartedAt,
        });
        const trackTileReadiness = !captureAnyway;
        const faceTileReadiness = [];
        let capturedFaces = 0;
        const captureRevision = Number.isSafeInteger(this._panoramaCaptureRevision)
            ? this._panoramaCaptureRevision + 1
            : 1;
        this._panoramaCaptureRevision = captureRevision;
        const captureTileset = this._panoramaTileset;
        const captureLoadState = this._panoramaTileLoadState;
        const tileErrorCountAtStart = Math.max(0, Number(captureLoadState?.errorCount) || 0);
        this._panoramaCaptureActiveCount = Math.max(0, Number(this._panoramaCaptureActiveCount) || 0) + 1;

        const readinessSnapshot = (captureComplete = false) => {
            const frozenFaces = Object.freeze(faceTileReadiness.slice());
            const readyFaces = frozenFaces.reduce(
                (count, face) => count + (face.readyWhenCopied === true ? 1 : 0),
                0,
            );
            const allFaceFlagsReady = captureComplete
                && frozenFaces.length === PANORAMA_FACE_DEFS.length
                && readyFaces === PANORAMA_FACE_DEFS.length;
            const rawLastErrorAt = captureLoadState?.lastErrorAt;
            const lastErrorAt = Number(rawLastErrorAt);
            const tileError = (
                (Math.max(0, Number(captureLoadState?.errorCount) || 0) > tileErrorCountAtStart)
                || (rawLastErrorAt != null
                    && Number.isFinite(lastErrorAt)
                    && lastErrorAt >= totalStartedAt - 5000)
            );
            const allFacesTileReady = allFaceFlagsReady && !tileError;
            return Object.freeze({
                faceTileReadiness: frozenFaces,
                readyFaces,
                allFacesTileReady,
                readinessReason: tileError
                    ? 'tile-error'
                    : allFacesTileReady
                    ? 'tiles-ready'
                    : captureComplete
                    ? 'tiles-partial'
                    : 'capture-incomplete',
                tileError,
            });
        };

        try {
            if (frustum) {
                if ('fov' in frustum) frustum.fov = faceFovDeg * Math.PI / 180;
                if ('near' in frustum) frustum.near = 0.03;
                if ('far' in frustum) frustum.far = this.panoramaFarMeters || 1200;
            }

            for (let faceIndex = 0; faceIndex < PANORAMA_FACE_DEFS.length; faceIndex++) {
                throwIfAborted();
                const faceDef = PANORAMA_FACE_DEFS[faceIndex];
                if (progressCb) progressCb(`face ${faceIndex + 1}/${PANORAMA_FACE_DEFS.length} ${faceDef.name}`);
                camera.setView({
                    destination,
                    orientation: {
                        direction: this._componentDirectionToFixed(basis, faceDef.dir),
                        up: this._componentDirectionToFixed(basis, faceDef.up),
                    },
                });
                viewer.scene.requestRender();
                let sceneRenderStartedAt = performance.now();
                this._renderViewerNow(viewer);
                sceneRenderMs += performance.now() - sceneRenderStartedAt;
                if (frameDelayMs > 0) {
                    await sleep(frameDelayMs);
                    throwIfAborted();
                    viewer.scene.requestRender();
                    sceneRenderStartedAt = performance.now();
                    this._renderViewerNow(viewer);
                    sceneRenderMs += performance.now() - sceneRenderStartedAt;
                }
                let faceTilesReady = true;
                if (trackTileReadiness && tileTimeoutMs > 0) {
                    const tileWaitStartedAt = performance.now();
                    const waitRenderTimings = { renderMs: 0, renderCount: 0 };
                    faceTilesReady = await this.waitForTilesIdle(
                        tileTimeoutMs,
                        tileQuietMs,
                        this._panoramaTileset,
                        this._panoramaTileLoadState,
                        viewer,
                        waitRenderTimings,
                        signal
                    );
                    const waitElapsedMs = performance.now() - tileWaitStartedAt;
                    const waitRenderElapsedMs = Math.max(0, Number(waitRenderTimings.renderMs) || 0);
                    waitRerenderMs += waitRenderElapsedMs;
                    tileWaitMs += Math.max(0, waitElapsedMs - waitRenderElapsedMs);
                    throwIfAborted();
                    if (!faceTilesReady && !continueOnTileTimeout) {
                        const readiness = readinessSnapshot(false);
                        return {
                            canvas: null,
                            complete: false,
                            ready: false,
                            loadingTiles: true,
                            faceIndex,
                            faces: PANORAMA_FACE_DEFS.length,
                            ...readiness,
                            timings_ms: captureTimings(),
                        };
                    }
                } else if (trackTileReadiness) {
                    faceTilesReady = !!captureTileset && captureTileset.tilesLoaded === true;
                }
                const faceUploadStartedAt = performance.now();
                projector.updateFace(faceDef.name, viewer.scene.canvas);
                faceUploadMs += performance.now() - faceUploadStartedAt;
                capturedFaces++;
                if (trackTileReadiness) {
                    faceTileReadiness.push(Object.freeze({
                        face: faceDef.name,
                        readyWhenCopied: faceTilesReady,
                    }));
                }

                if ((faceIndex + 1) % facesPerSlice === 0 && faceIndex + 1 < PANORAMA_FACE_DEFS.length) {
                    const yieldStartedAt = performance.now();
                    await yieldFrame();
                    schedulerMs += performance.now() - yieldStartedAt;
                }
            }

            throwIfAborted();
            const projectStartedAt = performance.now();
            const canvas = projector.render(width, height, verticalFovDeg, faceFovDeg, topPoleGuardDeg, bottomPoleGuardDeg);
            projectMs = performance.now() - projectStartedAt;
            const complete = !!canvas
                && capturedFaces === PANORAMA_FACE_DEFS.length;
            const readiness = trackTileReadiness ? readinessSnapshot(complete) : null;
            if (canvas && viewer === this._panoramaViewer && captureTileset === this._panoramaTileset
                && captureRevision > (this._lastCompletedPanoramaCapture?.revision || 0)) {
                this._lastCompletedPanoramaCapture = Object.freeze({
                    revision: captureRevision,
                    viewer,
                    tileset: captureTileset,
                    transform: Object.freeze({
                        position: Object.freeze({ ...transform.position }),
                        orientation: Object.freeze({ ...transform.orientation }),
                    }),
                    width,
                    height,
                    faceSize,
                    verticalFovDeg,
                    complete,
                    ready: readiness ? readiness.allFacesTileReady : complete,
                    faceTileReadiness: readiness?.faceTileReadiness || Object.freeze([]),
                    readyFaces: readiness?.readyFaces ?? (complete ? PANORAMA_FACE_DEFS.length : 0),
                    allFacesTileReady: readiness?.allFacesTileReady ?? null,
                    readinessReason: readiness?.readinessReason || (
                        complete ? 'capture-complete' : 'capture-incomplete'
                    ),
                    tileError: readiness?.tileError === true,
                    completedAt: performance.now(),
                });
            }
            return {
                canvas,
                complete,
                ready: readiness ? readiness.allFacesTileReady : complete,
                ...(readiness || {}),
                faces: PANORAMA_FACE_DEFS.length,
                timings_ms: captureTimings(),
            };
        } finally {
            this._panoramaCaptureActiveCount = Math.max(0, this._panoramaCaptureActiveCount - 1);
            if (frustum) {
                if (saved.fov !== undefined && 'fov' in frustum) frustum.fov = saved.fov;
                if (saved.near !== undefined && 'near' in frustum) frustum.near = saved.near;
                if (saved.far !== undefined && 'far' in frustum) frustum.far = saved.far;
            }
        }
    }

    async preloadPanoramaAtTransform(transform, options = {}) {
        if (!this.viewer || !this.ready || !transform || !transform.position || !transform.orientation) {
            return { canvas: null, complete: false, ready: false };
        }

        const width = Math.max(256, Math.round(options.width || 512));
        const height = Math.max(128, Math.round(options.height || Math.round(width / 2)));
        const faceSize = Math.max(64, Math.round(options.faceSize || 128));
        const verticalFovDeg = Math.max(1, Math.min(180, Number(options.verticalFovDeg) || 180));
        const viewer = await this._ensurePanoramaCaptureViewer(faceSize);
        return this._capturePanoramaHybridWithViewerAsync(
            viewer,
            transform,
            width,
            height,
            faceSize,
            verticalFovDeg,
            {
                faceFovDeg: options.faceFovDeg,
                topPoleGuardDeg: options.topPoleGuardDeg,
                bottomPoleGuardDeg: options.bottomPoleGuardDeg,
                frameDelayMs: options.frameDelayMs,
                tileTimeoutMs: options.tileTimeoutMs,
                tileQuietMs: options.tileQuietMs,
                captureAnyway: options.captureAnyway,
                continueOnTileTimeout: options.continueOnTileTimeout,
                facesPerSlice: options.facesPerSlice,
                signal: options.signal,
                progressCb: options.progressCb,
            },
        );
    }

    async capturePanoramaIncrementalAsync(transform, options = {}) {
        if (!this.viewer || !this.ready || !transform || !transform.position || !transform.orientation) {
            return { canvas: null, complete: false, ready: false };
        }

        const width = Math.max(256, Math.round(options.width || 512));
        const height = Math.max(128, Math.round(options.height || Math.round(width / 2)));
        const faceSize = Math.max(64, Math.round(options.faceSize || 128));
        const verticalFovDeg = Math.max(1, Math.min(180, Number(options.verticalFovDeg) || 180));
        const viewer = await this._ensurePanoramaCaptureViewer(faceSize);
        return this._capturePanoramaHybridWithViewerAsync(viewer, transform, width, height, faceSize, verticalFovDeg, {
            faceFovDeg: options.faceFovDeg,
            topPoleGuardDeg: options.topPoleGuardDeg,
            bottomPoleGuardDeg: options.bottomPoleGuardDeg,
            frameDelayMs: options.frameDelayMs,
            tileTimeoutMs: options.tileTimeoutMs,
            tileQuietMs: options.tileQuietMs,
            captureAnyway: options.captureAnyway,
            facesPerSlice: options.facesPerSlice,
            signal: options.signal,
            progressCb: options.progressCb,
        });
    }

    async capturePanoramaAsync(transform, options = {}) {
        return this.capturePanoramaIncrementalAsync(transform, options);
    }

    describeLocal(local) {
        if (!local) return '';
        const carto = this.localToCartographic(local);
        return [
            `lon ${this.Cesium.Math.toDegrees(carto.longitude).toFixed(6)}`,
            `lat ${this.Cesium.Math.toDegrees(carto.latitude).toFixed(6)}`,
            `alt ${local.y.toFixed(1)} m`,
        ].join(' | ');
    }

    describeSpawn(local, altitudeMeters) {
        if (!local) return '';
        const carto = this.localToCartographic({ x: local.x, y: 0, z: local.z });
        return [
            `lon ${this.Cesium.Math.toDegrees(carto.longitude).toFixed(6)}`,
            `lat ${this.Cesium.Math.toDegrees(carto.latitude).toFixed(6)}`,
            `alt ${Number(altitudeMeters || 0).toFixed(1)} m`,
        ].join(' | ');
    }

    /**
     * Sample sparse metric-depth anchors via Cesium ray-casting against
     * the currently-loaded 3D Tiles (Google Photorealistic).
     *
     * Each anchor is an ERP grid cell centre projected through the exact same
     * sensor-NWU → cubemap-component → capture-transform path as the RGB
     * panorama, then ray-cast against that panorama capture viewer's own scene
     * and tileset.  The returned object contains both successful hits and
     * per-anchor failure reasons so the downstream metric-fitting stage can
     * decide how to handle missing data.
     *
     * @param {object} transform  – { position: {x,y,z}, orientation: quaternion }
     * @param {object} [options]
     * @param {number} [options.gridCols=16]
     * @param {number} [options.gridRows=8]
     * @param {number} [options.maxRangeM=100]
     * @param {number} [options.excludeTopDeg=15]   – skip anchors within N° of top pole
     * @param {number} [options.excludeBottomDeg=5] – skip anchors within N° of bottom pole
     * @param {number} [options.imageWidth=384]     – ERP width for geometry
     * @param {number} [options.imageHeight=192]    – ERP height for geometry
     * @param {number} [options.verticalFovDeg=180]
     * @param {string} options.sessionId            – stable collection-session ID
     * @param {string} [options.locationId]         – stable physical-site ID for held-out validation
     * @param {string} [options.captureId]          – ID shared by RGB/raw/anchor artifacts
     * @param {string} [options.frameId]            – perception frame that supplied the RGB
     * @returns {{ anchors: Array, failures: Array, metadata: object }}
     */
    sampleMetricDepthAnchors(transform, options = {}) {
        const defaults = {
            gridCols: 16, gridRows: 8,
            maxRangeM: 100,
            excludeTopDeg: 15, excludeBottomDeg: 5,
            imageWidth: 384, imageHeight: 192,
            verticalFovDeg: 180,
        };
        const opts = { ...defaults, ...options };

        if (!transform || !transform.position || !transform.orientation) {
            throw new TypeError('sampleMetricDepthAnchors requires a panorama capture transform');
        }
        const positionValues = [transform.position.x, transform.position.y, transform.position.z];
        const orientationValues = [
            transform.orientation.x,
            transform.orientation.y,
            transform.orientation.z,
            transform.orientation.w,
        ];
        if (![...positionValues, ...orientationValues].every(Number.isFinite)) {
            throw new TypeError('sampleMetricDepthAnchors requires a finite capture transform');
        }
        const quaternionNorm = Math.hypot(...orientationValues);
        if (Math.abs(quaternionNorm - 1) > 1e-3) {
            throw new RangeError('sampleMetricDepthAnchors requires a unit capture quaternion');
        }
        for (const key of ['gridCols', 'gridRows', 'imageWidth', 'imageHeight']) {
            if (!Number.isInteger(opts[key]) || opts[key] <= 0) {
                throw new RangeError(`${key} must be a positive integer`);
            }
        }
        for (const key of ['maxRangeM', 'excludeTopDeg', 'excludeBottomDeg', 'verticalFovDeg']) {
            if (!Number.isFinite(opts[key])) throw new RangeError(`${key} must be finite`);
        }
        if (opts.maxRangeM <= 0) throw new RangeError('maxRangeM must be positive');
        if (opts.excludeTopDeg < 0 || opts.excludeTopDeg >= 90
            || opts.excludeBottomDeg < 0 || opts.excludeBottomDeg >= 90) {
            throw new RangeError('ERP pole exclusions must be in [0, 90) degrees');
        }
        if (opts.verticalFovDeg <= 0 || opts.verticalFovDeg > 180) {
            throw new RangeError('verticalFovDeg must be in (0, 180]');
        }
        const identity = {};
        for (const key of ['sessionId', 'captureId', 'locationId', 'frameId']) {
            if (typeof opts[key] !== 'string' || opts[key].trim() === '') {
                throw new TypeError(`${key} is required for metric anchor capture`);
            }
            identity[key] = opts[key];
        }
        const vfovRad = opts.verticalFovDeg / 180 * Math.PI;
        const raySource = this._metricAnchorRaySource(transform, opts);
        const tilesReady = this._tilesReady(raySource.tileset);

        const anchors = [];
        const failures = [];
        const basis = getTransformBasisLocal(transform);
        const samples = sampleAnchorDirections(
            opts.gridCols,
            opts.gridRows,
            opts.imageWidth,
            opts.imageHeight,
            vfovRad
        );

        for (const sample of samples) {
            const { col, row, u, v, yaw, pitch: pitchRad } = sample;
            const pitchDeg = pitchRad * 180 / Math.PI;

            // Pole exclusion
            if (pitchDeg > (90 - opts.excludeTopDeg) || pitchDeg < (-90 + opts.excludeBottomDeg)) {
                failures.push({ col, row, u, v, reason: 'pole_excluded', pitchDeg });
                continue;
            }

            // The RGB projector first maps canonical sensor NWU into its
            // cubemap component axes, then rotates those axes by exactly
            // this capture transform.  Metric rays must follow the same
            // path or anchors and pixels describe different directions.
            const componentDirection = erpDirectionToComponent(sample);
            const dir = componentDirectionToLocal(basis, componentDirection);
            if (Math.hypot(dir.x, dir.y, dir.z) < 1e-9) {
                failures.push({ col, row, u, v, reason: 'zero_direction' });
                continue;
            }

            // Ray cast
            const origin = transform.position;
            const hit = this._pickLocalRayFromViewer(
                raySource.viewer,
                origin,
                dir,
                opts.maxRangeM,
            );

            if (!hit) {
                // Distinguish failure modes
                const reason = tilesReady
                    ? 'no_hit' : 'tile_not_ready';
                failures.push({ col, row, u, v, reason });
                continue;
            }

            if (!Number.isFinite(hit.distance) || hit.distance <= 0 || hit.distance > opts.maxRangeM) {
                failures.push({ col, row, u, v, reason: 'out_of_range', distance: hit.distance });
                continue;
            }

            anchors.push({
                col, row,
                u, v,
                yawDeg: yaw * 180 / Math.PI,
                pitchDeg,
                sensorDirection: { x: sample.dx, y: sample.dy, z: sample.dz },
                componentDirection,
                direction: dir,
                distance: hit.distance,
                position: hit.position,
            });
        }

        return {
            anchors,
            failures,
            metadata: {
                schemaVersion: 1,
                identity,
                image: {
                    width: opts.imageWidth,
                    height: opts.imageHeight,
                    pixelCoordinateConvention: 'integer-pixel-centres',
                },
                erp: {
                    verticalFovDeg: opts.verticalFovDeg,
                    sensorFrame: 'NWU(+x forward,+y left,+z up)',
                    componentFrame: '(+x body-left,+y up,+z back)',
                },
                sampling: {
                    gridCols: opts.gridCols,
                    gridRows: opts.gridRows,
                    maxRangeM: opts.maxRangeM,
                    excludeTopDeg: opts.excludeTopDeg,
                    excludeBottomDeg: opts.excludeBottomDeg,
                },
                transform: JSON.parse(JSON.stringify(transform)),
                totalCells: opts.gridCols * opts.gridRows,
                validAnchors: anchors.length,
                failureCount: failures.length,
                raycastSource: 'panorama-capture-viewer',
                tilesetSharedWithRgb: true,
                panoramaFaceSize: this._panoramaFaceSize || null,
                panoramaCaptureRevision: raySource.capture.revision,
                panoramaSourceImage: {
                    width: raySource.capture.width,
                    height: raySource.capture.height,
                    verticalFovDeg: raySource.capture.verticalFovDeg,
                },
                panoramaFaceTileReadiness: raySource.capture.faceTileReadiness,
                tileState: tilesReady ? 'ready' : 'loading',
                timestamp: Date.now(),
            },
        };
    }

    _metricAnchorRaySource(transform, options) {
        const viewer = this._panoramaViewer;
        const tileset = this._panoramaTileset;
        if (!viewer || (typeof viewer.isDestroyed === 'function' && viewer.isDestroyed()) || !viewer.scene) {
            throw new Error('panorama capture viewer unavailable for metric anchor sampling');
        }
        if (!tileset) {
            throw new Error('panorama capture tileset unavailable for metric anchor sampling');
        }
        if (this._panoramaCaptureActiveCount > 0) {
            throw new Error('panorama capture is in progress; retry metric anchor sampling after the frame is frozen');
        }
        const capture = this._lastCompletedPanoramaCapture;
        if (!capture || capture.viewer !== viewer || capture.tileset !== tileset) {
            throw new Error('no completed RGB panorama capture matches the metric anchor ray source');
        }
        if (!captureTransformsEquivalent(capture.transform, transform)) {
            throw new Error('metric anchor transform does not match the completed RGB panorama capture');
        }
        if (Math.abs(capture.verticalFovDeg - options.verticalFovDeg) > 1e-6) {
            throw new Error('metric anchor vertical FOV does not match the completed RGB panorama capture');
        }
        if (capture.allFacesTileReady !== true) {
            throw new Error('RGB panorama was copied before every cubemap face reported tiles ready');
        }
        // The upload canvas rounds width and height independently. Accept a
        // target only when both rounded dimensions can come from one common
        // source scale; this permits half-pixel rounding without admitting a
        // genuinely distorted ERP aspect ratio.
        const widthScaleRange = [
            (options.imageWidth - 0.5) / capture.width,
            (options.imageWidth + 0.5) / capture.width,
        ];
        const heightScaleRange = [
            (options.imageHeight - 0.5) / capture.height,
            (options.imageHeight + 0.5) / capture.height,
        ];
        const scaleRangesOverlap = Math.max(widthScaleRange[0], heightScaleRange[0])
            <= Math.min(widthScaleRange[1], heightScaleRange[1]) + 1e-12;
        if (!scaleRangesOverlap) {
            throw new Error('metric anchor image aspect does not match the completed RGB panorama capture');
        }
        const primitives = viewer.scene.primitives;
        if (primitives && typeof primitives.contains === 'function' && !primitives.contains(tileset)) {
            throw new Error('panorama capture tileset is not attached to the RGB capture viewer');
        }
        return { viewer, tileset, capture };
    }

    _pickLocalRayFromViewer(viewer, originLocal, directionLocal, maxDistance) {
        if (!viewer || !viewer.scene) return null;
        const Cesium = this.Cesium;
        const scene = viewer.scene;
        if (!Cesium || typeof scene.pickFromRay !== 'function') return null;

        const dir = normalize3(directionLocal);
        if (Math.hypot(dir.x, dir.y, dir.z) < 1e-6) return null;

        const origin = this.localToCartesian(originLocal);
        const direction = this.localDirectionToFixed(dir);
        const ray = new Cesium.Ray(origin, direction);

        let hit;
        try {
            // The dedicated capture scene contains only its Google 3D Tileset,
            // so exclusions from the main viewer must not be mixed into this pick.
            hit = scene.pickFromRay(ray);
        } catch (error) {
            reportUserError('Panorama scene pickFromRay failed during metric anchor capture', error, {
                key: 'panorama-anchor-pick-from-ray',
                intervalMs: 10000,
            });
            return null;
        }
        if (!hit || !Cesium.defined(hit.position)) return null;

        const local = this.cartesianToLocal(hit.position);
        const distance = Math.hypot(
            local.x - originLocal.x,
            local.y - originLocal.y,
            local.z - originLocal.z,
        );
        if (!Number.isFinite(distance) || distance <= 0 || distance > maxDistance) return null;
        return { position: local, distance };
    }

    _tilesReady(tileset = this.tileset) {
        if (tileset && typeof tileset.tilesLoaded !== 'undefined') {
            return tileset.tilesLoaded;
        }
        return true; // optimistic
    }
}
