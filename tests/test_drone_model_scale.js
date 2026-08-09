/**
 * 无人机视觉模型尺寸一致性测试。
 * 运行: node tests/test_drone_model_scale.js
 *
 * 直接解析 CesiumDrone.glb 的顶点包围盒，校验默认 droneScale 渲染出来的
 * 机体跨度确实对应约 0.4 m 的物理半径。换模型或改默认缩放都会让本测试失效。
 *
 * 背景：默认缩放曾是 1.35，渲染跨度 6.3 m（等效半径 3.15 m），
 * 而物理碰撞半径只有 0.6 m，视觉比物理大一个数量级。
 */

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');

let passed = 0, failed = 0;
function assert(cond, msg) {
    if (cond) { passed++; return; }
    failed++;
    console.error(`FAIL: ${msg}`);
}

/** 解析 glTF 二进制容器的 JSON chunk，求所有 POSITION 访问器的并集包围盒。 */
function glbBoundingBox(path) {
    const buf = readFileSync(path);
    if (buf.readUInt32LE(0) !== 0x46546c67) throw new Error('不是合法的 glb 文件');
    const jsonLen = buf.readUInt32LE(12);
    const gltf = JSON.parse(buf.subarray(20, 20 + jsonLen).toString('utf8'));

    const lo = [Infinity, Infinity, Infinity];
    const hi = [-Infinity, -Infinity, -Infinity];
    for (const mesh of gltf.meshes || []) {
        for (const prim of mesh.primitives || []) {
            const acc = gltf.accessors[prim.attributes.POSITION];
            if (!acc || !acc.min || !acc.max) continue;
            for (let i = 0; i < 3; i++) {
                lo[i] = Math.min(lo[i], acc.min[i]);
                hi[i] = Math.max(hi[i], acc.max[i]);
            }
        }
    }
    return { size: [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]] };
}

// ── 从源码读出默认缩放（cesium-world.js 依赖 Cesium 全局，无法直接 import）──
function readDefaultDroneScale() {
    const src = readFileSync(join(ROOT, 'src', 'cesium-world.js'), 'utf8');
    const m = src.match(/urlNumber\(\s*'droneScale'\s*,\s*([0-9.]+)\s*\)/);
    if (!m) throw new Error('未能在 cesium-world.js 中找到 droneScale 默认值');
    return parseFloat(m[1]);
}

const TARGET_RADIUS_M = 0.4;      // 目标物理半径
const TOLERANCE = 0.15;           // 允许 15% 偏差

const { size } = glbBoundingBox(join(ROOT, 'asset', 'models', 'CesiumDrone.glb'));
const span = Math.max(size[0], size[2]);   // 水平最大跨度
const scale = readDefaultDroneScale();
const renderedRadius = span * scale / 2;

console.log(`模型原始尺寸 (X,Y,Z) = ${size.map(v => v.toFixed(3)).join(' × ')}`);
console.log(`水平最大跨度 = ${span.toFixed(3)} 单位`);
console.log(`默认 droneScale = ${scale}`);
console.log(`渲染等效半径 = ${renderedRadius.toFixed(3)} m (目标 ${TARGET_RADIUS_M} m)`);

assert(Number.isFinite(span) && span > 0, '包围盒解析应得到正的跨度');

const relErr = Math.abs(renderedRadius - TARGET_RADIUS_M) / TARGET_RADIUS_M;
assert(relErr <= TOLERANCE,
    `渲染半径 ${renderedRadius.toFixed(3)}m 与目标 ${TARGET_RADIUS_M}m 偏差 ${(relErr * 100).toFixed(1)}%，应 <= ${TOLERANCE * 100}%`);

// 回归：旧默认值 1.35 必须不再通过
{
    const oldRadius = span * 1.35 / 2;
    assert(Math.abs(oldRadius - TARGET_RADIUS_M) / TARGET_RADIUS_M > TOLERANCE,
        `回归检查：旧默认 1.35 对应半径 ${oldRadius.toFixed(2)}m，本测试必须能拒绝它`);
}

console.log(`\n无人机模型尺寸测试: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
