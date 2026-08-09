/**
 * SO3 推力矢量倾角限幅测试 —— 纯函数，不需要 DOM / Cesium / 推理服务。
 * 运行: node tests/test_so3_tilt_limit.js
 *
 * 对照实现: YOPO_360_v15/Controller/src/so3_control/src/NetworkControl.cpp
 *           的 get_Q_from_ACC()，以及 SO3Control.cpp 的 calculateControl()。
 *
 * 核心不变量：限幅**只能削弱加速度项，不得削弱重力补偿**。
 * 历史 bug 是对整个 F_d 等比缩放（FdX*=s; FdY*=s; FdZ*=s），
 * 把 m*g 一起缩掉 → 垂直力低于自重 → 设目标即掉高。
 */

globalThis.pc = {
    Quat: class { constructor() { this.x = 0; this.y = 0; this.z = 0; this.w = 1; } },
    Mat4: class {},
    Vec3: class { constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; } },
};
globalThis.pc.Vec3.ZERO = new globalThis.pc.Vec3(0, 0, 0);
globalThis.pc.Vec3.ONE = new globalThis.pc.Vec3(1, 1, 1);

const { limitTiltPreservingGravity } = await import('../src/drone.js');

let passed = 0, failed = 0;
function assert(cond, msg) {
    if (cond) { passed++; return; }
    failed++;
    console.error(`FAIL: ${msg}`);
}

const G = 9.81;
const MASS_KG = 0.98;                 // YOPO Hummingbird
const WEIGHT = MASS_KG * G;           // 9.61 N
const tiltOf = (F) => Math.atan2(Math.hypot(F.x, F.z), F.y) * 180 / Math.PI;

// ── 1. 未超限时原样透传 ──
{
    const F = limitTiltPreservingGravity(2.0, 20.0, 1.0, WEIGHT, 60);
    assert(F.x === 2.0 && F.y === 20.0 && F.z === 1.0, '倾角未超限应原样返回');
}

// ── 2. 重力补偿不被削弱（关键不变量）──
{
    // 复现故障场景：错误参考轨迹导致的巨大水平力需求
    const F = limitTiltPreservingGravity(-190.0, WEIGHT, 200.0, WEIGHT, 60);
    assert(F.y >= WEIGHT - 1e-9,
        `垂直力 ${F.y.toFixed(3)}N 必须 >= 自重 ${WEIGHT.toFixed(3)}N`);

    // 旧的等比缩放做法作为对照：必然低于自重
    const Fmax = 2600 * G / 1000 * 0.95;
    const n = Math.hypot(-190.0, WEIGHT, 200.0);
    const oldFdY = WEIGHT * (Fmax / n);
    assert(oldFdY < WEIGHT,
        `对照：旧等比缩放给出垂直力 ${oldFdY.toFixed(2)}N < 自重，证明该做法必然掉高`);
}

// ── 3. 输出倾角恰好被限制在上限 ──
{
    for (const maxTilt of [30, 45, 60]) {
        const F = limitTiltPreservingGravity(-190.0, WEIGHT, 200.0, WEIGHT, maxTilt);
        const t = tiltOf(F);
        assert(Math.abs(t - maxTilt) < 1e-6,
            `上限 ${maxTilt}°: 输出倾角 ${t.toFixed(4)}° 应贴合上限`);
    }
}

// ── 4. 随机属性测试：两条不变量在整个输入域上都成立 ──
{
    let violGravity = 0, violTilt = 0, n = 0;
    // 确定性伪随机，保证可复现
    let seed = 12345;
    const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };

    for (let i = 0; i < 20000; i++) {
        const FdX = (rnd() - 0.5) * 600;
        const FdZ = (rnd() - 0.5) * 600;
        // 只测试"指令垂直力不低于自重"的情形（即未主动要求下降）
        const FdY = WEIGHT + rnd() * 300;
        const maxTilt = 20 + rnd() * 50;
        const F = limitTiltPreservingGravity(FdX, FdY, FdZ, WEIGHT, maxTilt);
        n++;
        if (F.y < WEIGHT - 1e-6) violGravity++;
        if (tiltOf(F) > maxTilt + 1e-6) violTilt++;
    }
    assert(violGravity === 0, `${n} 组随机输入中有 ${violGravity} 组削弱了重力补偿`);
    assert(violTilt === 0, `${n} 组随机输入中有 ${violTilt} 组超出倾角上限`);
}

// ── 5. 退化输入回退到悬停，不产生 NaN ──
{
    for (const [x, y, z, label] of [
        [0, 0, 0, '零力'],
        [NaN, NaN, NaN, 'NaN'],
        [1e-12, 1e-12, 1e-12, '近零力'],
    ]) {
        const F = limitTiltPreservingGravity(x, y, z, WEIGHT, 60);
        assert(Number.isFinite(F.x) && Number.isFinite(F.y) && Number.isFinite(F.z),
            `${label} 输入不得产生 NaN`);
        assert(Math.abs(F.y - WEIGHT) < 1e-9, `${label} 输入应回退到悬停力`);
    }
}

// ── 6. 主动下降指令仍被允许（不是把所有情况都抬到自重）──
{
    // 指令垂直力明显小于自重 = 要求下降，限幅器不应强行抬升
    const F = limitTiltPreservingGravity(0.5, WEIGHT * 0.3, 0.5, WEIGHT, 60);
    assert(F.y < WEIGHT, '明确的下降指令应被保留，限幅器不得强制悬停');
}

console.log(`\nSO3 倾角限幅测试: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
