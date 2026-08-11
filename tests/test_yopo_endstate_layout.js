/**
 * YOPO endstate 数组布局契约测试 —— 纯逻辑，不需要 DOM / Cesium / 推理服务。
 * 运行: node tests/test_yopo_endstate_layout.js
 *
 * 锁定 scripts/yopo_bridge.py 与 src/drone.js 之间的数据契约：
 *   endstate 为**轴主序** [px,vx,ax, py,vy,ay, pz,vz,az]，sim 世界系 (x=east, y=up, z=north)。
 *
 * 历史 bug：drone.js 曾按量主序 [px,py,pz, vx,vy,vz, ax,ay,az] 读取，
 * 导致 X 轴终端速度被填成高度值(~100)、Z 轴终点位置被填成加速度值，
 * 参考轨迹发散 → SO3 控制器需求力冲到 277N → 掉高。
 */

// drone.js 在模块顶层构造 pc 对象，先装最小桩件再动态 import。
globalThis.pc = {
    Quat: class { constructor() { this.x = 0; this.y = 0; this.z = 0; this.w = 1; } },
    Mat4: class {},
    Vec3: class { constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; } },
};
globalThis.pc.Vec3.ZERO = new globalThis.pc.Vec3(0, 0, 0);
globalThis.pc.Vec3.ONE = new globalThis.pc.Vec3(1, 1, 1);

const { Drone } = await import('../src/drone.js');

let passed = 0, failed = 0;
function assert(cond, msg) {
    if (cond) { passed++; return; }
    failed++;
    console.error(`FAIL: ${msg}`);
}
function assertClose(a, b, msg, eps = 1e-6) {
    assert(Math.abs(a - b) < eps, `${msg}: 期望 ${b}, 实际 ${a}`);
}

const T = 1.125;

// ── 1. 轴主序读取：每个轴的多项式终点必须落在该轴自己的 位置/速度/加速度 上 ──
{
    // 刻意让 9 个分量互不相同，任何错位都会被抓到
    const px = 16.9, vx = 15.0, ax = 2.0;
    const py = 100.0, vy = -0.5, ay = 0.25;
    const pz = 3.0, vz = 1.0, az = 0.5;
    const endstate = [px, vx, ax, py, vy, ay, pz, vz, az];

    const d = new Drone();
    d.x = 0; d.y = 100; d.z = 0;
    d.vx = 15; d.vy = 0; d.vz = 0;
    d.setYopoTrajectory(endstate, T);

    assertClose(d._yopoPolyX.position(T), px, 'X 轴终点位置');
    assertClose(d._yopoPolyX.velocity(T), vx, 'X 轴终点速度');
    assertClose(d._yopoPolyX.acceleration(T), ax, 'X 轴终点加速度');

    assertClose(d._yopoPolyY.position(T), py, 'Y 轴(高度)终点位置');
    assertClose(d._yopoPolyY.velocity(T), vy, 'Y 轴终点速度');
    assertClose(d._yopoPolyY.acceleration(T), ay, 'Y 轴终点加速度');

    assertClose(d._yopoPolyZ.position(T), pz, 'Z 轴终点位置');
    assertClose(d._yopoPolyZ.velocity(T), vz, 'Z 轴终点速度');
    assertClose(d._yopoPolyZ.acceleration(T), az, 'Z 轴终点加速度');

    // 回归：量主序读法会让 X 轴终端速度 == py(高度值)。必须不成立。
    assert(Math.abs(d._yopoPolyX.velocity(T) - py) > 1.0,
        '回归检查：X 轴终端速度不得等于高度值(量主序错位特征)');
    // 回归：量主序读法会让 Z 轴终点位置 == ax。必须不成立。
    assert(Math.abs(d._yopoPolyZ.position(T) - ax) > 0.5,
        '回归检查：Z 轴终点位置不得等于 X 加速度(量主序错位特征)');
}

// ── 2. 物理合理性：巡航轨迹的参考速度必须有界 ──
// 布局一旦错位，高度值(50~100)会流进速度通道，参考速度立刻爆表。
{
    const VEL_BOUND = 25.0;   // YOPO vel_max_train=16, 留足裕度
    for (const alt of [50, 100, 300]) {
        // 典型巡航：15m/s 向东，高度保持在 alt
        const endstate = [16.9, 15.0, 2.0, alt, 0.0, 0.0, 1.2, 0.8, 0.3];
        const d = new Drone();
        d.x = 0; d.y = alt; d.z = 0;
        d.vx = 15; d.vy = 0; d.vz = 0;
        d.setYopoTrajectory(endstate, T);

        let maxRefSpeed = 0;
        for (let t = 0; t <= T; t += T / 40) {
            const s = Math.hypot(d._yopoPolyX.velocity(t), d._yopoPolyZ.velocity(t));
            maxRefSpeed = Math.max(maxRefSpeed, s);
        }
        assert(maxRefSpeed < VEL_BOUND,
            `高度 ${alt}m: 参考水平速度峰值 ${maxRefSpeed.toFixed(1)} m/s 应 < ${VEL_BOUND}`);
    }
}

// ── 3. _getYopoReference 的高度应来自 Y 多项式，而非硬覆盖 ──
{
    const d = new Drone();
    d.x = 0; d.y = 100; d.z = 0;
    // 单段安全高度步进到 104m；更大的目标高差由 bridge 连续重规划逼近。
    d.setIdealGoal({ x: 200, y: 104, z: 0 });
    // endstate 轴主序：[px,vx,ax, py,vy,ay, pz,vz,az]
    d.setYopoTrajectory([16.9, 15.0, 2.0, 104.0, 4.0, 0.0, 1.2, 0.8, 0.3], 1.125);

    // 中点：Y 多项式应给出介于 100→104 之间的中间值
    const ref = d._getYopoReference(0.5625);
    assert(ref.y > 100.5 && ref.y < 103.5,
        `中点高度 ${ref.y.toFixed(1)}m 应在 100.5~103.5 之间（Y 多项式平滑插值），不得被固定值覆盖`);
    assert(Math.abs(ref.vy) > 1e-6,
        `中点垂直速度 ${ref.vy.toFixed(4)} m/s 不得为 0（Y 多项式非恒值）`);
    // 验证 X/Z 通道仍来自对应多项式
    assert(Math.abs(ref.x - d._yopoPolyX.position(0.5625)) < 1e-9, '参考 x 取自 X 多项式');
    assert(Math.abs(ref.z - d._yopoPolyZ.position(0.5625)) < 1e-9, '参考 z 取自 Z 多项式');
}

console.log(`\nYOPO endstate 布局契约测试: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
