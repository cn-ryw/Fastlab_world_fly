/**
 * SO3 + YOPO 闭环积分测试 —— 无头运行真实物理循环。
 * 运行: node tests/test_so3_closed_loop.js
 *
 * 使用仓库自带的真实 PlayCanvas 数学库(asset/vendor/playcanvas.min.js)，
 * 不自写四元数桩件，避免桩件保真度影响结论。
 *
 * 复现用户报告的场景：SO3 模式下在 100m 高度设一个 200m 外的目标点。
 * 修复前的行为是"直接下降却不朝目标飞"，本测试把正确行为固化下来：
 *   - 高度保持
 *   - 水平距离持续收敛
 *   - 推力矢量倾角不超上限
 */

import { createRequire } from 'node:module';
const pc = createRequire(import.meta.url)('../asset/vendor/playcanvas.min.js');
globalThis.pc = pc;
// drone.js 只在 readSettings() 和 getFixedYaw() 里碰 DOM；
// 返回 null 即走各自的默认分支（yaw-lock 默认开启）。
globalThis.document = { getElementById: () => null };

let passed = 0, failed = 0;
function assert(cond, msg) {
    if (cond) { passed++; return; }
    failed++;
    console.error(`FAIL: ${msg}`);
}

const { Drone } = await import('../src/drone.js');

const NEUTRAL_INPUT = {
    armed: true, boost: false,
    pitch: 0, roll: 0, yaw: 0, throttle: 0,
    rates: { roll: 1, pitch: 1, yaw: 1 },
    cameraTiltKeyboard: 0, cameraTiltAxisChanged: false,
};

const TRAJ_T = 1.125;      // 2*radio_range/vel_max_train = 2*9/16
const CRUISE = 15.0;       // YOPO cruise_target_mps

/**
 * 模拟 yopo_bridge.py 的输出：朝目标前进一段，高度落在目标所在的高度平面。
 * 返回**轴主序** [px,vx,ax, py,vy,ay, pz,vz,az]。
 */
function mockYopoEndstate(drone, goal) {
    const dx = goal.x - drone.x, dz = goal.z - drone.z;
    const d = Math.hypot(dx, dz);
    const ux = d > 1e-6 ? dx / d : 0;
    const uz = d > 1e-6 ? dz / d : 0;
    const step = Math.min(CRUISE * TRAJ_T, d);
    const spd = d > CRUISE * TRAJ_T ? CRUISE : 0;   // 临近目标减速
    return [
        drone.x + ux * step, ux * spd, 0,
        goal.y,              0,        0,
        drone.z + uz * step, uz * spd, 0,
    ];
}

/** 跑一段闭环，返回轨迹统计。scramble=true 时模拟修复前的量主序错位。 */
function flyToGoal(goal, { seconds = 25, scramble = false } = {}) {
    const d = new Drone();
    d.flightMode = 'so3';
    d.setSpawnPoint(0, 100, 0);
    d.setIdealGoal({ x: goal.x, y: goal.y, z: goal.z });

    const dt = 1 / 60;
    const steps = Math.round(seconds / dt);
    let minAlt = Infinity, maxAlt = -Infinity, maxTilt = 0, sawNaN = false;
    const distAt = [];

    for (let i = 0; i < steps; i++) {
        if (i % 6 === 0) {                       // 10 Hz 重规划
            let es = mockYopoEndstate(d, goal);
            if (scramble) {
                // 量主序错位：[px,py,pz, vx,vy,vz, ax,ay,az] 被当成轴主序读
                es = [es[0], es[3], es[6], es[1], es[4], es[7], es[2], es[5], es[8]];
            }
            d.setYopoTrajectory(es, TRAJ_T);
        }
        d.update(dt, NEUTRAL_INPUT, null);

        if (![d.x, d.y, d.z, d.vx, d.vy, d.vz].every(Number.isFinite)) { sawNaN = true; break; }
        minAlt = Math.min(minAlt, d.y);
        maxAlt = Math.max(maxAlt, d.y);
        // 由姿态推出推力矢量与世界竖直方向的夹角
        const m = new pc.Mat4();
        m.setTRS(pc.Vec3.ZERO, d.orientation, pc.Vec3.ONE);
        const up = new pc.Vec3();
        m.getY(up);
        maxTilt = Math.max(maxTilt, Math.acos(Math.max(-1, Math.min(1, up.y))) * 180 / Math.PI);
        if (i % 60 === 0) distAt.push(Math.hypot(goal.x - d.x, goal.z - d.z));
    }
    return {
        drone: d, minAlt, maxAlt, maxTilt, sawNaN, distAt,
        finalDist: Math.hypot(goal.x - d.x, goal.z - d.z),
    };
}

// ── 1. 同高度 200m 外目标：应平飞过去，不掉高 ──
{
    const goal = { x: 200, y: 100, z: 0 };
    const r = flyToGoal(goal);

    assert(!r.sawNaN, '物理状态不得出现 NaN');
    console.log(`  同高目标: 高度 ${r.minAlt.toFixed(1)}~${r.maxAlt.toFixed(1)}m, ` +
        `末端水平距 ${r.finalDist.toFixed(1)}m, 最大倾角 ${r.maxTilt.toFixed(1)}°`);

    assert(r.minAlt > 95, `最低高度 ${r.minAlt.toFixed(1)}m 应保持在 95m 以上(不掉高)`);
    assert(r.maxAlt < 110, `最高高度 ${r.maxAlt.toFixed(1)}m 不应过冲`);
    assert(r.finalDist < 10, `末端水平距离 ${r.finalDist.toFixed(1)}m 应收敛到 10m 内`);
    assert(r.maxTilt <= 62, `最大倾角 ${r.maxTilt.toFixed(1)}° 应在 60° 上限附近`);

    // 距离必须单调收敛（允许末端小幅震荡）
    let monotone = true;
    for (let i = 1; i < Math.min(r.distAt.length, 8); i++) {
        if (r.distAt[i] > r.distAt[i - 1] + 1.0) monotone = false;
    }
    assert(monotone, `水平距离应持续收敛: ${r.distAt.slice(0, 8).map(v => v.toFixed(0)).join(' → ')}`);
}

// ── 2. 异面目标：目标高度平面高出 30m，应爬升过去 ──
{
    const goal = { x: 150, y: 130, z: 0 };
    const r = flyToGoal(goal, { seconds: 30 });
    console.log(`  升高目标: 末端高度 ${r.drone.y.toFixed(1)}m (目标 130m), ` +
        `末端水平距 ${r.finalDist.toFixed(1)}m`);
    assert(!r.sawNaN, '异面目标不得出现 NaN');
    assert(r.drone.y > 120, `末端高度 ${r.drone.y.toFixed(1)}m 应爬升到目标平面附近`);
    assert(r.finalDist < 15, `异面目标末端水平距 ${r.finalDist.toFixed(1)}m 应收敛`);
}

// ── 3. 对照组：还原修复前的量主序错位，必须复现"掉高且不到达" ──
{
    const goal = { x: 200, y: 100, z: 0 };
    const r = flyToGoal(goal, { scramble: true });
    console.log(`  [对照] 量主序错位: 高度最低 ${r.minAlt.toFixed(1)}m, ` +
        `末端水平距 ${r.finalDist.toFixed(1)}m, 最大倾角 ${r.maxTilt.toFixed(1)}°`);
    assert(r.minAlt < 95 || r.finalDist > 10,
        '对照组应复现故障（掉高或无法到达），否则本测试无法证明修复有效');
}

console.log(`\nSO3 闭环积分测试: ${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
