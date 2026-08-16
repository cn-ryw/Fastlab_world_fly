# 控制与感知基线存档（2026-08-11）

本文记录 `feat/da360-metric-depth-v2` 分支在继续优化实时规划和控制器之前的可运行基线。
该存档用于后续对比与回退，不代表 15 Hz 城市闭环已经通过正式验收。

## 一、本次存档范围

### 1. 飞行控制器

- 保留 `drone / fpv / stabilized / so3` 四个模式 ID 和界面入口，统一内部控制链。
- FPV 保持原有杆量、expo、角速度和油门行为，并增加固定时基黄金回归。
- Level 改为组合倾角受限的姿态模式，油门按 `0 / hover / max` 分段映射。
- Easy 改为位置—速度级联控制，加入制动、锁点、jerk/加速度限制和 anti-windup。
- SO3 改为 YOPO 自动模式：活动轨迹只执行 Poly5 参考加速度，不叠加位置/速度反馈；
  等待、过期、碰撞和到达后才进入位置保持。
- 公共姿态执行采用 reduced quaternion、角速度伺服和四元数积分；删除无真实转动惯量支撑的
  `KR/KOmega` 伪力矩语义。
- 修复 Easy/Level 左右方向、SO3 yaw 状态、模式切换物理状态不连续、旧姿态轴投影等问题。
- 模式切换只重置控制器记忆，不再清零物理位置、线速度、姿态或角速度。

### 2. 固定时基与轨迹安全

- 控制与公共刚体使用 200 Hz 固定步长；每个渲染帧最多追赶 20 步，超过 100 ms 时丢弃多余墙钟时间并触发安全保持。
- YOPO 轨迹使用独立 tracker 和模拟时钟，删除人工 0.5 秒 decay。
- Poly5 安装检查整个区间的速度/加速度极值，而不仅检查端点；加入时长、位移、速度和加速度门禁。
- 活动 SO3 每个样本进行有限性检查、25 m/s²模长限制、60°总倾角限制和总推力限制。
- 碰撞、超时、轨迹过期和控制 overrun 会使旧轨迹失效并请求新规划。

### 3. DA360、YOPO 与全景感知

- DA360 使用 2026-08-11 经实时效果人工检查后接受的 scale-only 米制标定作为 sim-to-sim 基线；
  自动 LOLO 精度门禁失败的事实仍完整保留，不将其描述成真实传感器精度验收通过。
- `start-all.sh` 默认启动 `da360-metric`，YOPO 规划响应携带稳定的标定和服务身份。
- 全景 RGB 方位恢复为 YOPO ERP 契约，修复左右镜像；深度 top-down 可视化同步修正。
- 全景捕获区分“六面已投影”和“瓦片全部就绪”，瓦片完整度仅显示和记录，不作为 YOPO 硬门禁。
- 修复预热参数分流和 Firefox 启动脚本代理继承；外网代理保留，本地服务写入 `NO_PROXY/no_proxy`。
- Firefox 启动输出和日志会隐藏 URL 查询参数、userinfo 与代理地址，警告写入独立日志。

### 4. 证据与配置

- 控制器配置升级为 v6，四种模式各自保存完整 profile，并严格校验模式枚举和数值。
- FlightLogger/evaluator 使用 schema v2，绑定 goal、generation、frame、标定和服务身份，区分 applied/stale/blocked/rejected。
- 增加 RGB 瓦片、捕获、网络、DA360、YOPO、应用和外层飞行循环等分段指标。
- 修正 Y-Up 标识、SO3 自动模式标签、HUD/OSD 和按键说明。

## 二、当前验证边界

已经具备自动测试覆盖：

- FPV 固定步行为保持；
- Level 组合倾角和油门端点；
- Easy 方向、制动、锁点、饱和反向和 anti-windup；
- SO3 direct-acceleration、25 m/s²、60°、轨迹替换、过期、碰撞和模式连续性；
- Poly5 区间极值、固定时基、配置迁移、全景方位、metric anchor、后端契约和 evaluator 门禁。

当前 Firefox 飞行记录显示，米制深度、YOPO、轨迹安装、SO3 执行和到达链路能够完成一次闭环运行。
但最近两次城市飞行仍未通过正式实时性验收：有效规划约 2.6/6.0 Hz，
capture-to-apply p95 约 249/244 ms，尚未达到 15 Hz 和 150 ms 门槛。

## 三、已知待优化项

后续按以下优先级推进，重新训练策略排在最后：

1. 日志记录原始 YOPO endstate、候选 ID/score、终端速度、Poly5 极值、捕获/应用位移、参考速度和控制饱和。
2. 将规划控制响应与深度预览解耦，减少主线程串行等待、陈旧请求和无效 GPU 工作。
3. 修复捕获时规划状态与应用时 Poly5 的时间对齐，避免陈旧绝对终点重新跑满整个轨迹时域。
4. 建立终端制动/停稳到达状态，不能只在高速穿入 4 m 半径时立即报告 arrived。
5. 依据新增诊断调研并优化姿态伺服、推力投影、轨迹可执行性和换轨连续性；不盲目提高倾角、推力或安全风险。
6. 只有在运行时链路和控制执行不再成为瓶颈后，才评估局部 subgoal、候选重排及策略重新训练。

## 四、基线测试命令

```bash
for test_file in tests/*.js; do node "$test_file" || break; done

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_fit_da360_metric.py \
  tests/test_serve_security.py \
  tests/test_evaluation_gates.py

docker run --rm --entrypoint /bin/bash -v "$PWD:/workspace:ro" \
  mindcloud-da360-yopo:latest \
  -lc 'cd /workspace && python3 tests/test_backend_contract.py'
```

真实 Firefox/GPU 与城市 tiles 的 15 Hz 闭环必须在后续优化完成后重新飞行采集日志，
不能用本存档的自动测试或旧仿真结果替代。
