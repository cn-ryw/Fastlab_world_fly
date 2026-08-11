# 控制器与实时规划优化决策（v7，2026-08-11）

## 1. 本轮目标与边界

本轮先解决运行时可观测性、规划时延、轨迹时间对齐和终端到达控制，再讨论策略重训。
不得用放宽 250 ms 陈旧帧门限、强行提高倾角/推力、降低避障权重或给 YOPO 活动段叠加
`15 m/s` 速度反馈来掩盖问题。

最近两次 Firefox 城市飞行已经证明 DA360 metric → YOPO → Poly5 → SO3 的功能闭环可达，
但正式实时性尚未通过：有效规划约 2.6/6.0 Hz，capture-to-apply p95 约 249/244 ms。
第二次飞行在建筑绕行低谷中，实际速度约 2.4 m/s、安装后的参考速度约 3.8 m/s，
位置跟踪误差很小。这说明主要矛盾是局部规划输出和高龄安装的耦合，不是简单的控制增益不足。

## 2. 调研结论

### YOPO 权威链

- READY 活动段直接执行世界系期望加速度，控制力为 `m(g e_up + a_ref)`。
- 位置/速度反馈用于等待、失效和悬停，不应混进有效网络轨迹。
- `Cruise-15` 是策略训练工况和任务标签，不是运行时恒速闭环。
- 运行时终点由捕获时状态生成；若应用端在约 200 ms 后从新状态重新跑满 1.125 秒 Poly5，
  会混用两套时刻并缩短或扭曲原轨迹。

### PX4 可借鉴部分

- reduced-quaternion 姿态误差、统一角速度上限和四元数积分；
- 平移环的推力优先级、饱和诊断和 anti-windup；
- 到达不是穿过位置半径，而是位置和速度共同进入稳定状态。

### 不采用部分

- 不引入电机 mixer、RPM、转动惯量、rate PID 或状态估计器；这些会改变当前 sim-to-sim plant 分布。
- 不把 PX4 的真实飞控增益直接套到当前简化角速度伺服。
- 不把 Fly360/AirSim 的速度转姿态增益复制进项目。

## 3. 选择的运行时控制方案

1. YOPO 活动段仍为 `DIRECT_ACCELERATION`，保持规划力方向，经过有限性、25 m/s²、60°和总推力包络。
2. 捕获时冻结模拟时钟、实际状态、参考状态和 RGB；应用时按模拟年龄 fast-forward 原 Poly5，
   只执行剩余时域，绝不平移障碍相关终点。
3. 终端使用 `approach → terminal-track → terminal-decelerating → settling → arrived`。
   候选终点必须进入 4 m、Poly5 峰值加速度不大于 25 m/s²且 suffix 状态一致；终端速度
   可高于 1.5 m/s，但不得超过 12 m/s，并且按当前质量、推力、60°倾角、12 m/s²设计减速度
   和 75 ms 姿态建立时间预测的切向停止点必须仍在 5 m 恢复球内。满足后才冻结候选并沿
   原绕障 Poly5 执行到底，绝不提前截断或改用指向 goal 的位置环。轨迹结束时按实际位置/
   速度重新检查同一停止包络；高速时以当时实际位姿为零速锚点，使用 25 m/s²、60°和总
   推力限幅的专用减速环，减速方向来自终端速度与锚点误差而非 goal。速度降到 1.5 m/s
   且仍在 4 m 内后锁定一次当前实际位姿进入 `settling`；三维速度不大于 0.75 m/s、无碰撞
   并连续稳定 0.4 s 才报告 `arrived`。减速超过 2.5 s、碰撞、overrun、非有限状态/命令或
   退出 5 m 恢复球，均退回 `approach`、保持当前位姿并请求新规划。
4. 规划控制响应优先于深度预览：飞行期 planning 固定 `include_preview=0`，轨迹完成校验和安装后立即释放请求 gate；操作员深度图约每2s按 frame/goal/generation 从服务端4帧LRU读取，异步、latest-only且不重复DA360。
5. 每次飞行必须记录规划选择、轨迹可执行性和控制饱和，之后才允许调整姿态带宽、投影或轨迹包络。

## 4. 实施顺序

### 阶段 A：日志与证据

- 原始网络系数/世界系 endstate；前者为 lattice 解码前的无量纲 tanh 输出，后者为 sim Y-up 轴主序物理状态；
- selected action、lattice/candidate ID 和 score；candidate 等同 CNN flatten action，lattice 为反序格点，score 是越低越优的组合代价而非置信度；
- 终端速度、终端加速度、Poly5 区间峰值速度/加速度；
- 捕获状态、应用状态及其位移；
- 参考速度/加速度、raw/limited acceleration、投影比、倾角、推力和饱和轴；
- fetch/headers/json/depth-preview 的分段年龄和字节数；
- 固定步 steps、overrun 与丢弃墙钟时间。

控制日志的 `frame` 固定为 `sim-world-y-up`：reference velocity 和 raw/limited acceleration
分别使用 m/s 与 m/s²，requested/allocated force 使用 N，`thrustGf` 使用 gram-force，
`tiltDeg` 使用度。HUD/OSD 的 ACT/POLY5 是水平速度，后端 `terminalSpeedMps` 是三维速度模长。

### 阶段 B：实时链路

- 后端所有飞行规划响应都不编码深度预览；兼容调用仍可显式请求内嵌预览；
- 前端深度预览与控制响应解耦，按2s周期读取有界身份缓存；
- fetch 前进行年龄预算，淘汰必然超过 250 ms 的帧；
- 只保留最新帧，不积压旧请求；
- 将六面切片默认值从 2 个面提高到 3 个面，减少一次 RAF yield，但不使用一次性 6 面阻塞控制循环；
- 用真实 Firefox 日志验证有效规划至少 15 Hz、capture-to-apply p95 不高于 150 ms，目标为 70–100 ms。

### 阶段 C：控制与轨迹

- 修复捕获到应用的时间轴；
- 加入 suffix 位置/速度一致性门、终端轨迹完整跟踪和停稳到达；
- 检查换轨 jerk、25 m/s²削顶、60°倾角、总推力与姿态投影占比；
- 只有证据显示姿态执行持续落后时，才 A/B 姿态带宽或推力投影策略。

### 阶段 D：重构

- 保持 `Drone.update()` 等公共接口，逐步把导航状态、轨迹 tracker、模式 setpoint、姿态执行和 plant 解耦；
- 不在一次提交中同时重写 planner 协议、控制律和物理模型；
- FPV 黄金轨迹和四模式物理连续性必须持续通过。

### 阶段 E：策略层（后排）

只有 A–D 通过后，才评估局部 subgoal、候选可执行性重排、跨帧换边迟滞、城市 hard-negative、
更长可靠深度/轨迹地平线及重新训练。策略层不能替代运行时修复。

## 5. 验收口径

- 日志中的每个 applied 规划帧可追溯到同一 goal/generation/frame/capture state；
- 有效规划平均不低于 15 Hz，规划间隔 p95 不高于 100 ms；
- capture-to-apply p95 不高于 150 ms，服务 p95 不高于 50 ms；
- 正常航段无 trajectory-expired、control-overrun 或跨会话轨迹复活；
- 到达时距离不高于 4 m、速度不高于 0.75 m/s，并完成 0.4 s 稳定驻留；
- 活动段总倾角不高于 60°，Level/Easy 不高于 45.5°，推力和角速度不越界；
- 30/60/120 FPS 合成输入结果保持固定步一致，FPV 黄金轨迹不回归。

真实 Firefox 城市闭环是最终性能证据；JS/Python/backend 自动测试只证明数学、协议和状态机正确。

## 6. 本地回归与后续实飞

本轮本地收口已通过：25 个 JS 测试文件、129 个宿主 Python 测试、Docker 内 30 个后端
契约测试，以及依赖锁、JS/Python 语法、差异格式和 SHA 变更扫描。50 ms Poly5 在 200 Hz
下已锁定为 10/10 个直接加速度控制样本，首个超过终点时刻的固定步才进入终端或保持逻辑。

这些结果尚不能宣称实时性门禁通过。后续需重建并重启 combined 服务、完全重启 Firefox，使用
城市 tiles 连续飞行至少 60 s、保存不少于 900 个有效规划帧，再用 evaluator 核对规划频率、
规划间隔、capture-to-apply、服务和物理更新 p95。若仍未达到 15 Hz / 150 ms，应先用新增的
fetch、response bytes、候选、suffix 误差和控制饱和字段定位运行时瓶颈；策略重训继续排在
运行时链和控制执行验证之后。
