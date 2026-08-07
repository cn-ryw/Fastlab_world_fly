# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

浏览器内的 Google Photorealistic 3D Tiles 穿越机仿真器 + 360° ERP 全景 → DA360 深度估计 → YOPO 端到端导航。

## 任务目标：sim-to-sim 迁移

本项目**不是**从零写一个规划器，而是把已经在 ROS/RViz 端跑通的 YOPO_360 闭环，原样搬到浏览器仿真器里。

- **源（已验证 work）**：`/home/ykx/ros1/YOPO_360_v15`，分支 `feat/x5-cruise15-release`。权威实现是 `YOPO/test_yopo_ros.py`（感知→推理→Poly5 轨迹）和 `Controller/src/so3_control/src/NetworkControl.cpp`（SO3 控制器）。
- **目标**：MindCloud_World_Fly。`scripts/yopo_bridge.py` 对应 `test_yopo_ros.py`，`src/drone.js` 的 `_controlSO3` 对应 `NetworkControl.cpp`。
- **差异只允许出现在两处**：深度来源（ROS 是仿真栅格 raycast，这里是 Cesium 渲染 + DA360 单目估计）、控制指令下发方式（ROS 发 SO3Command 给 mavros，这里直接积分刚体）。**其余任何行为差异都应视为移植 bug，先去对照参考实现，而不是自行调参。**

改 `_controlSO3` / `yopo_bridge.py` / 坐标变换前，先打开对应的参考文件逐行比对。历史上的几个严重 bug 都源于"照着文档字符串写，而没照着参考代码写"。

## 常用命令

### 全栈启停（一条命令搞定）

```bash
./start-all.sh              # DA360+YOPO(5688) + Web本地(8080) + 健康检查 + Clash规则
./launch-firefox-gpu.sh     # Firefox + NVIDIA prime offload 打开页面
./stop-all.sh               # 全部停掉（容器 + 本地Web服务）
```

`start-all.sh` 内部机制：DA360+YOPO 容器 bind-mount `scripts/`（Python 改动 restart 即生效），Web 走本地 `scripts/serve.py`（JS 改动刷新即生效）。不再启动 Docker Web 容器。

### 改了什么，怎么生效

| 改了什么 | 命令 | 生效方式 |
|---|---|---|
| `src/*.js` / `index.html` | 刷新浏览器 | 即时（本地 Python 从磁盘直读） |
| `scripts/*.py` | `docker restart mindcloud-da360-yopo` | ~1 秒 |
| `third_party/` / Dockerfile | `docker build -f Dockerfile.da360-yopo ... && ./start-all.sh` | ~2 分钟 |

### 镜像构建（低频，镜像不存在或改 third_party/Dockerfile 时才需要）

```bash
docker build -f Dockerfile.da360-yopo -t mindcloud-da360-yopo:latest .   # 基镜像 dzp_yopo:sim-u2004-noetic-py38
```

### 模型下载

```bash
python3 -m pip install --user gdown
./scripts/download_da360_model.sh large    # → third_party/DA360/checkpoints/DA360_large.pth (1.34 GB)
```

YOPO checkpoint 不在本仓库，`start-all.sh` 从宿主 `/home/ykx/ros1/YOPO_360_v15/YOPO/saved/YOPO_55/epoch10.pth` 挂载。

### 测试

JS 测试无需服务、无需浏览器，直接 node 跑（全部 7 个文件共约 500 项断言）：

```bash
for t in tests/*.js; do node "$t" || break; done   # 跑全部

node tests/test_erp_geometry.js            # ERP 像素↔方向 往返、单位长度、接缝
node tests/test_metric_anchor_direction.js
node tests/test_erp_anchor_seam.js
node tests/test_yopo_endstate_layout.js    # yopo_bridge ↔ drone.js 的 endstate 布局契约
node tests/test_so3_tilt_limit.js          # 倾角限幅：重力补偿不可被削弱（含 2 万组随机属性测试）
node tests/test_so3_closed_loop.js         # SO3+YOPO 闭环积分：高度保持 + 收敛到目标
node tests/test_drone_model_scale.js       # 渲染尺寸与物理机体尺寸一致性
```

其中 `test_so3_closed_loop.js` 用 `createRequire` 载入仓库自带的**真实** PlayCanvas
（`asset/vendor/playcanvas.min.js`，UMD 包在 Node 下走 CommonJS 分支）跑无头物理循环，
不自写四元数桩件。需要 DOM 的地方只有 `readSettings()` 和 `getFixedYaw()`，
用 `document.getElementById: () => null` 打桩即可走默认分支。

Python 测试**需要 DA360 服务已在跑**（默认 `http://127.0.0.1:5688`，用 `DA360_URL` 覆盖）：

```bash
python3 -m pytest tests/ -v
python3 -m pytest tests/test_da360_raw_output.py -v                      # 单文件
python3 -m pytest tests/test_da360_raw_output.py::test_raw_returns_200 -v # 单用例
DA360_URL=http://other-host:5688 python3 -m pytest tests/ -v
```

宿主 Python 只用于测试和离线脚本；推理本身在容器内（Python 3.8/3.10 + PyTorch）。

### 深度离线工具

```bash
python3 scripts/request_da360_raw.py test.jpg -o raw.npz     # 取 /depth/raw 的 float32 pred_disp
python3 scripts/inspect_da360_raw.py raw.npz --vis preview.png
python3 scripts/fit_da360_metric.py \                        # 1/z = a*pred_disp + b (Huber, 可选 --ransac)
    --raw depth_raw.npz --anchors cesium_anchors.json --output experiment_data/metric_fit_sample/
```

## 架构

### 进程拓扑

```
浏览器 (无构建步骤, 原生 ES module)          Docker: mindcloud-da360-yopo :5688
┌──────────────────────────────┐            ┌────────────────────────────────┐
│ Cesium + Google 3D Tiles     │            │ combined_server.py             │
│  6 面立方体采样 → GPU 重投影  │  JPEG      │  ├ DA360Runner  → /depth       │
│  → ERP 全景 384×192  ────────┼───────────▶│  │                /depth/raw   │
│                              │            │  └ YopoRunner   → /yopo/plan   │
│ drone.js 物理 + 控制律       │◀───────────┤                   /yopo/plan_full│
│ main.js 状态机 + 目标点交互  │  endstate  │  (两个模型并行加载, YOPO 有 lock)│
└──────────────────────────────┘            └────────────────────────────────┘
        Docker: google-tiles-flight :8080 (node scripts/server.js)
```

`/yopo/plan_full` 是在线闭环真正走的那条路：一次 POST 上传 JPEG，服务端内部 DA360 推理出深度再喂 YOPO，位姿/目标/速度/yaw 走 query string（`px,py,pz,gx,gy,gz,vx,vy,vz,yaw`）。`/yopo/plan` 收现成的深度数组，只在离线调试用。

### 前端模块（src/，全部原生 ES module，无打包）

`index.html` 加载 CesiumJS（本地 `/ThirdParty/Cesium/` + CDN fallback）和 `asset/vendor/playcanvas.min.js`（`pc.Quat`/`pc.Mat4` 全局可用），然后 import `src/main.js`。

```
main.js            模式状态机 loading → placement → view-select → flight
  ├ cesium-world.js    Cesium Viewer 封装、Google Tiles、局部坐标互转、
  │                    隐藏 viewer 的 6 面全景采样 + ERP shader、
  │                    waitForTilesIdle / 起飞前区域预加载 / 覆盖率采样
  ├ tiles-collision.js  sampleHeight + pickFromRay + swept ray 三层碰撞代理
  ├ controller.js       键盘 / Gamepad API / WebHID 遥控器、通道映射、
  │                     设置面板与 localStorage 持久化(CONFIG_VERSION=4 + 迁移)
  ├ drone.js            四元数刚体物理 + 4 种控制律 + YOPO Poly5 轨迹跟踪
  ├ panorama-sensor.js  采样节流、DA360 深度请求、YOPO 规划触发
  ├ erp-geometry.js     ERP 像素↔方向（与 shader、与 YOPO_360 必须一致）
  ├ flight-logger.js    设目标自动开始录制，到达/取消自动下载 JSON
  ├ hud.js / osd.js     飞行 HUD、FPV OSD
  ├ gates.js / path-editor.js / path-store.js / catmull-rom.js   赛道子系统
  └ error-report.js     去重限频的用户可见错误弹窗
```

### 飞行模式（`drone.js` 的 `flightMode`，settings 面板下拉框 / `M` 键切换）

| 值 | 控制律 | 说明 |
|---|---|---|
| `so3` | `_controlSO3` | 几何控制器，YOPO 自动导航走这条；增益 `so3Kx/Kv/KR/KOmega` 面板可调 |
| `stabilized` | `_controlStabilized` | PX4 自稳：摇杆→角度指令，回中自动水平 |
| `fpv` | `_controlFPV` | 角速率手动（acro），无自稳 |
| `drone` | `_controlDrone` | 级联 PID 速度指令 |

只有 `so3` 和 `stabilized` 允许设目标点（`G`+左键 / 双击 / 点雷达小地图；`G`+滚轮调高度；`C` 取消）。目标一设，`main.js` 同时喂给 `drone.setIdealGoal()`、`panoramaSensor.setYopoGoal()` 和 `flightLogger.start()`。

飞控参数对齐 YOPO Hummingbird：质量 980 g、最大推力 2600 gf、`collisionRadius=0.6`、`ARRIVAL_DISTANCE_M=4.0`。改这些默认值时注意 `controller.js` 的 `_migrateConfig` —— 旧 localStorage 会覆盖新默认值，必须同步 bump `CONFIG_VERSION`。

### 坐标系（改任何几何代码前先读这一节）

三套坐标系，任何一处符号错都表现为"无人机往反方向飞"：

| 坐标系 | 约定 |
|---|---|
| Sim local | x=east, **y=up**, z=north |
| Body frame | X=right, **Y=up/thrust**, Z=backward，forward = −Z；yaw=0 朝南 |
| YOPO world | x=east, y=north, **z=up**（相对 sim 是 y/z 交换） |
| ERP 像素 | `yaw = π − (u+0.5)/W·2π`，`pitch = vfov/2 − (v+0.5)/H·vfov`，`dy` 取负做左右镜像 |

`yopo_bridge.py:170-240` 是这些转换的唯一权威实现，包含两个非显然的处理：

- **yaw 映射** `yopo_yaw_rad = deg2rad(-yaw - 90)`（sim yaw 0=南、顺时针为正 → YOPO 0=东）。
- **高度平面跟随目标高度，不做任何高度平移**。网络输入 `obs = [vel_c, acc_c, goal_c]` 里 `goal_c = R_cw·(goal − pos)` 只含**相对**位移，绝对高度根本不进网络。历史上的 `altitude_shift` 把 pos.z 和 goal.z 同减一个量，对 obs 是恒等变换（数值验证各高度下误差 < 3e-15），已删除。现在按参考实现 `test_yopo_ros.py:callback_set_goal_3d` 的做法：`height_plane = goal.z`，再用 `endstate_w[:,2,0] = height_plane − pos.z` 把轨迹末端拉到该平面。前端 `main.js` 的 `goalAltitudeOverride` 与之对应——默认 null（目标落在当前高度，等价 `callback_set_goal`），按住 G 滚滚轮后变成显式高度（等价 `callback_set_goal_3d`），按 C 复位。

`erp-geometry.js` 的公式和 `cesium-world.js` 里的全景 shader 必须逐字对应，`tests/test_erp_geometry.js` 守这条不变量。

### 配置面：URL query string

运行参数几乎全部通过 URL 覆盖，没有配置文件。定义点在 `main.js` 和 `panorama-sensor.js` 顶部的 `urlNumber()` 常量。

```
panoWidth=384 panoHeight=192 panoFace=144 panoVfov=180   全景输出/采样分辨率
panoMs=33 depthMs=100                                    采样/推理间隔
panoTopPoleGuard=10 panoBottomPoleGuard=2                ERP 极区 guard（防天空伪影）
panoPreloadRequired=0 panoPreloadTimeoutMs=60000         首帧预加载策略
flightPreloadRadius=420 flightPreloadMinCoverage=0.95    起飞前主视图 tiles 预加载
da360Url= / da360Host= / da360Port=5688                  推理服务地址
da360UploadScale=0.35 da360UploadWidth= da360UploadHeight=  仅影响上传给 DA360 的尺寸
droneScale=1.35                                          模型显示大小
```

`panoWidth=384` 是刻意匹配 YOPO 的 384×192 训练分辨率，别随手调大。

### 服务端环境变量

```
DA360_INPUT_SCALE=0.5     模型输入缩放（低于 0.46 会让 large 输出条带化深度）
DA360_DEPTH_SCALE=2.0     粗糙的米制缩放（见下方已知问题）
DA360_RESAMPLE=bicubic    服务端 resize 插值
DA360_MODEL / DA360_MODEL_PATH / DA360_JPEG_QUALITY / DA360_AMP / DA360_CHANNELS_LAST
YOPO_MODEL_PATH / YOPO_CONFIG=x5_cruise15_18m_a12_mask_wc3.yaml / YOPO_AMP
```

## 开发时的坑

- **`start-all.sh` 不会重建 Web 镜像**（只在镜像不存在时 build），而 `Dockerfile.cesium` 是把 `src/` COPY 进镜像的。改完 `src/*.js` 直接 `./start-all.sh`，浏览器拿到的是旧代码。改前端要么 `./launch.sh`（每次都 build），要么 `./launch.sh --local`（从磁盘直读），要么手动 `docker build -f Dockerfile.cesium ...`。
- 排查"改了没生效"时先在控制台确认运行时值（如 `__drone.mass`），再考虑清 `localStorage` 的 `drone_sim_controller_config`。
- **GPU 渲染必须用 Firefox**：本机 `prime-select on-demand`，Chrome 与 prime offload GLX 不兼容（WebGL 初始化失败已确认）。走 `./launch-firefox-gpu.sh`。
- **Clash Verge TUN 模式会拦 `cesium.com` / `tile.googleapis.com`**，表现为 504 或 tile 加载不出来。快速诊断：`curl -m 5 -s -o /dev/null -w '%{http_code}' https://tile.googleapis.com/`，若输出 `000`（未完成）而非 `200`/`404` → 代理在拦。`./fix-clash-rules.sh` 直接改订阅 profile 并重启 Clash，`start-all.sh` 末尾会自动调。若 TUN 模式在更底层拦截，需 Clash 内加 bypass 规则或切到 System Proxy 模式。
- `third_party/DA360/` 在 `.gitignore` 里；`third_party/YOPO/`（config + policy）和 `third_party/wheels/`（离线装 flask-cors、timm）是入库的。

## 已知问题 / 当前状态

进度、已修 bug 清单、调试线索维护在 `../reference_notes/handoff.md`（不入库），开工前先读一遍。

- **已修复：SO3 模式下 G+click 设目标后掉高、且不朝目标飞**。两个独立根因各治一个症状（`reference_notes/handoff.md` 里"浏览器缓存旧 JS"的旧假设已被推翻）：
  1. **endstate 数组布局错配 → 飞错方向**。`yopo_bridge.py` 输出**轴主序** `[px,vx,ax, py,vy,ay, pz,vz,az]`（同 `test_yopo_ros.py` 的 `endstate_w[id, axis, order]`），而 `drone.js:setYopoTrajectory` 曾按**量主序** `[px,py,pz, vx,vy,vz, ax,ay,az]` 读取，把高度值(~100)填进 X 轴终端速度、把加速度填进 Z 轴终点位置。`yopo_bridge.py` 的 docstring 当时写的正是错误的量主序——**bug 源于照着文档字符串写代码，而不是照着参考实现**。现由 `tests/test_yopo_endstate_layout.js` 锁定。
  2. **力上限做整体等比缩放 → 掉高**。`_controlSO3` 曾用 `if (Fnorm > Fmax) { FdX*=s; FdY*=s; FdZ*=s; }`，把重力补偿一起缩掉，垂直分量低于 `m·g` 必然下沉。现改为移植自 `NetworkControl.cpp: get_Q_from_ACC()` 的**倾角限幅**：拆出 `f = F_d − m·g·e_up`，**只缩 f**（`F_d' = s·f + m·g·e_up`），解析解见 `limitTiltPreservingGravity()`。上限 `so3MaxTiltDeg = 60`（同参考实现默认值），另按推重比兜底以防设置面板把 mass/thrust 改到不可行。

  闭环积分测试（`test_so3_closed_loop.js`）的对照组显示：单独还原布局错位时高度仍稳（倾角限幅在护着），但会飞到离目标 1125 m 之外——两个修复缺一不可。
- **深度没有米制标度**：DA360 输出的是逐帧归一化的相对视差，YOPO 期望"米制深度 / 20"。在线路径目前只用 `DA360_DEPTH_SCALE=2.0` 粗调。离线拟合管线（`fit_da360_metric.py`，`1/z = a·pred_disp + b`）已完成但**未接入在线路径**，也还没有端到端 benchmark。
- **50–100 m 高空的分布外问题，真正的病灶在深度图而不在高度状态**。网络看不到绝对高度（见坐标系一节），所以 `fixed_height` 只决定轨迹末端被拉到哪个水平面。真正 OOD 的是深度输入：训练数据是 `depth_max_m=20` 归一化的地面街谷，100 m 高空视野里大半是天空（无效深度）和远处屋顶，clamp 到 20 m 后几乎全为 1.0，网络读到"四周全空"，于是径直平飞不避障。**在深度标定完成前，空高飞行结果没有参考价值；建议先在 5–30 m 街谷高度验证闭环。**

### 深度米制标定（在线路径尚未启用）

离线拟合管线 `fit_da360_metric.py` 已算出 `1/z = a·pred_disp + b`，参数 (a,b) 存在 `fit_report.json`。在线路径的架子已搭好：

- **标定参数文件**：`experiment_data/depth_calibration.json`（当前占位，a/b 为 null）
- **服务端加载**：`da360_server.py` 新增 `infer_metric()` 方法和 `_load_depth_calibration()` 静态函数。通过环境变量 `DA360_DEPTH_CALIB_PATH` 指向 JSON 文件
- **接入点**：`combined_server.py:/yopo/plan_full` 有注释标记，把 `da360_runner.infer(image)` 改为 `da360_runner.infer_metric(image)` 即完成切换
- **采集流程**：在浏览器控制台调用 `__world.sampleMetricDepthAnchors(__drone.getBodyTransform(), {...})` 获取 anchors；同时调 `POST /depth/raw` 获取对应帧的 `pred_disp.npz`；用 `fit_da360_metric.py` 拟合后将 (a,b) 填入 `depth_calibration.json`

**标定文件无效时自动回退到旧的 per-frame min-归一化 + `DA360_DEPTH_SCALE` 行为**，不影响正常运行。完整流程文档见 `../reference_notes/handoff.md` 的深度问题章节。
- **已修复：视觉模型尺寸与物理尺寸脱节**。`CesiumDrone.glb` 原始包围盒 3.96×1.12×4.67 单位，`droneScale` 旧默认 1.35 → 渲染跨度 6.3 m（等效半径 3.15 m）。现默认 `0.171`（= 0.8/4.668，对应 0.4 m 半径），`minimumPixelSize` 44→8，设置面板滑块范围同步改为 0.05–2.0。附带修掉了 `main.js` 里 `world._aircraftModelEntity`（正确名是 `aircraftModelEntity`，多了下划线）导致 Drone Model Scale 滑块一直是空操作的问题。由 `tests/test_drone_model_scale.js` 锁定。

  **注意物理侧仍是 `collisionRadius = 0.6 m`**（YOPO `vehicle_radius_m` 训练值），刻意不跟着改到 0.4——碰撞判据放宽会让规划器以为贴着飞也安全。`droneSize = 0.3`（相机前移量）同样未动，改它会移动全景采样点、扰动感知闭环。

## 相关项目

| 项目 | 用途 |
|---|---|
| [zwhhhhh9/YOPO_360 @velocity_15ms](https://github.com/zwhhhhh9/YOPO_360/tree/velocity_15ms) | YOPO 全景版本上游 |
| [cn-ryw/YOPO_360_X5_PR @feat/x5-cruise15-release](https://github.com/cn-ryw/YOPO_360_X5_PR/tree/feat/x5-cruise15-release) | 我们的 YOPO fork，对应本地 `/home/ykx/ros1/YOPO_360_v15` |
| [superboySB/MindCloud_World_Fly](https://github.com/superboySB/MindCloud_World_Fly) | 本仓库 upstream |
| [ManifoldTechLtd/MindCloud_World_Fly](https://github.com/ManifoldTechLtd/MindCloud_World_Fly) | 更上游，racing gate / 自适应碰撞检测 / 小巧飞机控制器的来源 |

## Git

- `origin` = `cn-ryw/MindCloud_World_Fly`，`upstream` = `superboySB/MindCloud_World_Fly`
- 工作分支 `feat/da360-metric-depth-prototype`
- 不直接改 main，不 force push，不向 upstream push
