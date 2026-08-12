# MindCloud World Fly

浏览器内的 Google Photorealistic 3D Tiles 无人机仿真器。目标导航链路为：Cesium 六面渲染生成 384×192 ERP RGB，DA360 估计米制深度，YOPO 输出 Poly5 轨迹，浏览器内 SO3 控制器跟踪轨迹。`start-all.sh` 默认使用项目负责人批准的 sim-to-sim metric 标定；显式 `da360-relative` 只用于预览和标定采集，不授权 YOPO 轨迹。

本项目正在做 YOPO_360 的 sim-to-sim 迁移。权威参考是 `/home/ykx/ros1/YOPO_360_v15`；除深度来源和控制下发方式外，其余行为应与参考实现一致。

## 运行要求

- NVIDIA GPU、驱动和 NVIDIA Container Toolkit
- Docker Engine、Python 3、curl
- 宿主无 GPU 测试需 pytest、NumPy、SciPy 和 Pillow；缺少 Flask 时 backend contract 模块会整体 skip，可改用 combined 镜像执行该 contract
- Firefox；当前机器上 Chrome/Chromium 与 PRIME offload 的 WebGL 组合不稳定
- 可访问 Cesium Ion 和 Google Photorealistic 3D Tiles
- DA360 large 与 YOPO epoch10 checkpoint；可读版本、来源与本地路径见 `dependencies.versions.json`

## 推荐启动方式

```bash
# 首次或 Dockerfile/third_party 变化后构建
python3 scripts/verify_dependencies.py
docker build -f Dockerfile.da360-yopo -t mindcloud-da360-yopo:latest .

# 启动 combined API 与本地 Web
./start-all.sh

# 打开启用 NVIDIA offload 的 Firefox
./launch-firefox-gpu.sh

# 停止本项目服务
./stop-all.sh
```

`start-all.sh` 的实际拓扑只有两个本机监听端点：

```text
Firefox ── http://127.0.0.1:8080 ── scripts/serve.py（直接读取工作树）
   │
   └───── http://127.0.0.1:5688 ── Docker combined_server.py
                                      ├─ DA360
                                      └─ YOPO
```

- 8080 不是 Docker Web 容器；修改 `src/*.js` 或 `index.html` 后刷新即可。
- 5688 是单进程、单端口的 DA360+YOPO 服务，只发布到 loopback。
- 启动检查显式绕过代理，并要求 `/health` 与 `/yopo/health` 同时通过；失败会非零退出。
- `launch-firefox-gpu.sh` 保留系统/环境代理、本机地址走 `NO_PROXY`，并用不含凭据的 Google Tiles 根地址做连通性预检。Google API key 通过官方支持的 `X-Goog-Api-Key` 请求头传递，不再进入浏览器请求 URL；应用错误输出还会删除 URL 的 userinfo、query 与 fragment，并对 tile 错误指数节流。
- Clash 配置不会再被默认修改；只有设置 `MINDCLOUD_FIX_CLASH=1` 才执行修复脚本。

默认 checkpoint 路径：

```text
third_party/DA360/checkpoints/DA360_large.pth
/home/ykx/ros1/YOPO_360_v15/YOPO/saved/YOPO_55/epoch10.pth
```

可分别用 `DA360_MODEL_PATH_HOST` 和 `YOPO_MODEL_PATH_HOST` 覆盖。

## 使用方式

1. 打开 `http://127.0.0.1:8080`，进入 Google 3D Tiles 模式。
2. 在 placement 模式按住 `I` 点击地面或建筑，设置出生点。
3. 用 `W/A/S/D` 微调，设置出生高度，按 `O` 确认。
4. 选择视角并进入飞行。
5. SO3 模式下按住 `G` 点击场景或雷达设置目标；`G`+滚轮修改目标高度；`C` 取消。

### T8L 串口遥控

RadioMaster T8L 以 VCP 模式接入后，先运行 `./launch.sh --input-status` 检查设备；若提示权限不足，运行一次
`./launch.sh --setup-input`，重新插拔遥控器（新增用户组后可能需要重新登录）。使用 Chrome/Chromium 打开
`127.0.0.1` 页面，在 Settings 中点击 **Connect T8L** 并在浏览器设备选择器中选中 RadioMaster。

- 默认 CH1/CH2/CH3/CH4 为 Roll/Pitch/Throttle/Yaw；通道映射、反向、死区、Rate/Expo 继续在 Settings 中配置。
- Arm 和 Mode 默认不绑定。点击对应的 Assign 后拨动开关完成学习；T8L 开关自动使用 level 模式，避免首帧误触。
- Mode 低位为 Easy，高位为 SO3。SO3 中 CH1/CH2 按 YOPO 遥控算法持续移动黄色 8 m 航点，高度保持为无人机当前高度。
- 250 ms 未收到合法串口帧会清除活动航点和旧轨迹、强制 SO3 原地悬停，但保留当前解锁状态；物理断开后需要再次点击 Connect T8L。

串口参数固定为 460800 baud，识别 VID/PID `19f5:5740`；当前稳定设备名通常为
`/dev/serial/by-id/usb-RADIOMASTER_RadioMaster_T8L_RADIOMASTER-if00`。Web Serial 必须运行在 localhost 或 HTTPS 安全上下文中。

到达阈值为 **4.0m**，来自权威 YOPO `traj_opt.yaml`。`radio_range` 不是到达距离。
终端候选不会被目标点直线环截断，而是沿原 Poly5 执行到底；随后须在无碰撞条件下将三维速度降到
0.75m/s以内并连续稳定0.4s，才报告 arrived。

## DA360 深度状态

右下角只有一个可见 depth canvas。服务返回的 JPEG 完成异步解码并成功绘制后，前端才设置 `hasDepth=true`。

状态含义：

| 状态 | 含义 |
|---|---|
| `offline` | API 不可达或请求超时 |
| `preview` | 无活动目标，调用 `/depth` 持续预览 DA360 深度 |
| `planning` | 有活动目标，下一完整帧调用 `/yopo/plan_full`；只有合格 metric 模式才安装轨迹 |
| `error` | 服务可达，但响应、解码或契约校验失败 |
| `preview/planning · stale` | 响应属于旧 frame、goal 或 generation，已丢弃 |
| `planning · blocked` | 深度画面已更新，但当前 relative 模式不授权轨迹 |

到达或取消后出现 `preview` 是正常行为，不表示 DA360 离线。若设置目标后已经产生新的完整 RGB 帧，状态仍不进入 `planning`，才属于目标/帧状态故障。

## 在线 API

combined 服务与 standalone DA360 服务共享 API v2 深度契约：

| 路径 | 用途 |
|---|---|
| `GET /health` | DA360 模型、输入尺寸、resample、depth mode、标定和 checkpoint 覆盖率 |
| `POST /depth` | JPEG → 深度预览 JPEG 与轻量 metadata |
| `POST /depth/raw` | JPEG → raw `pred_disp`、相对深度、valid mask 和 metadata 的 NPZ |
| `GET /yopo/health` | YOPO 模型、配置和 checkpoint 覆盖率 |
| `POST /yopo/plan_full` | JPEG + 完整状态 → identity 与 YOPO endstate；兼容调用可用 `include_preview=1` 附带深度预览，飞行链固定使用 `0` |
| `GET /yopo/preview` | 按 frame/goal/generation 读取最近 4 个规划帧之一的缓存深度预览，不重复运行 DA360 |
| `POST /yopo/plan` | 兼容/离线调试路径；relative 模式返回 409，不能绕过授权 |

`/yopo/plan_full` 使用 `frame_id/goal_id/generation` 防止旧响应覆盖新会话。前端 `PerceptionFrame` 将 RGB、采集 transform、模拟时钟、实际状态、参考状态、yaw、capture profile 和投影配置绑定为不可变快照，避免把旧 RGB 与新位姿配对。回包后按捕获时域 fast-forward 原 Poly5；安装年龄不得超过 `min(0.25s, 0.25T)`，应用状态相对 suffix 参考的位置/速度误差分别不得超过 1.2m/3m/s。规划响应保留兼容的顶层 `endstate/score/traj_time`，并附带 `planning_diagnostics.schema_version=1` 的选中候选诊断。顶层 `endstate` 才是 sim Y-up 世界系轴主序 `[px,vx,ax, py,vy,ay, pz,vz,az]`；`selected_endstate_raw` 是网络 tanh 后、lattice 解码前的9个无量纲系数，不是物理endstate。candidate ID 等同 CNN flatten action ID，lattice ID 是反序格点 ID，score 是越低越优的组合代价而非置信度；`terminal_speed_mps` 是三维速度模长。所有 JPEG 深度请求还会用 `X-Projection-Config` 发送该帧的 ERP、上传尺寸和 JPEG 配置契约；metric 模式在 GPU 推理前逐字段与 accepted calibration 核对。

## 当前默认值

代码默认值是唯一真相；下表对应当前 `src/panorama-sensor.js` 与 `start-all.sh`：

| 参数 | 默认值 |
|---|---:|
| `panoProfile` | `flight` |
| ERP 输出 | 384×192 |
| `panoFace` | 96 |
| `panoFacesPerSlice` | 2 |
| `panoramaFarMeters` | 1200m（仅隐藏六面相机；不改变主视图） |
| `panoramaLeanStreaming` | 1（关闭隐藏相机的无效瓦片过取；0 恢复旧策略） |
| `flightPreloadRadius` | 500m |
| `panoMs` | 20ms |
| `depthMs` | 20ms |
| `yopoMaxFrameAgeMs` | 250ms（冻结帧绝对硬上限；单段还限制为不超过 25% 时域） |
| `panoFrameDelayMs` | 0ms |
| `da360UploadScale` | 0.35 |
| `DA360_INPUT_SCALE` | 0.46（约 476×238） |
| `DA360_RESAMPLE` | bicubic |
| `DA360_CHANNELS_LAST` | 0 |
| `DA360_DEPTH_MODE` | da360-metric（`start-all.sh` 默认） |
| 到达条件 | 合格终端 Poly5 完整执行后，≤4.0m、无碰撞、三维速度≤0.75m/s并持续0.4s |
| 碰撞半径 | 0.6m |

URL 参数示例：

```text
http://127.0.0.1:8080/?panoMs=40&depthMs=40&panoFace=96
http://127.0.0.1:8080/?da360UploadScale=1
http://127.0.0.1:8080/?panoTopPoleGuard=0&panoBottomPoleGuard=0
```

## 米制标定状态

**2026-08-10 LOLO scale-only 标定已由项目负责人于 2026-08-11 人工批准用于 sim-to-sim，`start-all.sh` 默认加载正式路径 `../experiment_data/depth_calibration.json`。** 当前已实现：

- combined 与 standalone 的 `/depth/raw`
- canonical ERP anchor 方向与 capture transform 旋转
- 不同分辨率间的像素中心映射、水平 wrap 双线性采样
- `1/z = a·pred_disp+b` 的单次 Huber 拟合、留出验证和 fail-closed 标定加载
- `PerceptionFrame` 原子绑定 RGB 与采集/规划上下文
- `window.__captureMetricCalibration(locationId, options)` 从同一冻结帧导出同一 `captureId` 的 RGB、anchors、manifest 和 raw NPZ
- `flight`/`calibration` 两套 capture profile；标定导出只接受 `calibration`，设置飞行目标会强制切回 `flight`

2026-08-10 已完成 4 地点×3 静止姿态、共 12 captures 的原子采集和 leave-one-location-out 拟合。证据在 `../experiment_data/metric_fit-lolo-20260810-12capture/fit_report.json`；样本为 site-a `try-03/04/05`、site-b `try-01/02/03`、site-c `try-02/03/04`、site-d `try-02/04/05`。四件套 identity、地点覆盖、输入/投影契约和 LOLO 完整性均通过：1536 个 anchors 中 1140 个有效（74.22%），其中 833 个在 0.5–20m 范围。

自动评估事实保持不变：**拟合结果未通过预设精度门禁**。报告中的 `success=true` 只表示 fitter 正常完成，自动 `acceptance.passed` 为 false：

| 留出地点 | median AbsRel | p90 AbsRel | 10m 内 p90 绝对误差 |
|---|---:|---:|---:|
| site-a | 0.399 | 0.544 | 3.326m |
| site-b | 0.366 | 2.125 | 5.959m |
| site-c | 0.383 | 0.443 | 3.545m |
| site-d | 0.313 | 0.982 | 4.051m |

对应门禁分别为 ≤0.15、≤0.30、≤1m，四个 LOLO fold 均失败。全量 scale-only 结果为 `a=0.0011892812185910185,b=0`，全量 median AbsRel=0.376、p90 AbsRel=1.507、10m 内 p90 绝对误差=4.054m。项目负责人在检查实时深度效果后，明确接受这组固定尺度作为本项目后续 sim-to-sim 基线；正式文件同时保留 `acceptance.passed=false` 和上述原始指标，并通过 `manual_acceptance` 记录人工批准，不会伪装成自动精度门禁通过。运行 health/规划响应将其报告为 `validated-da360-metric`，同时报告 `calibration_acceptance_method=manual-user`、`calibration_automatic_gate_passed=false` 和 `calibration_acceptance_scope=sim-to-sim`。

标定采集必须显式使用 `panoProfile=calibration`，让六个 cubemap 面逐面等待 tiles ready；同时把投影契约的全部参数写在 URL 中。旧参数 `panoCaptureAnyway=0` 仅保留为向后兼容映射，不应再作为新命令的主入口。例如：

```bash
./launch-firefox-gpu.sh \
'http://127.0.0.1:8080/?panoProfile=calibration&panoPreloadRequired=0&panoWidth=384&panoHeight=192&panoFace=96&panoVfov=180&panoJpeg=0.74&da360UploadScale=0.35&panoFaceTileTimeoutMs=6000&panoFaceTileQuietMs=650'
```

在无人机静止、没有活动目标且已有完整 `PerceptionFrame` 时，可从 Firefox 控制台一次导出四个同 ID 文件。上述标定 URL 对每个面设置 650ms tile quiet，六面理论静默等待就是 3.9s；加上渲染、复制和投影后，实测约 4.2s/capture。旧日志把等待时间合并进 `render`，不能据此断言 GPU 渲染本身耗时 4.1s。该等待只用于静态标定，因此显式给出较宽的 freshness 窗口：

```js
await window.__captureMetricCalibration('site-a', {
  captureId: 'site-a-01',
  maxFrameAgeMs: 60000
});
// site-a-01-rgb.jpg / site-a-01-anchors.json /
// site-a-01-manifest.json / site-a-01-raw.npz
```

`manifest.json` 记录 session/frame/capture/location ID、三份 artifact 文件名与字节数、pose、capture transform、ERP/JPEG/上传尺寸配置和模型 metadata。默认会触发四次下载；传入 `{ download: false }` 可只返回内存中的 artifacts。同一帧和同一 capture ID 不能重复消费，fitter 还会拒绝重复 session/frame、capture ID 和近重复姿态。

若重采或复现实验，拟合命令仍要求每个地点三个不同静止姿态、共 4 地点×3 capture；下面是通用展开模板，不代表上述已归档样本的实际 ID：

```bash
fit_args=()
for site in a b c d; do
  for pose in 01 02 03; do
    prefix="site-${site}-${pose}"
    fit_args+=(
      --raw "${prefix}-raw.npz"
      --anchors "${prefix}-anchors.json"
      --manifest "${prefix}-manifest.json"
      --rgb "${prefix}-rgb.jpg"
    )
  done
done
python3 scripts/fit_da360_metric.py "${fit_args[@]}" \
  --output experiment_data/metric_fit-v2
```

只有 `fit_report.json` 同时通过留出误差、有效 anchor、至少 4 地点和至少 12 次采集门禁，fitter 才会自动生成 `depth_calibration.json`；人工接受则必须像当前正式文件一样保留失败的自动报告并提供完整 `manual_acceptance` 元数据。当前默认 metric 服务直接启动：

```bash
./start-all.sh
```

如需回到只预览、不向 YOPO 提供可应用轨迹的相对深度模式，必须显式运行：

```bash
DA360_DEPTH_MODE=da360-relative \
./start-all.sh
```

标定 schema、有限数值、模型/模型输入尺寸、请求 RGB 尺寸、runtime projection、JPEG 和 resample 契约仍必须完全匹配，否则 metric 模式会在启动或请求阶段拒绝运行；原始 `da360-relative` 也仍然只能预览，绝不会被直接送入 YOPO。人工批准范围仅为 sim-to-sim，不等价于真实传感器绝对深度精度或自动 LOLO 门禁通过。Anchor 还要求六个 cubemap 面在各自像素被复制时均已报告 tiles ready；仅在导出时看到 tiles idle 不算合格。

### DA360 输入分辨率 pilot

2026-08-10 的 native-resolution pilot 使用同一张解码后仅 134×67 的 JPEG，分别送入 476×238 和 1036×518 模型输入。同步推理从 25.6ms 增至 151.9ms（5.94×）。site-d 指标仅小幅混合变化，而 site-a 明显恶化：median AbsRel 0.276→0.569、p90 0.544→2.468、10m 内 p90 误差 2.666→19.945m。由于上游 JPEG 没有新增空间信息，**不要把 live `DA360_INPUT_SCALE` 从约 476×238 改到 1036×518**；该 pilot 否决了升分辨率候选，不构成 metric 验收。

## 测试与当前门禁

```bash
# 无浏览器/GPU的 JS 测试
for test_file in tests/*.js; do node "$test_file" || break; done

# 无 GPU 的静态服务与拟合单元测试
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_fit_da360_metric.py \
  tests/test_serve_security.py \
  tests/test_evaluation_gates.py

# 宿主没装 Flask 时，在已有 combined 镜像中跑 API contract
docker run --rm --entrypoint /bin/bash -v "$PWD:/workspace:ro" \
  mindcloud-da360-yopo:latest \
  -lc 'cd /workspace && python3 tests/test_backend_contract.py'

# 需要实际运行服务的 19 项集成测试必须显式选择
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -o addopts='' -m integration tests/ -v
```

也可以在装齐测试依赖的宿主环境中运行 backend contract。宿主缺少 Flask 时，`tests/test_backend_contract.py` 会在 module level 标记 skip，而不是 collection 失败；要获得 contract 通过证据，必须在 combined 镜像中运行或先安装 Flask，并将 skip 与 pass 分开报告。

当前工作树的自动测试已复验通过；精确的 passed/skipped/deselected 计数以最终 test sweep 保存的原始输出为准，不在这里绑定。真实 Firefox+GPU、live 服务加载当前工作树后的端到端链路和可信 15Hz 仍未验收；旧 CUDA integration 或旧镜像结果不能替代这些 live 证据。

真实 flight log 和同场景质量数据生成后，使用已实现的门禁工具出报告：

```bash
python3 scripts/evaluate_closed_loop.py flight-log.json \
  --warmup-frames 30 \
  --min-duration-s 60 \
  --min-planning-frames 900 \
  --min-physics-coverage 0.95 \
  --output closed-loop-report.json
python3 scripts/evaluate_perception_quality.py \
  --baseline baseline.npz \
  --candidate 6x96-sse512=candidate-96.npz \
  --candidate 6x80-sse768=candidate-80.npz \
  --candidate 6x64-sse1024=candidate-64.npz \
  --output perception-quality-report.json
```

evaluator v2 会先丢弃 30 个合法唯一 planning frames，再要求至少 60s、900 个计量 frames。每份日志必须声明且只包含一个 `navigationSession(goalId,generation)`，其中 generation 必须是非负 JavaScript-safe integer；日志还需提供有限的 `monotonicStartMs` 和正数 `duration_s`。所有合法 apply 时间都必须落在声明的 session 起点至终点内（终点只容许 5.1ms 的日志取整余量）；第一次和最后一次合法 apply 距相应 session 边界都不得超过 `250+5.1ms`，避免用中段高频规划掩盖目标刚设置后或结束前的停摆。每帧必须有浏览器实际轨迹安装回执 `trajectoryApplied=true`、完整 goal/generation/frame identity、允许的 `da360-metric` 或 `cesium-truth`、稳定且非空的 calibration ID 与 service session ID，以及严格递增的 apply/physics 时间。任何声称 applied 或 `trajectoryApplied=true` 的 planning 事件，都必须同时满足 `outcome=applied`、`planningAuthorized=true` 和 `trajectoryApplied=true`；blocked/rejected/unauthorized 事件不得声称轨迹已安装。日志中的 `resolvedUrl` 只接受本机 `127.0.0.1|localhost` 的 `/|/index.html` 入口，采用已审查的全景/性能参数正向白名单，并保留实际影响预加载/cache 状态的 5 个 `flightPreload*` 数值参数；userinfo、fragment、端点 URL、未知参数及 token/secret/password/API key/auth/credential 等查询项均不记录。evaluator 会再次独立校验该 URL，缺失、解析失败、非本机/非法路径或重复参数均 fail closed。

物理时间戳还必须覆盖至少 95% 的 planning 计量窗口，并满足最小帧数 `ceil(measurementDurationMs × 0.95 / 33.3ms) + 1`；窗口恰为 60s 时至少需要 1713 个 physics frames，不能用首尾两个时间戳伪造 p95。`da360-relative`、只收到服务响应、重复帧、矛盾回执、session 越界或 service session 漂移都会 fail closed；因此当前 relative 模式不可能被误报成 15Hz 通过。

六面 capture 仍是六次 Cesium scene render，默认按 `panoFacesPerSlice=2` 分成三批，限制单次同步 Cesium 工作；3-face 实验在最新四份真实日志中使 capture/physics p95 明显退化，因此恢复为 2，URL 仍可显式选择 1/3/6 做 A/B。隐藏相机的默认远裁剪为 `panoramaFarMeters=1200`，并关闭 sibling/flight-destination 等无效过取；这不改变 384×192、6×96、FOV、ERP 方向或米制标定参数。旧策略可用 `panoramaFarMeters=15000000&panoramaLeanStreaming=0` 完整回滚。projector texture 在尺寸不变时改用 `texSubImage2D` 复用。飞行期 planning 固定 `include_preview=0`，轨迹安装后立即释放 request gate；操作员深度图约每 2s 从按 frame/goal/generation 索引的 4 帧 LRU 读取，后台 latest-only 合并且不重复运行 DA360。2s 周期用于降低单线程服务中 CPU 着色/JPEG 对规划 p95 的影响，不是控制门禁。timing 现拆为 capture/render/scheduler、fetch/header/json、response bytes、DA360、YOPO、trajectory apply、preview fetch/decode/draw 和 gate hold。

2026-08-11 在同一台 Firefox 153、RTX 5070 Ti、500 m 完整预载和相同 30 s 计量窗口中，默认候选两次得到 `capture p95=53/45 ms`、`RAF/physics-frame p95=17.1/17.1 ms`、preview age p95=`156/97 ms`；旧远裁剪/流式对照为 `62/33.1/187 ms`，未通过 capture 60 ms 门槛。因此保留 1200 m + lean streaming 默认值。该 smoke 只证明六面 preview/主线程预算改善，不能替代带航点的 60 s、900 个有效 planning frame 的 15 Hz/150 ms 正式闭环验收。可重复执行：

```bash
python3 scripts/benchmark_firefox_preview.py --sample-seconds 30 --startup-timeout 360
```

流水化不会再因 RGB N+1 先完成就丢弃仍新鲜的规划响应 N：轨迹应用只核对 goal/generation/mode 和 250ms 硬龄限；canvas 则按递增 request ID 显示最新可用 depth。因此画面允许比当前 RGB 落后一帧，日志用 `depthPreviewLagFrames/depthPreviewAgeMs` 明示，而旧会话或已被更新 depth 超越的结果仍 fail closed。规划 control outcome 在 JPEG 解码前同步记录且每请求唯一；随后画面成功、失败或 stale 只产生 `mode=depth-preview` diagnostic，不进入 15Hz 规划计数。

2026-08-11 的最小 live performance smoke 应显式使用 flight profile。进入飞行前的一次性 panorama preload 会逐面等待 tiles 稳定；进入 flight 后的实时 capture 仍是 zero-wait/capture-anyway，不会把 6×650ms tile quiet 带入 YOPO 循环，也不会把 `rgbTiles=6/6` 作为自动飞行硬门禁：

```bash
./launch-firefox-gpu.sh \
'http://127.0.0.1:8080/?panoProfile=flight&panoPreloadRequired=0&panoWidth=384&panoHeight=192&panoFace=96&panoFacesPerSlice=2&panoVfov=180&panoJpeg=0.74&da360UploadScale=0.35&panoMs=20&depthMs=20&panoramaTileSse=512&panoramaFarMeters=1200&panoramaLeanStreaming=1'
```

如果当前地址仍含旧 `panoCaptureAnyway=0`，单独执行 `location.reload()` 会原样保留查询参数，页面仍会进入 `calibration`。最稳妥的做法是直接打开上面的显式 `panoProfile=flight` URL；也可在当前 Firefox 控制台执行下列命令，删除 legacy/别名参数、写入显式 flight profile 后跳转：

```js
(() => {
  const url = new URL(location.href);
  url.searchParams.delete('panoCaptureAnyway');
  url.searchParams.delete('panoCaptureProfile');
  url.searchParams.set('panoProfile', 'flight');
  location.assign(url.href);
})();
```

页面重新就绪后用 `window.__getPanoramaCaptureProfile()` 核对返回值为 `flight`。`window.__setPanoramaCaptureProfile('flight')` 只切换当前页面运行态并取消在途 capture；若地址仍含 legacy 参数，下一次 reload 仍会按 URL 恢复为 calibration。设置目标也会自动强制 flight。本次 smoke 先保存 timing 拆分和原始 log；只有 evaluator v2 对授权 metric/truth 的实际轨迹回执报告通过，才能称为 15Hz。`cesium-truth` dense depth 目前只有 PostProcessStage depthTexture→eye-space ray range→RGB24/ERP→单次 readback→anchor parity 的设计，尚未实现或 live smoke，`/yopo/plan_depth` 二进制路由也不存在。详见 `docs/implementation-status-v2.md`。

## Git 与依赖

- `origin`：`cn-ryw/MindCloud_World_Fly`，只用于本项目分支
- `upstream`：`superboySB/MindCloud_World_Fly`，只拉取参考，不向其 push
- 当前实现分支：`feat/da360-metric-depth-v2`
- 原型归档：`archive/da360-prototype-c0b82d5-20260809` 和 tag `archive-c0b82d5-20260809`
- 依赖版本、来源与本地路径：`dependencies.versions.json`

启动前也可单独核对本地 bundle、配置和 checkpoint：

```bash
python3 scripts/verify_dependencies.py
```

不要提交 PID、checkpoint、raw NPZ/NPY 或实验数据大文件。
