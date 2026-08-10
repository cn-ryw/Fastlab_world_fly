# CLAUDE.md

本文件是本仓库的开发约束。开始工作前还应阅读 `README.md`、`docs/implementation-status-v2.md` 和 `../reference_notes/handoff.md`。

## 项目目标与权威实现

这是 YOPO_360 的 sim-to-sim 迁移，不是重新设计规划器。

- 权威仓库：`/home/ykx/ros1/YOPO_360_v15`
- 权威分支：`feat/x5-cruise15-release`
- 感知/规划/Poly5：`YOPO/test_yopo_ros.py`
- SO3 控制：`Controller/src/so3_control/src/NetworkControl.cpp`
- 权威配置：`YOPO/config/traj_opt.yaml` 与 `x5_cruise15_18m_a12_mask_wc3.yaml`

只允许两类迁移差异：深度来自 Cesium+DA360，而不是仿真 raycast；控制指令直接作用于浏览器刚体，而不是 ROS/mavros。其他差异首先按移植 bug 处理。

关键契约：

- 到达距离固定为 4.0m；不能用 9m `radio_range` 代替。
- YOPO endstate 为轴主序 `[px,vx,ax, py,vy,ay, pz,vz,az]`。
- Sim local 为 `x=east,y=up,z=north`；YOPO world 为 `x=east,y=north,z=up`。
- ERP canonical sensor frame 为 NWU：`+x forward,+y left,+z up`。
- observation 使用 `goal-referencePosition` 和参考加速度，Poly5 起点使用实际位置/速度。

## 启停和拓扑

```bash
./start-all.sh
./launch-firefox-gpu.sh
./stop-all.sh
```

实际运行拓扑：

```text
Firefox
  ├─ 127.0.0.1:8080  scripts/serve.py，直接读取工作树
  └─ 127.0.0.1:5688  Docker combined_server.py
                         ├─ DA360
                         └─ YOPO
```

- combined 服务是单进程、单端口，不存在独立的 5699 YOPO 端口。
- Web 默认不是 Docker 容器；改前端后刷新浏览器即可。
- 两个服务均默认只绑定 loopback。若添加远程访问，必须同时设计认证、CORS 和暴露面，不能只改 host。
- `start-all.sh` 先检查模型、GPU、端口和镜像；health 使用 `curl --noproxy '*'`。只有 Web、`/health` 与 `/yopo/health` 全部可达才打印就绪。
- 不要恢复 `fuser -k` 或默认修改 Clash 的行为。

Python 改动通过以下命令生效：

```bash
docker restart mindcloud-da360-yopo
```

Dockerfile、third_party 或 Python 依赖变化后重建：

```bash
lock_sha="$(sha256sum dependencies.lock.json | awk '{print $1}')"
recipe_sha="$(sha256sum Dockerfile.da360-yopo | awk '{print $1}')"
docker build \
  --build-arg "MINDCLOUD_DEPENDENCY_LOCK_SHA256=$lock_sha" \
  --build-arg "MINDCLOUD_IMAGE_RECIPE_SHA256=$recipe_sha" \
  -f Dockerfile.da360-yopo -t mindcloud-da360-yopo:latest .
```

2026-08-10 已用 base/lock/recipe 三重指纹镜像完成 live 启动。实际 health 为 API v2、`resample=bicubic`、`input_scale≈0.459`、loopback publish，DA360/YOPO checkpoint coverage 100%，19 项 live API integration 全部通过。Firefox 已加载完整页面与模块，但尚未人工完成出生点、真实 depth canvas、目标切换和飞行验收。后续代码或锁文件变化仍必须重新核验，不能沿用本次结论。

## 前端感知与导航状态

`src/panorama-sensor.js` 只有一个可见 depth canvas。DA360 JPEG 必须完成异步解码、请求 identity 校验并成功 `drawImage()` 后，才能设置 `hasDepth=true`。

深度 UI 状态：

- `offline`：API 不可达/超时
- `preview`：无活动目标，调用 `/depth`
- `planning`：有活动目标，调用 `/yopo/plan_full`
- `error`：响应或解码错误
- `stale` outcome：旧 frame、goal 或 generation 已丢弃
- `blocked` outcome：深度 JPEG 已画出，但 relative 模式未授权 YOPO 轨迹

到达和取消后回到 preview，但保留最后一张成功深度图。无目标时日志出现 preview 是正确行为。

目标生命周期由 `goalId + generation` 守护。设置新目标时必须：

1. 递增 generation 并生成新 goalId；
2. abort 在途旧请求；
3. 等待设置目标后的下一张完整 RGB；
4. 响应 identity 全部匹配后才允许画深度或安装 YOPO 轨迹。

## PerceptionFrame 原子契约

`src/perception-frame.js` 将以下内容绑定为不可变快照：

```text
frameId, capturedAt, capture transform, RGB Blob,
actual position/velocity,
reference position/velocity/acceleration,
yaw, projection config
```

全景 capture 开始时快照 planning state 和 transform；capture 完成后才形成该 RGB 的 frame context。`/yopo/plan_full` 必须从对应 `PerceptionFrame` 取 observation，禁止从持续变化的当前 `_yopoPose` 拼接旧 RGB。

该原语也用于原子标定导出。无人机静止、没有活动目标且完整帧就绪时，`window.__captureMetricCalibration(locationId, options)` 会从同一个冻结 `PerceptionFrame` 生成同一 `captureId` 的 RGB、anchors、manifest 和 raw NPZ；禁止手工拼接来自不同帧的文件。manifest 必须保留 frame/location/capture ID、RGB/raw SHA、pose、capture transform、ERP 配置和模型 metadata。

## 服务端 API v2

standalone `da360_server.py` 和 combined `combined_server.py` 通过共享 route registration 提供一致的深度契约：

| 路径 | 契约 |
|---|---|
| `/health` | 模型、尺寸、resample、depth mode、标定、checkpoint 指纹 |
| `/depth` | JPEG 请求；返回 depth JPEG 与轻量 metadata |
| `/depth/raw` | NPZ：raw pred_disp、relative depth、valid mask、metadata |
| `/yopo/health` | YOPO 模型/config/checkpoint 信息 |
| `/yopo/plan_full` | JPEG+完整状态；总返回 depth/identity，只有授权 metric 模式返回 9 元 endstate |
| `/yopo/plan` | 兼容调试路径；relative 模式 409，授权时缓存仍有最大年龄限制 |

所有 planning 数字、frame/goal/generation identity 和参考加速度都是必填；缺字段返回 400，模型异常返回无内部路径的 500。API CORS 只允许本机 Web origin，请求体有上限。

## 默认配置

以代码为准，每次修改默认值都同步更新本文和 README：

```text
panoWidth=384 panoHeight=192 panoFace=96 panoVfov=180
panoMs=20 depthMs=20 panoFrameDelayMs=0 panoFacesPerSlice=2
panoTopPoleGuard=10 panoBottomPoleGuard=2
panoPreloadTimeoutMs=60000
da360UploadScale=0.35

DA360_INPUT_SCALE=0.46
DA360_RESAMPLE=bicubic
DA360_CHANNELS_LAST=0
DA360_DEPTH_MODE=da360-relative

ARRIVAL_DISTANCE_M=4.0
collisionRadius=0.6
droneScale=0.171
```

`DA360_CHANNELS_LAST=1` 曾在真实 Flask 进程中造成约 200 倍退化；没有同进程 benchmark 不得重新启用。

## 米制标定

DA360 输出 raw disparity，YOPO 期望米制深度。正式关系固定为：

```text
inverse_depth_1_per_m = a * pred_disp + b
depth_m = 1 / inverse_depth_1_per_m
```

已编码能力：

- `/depth/raw` 输出 raw pred_disp 和输入/模型/frame metadata
- anchor 使用 canonical ERP helper 与真实 capture transform
- anchor 分辨率到 raw 分辨率采用像素中心映射、水平 wrap 双线性采样
- fitter 只应用一次 Huber loss，支持多 capture 和 leave-one-location-out
- metric calibration 启动时一次加载并绑定模型、输入尺寸、resample 和 checkpoint SHA
- metric loader 必须校验标定的 projection/JPEG fingerprint；每次请求通过 `X-Projection-Config` 携带冻结帧的完整投影配置，并在 GPU 推理前同时匹配 RGB request size、ERP/FOV、pole guard、JPEG quality 和 upload scale
- calibration anchor 只有在六个 cubemap 面各自在复制 RGB 像素时都已报告 tiles ready 才可导出；不能用采集结束后的单次 `tilesLoaded` 状态代替逐面 provenance
- `DA360_DEPTH_MODE=da360-metric` fail closed；校准不合格时拒绝启动
- `window.__captureMetricCalibration()` 原子导出同 captureId 的 RGB、anchors、manifest 与 raw NPZ

尚未完成/验收：

- `../experiment_data/depth_calibration.json` 仍是 `a/b=null` 的旧占位文件
- 没有真实 RGB/raw/anchor 数据集
- 没有达到 4 地点×3 capture 的收集门槛
- 没有 DA360 metric、Cesium truth parity 或低空闭环结果

验收门槛：0.5–20m 留出数据 median AbsRel ≤15%、p90 AbsRel ≤30%、10m 内 p90 绝对误差 ≤1m、有效 anchor ≥70%，至少 4 地点、12 captures，并使用 leave-one-location-out。

未通过前只能称为 `da360-relative`，它只能 preview/采集，不能产生可应用轨迹；不得在文档、health 或实验结论中称为米制深度。`cesium-truth` fallback 尚未实现，不能写成现有能力。

## 测试

无浏览器/GPU测试：

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

这些宿主单元测试要求 NumPy/SciPy/Pillow/pytest；backend contract 放到已有 combined 镜像中运行，避免依赖宿主 Flask。宿主缺少 Flask 时 backend contract 模块会整体 skip；若要在宿主实际执行它，则需额外安装 Flask。连接 5688 的旧测试已标记 `integration` 并被 `pytest.ini` 默认排除；必须用下列命令覆盖默认 `addopts`：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -o addopts='' -m integration tests/ -v
```

执行后应分别报告单元测试与真实集成测试，不能混写成“全部验证”。

真实验收仍需 Firefox+GPU+城市 tiles：

- 设置目标后的下一张完整帧进入 planning
- 到达/取消后回 preview，深度仍更新
- 3.9m 到达、4.1m 不到达、8.9m 不提前结束
- 平均有效规划率 ≥15Hz、p95 规划间隔 ≤100ms
- p95 capture-to-apply ≤150ms、服务 p95 ≤50ms
- 低空街谷无碰撞、NaN、旧轨迹复活

截至 2026-08-10，上述真实门禁尚未完成。修复后一次 relative `/yopo/plan_full` 诊断为 DA360 44.6ms、端到端 63.3ms，但它未授权 YOPO，既不是统计样本也不能证明 15Hz。修复前基线约为 depth 3.1Hz、YOPO 2.9Hz；不要把 API/单元测试通过写成 15Hz 已达标。

六面 capture 当前每两面 `requestAnimationFrame` yield，并支持 AbortSignal；FlightLogger 已记录分段耗时、唯一 planning frame、drop reason 和 p95。它们属于“已实现、待 live benchmark”，不能写成 15Hz 已通过。

门禁实现位于 `config/perception-sweep.json`、`scripts/evaluate_closed_loop.py` 和 `scripts/evaluate_perception_quality.py`。真实数据必须按该配置的候选顺序跑完并保存报告，不能凭 URL 参数或平均延迟口头选择配置。

## 安全和文件边界

`scripts/serve.py` 不是项目根目录文件服务器，只允许：

- `/`、`/index.html`
- `/src/`
- `/asset/`
- `/ThirdParty/Cesium/`
- 同源 `/api/path/<safe-name>.json`

禁止重新暴露 `.git`、Markdown、scripts、tests、checkpoint、目录列表或任意 traversal。路径 API 只允许同源且原子写入。

模型、PID、raw NPZ/NPY 和实验大文件不得进入 Git。锁定的版本与本机 checkpoint SHA 记录在 `dependencies.lock.json`。

依赖核验命令为 `python3 scripts/verify_dependencies.py`；只有核验通过并记录镜像 ID 后，才可开始 live 验收。

## Git

- `origin=https://github.com/cn-ryw/MindCloud_World_Fly.git`
- `upstream=https://github.com/superboySB/MindCloud_World_Fly.git`
- 工作分支：`feat/da360-metric-depth-v2`
- 归档分支：`archive/da360-prototype-c0b82d5-20260809`
- 归档 tag：`archive-c0b82d5-20260809`
- 仓外 bundle：`../backups/MindCloud_World_Fly-c0b82d5-20260809.bundle`

不向 upstream push，不 force-push。`origin` 的 v2 分支未确认推送前，不得声称本轮工作已有远端备份。

依赖锁定：Cesium 1.117、PlayCanvas 2.17.2（revision `2892d5e`）、DA360 commit `93dd3fc32e8e8751ac1e4b26ff1a575adfc55661`。本地 Cesium 是 1.117；`index.html` 的 CDN fallback 必须同步保持 1.117，不能混用 1.121。
