# MindCloud World Fly

浏览器内的 Google Photorealistic 3D Tiles 无人机仿真器。目标导航链路为：Cesium 六面渲染生成 384×192 ERP RGB，DA360 估计米制深度，YOPO 输出 Poly5 轨迹，浏览器内 SO3 控制器跟踪轨迹。当前默认 `da360-relative` 只用于预览和标定采集，不授权 YOPO 轨迹。

本项目正在做 YOPO_360 的 sim-to-sim 迁移。权威参考是 `/home/ykx/ros1/YOPO_360_v15`；除深度来源和控制下发方式外，其余行为应与参考实现一致。

## 运行要求

- NVIDIA GPU、驱动和 NVIDIA Container Toolkit
- Docker Engine、Python 3、curl
- 宿主无 GPU 测试需 pytest、NumPy、SciPy 和 Pillow；缺少 Flask 时 backend contract 模块会整体 skip，可改用 combined 镜像执行该 contract
- Firefox；当前机器上 Chrome/Chromium 与 PRIME offload 的 WebGL 组合不稳定
- 可访问 Cesium Ion 和 Google Photorealistic 3D Tiles
- DA360 large 与 YOPO epoch10 checkpoint，具体哈希见 `dependencies.lock.json`

## 推荐启动方式

```bash
# 首次或 Dockerfile/third_party 变化后构建
lock_sha="$(sha256sum dependencies.lock.json | awk '{print $1}')"
recipe_sha="$(sha256sum Dockerfile.da360-yopo | awk '{print $1}')"
docker build \
  --build-arg "MINDCLOUD_DEPENDENCY_LOCK_SHA256=$lock_sha" \
  --build-arg "MINDCLOUD_IMAGE_RECIPE_SHA256=$recipe_sha" \
  -f Dockerfile.da360-yopo -t mindcloud-da360-yopo:latest .

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

到达阈值为 **4.0m**，来自权威 YOPO `traj_opt.yaml`。`radio_range` 不是到达距离。

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
| `GET /health` | DA360 模型、输入尺寸、resample、depth mode、标定和 checkpoint 指纹 |
| `POST /depth` | JPEG → 深度预览 JPEG 与轻量 metadata |
| `POST /depth/raw` | JPEG → raw `pred_disp`、相对深度、valid mask 和 metadata 的 NPZ |
| `GET /yopo/health` | YOPO 模型、配置和 checkpoint 指纹 |
| `POST /yopo/plan_full` | JPEG + 完整状态 → 深度 JPEG 和 identity；仅授权模式返回 YOPO endstate |
| `POST /yopo/plan` | 兼容/离线调试路径；relative 模式返回 409，不能绕过授权 |

`/yopo/plan_full` 使用 `frame_id/goal_id/generation` 防止旧响应覆盖新会话。前端 `PerceptionFrame` 将 RGB、采集 transform、实际状态、参考状态、yaw 和投影配置绑定为不可变快照，避免把旧 RGB 与新位姿配对。所有 JPEG 深度请求还会用 `X-Projection-Config` 发送该帧的 ERP、上传尺寸和 JPEG 指纹；metric 模式在 GPU 推理前逐字段与 accepted calibration 核对。

## 当前默认值

代码默认值是唯一真相；下表对应当前 `src/panorama-sensor.js` 与 `start-all.sh`：

| 参数 | 默认值 |
|---|---:|
| ERP 输出 | 384×192 |
| `panoFace` | 96 |
| `panoFacesPerSlice` | 2 |
| `panoMs` | 20ms |
| `depthMs` | 20ms |
| `panoFrameDelayMs` | 0ms |
| `da360UploadScale` | 0.35 |
| `DA360_INPUT_SCALE` | 0.46（约 476×238） |
| `DA360_RESAMPLE` | bicubic |
| `DA360_CHANNELS_LAST` | 0 |
| `DA360_DEPTH_MODE` | da360-relative |
| 到达距离 | 4.0m |
| 碰撞半径 | 0.6m |

URL 参数示例：

```text
http://127.0.0.1:8080/?panoMs=40&depthMs=40&panoFace=96
http://127.0.0.1:8080/?da360UploadScale=1
http://127.0.0.1:8080/?panoTopPoleGuard=0&panoBottomPoleGuard=0
```

## 米制标定状态

**DA360 metric 尚未验收；默认 `da360-relative` 是 preview/采集模式，正式闭环当前 fail closed。** 当前已实现：

- combined 与 standalone 的 `/depth/raw`
- canonical ERP anchor 方向与 capture transform 旋转
- 不同分辨率间的像素中心映射、水平 wrap 双线性采样
- `1/z = a·pred_disp+b` 的单次 Huber 拟合、留出验证和 fail-closed 标定加载
- `PerceptionFrame` 原子绑定 RGB 与采集/规划上下文
- `window.__captureMetricCalibration(locationId, options)` 从同一冻结帧导出同一 `captureId` 的 RGB、anchors、manifest 和 raw NPZ

当前 `../experiment_data/depth_calibration.json` 的 `a/b` 仍为 null，没有任何标定通过验收。原子导出已实现，但 4 地点×3 静止姿态的实际采集、leave-one-location-out 报告和 accepted calibration 仍待完成。

标定采集应显式使用 `panoCaptureAnyway=0`，让六个 cubemap 面逐面等待 tiles ready；同时把所有会进入 projection fingerprint 的参数写在 URL 中。例如当前 smoke 配置：

```bash
./launch-firefox-gpu.sh \
'http://127.0.0.1:8080/?panoPreloadRequired=0&panoWidth=384&panoHeight=192&panoFace=96&panoVfov=180&panoJpeg=0.74&da360UploadScale=0.35&panoCaptureAnyway=0&panoFaceTileTimeoutMs=6000&panoFaceTileQuietMs=650'
```

在无人机静止、没有活动目标且已有完整 `PerceptionFrame` 时，可从 Firefox 控制台一次导出四个同 ID 文件。逐面等瓦片可能让一次 capture 超过 1 秒，因此显式给出只用于静态标定的 freshness 窗口：

```js
await window.__captureMetricCalibration('site-a', {
  captureId: 'site-a-01',
  maxFrameAgeMs: 60000
});
// site-a-01-rgb.jpg / site-a-01-anchors.json /
// site-a-01-manifest.json / site-a-01-raw.npz
```

`manifest.json` 记录 session/frame/capture/location ID、RGB/raw/anchors SHA、pose、capture transform、ERP/JPEG/上传尺寸配置和模型 metadata。默认会触发四次下载；传入 `{ download: false }` 可只返回内存中的 artifacts。同一帧和同一 capture ID 不能重复消费，fitter 还会拒绝重复 RGB、raw 和近重复姿态。

拟合示例（从每个地点的三个不同静止姿态各采一次，共 4 地点×3 capture；
下面命令会把 12 组四件套完整展开）：

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

只有 `fit_report.json` 同时通过留出误差、有效 anchor、至少 4 地点和至少 12 次采集门禁，才会生成可用于在线模式的 `depth_calibration.json`。启动 metric 模式示例：

```bash
DA360_DEPTH_MODE=da360-metric \
DA360_DEPTH_CALIB_PATH_HOST=/absolute/path/to/depth_calibration.json \
./start-all.sh
```

标定缺失、未通过或模型/模型输入尺寸/请求 RGB 尺寸/runtime projection/JPEG/resample/checkpoint 指纹不匹配时，metric 模式会在进入 GPU 前拒绝启动或请求，不会静默伪装成米制深度。Anchor 还要求六个 cubemap 面在各自像素被复制时均已报告 tiles ready；仅在导出时看到 tiles idle 不算合格。

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

2026-08-10 当前收口证据：15 个 JS 测试文件通过；默认 Python 为 50 passed、1 skipped、19 deselected；镜像内 backend contract 18/18；真实 CUDA 服务的 19 项 integration 全部通过。Firefox 已加载完整仿真 URL，但尚未人工完成出生点后的 depth canvas 与导航状态验收。

真实 flight log 和同场景质量数据生成后，使用已实现的门禁工具出报告：

```bash
python3 scripts/evaluate_closed_loop.py flight-log.json --output closed-loop-report.json
python3 scripts/evaluate_perception_quality.py \
  --baseline baseline.npz \
  --candidate 6x96-sse512=candidate-96.npz \
  --candidate 6x80-sse768=candidate-80.npz \
  --candidate 6x64-sse1024=candidate-64.npz \
  --output perception-quality-report.json
```

六面 capture 已改为每两面向浏览器调度器让出一次，并记录 render/project/scheduler/network/DA360/YOPO/capture-to-apply 等分段指标；这只证明调度机制存在，不证明吞吐已达标。2026-08-09 修复前日志只有约 3.1Hz depth、2.9Hz YOPO；当前实现尚未在真实 Firefox+GPU+城市 tiles 上证明 15Hz，也尚未完成 DA360 metric 或低空闭环门禁。详见 `docs/implementation-status-v2.md`。

## Git 与依赖

- `origin`：`cn-ryw/MindCloud_World_Fly`，只用于本项目分支
- `upstream`：`superboySB/MindCloud_World_Fly`，只拉取参考，不向其 push
- 当前实现分支：`feat/da360-metric-depth-v2`
- 原型归档：`archive/da360-prototype-c0b82d5-20260809` 和 tag `archive-c0b82d5-20260809`
- 依赖版本与模型哈希：`dependencies.lock.json`

启动前也可单独核对本地 bundle、配置和 checkpoint：

```bash
python3 scripts/verify_dependencies.py
```

不要提交 PID、checkpoint、raw NPZ/NPY 或实验数据大文件。
