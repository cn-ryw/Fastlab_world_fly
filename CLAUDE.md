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

- 到达球半径固定为 4.0m；不能用 9m `radio_range` 代替。终端候选必须沿原 Poly5 完整执行，随后还要满足无碰撞、三维速度不大于 0.75m/s并连续稳定0.4s，才可声明 arrived。
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
python3 scripts/verify_dependencies.py
docker build -f Dockerfile.da360-yopo -t mindcloud-da360-yopo:latest .
```

历史镜像曾使用内容哈希做启动绑定；该机制已撤除。现在以可读的模型文件名、YOPO 配置名、标定 ID 和进程级 service session ID 记录运行身份；checkpoint coverage 仍做模型结构完整性检查。

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
frameId, capturedAt, captureSimTimeS, capture transform, RGB Blob,
actual position/velocity,
reference position/velocity/acceleration,
yaw, captureProfile, projection config
```

全景 capture 开始时快照 planning state 和 transform；capture 完成后才形成该 RGB 的 frame context。`/yopo/plan_full` 必须从对应 `PerceptionFrame` 取 observation，禁止从持续变化的当前 `_yopoPose` 拼接旧 RGB。

该原语也用于原子标定导出。无人机静止、没有活动目标、完整帧就绪且 capture profile 为 `calibration` 时，`window.__captureMetricCalibration(locationId, options)` 会从同一个冻结 `PerceptionFrame` 生成同一 `captureId` 的 RGB、anchors、manifest 和 raw NPZ；禁止手工拼接来自不同帧的文件。manifest 必须保留 frame/location/capture/session ID、三份文件名与字节数、pose、capture transform、ERP 配置和模型 metadata。

## 服务端 API v2

standalone `da360_server.py` 和 combined `combined_server.py` 通过共享 route registration 提供一致的深度契约：

| 路径 | 契约 |
|---|---|
| `/health` | 模型、尺寸、resample、depth mode、标定、checkpoint 参数覆盖率 |
| `/depth` | JPEG 请求；返回 depth JPEG 与轻量 metadata |
| `/depth/raw` | NPZ：raw pred_disp、relative depth、valid mask、metadata |
| `/yopo/health` | YOPO 模型/config/checkpoint 信息 |
| `/yopo/plan_full` | JPEG+完整状态；总返回 identity，授权 metric 模式返回9元endstate及选中候选诊断；兼容调用仅在 `include_preview=1` 时返回深度预览，飞行链固定使用 `0` |
| `/yopo/preview` | 按 frame/goal/generation 从最近4帧有界缓存读取规划深度预览，不重复运行DA360 |
| `/yopo/plan` | 兼容调试路径；relative 模式 409，授权时缓存仍有最大年龄限制 |

所有 planning 数字、frame/goal/generation identity 和参考加速度都是必填；缺字段返回 400，模型异常返回无内部路径的 500。API CORS 只允许本机 Web origin，请求体有上限。

## 默认配置

以代码为准，每次修改默认值都同步更新本文和 README：

```text
panoProfile=flight
panoWidth=384 panoHeight=192 panoFace=96 panoVfov=180
panoMs=20 depthMs=20 yopoMaxFrameAgeMs=250 panoFrameDelayMs=0 panoFacesPerSlice=2
panoTopPoleGuard=10 panoBottomPoleGuard=2
panoPreloadTimeoutMs=60000
da360UploadScale=0.35

DA360_INPUT_SCALE=0.46
DA360_RESAMPLE=bicubic
DA360_CHANNELS_LAST=0
DA360_DEPTH_MODE=da360-metric

ARRIVAL_DISTANCE_M=4.0
ARRIVAL_SETTLE_SPEED_MPS=0.75 ARRIVAL_SETTLE_TIME_S=0.4
collisionRadius=0.6
droneScale=0.171
```

`DA360_CHANNELS_LAST=1` 曾在真实 Flask 进程中造成约 200 倍退化；没有同进程 benchmark 不得重新启用。

Capture profile 契约：

- `panoProfile=flight`（别名 `panoCaptureProfile`）禁用逐面 frame delay、tile timeout 和 tile quiet，并令 `captureAnyway=true`；它是默认值，设置 YOPO 目标也会强制切回该 profile。
- `panoProfile=calibration` 使用 URL 配置的逐面 tile wait/quiet；`window.__captureMetricCalibration()` 在其他 profile 下必须拒绝。
- 活动导航期间切换到 `calibration` 必须拒绝；换目标会结束旧日志/请求 generation 并清除旧轨迹，但保持实际速度连续。
- 旧 `panoCaptureAnyway=0` 只作为 calibration 的兼容映射；显式 `panoProfile` 优先。若当前 URL 仍有 legacy 参数，`location.reload()` 会保留它并再次进入 calibration。控制台接口为 `window.__getPanoramaCaptureProfile()` 与 `window.__setPanoramaCaptureProfile(profile)`；切换会 abort 在途 capture，但只改变当前页面运行态，不会清理 URL。
- `yopoMaxFrameAgeMs=250` 是冻结 observation 的硬安全龄限；超过即丢弃轨迹。它不替代正式门禁的 capture-to-apply p95 ≤150ms。
- Poly5 安装还要求年龄不超过 `min(0.25s, 0.25T)`，且当前实际状态与 fast-forward suffix 参考的位置/速度误差分别不超过 1.2m/3m/s；不允许通过平移障碍相关终点来修补陈旧轨迹。
- 异步 freshness 分三层：请求源在发送前必须仍是最新完整 RGB；轨迹响应只要求同 goal/generation/mode 且不超过 250ms；深度 canvas 按递增 request ID 提交。因此流水化时 depth 可以是“最新可用帧”并比 RGB 落后一帧，但旧目标、旧 generation 或已被更新 depth 超越的响应永不绘制。
- 每个 planning 请求只记录一个同步 control outcome（applied/blocked/rejected）；JPEG canvas 的 commit/error/stale 另记为 `mode=depth-preview` diagnostic，不能计入规划频率。
- 650ms quiet×6 面本身就是 3.9s，叠加渲染/复制/投影后约 4.2s。旧 aggregate `render≈4.1s` 混入 tile wait，不是 GPU scene render 证据。

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
- metric calibration 启动时一次加载并绑定可读模型名、输入尺寸和 resample
- metric loader 必须校验标定的 projection/JPEG contract；每次请求通过 `X-Projection-Config` 携带冻结帧的完整投影配置，并在 GPU 推理前同时匹配 RGB request size、ERP/FOV、pole guard、JPEG quality 和 upload scale
- calibration anchor 只有在六个 cubemap 面各自在复制 RGB 像素时都已报告 tiles ready 才可导出；不能用采集结束后的单次 `tilesLoaded` 状态代替逐面 provenance
- `DA360_DEPTH_MODE=da360-metric` fail closed；校准结构、输入/投影契约或接受来源无效时拒绝启动
- `window.__captureMetricCalibration()` 原子导出同 captureId 的 RGB、anchors、manifest 与 raw NPZ

2026-08-10 已完成 4 地点×3 captures 的真实原子数据集和 LOLO 报告：`../experiment_data/metric_fit-lolo-20260810-12capture/fit_report.json`。1536 anchors 中 1140 有效（74.22%），833 个落在 0.5–20m；四件套 identity、输入/投影契约和地点覆盖完整。**自动 calibration 精度门禁失败**：

| 留出地点 | median AbsRel | p90 AbsRel | 10m 内 p90 绝对误差 |
|---|---:|---:|---:|
| site-a | 0.399 | 0.544 | 3.326m |
| site-b | 0.366 | 2.125 | 5.959m |
| site-c | 0.383 | 0.443 | 3.545m |
| site-d | 0.313 | 0.982 | 4.051m |

报告 `success=true` 仅表示 fitter 完成；自动 `acceptance.passed=false`。全量 scale-only 结果 `a=0.0011892812185910185,b=0` 的 median/p90 AbsRel 为 0.376/1.507，10m 内 p90 误差 4.054m。项目负责人于 2026-08-11 检查实时深度效果后，明确人工接受该固定尺度作为后续 sim-to-sim 基线；正式 `../experiment_data/depth_calibration.json` 保留自动失败事实，并记录 `manual_acceptance`。`start-all.sh` 默认使用该正式 metric 标定；health/规划为 `validated-da360-metric`，同时暴露 `manual-user`、`automatic_gate_passed=false` 与 `sim-to-sim` 范围。未完成项仍是 Cesium truth parity 与真实低空 metric 闭环。

验收门槛：0.5–20m 留出数据 median AbsRel ≤15%、p90 AbsRel ≤30%、10m 内 p90 绝对误差 ≤1m、有效 anchor ≥70%，至少 4 地点、12 captures，并使用 leave-one-location-out。

人工接受范围仅为 sim-to-sim；不得把它写成自动 LOLO 精度门禁通过或真实传感器绝对精度验收。显式 `da360-relative` 仍只能 preview/采集，不能产生可应用轨迹。`cesium-truth` fallback 尚未实现，不能写成现有能力。

输入分辨率 pilot 的结论也已固定：同一解码后 134×67 JPEG 输入，476×238 推理为 25.6ms，1036×518 为 151.9ms（5.94×）；site-a median/p90 AbsRel 从 0.276/0.544 恶化到 0.569/2.468，10m 内 p90 误差从 2.666m 恶化到 19.945m。不得把 live DA360 输入改到 1036×518；上游 RGB 没有新增细节，该候选既更慢也未改善跨地点精度。

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

真实验收仍需 Firefox+GPU+城市 tiles。闭环 evaluator 固定为 schema v2，并且必须：

- 丢弃前 30 个合法唯一 planning frames；其后至少计量 60s 和 900 frames
- 日志提供有限 `monotonicStartMs` 和正数 `duration_s`；全部合法 apply timestamp 位于 session 起点至终点内，终点只允许 5.1ms 取整余量；第一次/最后一次合法 apply 距相应 session 边界均不得超过 `yopoMaxFrameAgeMs+5.1ms`，防止首段或末段停摆被中段高频数据掩盖
- `resolvedUrl` 只接受本机 `127.0.0.1|localhost` 的 `/|/index.html` 入口，并保留正向白名单内的全景/性能参数（包括 5 个 `flightPreload*` 预加载参数）；删除 userinfo、fragment、端点 URL、未知参数与 token/secret/password/API-key/auth/credential 查询项。evaluator 会再次独立校验该 URL，缺失、解析失败、非本机/非法路径或重复参数都 fail closed
- 每个计量帧有浏览器实际安装轨迹的 `trajectoryApplied=true` 回执，而不只是服务返回 200；`outcome=applied`、`planningAuthorized=true`、`trajectoryApplied=true` 必须一致，blocked/rejected/unauthorized 不得声称已安装轨迹
- 日志声明单一 `navigationSession(goalId,generation)`，generation 必须是非负 JavaScript-safe integer；每个 planning identity 与其一致，frame 唯一，apply 与 physics timestamp 严格递增
- physics timestamp 覆盖至少 95% 的计量窗口；最少帧数为 `ceil(measurementDurationMs×0.95/33.3)+1`，60s 窗口至少 1713 帧
- depth mode 只能是 `da360-metric` 或 `cesium-truth`
- calibration ID 和 service session ID 非空、全段稳定；service session ID 只标识当前 combined-server 进程，模型与配置用 health 中的可读名称记录
- 设置目标后的下一张完整帧进入 planning
- 到达/取消后回 preview，深度仍更新
- 高速穿越4m球不提前结束；合格终端轨迹沿原 Poly5 到达后锁定实际位姿，并在无碰撞、三维速度≤0.75m/s连续0.4s才到达
- 平均有效规划率 ≥15Hz、p95 规划间隔 ≤100ms
- p95 capture-to-apply ≤150ms、physics update 间隔 ≤33.3ms、服务 p95 ≤50ms
- 低空街谷无碰撞、NaN、旧轨迹复活

截至 2026-08-10，上述真实门禁尚未完成。修复后一次 relative `/yopo/plan_full` 诊断为 DA360 44.6ms、端到端 63.3ms，但它未授权 YOPO，既不是统计样本也不能证明 15Hz。修复前基线约为 depth 3.1Hz、YOPO 2.9Hz；不要把 API/单元测试通过写成 15Hz 已达标。

六面 capture 当前仍执行六次 Cesium scene render，默认每两面 `requestAnimationFrame` yield，并支持 AbortSignal；1/3/6 面仅保留为显式 A/B 参数。projector texture 在尺寸相同时使用 `texSubImage2D` 复用。飞行期 planning 固定 `include_preview=0`；操作员深度预览约每2s从按身份索引的4帧LRU异步读取，latest-only且不占控制request gate。timing 必须分别保留 `scene_render/tile_wait/wait_rerender/face_upload/project/scheduler`、fetch/header/json、response bytes、DA360、YOPO、trajectory apply、preview fetch/decode/draw 和 gate hold，不能再用旧 aggregate `render` 归因。FlightLogger schema v2 记录真实轨迹安装回执、唯一 planning frame、选中候选、Poly5峰值、控制饱和、drop reason、单调时间和 p95。真实 Firefox preview A/B 已测得新默认 capture p95=53/45ms、RAF/physics-frame p95=17.1/17.1ms；这只证明 preview/主线程预算改善，仍不能写成带航点的 15Hz 闭环已通过。

门禁实现位于 `config/perception-sweep.json`、`scripts/evaluate_closed_loop.py` 和 `scripts/evaluate_perception_quality.py`。真实数据必须按该配置的候选顺序跑完并保存报告，不能凭 URL 参数或平均延迟口头选择配置。

2026-08-11 最小 live smoke URL：

```bash
./launch-firefox-gpu.sh \
'http://127.0.0.1:8080/?panoProfile=flight&panoPreloadRequired=0&panoWidth=384&panoHeight=192&panoFace=96&panoFacesPerSlice=2&panoVfov=180&panoJpeg=0.74&da360UploadScale=0.35&panoMs=20&depthMs=20&panoramaTileSse=512&panoramaFarMeters=1200&panoramaLeanStreaming=1'
```

先用 `window.__getPanoramaCaptureProfile()` 核对 `flight`，保存原始 timing/log；evaluator 命令必须显式或默认使用 `--warmup-frames 30 --min-duration-s 60 --min-planning-frames 900`。relative preview 可以用于性能归因，但绝不能通过正式 evaluator。

旧标定页若仍含 `panoCaptureAnyway=0`，不可只做 reload。直接打开上面的显式 flight URL，或在控制台执行以下跳转；`window.__setPanoramaCaptureProfile('flight')` 只改变当前运行态，未删除 legacy URL 时下次 reload 仍会回 calibration：

```js
(() => {
  const url = new URL(location.href);
  url.searchParams.delete('panoCaptureAnyway');
  url.searchParams.delete('panoCaptureProfile');
  url.searchParams.set('panoProfile', 'flight');
  location.assign(url.href);
})();
```

Dense Cesium truth 仅允许按阶段推进，目前全是设计、没有代码或 live 证据：T0 为 opt-in PostProcessStage 读取 Cesium 1.117 `depthTexture`，重建 eye-space position 后取 ray range（不是 camera-z），RGB24 打包并单次 `readPixels`，先对已有 anchor ray 做 parity；T1 才扩到六面并复用现有 ERP projector；T2 只有在 parity 和 readback 延迟通过后才增加 uint16-mm/invalid=65535 的二进制 `/yopo/plan_depth` 路由。任何阶段都不得先宣称 `cesium-truth` 或 15Hz 已实现。

## 安全和文件边界

`scripts/serve.py` 不是项目根目录文件服务器，只允许：

- `/`、`/index.html`
- `/src/`
- `/asset/`
- `/ThirdParty/Cesium/`
- 同源 `/api/path/<safe-name>.json`

禁止重新暴露 `.git`、Markdown、scripts、tests、checkpoint、目录列表或任意 traversal。路径 API 只允许同源且原子写入。

模型、PID、raw NPZ/NPY 和实验大文件不得进入 Git。可读的依赖版本、来源与本地路径记录在 `dependencies.versions.json`。

依赖核验命令为 `python3 scripts/verify_dependencies.py`；它核对可读版本标记、必需路径与镜像 tag，不计算内容哈希。

## Git

- `origin=https://github.com/cn-ryw/MindCloud_World_Fly.git`
- `upstream=https://github.com/superboySB/MindCloud_World_Fly.git`
- 工作分支：`feat/da360-metric-depth-v2`
- 归档分支：`archive/da360-prototype-c0b82d5-20260809`
- 归档 tag：`archive-c0b82d5-20260809`
- 仓外 bundle：`../backups/MindCloud_World_Fly-c0b82d5-20260809.bundle`

不向 upstream push，不 force-push。`origin` 的 v2 分支未确认推送前，不得声称本轮工作已有远端备份。

依赖锁定：Cesium 1.117、PlayCanvas 2.17.2（revision `2892d5e`）、DA360 commit `93dd3fc32e8e8751ac1e4b26ff1a575adfc55661`。本地 Cesium 是 1.117；`index.html` 的 CDN fallback 必须同步保持 1.117，不能混用 1.121。
