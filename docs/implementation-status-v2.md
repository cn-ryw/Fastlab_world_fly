# DA360/YOPO v2 Implementation Status

更新日期：2026-08-11

分支：`feat/da360-metric-depth-v2`

已验证实现基线：`63fd9f1`（严格 bundle、授权门禁、运行时尺寸契约和运维指纹）。当前工作树另有 capture profile、timing/evaluator v2 等改动；自动测试已复验通过（精确计数以最终 sweep 原始输出为准），但 live Firefox/GPU 与可信 15Hz 尚未验收。

本文件只报告证据，不把“代码已写”视为“真实系统已验收”。

仓库提供无浏览器 JS、metric/security/evaluation Python 单元测试，以及可在 combined 镜像内运行的 backend contract。测试文件会随实现变化，因此本文不固化数量；每次验收必须运行下方命令并保存该次原始输出。旧 live-service tests 和真实浏览器飞行必须单独报告。

## 状态定义

- **Implemented + unit tested**：代码存在，并有不依赖真实 GPU/浏览器的自动测试。
- **Code present; automated tests rerun; live validation pending**：当前工作树代码已通过自动复验，但尚无当前实现的 Firefox/GPU、真实飞行或 15Hz live 证据。
- **Live API verified; interactive flight pending**：真实 GPU/API 已验证，但 Firefox 交互、tiles、飞行或数据门禁仍未完成。
- **Not implemented / not accepted**：仍缺代码、数据或门禁结果。

## Implemented + unit tested

### Git 与运行安全

- 原型 `c0b82d5` 已生成仓外 bundle、SHA256、archive branch 和 annotated tag。
- `.dev-web.pid`、checkpoint、raw NPZ/NPY 与实验大文件已排除出 Git。
- Web 静态服务采用路径白名单，拒绝 `.git`、checkpoint、隐藏文件、目录浏览和 traversal。
- Web/API 默认绑定 `127.0.0.1`，API CORS 仅允许本机 Web origin并限制请求体。
- start/stop 通过 PID cmdline+cwd 验证进程归属；health 显式绕过代理并 fail closed。

### DA360 显示与导航状态

- 只有可见 depth canvas；无隐藏 depth image 双面状态。
- JPEG 解码和 canvas commit 完成后才设置 `hasDepth=true`。
- UI 区分 offline、preview、planning、error 和 stale outcome。
- relative `plan_full` 只更新 depth canvas，并以 blocked outcome 排除出 planning/15Hz 统计。
- `goalId/generation/frameId/requestId` 防止旧响应画面或轨迹覆盖。
- 新目标等待下一完整 RGB；取消/到达返回 preview 并保留最后深度图。
- 到达阈值为 4.0m；碰撞半径为 0.6m并带旧配置迁移。
- swept collision 每物理步执行，不受 no-hit cache 跳过。
- 六面 capture 默认每两面向调度器 yield，支持 AbortSignal，并输出 render/project/scheduler timings。
- FlightLogger 记录唯一 planning frame、stale/drop reason、各阶段延迟、capture-to-apply 和 p95。

### 原子感知和 YOPO 契约

- `PerceptionFrame` 原子绑定 RGB、capture transform、实际/参考状态、yaw、capture profile 和投影配置。
- capture 起点 planning snapshot 与完成后的 RGB frame context 绑定。
- planning wire 分别传 actual `px/py/pz` 与 reference `rpx/rpy/rpz`；YOPO goal observation 使用 reference position/acceleration，世界 endpoint 使用 actual position。
- 9 元 axis-major endstate、有限数值、traj time、实际/参考状态、加速度和响应 identity 有校验。
- 换目标先清理上一 generation 的 Poly5/decay；退出 flight 统一取消请求、轨迹、marker 和高度覆盖。

### 深度 API 与标定数学

- standalone 与 combined 共享 API v2 `/health`、`/depth`、`/depth/raw`。
- `/depth` 保持小 JPEG并恢复轻量 metadata；`/depth/raw` 输出 NPZ 和完整指纹。
- `/yopo/plan_full` 回显 frame/goal/generation、depth mode、calibration ID 和 timings。
- Anchor canonical 方向、姿态旋转和 sensor/component/world 映射已统一。
- Fitter 支持分辨率像素中心映射、seam wrap 双线性、多 captures、一次 Huber 和留出验证。
- metric loader 绑定模型、模型/请求尺寸、projection/JPEG、resample、checkpoint SHA 与 accepted 标志；每请求携带 `X-Projection-Config`，并在 GPU 前核对 RGB 尺寸和完整运行时投影指纹。
- `window.__captureMetricCalibration()` 从一个冻结 `PerceptionFrame` 原子导出同 captureId 的 RGB、anchors、manifest 与 raw NPZ，并记录三份 artifact SHA、session/pose/transform、ERP/JPEG/上传尺寸和模型 metadata。
- fitter 强制四件套配对、完整指纹、每地点 3 个 distinct poses、每地点 LOLO 过门及 0.5–20m 范围指标。
- Anchor ray 强制绑定生成 RGB 的同一 panorama viewer/tileset，并核验 capture revision、transform、FOV 和按像素取整容差计算的纵横比；六个 cubemap 面分别记录像素复制瞬间的 tile readiness，任一面未 ready 就 fail closed。导出会等待在途 capture 完成，并阻止下一轮 capture 破坏真值源。

覆盖这些行为的测试包括 JS navigation/perception/depth/collision/geometry tests，以及 Python backend contract、serve security 和 metric fitter tests。

## Code present; automated tests rerun; live validation pending

- `panoProfile=flight|calibration`（别名 `panoCaptureProfile`）已接入；默认/设置目标为 `flight`，标定导出只接受 `calibration`。显式 profile 优先于 legacy `panoCaptureAnyway=0`；控制台接口为 `window.__getPanoramaCaptureProfile()`、`window.__setPanoramaCaptureProfile(profile)`。
- `flight` 令逐面 delay/timeout/quiet 为 0；`calibration` 使用 URL 的 tile wait。标定约 4.2s 主要来自 6×650ms quiet（3.9s），旧 `render≈4.1s` 混入等待，不能归因给 GPU scene render。
- 冻结 planning observation 超过 `yopoMaxFrameAgeMs=250` 会在轨迹安装前 fail closed；正式 capture-to-apply p95 门槛仍是 150ms。
- RGB/推理流水线使用分层 freshness：同会话且 ≤250ms 的冻结帧轨迹可以在下一 RGB 已完成时应用；depth canvas 按 request ID 显示最新可用帧，并记录 `depthPreviewLagFrames/depthPreviewAgeMs`。旧 goal/generation 或被更新 depth 超越的响应不绘制。
- planning 的 applied/blocked/rejected control outcome 在可选 JPEG 解码前同步且唯一记录；canvas commit/error/stale 另记为 `mode=depth-preview`，不进入规划频率门禁。
- projector texture 同尺寸时复用并走 `texSubImage2D`；capture timing 拆为 `scene_render/tile_wait/wait_rerender/face_upload/project/scheduler`，HTTP header/body、DA360、YOPO 与 apply 另记。
- FlightLogger/evaluator schema v2 只计 `trajectoryApplied=true` 的实际轨迹安装；combined response 增加由 API version、两模型 checkpoint 和 YOPO effective config 组成的 service fingerprint。
- evaluator v2 丢弃 30 个合法唯一 warmup frames，再要求 ≥60s、≥900 frames、稳定 calibration/service fingerprint、允许的 `da360-metric|cesium-truth` 和严格递增的 identity/timestamp；自动测试已复验，但尚未在 Firefox+GPU 上证明 15Hz。

## Live API verified; interactive flight pending

- 已用镜像 `sha256:72b068847102…` 完成 local-only `start-all.sh` 成功路径；base、dependency lock `34fdb7e837c6…`、Dockerfile recipe `b0d0fd39d57d…`、本地 base image ID 与 RootFS 前缀均已核验。
- live `/health` 与 `/yopo/health` 为 API v2/CUDA，DA360 bicubic、0.459、checkpoint coverage 100%，YOPO base/overlay/effective hash 与锁一致。
- live integration 19/19 通过；`/depth`、`/depth/raw` 及 relative `/yopo/plan_full` 均 200，后者返回 depth 但明确 `planning_authorized=false`、无 endstate、未运行 YOPO。legacy relative `/yopo/plan` 为 409。
- 静态服务 live 拒绝 `.git`、Markdown、scripts/tests、checkpoint、明文和编码 traversal；恶意 Host/Origin 为 403，合法 Web/Cesium 资源为 200。
- 一次非统计用途的 relative `plan_full` 诊断为 DA360 44.6ms、端到端 63.3ms；它不能计入有效 planning 或 15Hz 门禁。
- preview→planning→preview 状态在假 DOM/网络测试通过，真实 Firefox canvas、AbortController 和目标交互仍需确认。
- 4m 到达代码和单元边界已修复，真实飞行尚未重新验证 3.9/4.1/8.9m。
- 原子 `PerceptionFrame` 已接入在线请求，但尚未对高速运动场景测量 frame age 和 pose/RGB 一致性。
- Firefox GPU 进程已加载完整 URL、Cesium 与所有前端模块；尚未人工选择出生点并进入飞行，因此不能算真实 depth canvas 或 Google tiles 验收。
- RGB/anchor 同 viewer/tileset provenance 已有单元覆盖；仍需在真实 tiles-loaded 场景确认 `tileState=ready` 和 ray-hit parity。

## Not implemented / not accepted

### DA360 metric

- 4 地点×3 captures、共 12 组四件套和 LOLO 报告已完成：`../experiment_data/metric_fit-lolo-20260810-12capture/fit_report.json`。1536 anchors 中 1140 有效（74.22%），833 个在 0.5–20m。
- 数据完整性通过，但四个留出地点的 median/p90 AbsRel、10m 内 p90 误差分别为：site-a 0.399/0.544/3.326m，site-b 0.366/2.125/5.959m，site-c 0.383/0.443/3.545m，site-d 0.313/0.982/4.051m，均未过精度门禁。
- `success=true` 只代表 fitter 完成；acceptance=false。全量 scale-only 候选 `a=0.0011892812185910185,b=0` 的 median/p90 AbsRel=0.376/1.507、近距 p90=4.054m，不能安装；`depth_calibration.json` 仍为 `a/b=null`。
- 476×238→1036×518 pilot 在相同 134×67 source JPEG 下从 25.6ms 增至 151.9ms（5.94×），且 site-a 显著变差；不得提升 live DA360 输入到 1036×518。
- 没有 accepted metric、metric 驱动的低空闭环或 Cesium truth parity；继续 `da360-relative` fail closed。

上线门槛固定为：median AbsRel ≤15%、p90 AbsRel ≤30%、10m 内 p90 绝对误差 ≤1m、有效 anchor ≥70%、至少 4 地点与 12 captures。

### Dense Cesium truth（仅设计）

- T0 最小 smoke：opt-in PostProcessStage 读取 Cesium 1.117 `depthTexture`，重建 eye-space position 并取 ray range，RGB24 打包、一次 `readPixels`，只与现有 anchors 做 parity，不接 YOPO。
- T1 才扩为六面并复用现有 ERP projector；T2 只有 parity/readback 延迟合格后才新增 uint16-mm（invalid=65535）的二进制 `/yopo/plan_depth`。
- 当前没有上述实现、GPU depth readback 数据或路由，不能称为 `cesium-truth`，更不能据设计宣称 15Hz。

### 15Hz 与飞行验收

- 六次 Cesium scene render、跨 context face upload 和每两面一次 rAF yield 仍需 live 分段数据；目前没有真实吞吐、主线程长任务或物理更新收益数据。
- `config/perception-sweep.json` 及闭环/质量评估脚本已实现；它们只负责判定已有数据，不代表候选已经跑过。
- 6×96、6×80、6×64 的质量/性能矩阵尚未运行。
- 修复前实测仅 depth 3.1Hz、YOPO 2.9Hz；这不是当前代码的验收结果，也不能作为已达标证据。
- 50 次旧的未授权 relative 全链诊断显示服务 p95 40.7ms、HTTP p95 76.1ms，但该路径现已 fail closed，不能计作有效 planning；仍无真实 Firefox 的平均 planning ≥15Hz、p95 planning 间隔与 capture-to-apply 数据。
- 尚未完成直线街谷、转角避障、取消/换目标的 5 分钟真实飞行。

正式 evaluator v2 还要求 schema v2、30 warmup、60s/900 计量 frames、实际轨迹回执、唯一 identity、稳定 calibration ID/service fingerprint。每份日志声明单一 `navigationSession(goalId,generation)`，generation 必须是非负 JavaScript-safe integer，所有 planning frame 必须匹配；日志还需有有限 `monotonicStartMs`、正数 `duration_s`，全部合法 apply 时间位于 session 起止边界内（终点只允许 5.1ms 取整余量），且第一次/最后一次合法 apply 距相应 session 边界均不超过 `250+5.1ms`。任何 applied/`trajectoryApplied=true` 声称都必须与 `planningAuthorized=true` 一致，blocked/rejected/unauthorized 不得声称已安装轨迹。physics 时间戳须覆盖至少 95% 计量窗口，最少帧数为 `ceil(measurementDurationMs×0.95/33.3)+1`，所以 60s 窗口至少 1713 帧。relative preview 即使频率高也必须失败。飞行日志的 `resolvedUrl` 仅接受本机 `127.0.0.1|localhost` 的 `/|/index.html` 入口，保留已审查的全景/性能参数白名单（包含 5 个实际生效的 `flightPreload*` 参数），端点 URL、未知参数与凭据均不记录；evaluator 对该字段再做独立、失败关闭的 URL/schema 校验。

### 依赖与发布收尾

- `dependencies.lock.json` 已记录本地固定依赖和 checkpoint SHA。
- 本地 Cesium 与 `index.html` CDN fallback 均已固定为 1.117。
- v2 工作分支已推送到 `origin/feat/da360-metric-depth-v2`，首次实现提交为 `63fd9f1`；未向 upstream 推送。
- 文档外的 `../reference_notes` 和 `../experiment_data` 不受主仓库 Git 保护，需要单独归档。

## 下一次验收命令

```bash
# 1. 无 GPU 测试
for test_file in tests/*.js; do node "$test_file" || break; done
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_fit_da360_metric.py \
  tests/test_serve_security.py \
  tests/test_evaluation_gates.py
docker run --rm --entrypoint /bin/bash -v "$PWD:/workspace:ro" \
  mindcloud-da360-yopo:latest \
  -lc 'cd /workspace && python3 tests/test_backend_contract.py'
python3 scripts/verify_dependencies.py

# 2. 重建并启动真实服务
lock_sha="$(sha256sum dependencies.lock.json | awk '{print $1}')"
recipe_sha="$(sha256sum Dockerfile.da360-yopo | awk '{print $1}')"
docker build \
  --build-arg "MINDCLOUD_DEPENDENCY_LOCK_SHA256=$lock_sha" \
  --build-arg "MINDCLOUD_IMAGE_RECIPE_SHA256=$recipe_sha" \
  -f Dockerfile.da360-yopo -t mindcloud-da360-yopo:latest .
./start-all.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -o addopts='' -m integration tests/ -v

# 3. 核对 runtime identity
curl --noproxy '*' -s http://127.0.0.1:5688/health | python3 -m json.tool
curl --noproxy '*' -s http://127.0.0.1:5688/yopo/health | python3 -m json.tool

# 4. Firefox flight-profile 手工/日志 smoke
./launch-firefox-gpu.sh \
'http://127.0.0.1:8080/?panoProfile=flight&panoPreloadRequired=0&panoWidth=384&panoHeight=192&panoFace=96&panoFacesPerSlice=2&panoVfov=180&panoJpeg=0.74&da360UploadScale=0.35&panoMs=20&depthMs=20&panoramaTileSse=512'

# 5. 有授权 metric/truth flight log 后执行正式门禁
python3 scripts/evaluate_closed_loop.py flight-log.json \
  --warmup-frames 30 --min-duration-s 60 --min-planning-frames 900 \
  --min-physics-coverage 0.95 \
  --output closed-loop-report.json
```

若 Firefox 当前地址仍含 legacy `panoCaptureAnyway=0`，单独执行 `location.reload()` 会保留该参数并再次进入 `calibration`。应直接打开步骤 4 的显式 `panoProfile=flight` URL，或在控制台执行：

```js
(() => {
  const url = new URL(location.href);
  url.searchParams.delete('panoCaptureAnyway');
  url.searchParams.delete('panoCaptureProfile');
  url.searchParams.set('panoProfile', 'flight');
  location.assign(url.href);
})();
```

页面重新就绪后执行 `window.__getPanoramaCaptureProfile()`，预期为 `flight`。`window.__setPanoramaCaptureProfile('flight')` 会取消在途 capture，但只改变当前运行态；未清理 legacy URL 时，下次 reload 仍会回到 calibration。backend service-fingerprint 改动仍需按上面的镜像重建/identity 核对，不能只 reload 页面就视为生效。

验收报告必须分别列出：commit、工作树状态、镜像 ID、模型 SHA、Cesium/PlayCanvas 版本、场景、预热方式、原始 flight log 和 p50/p95 指标。
