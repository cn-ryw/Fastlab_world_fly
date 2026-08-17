# MindCloud World Fly 中文使用指南

本指南面向第一次运行项目、使用 RadioMaster T8L 或测试 YOPO/SO3 闭环的用户。项目仅用于仿真研究与演示。

## 1. 运行前准备

### 硬件与软件

- NVIDIA GPU、可用驱动和 NVIDIA Container Toolkit。
- Docker Engine、Python 3、Node.js、curl。
- Firefox；需要 Web Serial 遥控器时使用 Google Chrome/Chromium。
- DA360 large checkpoint 和 YOPO checkpoint。
- Cesium Ion 访问凭据，以及可访问 Google Photorealistic 3D Tiles 的网络。

### 中国网络环境与代理

Google 3D Tiles 在部分网络环境下可能连接缓慢或失败。若必须使用 Clash，优先选择延迟低、丢包少、可稳定访问 Google Maps Platform 的节点，并保持以下原则：

- 浏览器访问 Google/Cesium 的流量可以走代理。
- <code>127.0.0.1</code> 与 <code>localhost</code> 必须绕过代理。
- 不要把代理凭据、Cesium token 或节点信息提交到 Git。
- 节点切换后重新进入场景，让主视图和隐藏全景视图重新建立缓存。

启动脚本默认不会修改 Clash 配置。只有明确设置 <code>MINDCLOUD_FIX_CLASH=1</code> 时才启用项目中的辅助修复逻辑。

### Cesium Ion token

仓库不包含 Cesium Ion token。首次开始飞行时，页面会显示密码样式的 token 配置框；输入一个限制到本机来源的有效 token 后，页面会将它保存在当前浏览器配置中并直接继续。项目 Firefox 启动器使用独立、持久、非隐私的配置目录，正常关闭和再次启动后无需重复设置。

Firefox 和 Chrome 的 <code>localStorage</code> 相互独立，因此需要分别设置。该配置流程不会把 token 写入 MindCloud 页面 URL、浏览器启动命令或应用主动日志。不要把真实 token 写进 README、启动脚本、URL 截图或飞行日志；旧版本中曾经公开的 token 必须在 Cesium Ion 后台撤销并重新创建受限 token。

## 2. 模型与策略

模型不随仓库分发。推荐显式设置宿主机路径：

~~~bash
DA360_MODEL_PATH_HOST=/path/to/DA360_large.pth \
YOPO_MODEL_PATH_HOST=/path/to/epoch30.pth \
./start-all.sh
~~~

默认 YOPO 策略是 <code>d70_h30_epoch30</code>，使用 epoch30、70 m 感知和 30 m 最大轨迹距离。切换到原有 epoch10 基线策略：

~~~bash
MINDCLOUD_YOPO_STRATEGY=baseline ./start-all.sh
~~~

切换策略后应查看服务启动输出与 <code>/yopo/health</code>，确认实际 checkpoint 和策略身份。

## 3. 启动与连通性

### 启动后端和本地 Web

~~~bash
./start-all.sh
~~~

正常拓扑：

~~~text
浏览器 -> http://127.0.0.1:8080 -> 本地静态服务
浏览器 -> http://127.0.0.1:5688 -> DA360 + YOPO combined API
~~~

服务就绪状态与运行配置检查（不是账号认证或设备身份检查）：

~~~bash
curl --noproxy '*' http://127.0.0.1:5688/health
curl --noproxy '*' http://127.0.0.1:5688/yopo/health
~~~

### 启动 Firefox

~~~bash
./launch-firefox-gpu.sh
~~~

Firefox 适合主画面、G 航点和完整 Demo 测试。启动器固定使用独立、持久、非隐私的 MindCloud profile；可用 <code>FIREFOX_PROFILE_DIR</code> 覆盖默认目录。

### 启动 Chrome

~~~bash
./launch-chrome-gpu.sh
~~~

Chrome 使用持久的普通配置，不开启隐私模式。连接 RadioMaster T8L 时必须使用支持 Web Serial 的 Chrome/Chromium。

Chrome 默认以 <code>CHROME_GPU_MODE=auto</code> 启动：优先尝试 NVIDIA PRIME，若 GPU/WebGL 初始化失败则明确告警并自动回退到桌面 GPU。也可显式选择：

~~~bash
CHROME_GPU_MODE=nvidia ./launch-chrome-gpu.sh  # 强制 NVIDIA，失败即停止
CHROME_GPU_MODE=desktop ./launch-chrome-gpu.sh # 直接使用桌面 GPU
~~~

### 停止服务

~~~bash
./stop-all.sh
~~~

## 4. 出生点与主界面

1. 进入 Google 3D Tiles 模式。
2. 在 placement 阶段按住 <code>I</code> 点击地面或已解析建筑表面。
3. 使用 <code>W/A/S/D</code> 微调位置和高度。
4. 按 <code>O</code> 确认出生点并进入飞行。
5. 若预加载提示未就绪，可以继续等待瓦片加载；不要在建筑仍明显缺块时开始高速避障演示。

主视图与隐藏六面全景视图各自维护 Cesium 瓦片状态。<code>rgbCaptured=6/6</code> 表示六个方向已完成像素采集，不等价于所有建筑瓦片都已加载完整。

## 5. 三种飞行模式

### FPV

FPV 是直接手动飞行模式，适合观察场景和确认控制方向。Mode 三挡低挡进入 FPV。

### Drone (Easy)

Easy 模式把输入转换为更易控制的无人机运动。Mode 三挡中挡进入 Drone (Easy)。

### SO3

SO3 模式用于 YOPO 轨迹跟踪、G 固定航点和 T8L 滚动航点。Mode 三挡高挡进入 SO3。

SO3 进入时固定第一帧世界 yaw 参考。Roll/Pitch 按该固定世界航向解释，不随机体倾斜造成的视觉航向耦合旋转。当前油门通道不接入 YOPO 高度输入，航点保持设置时的高度。

## 6. RadioMaster T8L

### 系统连接检查

~~~bash
./launch.sh --input-status
~~~

若提示权限不足：

~~~bash
./launch.sh --setup-input
~~~

完成后重新插拔设备；用户组变化可能需要重新登录。

### 浏览器连接

1. 使用 <code>./launch-chrome-gpu.sh</code> 打开页面。
2. 打开 Tab/Settings。
3. 点击 <b>Connect T8L</b>。
4. 在浏览器设备选择器中选择 RadioMaster。
5. 控制台出现 <code>Gamepad connected</code> 或串口连接状态后再进行 Arm。

### 默认通道与三挡开关

| 功能 | 默认逻辑 |
|---|---|
| CH1 | Roll |
| CH2 | Pitch |
| CH3 | Throttle |
| CH4 | Yaw |
| Arm 低挡 | 锁定 |
| Arm 中挡 | 解锁 |
| Arm 高挡 | 解锁 |
| Mode 低挡 | FPV |
| Mode 中挡 | Drone (Easy) |
| Mode 高挡 | SO3 |

仓库中的共享默认配置会被 Firefox 和 Chrome 共同读取，但用户在浏览器中临时修改的 LocalStorage 配置彼此独立。需要完全同步自定义映射时，应在一个浏览器导出配置，再在另一个浏览器导入。

### SO3 滚动航点

在 Arm 且 Mode 为 SO3 时，Roll/Pitch 首次越过接管死区后启用滚动航点。水平目标距离为 50 m，摇杆方向按固定世界 yaw 解释；摇杆回中后保持最近一次目标。

设备断开、串口超过约 250 ms 没有合法帧、退出 SO3 或 Arm 失效时，系统会取消活动航点和旧轨迹并进入悬停保护。

## 7. G 固定航点

1. 切换到 SO3 并完成 Arm。
2. 按住 <code>G</code>。
3. 点击地面、道路或其他可通行且已解析的表面。
4. 松开 <code>G</code> 后观察目标标记和 planning 状态。
5. 按 <code>C</code> 可取消当前导航。

目标默认沿用无人机当前高度。建筑物上、建筑内部、没有可靠深度/几何解析的表面会被拒绝，避免把不可通行点交给规划器。

进入目标点约 4 m 半径后，系统提交完成状态。当前最后一小段 Poly5 可以执行结束，之后停止 YOPO 重规划并进入 SO3 位置保持。该项目不要求复杂的厘米级停稳判定。

## 8. 飞行历史路径和目标标记

Tab/Settings 中提供历史路径开关。开启后，主场景绘制无人机已经飞过的线条；关闭只影响显示，不影响日志、控制或规划。G 航点标记参与正常场景深度遮挡，建筑遮挡时不会穿透显示。

## 9. RGB、DA360 与规划状态

DA360 输出的是已经消除平移不确定性的 scale-invariant 相对视差，不会直接给出绝对米制深度。项目在运行时使用 `1/D=a·p` 的 scale-only 标定，并固定 `b=0`。当前尺度只经过 sim-to-sim 人工检查，自动跨地点精度门禁尚未通过；详细数据见 [DA360 米制尺度诊断](da360-metric-scale-diagnosis.md)。

| 字段或状态 | 含义 |
|---|---|
| <code>offline</code> | API 未连接、请求失败或传感器重置 |
| <code>preview</code> | 没有活动规划目标，显示诊断预览 |
| <code>planning</code> | 有活动目标，真实 RGB→DA360→YOPO 链路运行 |
| <code>stale</code> | 响应属于旧 frame、goal 或 generation，已丢弃 |
| <code>rgbCaptured=6/6</code> | 六面像素已经采集完成 |
| <code>depthLag=1f</code> | 可见深度预览相对当前 RGB 落后一帧 |
| <code>trajectory-ready</code> | 新轨迹通过检查并可交给控制器 |
| <code>planning-frame-too-old</code> | 观测超过应用时效，未安装为新轨迹 |

可见 DA360 预览是诊断显示。预览刷新频率不能直接代表 YOPO 的真实规划频率；规划时应查看 <code>mode=planning</code>、<code>trajectoryInstallHz</code>、<code>uniquePlanningHz</code> 和 capture-to-apply 延迟。

## 10. 日志与评估

每次导航结束、取消或切换目标时会下载一份飞行日志。公开分享前必须删除浏览器身份、设备信息、绝对路径和原始轨迹响应。

正式闭环评估示例：

~~~bash
python3 scripts/evaluate_closed_loop.py flight-log.json \
  --warmup-frames 30 \
  --min-duration-s 60 \
  --min-planning-frames 900 \
  --min-physics-coverage 0.95 \
  --output closed-loop-report.json
~~~

演示图生成：

~~~bash
python3 scripts/plot_demo_flights.py log-a.json log-b.json \
  --output-dir demo-figures \
  --summary demo-flight-summary.json
~~~

成功到达与“正式 15 Hz 门禁通过”是两个不同结论。短时零碰撞日志可以作为 Demo 证据，但不能替代 60 秒持续规划验收。

## 11. 常见问题

### Cesium WebGL 初始化失败

- 确认使用项目启动脚本，而不是普通浏览器快捷方式。
- 检查 <code>about:support</code> 或 <code>chrome://gpu</code> 中 WebGL/GPU 状态。
- Firefox 与 Chrome 的 GPU 参数不能机械照搬；ANGLE/EGL 失败时 Chrome 可能禁用 GPU 进程。
- 关闭占用 GPU 的大型程序后重试。

### Google Tiles 缺块或建筑穿模

- 检查代理节点延迟、丢包和 Google Tiles 请求错误。
- 等待主视图和全景视图建立缓存后再起飞。
- 不要把 <code>rgbCaptured=6/6</code> 误解为瓦片全部加载完成。
- 对未解析区域采用保守障碍处理，不能把缺块直接当成自由空间。

### Panorama preload 超时

- <code>panoPreloadRequired=0</code> 允许进入后继续实时加载。
- 若用于正式 Demo，应等待地图覆盖提示达到可接受状态。
- 不要在飞行中弹出阻塞式预加载窗口；飞行期间只允许后台、非阻塞加载。

### 遥控器有连接但没有控制

- 确认 Chrome 已获得串口权限。
- 确认 Arm 在中挡或高挡，Mode 在高挡 SO3。
- 查看通道监视器是否随摇杆变化。
- 检查 Roll/Pitch/Yaw/Throttle 映射与反向设置。

### 设置 G 航点后无人机不动

- 检查是否真正进入 SO3 且已 Arm。
- 查看是否出现 <code>trajectory-ready</code>。
- 连续 <code>planning-frame-too-old</code> 表示采集或网络延迟超过轨迹应用预算。
- 若目标落在建筑或未解析区域，系统会拒绝该目标。

## 12. 安全与隐私

- 仅绑定本机回环地址，不要把开发服务直接暴露到公网。
- 不提交 Cesium token、代理凭据、模型 checkpoint、原始日志或遥控器设备信息。
- 本项目没有真实飞行认证，不能直接部署到真实无人机。
