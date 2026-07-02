# Google 3D Tiles Flight

浏览器中的 Google Photorealistic 3D Tiles 穿越机驾驶器。进入页面后选择城市、放置出生点，然后用键盘、手柄或 RC 遥控器飞行。右下角可显示机头 360 ERP 全景 RGB 和 DA360 深度。

## 环境要求

- Docker Engine
- Chrome / Chromium
- 浏览器可以访问 Cesium Ion 和 Google 3D Tiles
- 本地开发模式需要 Python 3
- DA360 深度推理需要 NVIDIA GPU、NVIDIA Container Toolkit、Python 3 + pip，以及可访问模型下载地址的网络

## 启动主进程
默认
```bash
./launch.sh
```

打开：

```text
http://127.0.0.1:8080
```

可选常用方式：

```bash
# 端口被占用时
PORT=18081 ./launch.sh

# 只启动服务，不自动打开浏览器
./launch.sh --no-open

# Docker 后台运行
./launch.sh --detach

# 停止后台容器
docker rm -f google-tiles-flight

# 本地开发模式
./launch.sh --local
```

## 启动副进程（DA360 深度估计）

首次使用前下载模型并启动推理服务：

```bash
python3 -m pip install --user gdown
./scripts/download_da360_model.sh
./scripts/start_da360_api.sh

# 心跳包自检：curl http://127.0.0.1:5688/health
```

停止感知的推理服务，只保留主进程飞飞机的功能的话就关掉这个docker就行了：

```bash
docker rm -f mindcloud-da360-api
```

注意，默认使用 `DA360_large`，DA360 服务端以实时优先的 `DA360_INPUT_SCALE=0.65` 推理，模型输入约为 `672x336`；在 RTX 4070 Ti SUPER 上真实 HTTP 端到端约 70 ms。右下角 RGB 全景仍保持原始显示尺寸；只有发送给 DA360 的深度请求会单独缩小，前端默认按 `da360UploadScale=0.2` 上传约 `134x67` 的 JPEG，再由服务端 resize 到模型输入尺寸。前端默认 `depthMs=100`，推理未完成时不会堆积请求。

默认不建议换模型；实验中 `DA360_large` 的 fast 档比 `DA360_small` 保留了更好的深度排序和边缘一致性。只有显存、功耗或部署体积受限时，再自行覆盖模型名：

```bash
DA360_MODEL=<large|base|small> ./scripts/download_da360_model.sh
DA360_MODEL=<large|base|small> ./scripts/start_da360_api.sh
```

如需主动调整 DA360 服务端模型输入尺寸，可设置推理 scale 或指定模型输入宽高；过低的 `DA360_INPUT_SCALE` 可能让 large 模型输出条带化深度，不建议低于 `0.46`。服务端 resize 默认使用 `DA360_RESAMPLE=bicubic`，与 DA360 原项目的输入缩放方式保持一致：

```bash
DA360_INPUT_SCALE=1.0 ./scripts/start_da360_api.sh
DA360_INPUT_SCALE=0.46 ./scripts/start_da360_api.sh
DA360_INPUT_WIDTH=476 ./scripts/start_da360_api.sh
DA360_INPUT_WIDTH=672 DA360_INPUT_HEIGHT=336 ./scripts/start_da360_api.sh
DA360_RESAMPLE=bilinear ./scripts/start_da360_api.sh
```

推理服务不在本机时：

```text
http://127.0.0.1:8080/?da360Url=http://<host>:5688/depth
```

## 使用流程说明

1. 点击 **Start Google 3D Tiles Flight**。
2. 等页面进入 **PLACEMENT MODE**。
3. 用 Cesium 搜索框搜索城市或地点。
4. 按住 `I` 并点击建筑、道路或地面设置出生点。
5. 用 `W/A/S/D` 微调水平位置，`Shift` 加快微调。
6. 设置 **SPAWN ALTITUDE (m)**。
7. 按 `O` 确认出生点。
8. 选择 **First Person** 或 **Third Person** 开始飞行。

常用按键：

```text
↑ / ↓       前进 / 后退
← / →       左右平移
W / S       上升 / 下降
A / D       左右偏航
Shift       加速
R           重置
V           切换视角
P           返回放置模式
Tab         设置面板
```

键盘可直接使用，也支持手柄（但需要自己优化映射），手柄通常会被 Chrome 的 Gamepad API 自动识别。RC 遥控器或 WebHID 设备可在设置面板中连接；如需检查 Linux 输入权限：

```bash
./launch.sh --input-status
./launch.sh --setup-input
```

![](asset/display/screenshot-20260703-011815.png)
![](asset/display/20260703-005006.jpg)
![](asset/display/20260703-005023.jpg)

## 全景相机实现原理

全景 RGB 默认从机头 360 相机位置采集，输出 `672x336` ERP 图。实现方式是对 Cesium/Google Tiles 渲染结果进行 6 个方向采样，然后在 GPU 中按 ERP 射线模型重投影：

```text
yaw   = pi - (u + 0.5) / W * 2pi
pitch = vfov / 2 - (v + 0.5) / H * vfov
```

这保证投影模型与 YOPO_360 的 ERP 相机一致；区别是数据来源为 Cesium 渲染视图，而不是仿真栅格的直接 raycast。放置阶段会后台创建全景采样 viewer；确认出生点后会在用户可控前预采样一张全景首帧。飞行中默认 `panoMs=16`、`panoFace=192`、每个采样方向等待 `panoFrameDelayMs=8`，并最多等待 `panoFaceTileTimeoutMs=900` 让当前方向 tiles idle；首帧预加载使用 `panoPreloadFrameDelayMs=96`、`panoPreloadFaceTileTimeoutMs=6000` 和 `panoPreloadTimeoutMs=60000`，默认 `panoPreloadRequired=1`，未拿到完整 6 面首帧不会进入可控飞行。为了避免 Google Tiles 天空/极区采样在 ERP 顶部形成海市蜃楼状伪影，默认对顶部 10 度和底部 2 度做极区 guard；guard 区域保持 ERP 坐标，只向上下极点采样淡出，不会把整张图压缩到 guard 边界。可用 `panoTopPoleGuard` / `panoBottomPoleGuard` 调整或设为 0 关闭。

进入可控飞行前，主 Cesium 视图会预加载出生点周围区域，并分别等待第一人称和第三人称初始视角 tiles idle。默认 `flightPreloadStrict=0`，主视图只要目标区域覆盖率达标就继续；全景首帧预加载独立检查隐藏 viewer 的 6 个方向 tiles idle。只有显式设置 `?panoPreloadRequired=0` 时，才会允许全景首帧失败后进入飞行并让实时采样继续重试。

常用参数：

```text
# 更高输出分辨率
http://127.0.0.1:8080/?panoWidth=1036&panoFace=768

# 调整采样视图等待时间
http://127.0.0.1:8080/?panoFrameDelayMs=16&panoPreloadFrameDelayMs=120

# 调整首帧全景预加载超时；或允许首帧失败后继续进入飞行
http://127.0.0.1:8080/?panoPreloadTimeoutMs=90000&panoPreloadFaceTileTimeoutMs=9000
http://127.0.0.1:8080/?panoPreloadRequired=0

# 调整起飞前主视图预加载范围和覆盖率门槛
http://127.0.0.1:8080/?flightPreloadRadius=600&flightPreloadMinCoverage=0.98

# 调整 RGB / 深度更新间隔
http://127.0.0.1:8080/?panoMs=1000&depthMs=1200

# 调整 ERP 极区 guard
http://127.0.0.1:8080/?panoTopPoleGuard=0&panoBottomPoleGuard=0

# 调整仅用于 DA360 的上传尺寸或缩放，不影响 RGB 全景显示
http://127.0.0.1:8080/?da360UploadScale=0.35
http://127.0.0.1:8080/?da360UploadWidth=672
```
