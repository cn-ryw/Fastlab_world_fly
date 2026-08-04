# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 快速运行 Demo

### 1. 启动 Web 服务 (MindCloud 飞行器)

```bash
./launch.sh          # Docker 模式, 自动构建并打开 Chrome
./launch.sh --detach # 后台运行 (不自动打开浏览器)
./launch.sh --local  # 本地 Python 开发模式
```

浏览器访问: `http://127.0.0.1:8080`

> **GPU 渲染**: 本机 `prime-select on-demand`, 需用 Firefox + prime offload 才能用 RTX 5070 Ti 渲染 Cesium:
> ```bash
> __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia firefox http://127.0.0.1:8080/
> ```
> Chrome 不兼容 prime offload GLX (已确认 WebGL 初始化失败)。Firefox 已验证正常。

> **代理**: Clash Verge TUN 模式会拦截 `cesium.com` / `tile.googleapis.com`, 已添加 merge 规则绕过。如遇 504 或 tile 加载失败, 检查代理配置。

### 2. 下载 DA360 模型

```bash
python3 -m pip install --user gdown
./scripts/download_da360_model.sh large   # 推荐 (1.34 GB, README 首选)
./scripts/download_da360_model.sh small   # 快速验证用 (391 MB)
```

下载到 `third_party/DA360/checkpoints/DA360_<model>.pth`。

### 3. 构建并启动 DA360 推理服务

```bash
# 构建镜像 (基于本地 dzp_yopo, 无需网络)
docker build -f Dockerfile.da360.yopo -t mindcloud-da360:latest .

# 启动 GPU 推理 (DA360_small 示例)
docker run --rm -d --init \
  --name mindcloud-da360-api \
  --gpus all -p 5688:5688 \
  -v $(pwd)/third_party/DA360/checkpoints/DA360_small.pth:/models/DA360_small.pth:ro \
  -e DA360_INPUT_SCALE=0.65 \
  mindcloud-da360:latest \
  python3 /opt/mindcloud-da360/scripts/da360_server.py \
    --model-path /models/DA360_small.pth --host 0.0.0.0 --port 5688

# 自检
curl http://127.0.0.1:5688/health
# → {"ok":true,"model":"DA360_small","device":"cuda",...}
```

### 4. 飞行操作

1. 浏览器打开 `http://127.0.0.1:8080`
2. 点击 **Start Google 3D Tiles Flight**
3. 搜索城市 → 按住 `I` 点击建筑/地面设置出生点
4. `W/A/S/D` 微调 → `O` 确认 → 选择视角开始飞行

| 按键 | 功能 |
|------|------|
| ↑↓ | 前进/后退 |
| ←→ | 左右平移 |
| W/S | 上升/下降 |
| A/D | 左右偏航 |
| Shift | 加速 |
| R | 重置 |
| V | 切换视角 |
| P | 返回放置模式 |
| Tab | 设置面板 |

右下角面板显示 360° ERP 全景 RGB + DA360 深度。

### 5. DA360 深度接口 (新增)

| 端点 | 方法 | 说明 |
|------|------|------|
| `/health` | GET | 心跳 + 模型信息 |
| `/depth` | POST | 接收 JPEG → 返回伪彩深度图 (data URL) |
| `/depth/raw` | POST | 接收 JPEG → 返回 `pred_disp` float32 `.npz` (不逐帧归一化) |

```bash
# 测试 /depth/raw
python3 scripts/request_da360_raw.py test.jpg -o raw.npz
python3 scripts/inspect_da360_raw.py raw.npz --vis preview.png
```

### 6. 米制深度离线拟合管线 (M1→M2→M3)

```
Cesium ERP RGB → /depth/raw → pred_disp.npz
Cesium anchors  → sampleMetricDepthAnchors() → anchors.json
                                    ↓
            scripts/fit_da360_metric.py
                1/z = a * pred_disp + b
                                    ↓
              metric_depth.npy + fit_report.json
```

```bash
# 拟合 (需先采集 pred_disp.npz + anchors.json)
python3 scripts/fit_da360_metric.py \
    --raw experiment_data/metric_fit_sample/depth_raw.npz \
    --anchors experiment_data/metric_fit_sample/cesium_anchors.json \
    --output experiment_data/metric_fit_sample/
```

### 停止服务

```bash
docker rm -f google-tiles-flight   # 停止 Web
docker rm -f mindcloud-da360-api   # 停止 DA360
```

---

## 环境

- **宿主**: Ubuntu 24.04, RTX 5070 Ti 16GB, Driver 580.173.02, CUDA 12.8
- **Web**: Docker (`tumgis/3dcitydb-web-map:alpine`), 端口 8080
- **DA360**: Docker (`dzp_yopo:sim-u2004-noetic-py38` 基镜像), 端口 5688, GPU (CUDA)
- **Python**: 宿主 Python 3.12/3.13 (仅用于脚本和测试); DA360 推理在容器内 (Python 3.10 + PyTorch 2.8)
- **YOPO 参考**: `/home/ykx/ros1/YOPO_360_v15` (只读, `feat/x5-cruise15-release`)
- **项目根**: `/home/ykx/projects/urban_highspeed_uav/MindCloud_World_Fly`

## Git

- **origin**: `cn-ryw/MindCloud_World_Fly`
- **upstream**: `superboySB/MindCloud_World_Fly`
- **当前分支**: `feat/da360-metric-depth-prototype`
- 禁止直接改 main, 禁止 force push, 禁止向 upstream push

## 架构

### 前端模块图 (src/)
`index.html` 加载 CesiumJS (1.121, 本地 `/ThirdParty/Cesium/` + CDN fallback), 然后 ES module 导入 `src/main.js`:

```
main.js (模式状态机: loading → placement → view-select → flight)
  ├── cesium-world.js   — Cesium Viewer + Google 3D Tiles, pickLocalRay, sampleMetricDepthAnchors
  ├── tiles-collision.js — 碰撞检测 (sampleHeight + pickFromRay + swept ray)
  ├── controller.js     — 输入系统: 键盘/Gamepad API/WebHID RC 遥控器
  ├── drone.js          — 四元数刚体物理: FPV 角速率模式 + Drone 速度指令模式
  ├── panorama-sensor.js — 6 面立方体采样 → GPU 重投影 → ERP 全景 RGB
  ├── erp-geometry.js   — ERP 像素↔方向转换 (与 YOPO_360 一致)
  ├── hud.js / osd.js   — 飞行 HUD + FPV OSD
  ├── gates.js          — 赛道子系统
  ├── path-editor.js    — 路径编辑器
  ├── path-store.js     — 赛道 JSON 持久化
  └── error-report.js   — 用户可见错误弹窗
```

### 坐标约定
- **Local**: x=east, y=up, z=north (NWU 变体)
- **Drone body**: x=forward, y=left, z=up
- **ERP**: `yaw = π - (u+0.5)/W * 2π`, `pitch = vfov/2 - (v+0.5)/H * vfov`

### DA360 推理管线
- `scripts/da360_server.py`: Flask API (`/health`, `/depth`, `/depth/raw`)
- `Dockerfile.da360.yopo`: 基于 `dzp_yopo:sim-u2004-noetic-py38` (CUDA 12.8 + PyTorch 2.8)
- 离线 wheel 安装: `third_party/wheels/flask_cors*.whl` + `timm*.whl`

### URL 参数
所有运行参数通过 URL query string 覆盖: `?key=value`
- `panoWidth`, `panoFace` — 全景输出分辨率
- `panoMs`, `depthMs` — RGB/深度更新间隔
- `da360Url` — DA360 服务地址
- `panoPreloadTimeoutMs`, `panoPreloadFaceTileTimeoutMs` — 首帧全景超时
- `panoTopPoleGuard`, `panoBottomPoleGuard` — ERP 极区 guard (默认 10°/2°)
