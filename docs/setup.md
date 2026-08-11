# 安装与运行

## 运行层级

项目提供三个相互递进的入口：

1. `./launch.sh --local`：手动飞行和浏览器功能；
2. `./scripts/start_da360_api.sh`：独立 DA360 深度服务；
3. `./start-all.sh`：Web、DA360 与 YOPO combined 服务。

所有服务默认仅监听 `127.0.0.1`。如需跨主机调试，请自行增加经过认证的反向代理，
不要直接开放开发服务器或推理 API。

## Web 依赖

```bash
./scripts/bootstrap.sh web
export CESIUM_ION_TOKEN='your-restricted-token'
./launch.sh --local
```

bootstrap 会获取 CesiumJS 1.117 和 PlayCanvas 2.17.2，并使用仓库依赖清单中已有的
发布信息检查安装结果。已有但校验失败的目录不会被自动覆盖。

`CESIUM_ION_TOKEN` 只在请求 `/runtime-config.js` 时注入浏览器，响应禁止缓存。不要通过
URL query 传递 token，也不要把 token 写入脚本、README、Issue 或日志。按 Cesium Ion
官方建议使用最小权限 token，并限制允许访问的资产和 URL。

## 自治依赖

```bash
python3 -m pip install --user gdown
./scripts/bootstrap.sh autonomy
./scripts/build-autonomy-image.sh
```

autonomy 模式会在 web 依赖基础上获取固定版本的 DA360 源码、官方 DA360 large 权重，
以及 YOPO_55/e10 公开 Release。默认文件位置：

```text
models/da360/DA360_large.pth
models/yopo/YOPO_55/epoch10.pth
```

模型、源码 checkout 和 Web 构建均由 `.gitignore` 排除。运行时可使用：

- `DA360_MODEL_PATH_HOST`
- `YOPO_MODEL_PATH_HOST`
- `DA360_DEPTH_CALIB_PATH_HOST`
- `MINDCLOUD_API_IMAGE`
- `MINDCLOUD_WEB_PORT` / `MINDCLOUD_API_PORT`

覆盖默认值。

## 深度模式

`DA360_DEPTH_MODE=da360-relative` 只提供相对深度预览，不授权规划。

`DA360_DEPTH_MODE=da360-metric` 使用固定标定把视差映射为米制深度。仓库默认标定仅为
人工保留的 sim-to-sim 实验基线，自动精度门禁失败。服务会严格核对 checkpoint、输入
尺寸和 ERP 投影指纹；任何不匹配都会拒绝请求。

## 浏览器

Chrome/Chromium 和 Firefox 都可用于基本操作。`launch-firefox-gpu.sh` 是 Linux +
NVIDIA 环境的可选辅助脚本，不代表项目只能使用 Firefox。WebHID 是否可用取决于浏览器、
HTTPS/loopback 安全上下文和系统设备权限。

## 常见问题

- 页面提示缺少 Cesium token：在启动服务的同一 shell 中导出 `CESIUM_ION_TOKEN`。
- 找不到 Cesium/PlayCanvas：运行 `scripts/bootstrap.sh web`。
- 找不到模型：运行 `scripts/bootstrap.sh autonomy`，或通过环境变量指定文件。
- metric 请求返回尺寸错误：从 `/health` 读取 `calibration.request_width` 和
  `calibration.request_height`，不要硬编码旧尺寸。
- GPU 容器无法启动：检查 `nvidia-smi`、NVIDIA Container Toolkit 和 Docker GPU 支持。
