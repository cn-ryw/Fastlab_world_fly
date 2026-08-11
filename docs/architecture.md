# 架构说明

## 浏览器生命周期

`src/main.js` 负责 loading → placement → view-select → flight 生命周期。Cesium 世界层将
ECEF/ENU 坐标映射到仿真本地坐标，其中 `x` 指东、`y` 指上、`z` 指北。Drone、HUD、
控制器和轨迹执行器只处理本地米制坐标。

## 感知链

隐藏的 Cesium viewer 分别渲染前、后、左、右、上、下六个面。PlayCanvas/WebGL
投影器将六面纹理重投影为 ERP RGB，再按配置缩放为 DA360 请求。飞行和标定使用不同
capture profile：飞行优先时延，标定要求逐面等待 tiles ready。

`PerceptionFrame` 把 RGB、采集变换、模拟时间、实际/参考状态、目标 identity 和投影
配置绑定成不可变快照，避免旧图像与新位姿配对。前端只接受与当前 goal、generation 和
frame identity 一致的响应。

## 深度与规划

Combined 服务运行在单一 loopback 端口：

- `/health`：DA360、标定和输入指纹；
- `/depth`：相对或 metric 深度；
- `/yopo/health`：YOPO checkpoint、配置和规划授权；
- `/yopo/plan_full`：RGB → DA360 → YOPO → Poly5 完整链；
- `/yopo/preview`：复用最近规划深度生成预览，不重复推理。

只有结构完整且明确接受的 metric 标定会授权 YOPO。相对深度可显示，但规划必须失败
关闭。规划结果经 lattice/candidate 诊断、时域 fast-forward、安装年龄和状态偏差检查后，
才交给浏览器 SO3 控制器。

## 信任边界

Web 服务只暴露 `index.html`、`src/`、`asset/` 和 Cesium 静态目录；仓库根目录、Git、
模型、脚本和实验数据不能通过 HTTP 读取。gate-path API 只接受同源 loopback 请求，
并校验 Host、文件名、大小和 JSON。Combined API 只允许配置的 loopback origin。

Cesium token 是浏览器能力凭据，因此只能降低暴露风险，不能成为服务端秘密。项目通过
运行时注入、URL 限制、`no-store` 和日志排除减少意外传播；维护者仍需在 Cesium Ion
控制台设置最小权限、资产与 URL 限制。
