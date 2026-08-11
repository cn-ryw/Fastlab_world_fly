# 运行时身份与 SHA 机制撤除说明

2026-08-11 按项目负责人要求，撤除了近期加入、且不是服务必需的启动期内容哈希机制。这些改动不改变 DA360 深度标定系数、YOPO 配置选择或规划授权。

已撤除：

- `start-all.sh` 对 DA360/YOPO checkpoint、YOPO 配置、Dockerfile 和依赖清单的内容哈希计算、比对与打印。
- combined 镜像的内容哈希 label、基础镜像 digest 强绑定，以及启动时 RootFS 血缘比对。
- `/health` 和 `/yopo/health` 中的 checkpoint/config 哈希字段。
- 每个规划响应中由模型与配置哈希派生的 `service_fingerprint`。
- metric loader 对 checkpoint 内容哈希的运行时强绑定；正式标定 JSON、归档 candidate 和 fit report 中对应的历史字段也已移除。
- 独立 `start_da360_api.sh`/`Dockerfile.da360` 的 server-script 哈希 label 与模型哈希环境变量。挂载当前 server 脚本时可安全复用现有镜像；不挂载时默认重建，不会用无哈希的猜测冒充新鲜性检查。

替代的可读契约：

- 启动预检直接核对文件存在、文件名、Cesium/PlayCanvas 版本头、YOPO 配置名和容器 tag。
- 依赖来源、版本与路径记录在 `dependencies.versions.json`。
- DA360 仍检查 checkpoint 参数覆盖率；YOPO 仍检查参数覆盖率、实际导入配置路径和基础配置存在性。
- metric calibration ID 优先读取 JSON 中的 `calibration_id`；旧文件没有该字段时，使用“文件名 + schema 版本 + 模型名 + 请求尺寸”生成稳定、可读 ID。
- combined server 启动时生成一个进程级 `service_session_id`；前端和 evaluator 只检查它在单次飞行内非空且稳定，用于防止跨服务重启混入日志。

标定 bundle 同样不再计算或保存内容哈希。防串样与完整性检查改由现有语义完成：manifest 与三份 artifact 的文件名和非空尺寸必须匹配，`sessionId/captureId/locationId/frameId` 必须一致，投影、pose、transform 与数组维度/有限性必须合法；拟合器继续拒绝重复 capture、重复 session/frame，以及同一地点近似相同 pose。该契约不会进入 health 或 planning 响应。
