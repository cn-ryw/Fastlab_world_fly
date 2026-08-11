# 评估状态与边界

## Metric 标定

公开标定使用 4 个地点、12 次静止采集的聚合结果。原始 RGB、位置、轨迹、anchors、
manifest 和深度数组不公开。脱敏摘要位于
`config/calibration/da360-v1/fit-report-summary.json`。

| 指标 | 观察值 | 自动门禁 |
| --- | ---: | ---: |
| 有效 anchor 比例 | 0.742 | ≥ 0.70 |
| median AbsRel | 0.376 | ≤ 0.15 |
| p90 AbsRel | 1.507 | ≤ 0.30 |
| 10 m 内 p90 绝对误差 | 4.054 m | ≤ 1.0 m |
| 所有 LOLO folds | 失败 | 必须全部通过 |

因此自动门禁结论是 **失败**。固定尺度仅以人工保留的 sim-to-sim baseline 运行，不能
表述为精度验收通过。

## 自动测试

公开 CI 覆盖：

- 公共树隐私与密钥审计；
- 浏览器控制、ERP 几何、状态 identity、调度和轨迹契约的 JavaScript 测试；
- 非 GPU Python 拟合、评估、安全与服务契约测试；
- PlayCanvas 下载和依赖清单校验。

GPU 镜像验收还应执行 import smoke 和 `tests/test_backend_contract.py`。在线集成测试必须先
从 `/health` 获取 metric calibration 的请求宽高，再生成测试 JPEG；不得硬编码旧的
1036×518 尺寸。

## 15 Hz 与真实飞行

闭环评估器会检查 warm-up、测量时长、planning identity、轨迹实际安装回执、物理时间
覆盖和稳定的 calibration/service identity。这些约束存在不等于性能已经通过。目前没有
可公开复现的持续 15 Hz 通过报告，也没有真实低空闭环验证。

任何结果发布都应同时说明硬件、浏览器、GPU 驱动、输入尺寸、depth mode、标定身份、
测量窗口和失败项。相对深度预览或中段峰值频率不能作为 metric 闭环通过证据。
