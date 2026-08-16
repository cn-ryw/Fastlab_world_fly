# MindCloud World Fly 系统架构

![闭环仿真架构](assets/architecture/mindcloud-system-architecture.png)

## 可检索架构图

~~~mermaid
flowchart LR
    TASK[任务输入\nG 固定航点 / T8L 滚动航点]
    WORLD[CesiumJS 仿真世界\nGoogle 3D Tiles]
    CAMERA[六面相机采集]
    ERP[ERP RGB\n384×192]
    DA360[DA360\nscale-invariant 相对视差]
    SCALE[scale-only 标定\n1/D = a·p，b = 0]
    YOPO[YOPO\n候选评估]
    POLY5[Poly5 轨迹]
    SO3[SO3 控制器]
    UAV[无人机动力学]
    LOG[诊断支路\nRGB / 深度 / 日志 / 性能]

    TASK --> WORLD
    WORLD --> CAMERA --> ERP --> DA360 --> SCALE --> YOPO --> POLY5 --> SO3 --> UAV
    UAV -->|位姿与速度反馈| WORLD
    ERP -.-> LOG
    SCALE -.-> LOG
    POLY5 -.-> LOG
    UAV -.-> LOG
~~~

## 模块边界

| 子系统 | 职责 | 不承担的职责 |
|---|---|---|
| Cesium 主视图 | Demo 画面、任务交互、目标和路径显示 | 不直接生成 YOPO 轨迹 |
| 隐藏六面相机 | 从当前仿真状态采集六个方向并投影为 ERP | 不执行深度估计 |
| DA360 服务 | 从 ERP RGB 估计已消除平移不确定性的 scale-invariant 相对视差 | 不直接承诺米制尺度 |
| Scale-only 标定 | 以 `1/D=a·p`、固定 `b=0` 转换为仿真中的近似米制深度 | 不引入外部 disparity 偏移 |
| YOPO 服务 | 根据深度、目标和状态选择局部 Poly5 轨迹 | 不直接推进无人机物理状态 |
| SO3 控制器 | 跟踪轨迹并输出姿态/推力控制 | 不替代 YOPO 避障决策 |
| 诊断预览 | 展示缓存 RGB、深度和性能数据 | 预览频率不等于规划频率 |

## 感知帧与快速重规划

### DA360 米制尺度边界

DA360 在网络内部估计并消除了 disparity 的全局平移项，公开输出仍保留尺度不确定性。因此运行时只允许 `1/D=a·p` 的 scale-only 标定，外部偏移参数固定为 `b=0`。`a·p+b` 只可作为离线诊断，用来暴露输入、数据或模型契约问题，不得写入部署配置。当前 `a` 来自仿真真值到仿真预测的标定，是 sim-to-sim 近似尺度，不代表跨场景、跨相机天然成立的真实传感器标定。

每次请求必须把 RGB、采集时刻的位姿/速度、目标身份和 session 身份绑定，避免旧图像与新状态错配。这种绑定保证单个请求的一致性，但不锁死后续感知：latest-slot 会持续接受更新观测，一个请求完成后立即使用最新可用帧进行下一次重规划。

超过轨迹应用时效的响应会被丢弃，不允许旧轨迹覆盖新会话。该机制服务于快速重规划，而不是把整段飞行冻结在第一帧感知上。

## 坐标与航向

SO3 模式进入时保存第一帧世界 yaw。Roll/Pitch 的滚动航点方向在该固定世界航向下解释，避免机体倾斜造成的视觉航向耦合。Yaw 输入修改世界航向参考；当前 Throttle 不写入 YOPO 高度输入。

## 诊断支路

RGB PANORAMA 与 DA360 DEPTH 是诊断显示。规划链路使用真实的 RGB→DA360→YOPO 请求；为了主画面流畅，诊断窗口可以降低显示频率，但不能以此降低真实规划频率。
