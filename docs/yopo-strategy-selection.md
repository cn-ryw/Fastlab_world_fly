# 可选YOPO策略

项目保留原YOPO策略为默认值，并提供第二套宽楼恢复策略。两套策略使用相同网络结构和浏览器调用链；切换只改变checkpoint和与其匹配的YOPO overlay。

| 名称 | checkpoint | 感知深度 | 最大轨迹距离 | 时域 |
|---|---|---:|---:|---:|
| `baseline`（默认） | YOPO_55/epoch10 | 20 m | 18 m | 1.125 s |
| `d70_h30_epoch30` | YOPO_70/epoch30 | 70 m | 30 m | 1.875 s |

启动第二套策略：

```bash
MINDCLOUD_YOPO_STRATEGY=d70_h30_epoch30 ./start-all.sh
```

回到默认基线：

```bash
MINDCLOUD_YOPO_STRATEGY=baseline ./start-all.sh
```

启动完成后核对运行身份：

```bash
curl --noproxy '*' -s http://127.0.0.1:5688/yopo/health \
  | python3 -m json.tool
```

第二套策略应显示：

```text
strategy = d70_h30_epoch30
model = epoch30.pth
config = d70_h30_cruise15_recovery.yaml
```

checkpoint本地路径为`models/yopo/d70_h30_epoch30/epoch30.pth`。`models/`按项目规则不进入Git；迁移工作目录时必须单独复制该文件。配置和策略清单是可读文本，不使用内容哈希。

该策略在权威ROS仿真中完成固定seed初筛：空旷稳态约15.78 m/s，宽楼seed 3无碰撞、无1–4 m/s龟速、离障碍后2 s约15.52 m/s且未触发25 m/s²加速度裁剪。这里是simulator raycast结果，不代表Cesium+DA360闭环已经验收；在本项目中仍应先进行Firefox/GPU sim-to-sim测试，不用于真机。
