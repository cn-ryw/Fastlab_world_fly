# 贡献指南

感谢参与 MindCloud World Fly。请从当前 <code>main</code> 创建聚焦的功能分支，并在 PR 中说明用户可见行为和验证环境。

## 提交前

1. 不提交模型、原始标定数据、原始飞行日志、个人路径、凭据或设备信息。
2. 运行 <code>python3 scripts/audit_public_tree.py</code>。
3. 运行全部 JavaScript 测试和非 GPU pytest。
4. 自治链改动必须说明 depth mode、标定身份、YOPO 策略和仍未通过的门禁。
5. 第三方代码、模型或资产必须提供可核验来源、版本和许可证。

文档应区分“已实现”“自动测试通过”“GPU 在线验证”“真实飞行验证”。不要把相对深度称为米制深度，也不要把人工接受的 sim-to-sim 标定写成自动精度门禁通过。

## 测试

~~~bash
for test_file in tests/*.js; do node "$test_file" || break; done
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
python3 scripts/audit_public_tree.py
~~~

涉及浏览器/GPU 的改动还需要记录浏览器、GPU/驱动、启动脚本、实际 YOPO 策略和复现实验步骤。
