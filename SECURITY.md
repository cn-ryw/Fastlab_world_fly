# Security Policy

## Supported version

安全修复面向当前 GitHub 默认分支。旧分支和历史提交仅作归档，不保证持续更新。

## Reporting

请使用 GitHub **Private vulnerability reporting** 提交安全问题，不要公开创建包含利用细节、
token、轨迹或本机信息的 Issue。维护者会在确认后协调修复与披露。

## Local deployment

- 所有开发服务默认只绑定 `127.0.0.1`；不要直接暴露到公网。
- Cesium Ion token 应使用最小 scope，并限制资产和允许 URL；怀疑泄露时立即轮换。
- 不要把 `.env`、模型、原始标定样本、飞行日志或设备信息提交到 Git。
- 运行 `python3 scripts/audit_public_tree.py --staged` 检查准备提交的内容。

当前维护版本已移除源码中的历史 Cesium token 和本地工具文件，但本项目不改写既有 Git
历史；旧提交仍可能包含已经公开过的信息。任何曾公开的 token 都应视为已泄露并由所有者
撤销，而不是仅依赖删除当前文件。
