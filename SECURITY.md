# 安全策略

## 支持版本

安全修复面向 GitHub 当前默认分支。历史标签和归档分支不保证持续更新。

## 报告漏洞

请使用 GitHub Private vulnerability reporting，不要公开提交包含利用细节、token、轨迹、代理凭据或本机信息的 Issue。

## 本地部署

- 开发服务默认只应绑定 <code>127.0.0.1</code>。
- Cesium Ion token 应采用最小权限并限制允许 URL，怀疑泄露时立即轮换。
- 当前版本从浏览器 <code>localStorage</code> 读取 Cesium Ion token，不再提供源码默认 token。
- 不提交 <code>.env</code>、模型、原始标定样本、飞行日志或遥控器设备信息。
- 发布前运行 <code>python3 scripts/audit_public_tree.py --staged</code>。

本项目不改写既有 Git 历史。任何曾公开的 token 都应视为已经泄露并由所有者撤销，不能只依赖删除当前文件。
