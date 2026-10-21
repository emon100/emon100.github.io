# 个人网站维护

- `README.md` 说明 GitHub Pages 部署与本地预览。发布分支为 `master`，目录为仓库根目录，保留 `CNAME` 和 `.nojekyll`。
- 个人主页保留现有布局和样式；更新文案和项目入口时复用已有结构。简历事实同步维护 `index.html` 与 `resume.tex`，不补写未经用户确认的后续任职。
- `lab/roadster/` 是相邻 `../rodstar/site/dist/` 的静态发布副本。涉及 Roadster 时先读取 `../rodstar/AGENTS.md`，在源项目修复和验证后同步，保持两个目录一致。
- 提交前运行 `node test-site.mjs`，仅提交本次相关文件，保留已有未提交文件。
- 用户要求发布时，推送前检查远端变更；等待 GitHub Pages 成功，再检查实际域名上的页面、子目录资源和受影响功能。
