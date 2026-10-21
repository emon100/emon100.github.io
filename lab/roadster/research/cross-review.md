# 四种图稿

“A+B”表示A起稿、B审阅修改。十个镜头均可切换进度、保存SVG并选择版本；选择保存在当前浏览器，可导出。

| 路线 | 保留与修改 |
|---|---|
| Codex | 独立初稿保留。弯坡轮缘、Semi对位与旧车列布局存在已知简化，供比较 |
| Kimi k3 / max | 真实CLI独立生成。主站补齐宿主主题容器；原始动作分段和回正轴问题保留 |
| Codex + Kimi | Kimi修正弯坡轮心半径、Semi上方位置、车轮细节与力偶表达。集成时由Codex延长落地区，避免末帧与卡车相交 |
| Kimi + Codex | Codex修正接触、滚转、五车并排与镜头衔接；按官网和原片把车体细化为137个实体，修正遮挡排序，保留Kimi图面风格 |

四栏是解释图的风格比较，不是实测轨迹。209个网格的独立米制母模型见[精修外形](../model.html)。尺寸一致的场景、风机和真实控制结果见[飞行实验](../training.html)。

Kimi Code CLI 2.2.0，实际请求`kimi-code/k3`，`thinking.effort=max`。两条会话隔离。独立模块完成后停止后处理、由Codex检查（退出143）；交叉评审自然结束。没有把其他模型结果标成Kimi。

原始文件与哈希：[Kimi](kimi/kimi.freeze.json) / [Codex→Kimi](kimi/codex-kimi.freeze.json)。原件留档，集成文件另计哈希。[调用配置](kimi/config-evidence.json)
