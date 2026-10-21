# Roadster 研究与训练

- [驾驶沙盘高速旋转回正修复](driving-recovery.md)：制动分配、200 个独立初态与实际坡道验证，保留失败和 99% 目标的限制。

当前入口是[统一腾空控制](unified-airborne.md)。飞行实验的回正和完整 kickflip 使用同一个反馈器与同一组训练参数；驾驶沙盘的意外腾空回正使用独立修复。页面可以比较训练前后、选择各代检查点，并改变初态重新做物理试验。

网站入口包括[原片与物理](../index.html)、[飞行实验](../lab.html)、[驾驶沙盘](../driving.html)、[JEV 实验](../jev.html)、[素材库](../materials.html)与本资料页。车模与失败视频合并到素材库；旧入口保留跳转。

- [姿态喷气辅助的对照验证与供气](driving-assistance.md)。
- [驾驶模型、假设与验证](driving-dynamics.md)。
- [物理事实与边界](physics.md)、[尺度估计](measurement.md)、[风扇专利](fan-patents.md)、[低重心依据](ev-cg-sources.md)。
- [官方素材](media.md)、[发布会原文入口](transcript-index.md)、[90 秒视频计划](content-plan.md)。
- [最新车模使用清单](latest-asset-audit.md)、[材质与排气](materials-and-control.md)、[四路线评审](cross-review.md)。
- [失败视频](../materials.html#failures)、[训练问题](training-lessons.md)、[原 100 代训练](../r6-lab.html)、[v3 回正修复](recovery-repair.md)、[早期试验](../legacy-training.html)。

最新 Meshy 着色模型已用于首页十镜头、四个物理试件、统一飞行实验和素材库。原片与照片仍是原始证据。首页镜头路径是教学重建；飞行实验的位置和姿态来自 MuJoCo 积分，两者有明确区分。

动力与接触参数仍为假设。仿真成功不能证明 Tesla 实车采用同样布局或能完成同样动作。历史报告保留原数字和原参数，不把新的控制结构当成旧训练的后续代数。
