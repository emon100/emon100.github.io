# 失败视频素材包

三例真实仿真回放，非Tesla实拍。每例含无文字与说明版，均1080×1920、30 fps、H.264、无音轨。

| 案例 | 内容 | 首次失败 | 源片结束 | 视频时长 |
|---|---|---:|---:|---:|
| [r4 · 先升高，后滚转](late-roll-start/README.md) | [无文字](late-roll-start/clean.mp4) / [说明](late-roll-start/annotated.mp4) | 2.78s | 3.20s | 3.233s |
| [r5 · 提前转，仍然太低](early-roll-low-height/README.md) | [无文字](early-roll-low-height/clean.mp4) / [说明](early-roll-low-height/annotated.mp4) | 1.40s | 1.80s | 1.833s |
| [r6 · 飞得高，落地过重](high-jump-hard-landing/README.md) | [无文字](high-jump-hard-landing/clean.mp4) / [说明](high-jump-hard-landing/annotated.mp4) | 4.60s | 5.02s | 5.067s |

每帧使用不晚于输出时刻的原始20ms记录，未插值姿态或重模拟。所有片段保留失败后约0.4秒；封装到30fps只补齐不足0.05秒的末帧，没有叙事停帧。两版的运动、镜头、时序完全一致，说明版仅叠加文字。

每例的 trajectory.json、config.json、provenance.json、render-schedule.json 可复核原始状态、版本、文件/源码哈希、是否为人工开发样例及采样映射。r6是未调参开发记录，不是100代训练最终结果。

原始PNG与说明PNG保留在本地各例 frames/、annotated-frames/。ZIP不重复收录PNG，包含6段MP4、3份轨迹、来源、字幕和复现脚本。重新渲染需原项目的Three、着色Meshy与烟雾模块；视频本身可独立播放。

画面外壳与碰撞几何不同；烟雾只示意方向和相对强度，不是CFD。实际失败由源物理记录判断，不能仅凭外观接触判定。

本地浏览：启动原项目 site/dist 的静态服务后打开 failure-library.html。
