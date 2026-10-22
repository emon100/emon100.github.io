# Tesla Roadster 公开喷口布局核对

核对日期：2026-10-22。对象为 2026 年公开的 Roadster Rocket 原型，不是已完成标定的量产配置。

## 数量与大致方向

MotorTrend 对 Tesla 设计与工程负责人的现场采访，以及 Top Gear 的现场报道，给出一致的十口分组。方向以下列车身正立状态为准：

| 位置 | 数量 | 大致喷气方向 | 用途 |
| --- | ---: | --- | --- |
| 尾部 | 4 | 向后 | 向前推进 |
| 各车轮后方、车身四角 | 4 | 向下 | 起升与下降制动 |
| 车头左右 | 2 | 向左右外侧 | 偏航控制 |

依据：[MotorTrend 一手采访](https://www.motortrend.com/news/first-look-2028-tesla-roadster-performance-thrust-rocket)、[MotorTrend 技术说明](https://www.motortrend.com/news/tesla-roadster-thrust-system-spacex-technology-explained)、[Top Gear 现场报道](https://www.topgear.com/car-news/first-look/wow-new-tesla-roadster-can-actually-fly)。两家报道均发表于 2026-10-16。

[Tesla 官网](https://www.tesla.com/roadster)将 Thrust 版描述为尾部冷气推进，将 Rocket 版描述为车身周边喷口，支持向前、向上加速和横向旋转。官网没有给出完整逐口坐标和安装角度。

尾部下排四个大风扇属于吸地系统，与上排冷气喷口分开，不能加入这十个冷气喷口的计数。官网[正立／倒立图](https://digitalassets.tesla.com/tesla-contents/image/upload/f_auto,q_auto/Roadster-06-Gravity-Desktop.jpg)可辨认尾部四个小喷口、四个大风扇，以及后侧向下喷口；单张图不能核对全部十口。

## 未确认的参数

没有在本次查到的一手资料中找到侧口上仰 5°、全部喷口的三维坐标、逐口推力曲线、阀门响应时间或完整分组阀路。报道中的“向下”“向左右”只支持大致方向，不能换算成精确安装角。也不能据此宣称喷口能够转向。

用户已选定“侧口上仰 5°”作为实验假设；这不是 Tesla 已确认参数。原先提出的“四个底部 + 八个侧向，共十二口”也不是上述公开原型布局。项目中原有“四组正负竖直喷口 + 四个水平喷口”的配置更与之不符，旧成功率不能当作实车布局的验证。

## 驾驶沙盘采用的布局

用户已选定按公开十口分组重建。驾驶物理、喷力分配、阀门数组和烟雾位置统一改为 4 个底部向下喷气口、4 个尾部向后喷气口、2 个车头侧向喷气口。侧口上仰 5° 仍只是用户选定的假设。没有竖直向上喷气口。

精确坐标、18 / 10 kN 单口推力、90 kN 总流量上限、400 kg 储气与压力模型均未实车标定。主动 kickflip 使用同样的十口物理执行器，但继续沿用冻结的动作路径规划；冻结飞行与 JEV 实验不改写。逐口定义与方向检查见 [喷口审计](driving-nozzle-audit.json)，新布局的测试结果见 [驾驶回正报告](driving-recovery.md)。

## 本次核对的范围

网页工具读取 Tesla 动态页面不完整，因此另用浏览器检查了官网正文；已核对两家现场采访／报道并查看官网图片。官方发布会和 Top Gear 原始视频可以访问，但浏览器没有提供可用字幕，本记录不声称完成逐句回听。

原始视频：[Tesla 发布会](https://www.youtube.com/watch?v=qt5p-_tE8d0)、[Top Gear 对 Tesla 团队的访谈](https://www.youtube.com/watch?v=JdkIOhFmbKI&t=1643s)。本地官网图片保存在 `research/nozzle-evidence-20261022/`，未修改或发布图片。
