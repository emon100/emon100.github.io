# Roadster 车模参考

应以发布会 Rocket 原型为主体。官网普通展示图主要反映 Thrust；双座、展示轮毂和整洁底部不能直接套入飞行动作。官网与预订页均把 Rocket 列为单座。此轮未找到长宽高、轴距、轮径、质量或 CAD 文件。[官网](https://www.tesla.com/roadster)、[车型配置](https://www.tesla.com/roadster/reserve)

最值得先看的新图是 [近侧视，3440×1024](/Users/emon100/IdeaProjects/rodstar/media/official-images/Roadster-02-Pilot-Desktop.jpg) 和 [前 3/4，2800×2074](/Users/emon100/IdeaProjects/rodstar/media/official-images/Roadster-08-End-Desktop.jpg)。它们适合修车壳；实际零件状态以 [飞行侧视帧](/Users/emon100/IdeaProjects/rodstar/media/frames-hd/09-kickflip-x-aligned.jpg) 为准。

| 部位 | 建模依据 | 可靠度 |
|---|---|---|
| 车壳 | 低鼻楔形、上扬腰线、宽后肩；门板较平，轮肩与车顶圆滑。避免整车都是硬折平板 | 高 |
| 车头 | 宽梯形鼻板、两侧斜切灯槽、黑色下腔与薄前唇 | 高 |
| 轮拱 | 车壳内的圆弧切口，薄白边；车轮嵌入壳体 | 高 |
| 车轮 | 官网六向主辐条；原型飞行帧呈浅灰圆盘面、双环与小黑圆心 | 外观高；实体轮罩待核 |
| 座舱 | 倾斜大风挡连低弧顶，铜金深色玻璃、白色细边，侧窗后端收窄 | 高 |
| 车门 | 门板连侧窗向上抬起；精确铰链轴未知 | 外形高；机构低 |
| 尾翼 | 整宽翼板由左右细支座抬起，尾灯在其下；收纳和展开分开处理 | 官网形态高 |
| 尾部 | 上排四小喷口、下排四大风扇舱，另有左右朝下喷口；不能画成一排排气管 | 可见分区高；内部低 |
| 底盘 | 原型下沿较厚，周边喷流硬件外露；完整密封裙和管路不可见 | 中 |
| 悬架 | 新图可见横置弹簧减震器、摇臂与连杆，不足以恢复运动学 | 图中形态高；原型对应低 |

尾部细节用 [展开组合图](/Users/emon100/IdeaProjects/rodstar/media/official-images/Roadster-06-Gravity-Desktop.jpg)、[正后特写，3456×2800](/Users/emon100/IdeaProjects/rodstar/media/official-images/Roadster-07-Augments-Desktop.jpg)、[Tesla X 尾部图](/Users/emon100/IdeaProjects/rodstar/media/x-official/2110904620948267152-1.jpg)。[新增悬架图](/Users/emon100/IdeaProjects/rodstar/media/official-images/Roadster-03-Carousel-Slide-5-Desktop.jpg) 适合局部机械细节。

近侧视的手工图像比例约为轴距/车长 0.62–0.67、车顶高度/车长 0.24–0.29、轮径/车长 0.145–0.175。这是透视图中的美术检查范围，不是真实尺寸。原型09帧复核为 H/L .27–.30，旧180 px高度含错误底点。统一模型采用4.1×1.72×1.20 m作为明确假设，H/L≈.293；不称官方或标定尺寸。读点和局限见 `model-reference.json`。

共新增 5 张官网原图，均保留原 URL 与尺寸。未把太空合成图当作原型证据。图片版权归 Tesla，未确认开放复用授权。完整部件记录与 15 项来源见 [结构化清单](/Users/emon100/IdeaProjects/rodstar/research/model-reference.json)。

## 本轮车模路线

Kimi k3 以 max 思考读取四张官网实图，独立生成车壳。Codex 派生版修正插值波纹、玻璃与车漆交叠、轮圈比例和风挡前缘，原件保留供比较。两份 GLB 都可下载；前脸、后肩和窗框仍近似，不能视为精确 CAD。

Meshy 登录成功，但创建被免费套餐限制拒绝；用户选择暂不升级，未生成。第三方检索排除了名称含“2026”但实际发布于 2023 年的旧车型，本轮未找到已验证可用的新车体下载。

## 用户提供的 Meshy 模型

用户另行手动生成并提供 GLB。已按完全独立的连通组件删除外围四墙，汽车40,564面原样保留。它适合作为外形与中远景运动底稿；车侧、轮圈和封闭底部仍简化，文件没有UV、贴图或材质。网页新增“Meshy 去墙白模”供比较，未替换物理碰撞模型。
