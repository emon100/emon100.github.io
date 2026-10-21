# 第三方模型核查

本轮未选中可验证、可下载且匹配 2026 年 10 月 15 日新车体的模型。没有用旧 Roadster 凑数。

- [Tesla 官网](https://www.tesla.com/roadster)此前的页面与公开资源观察未发现 GLB、GLTF、OBJ、FBX、USDZ 或 BLEND 下载入口。本轮补查未找到新入口，直接 HTML 请求返回 403 后停止。这只能说明本轮未找到，不能断言官方不存在模型。
- [Sketchfab 的“2026 Tesla Roadster”](https://sketchfab.com/3d-models/2026-tesla-roadster-110c3b2a7b144014b66ec328c3190bc5)实际发布于 2023-11-09，车体检查属于旧设计。虽标为可下载及 CC BY，仍不适合本项目。
- [ArtStation 模型](https://impylse.artstation.com/store/dVPg/tesla-roadster-interior-3d-model-physics-rig)说明为 2020 原型，排除。
- 新查到的 [DigiSim 交互展项](https://digisim.io/museum/electric-roadster)声明外形为参考宣传图制作的程序模型，尺寸与隐藏结构暂定。已读取的 HTML 没有直接模型下载链接；脚本请求返回 403，复用授权也未确认，仅保留为参考入口。

继续使用本地制作的参考车模和官方图像依据。未下载上述模型、未操作 Ego、未升级 Meshy。来源、时间与检查范围见 [manifest.json](manifest.json)。
