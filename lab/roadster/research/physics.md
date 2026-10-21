# 物理与证据

**贴坡靠压差，离地靠喷气，回正靠力矩。** Tesla官网已确认风机、底部密封和冷气推进，但没有公布完整质量、推力曲线或控制遥测。[官方说明](https://www.tesla.com/roadster)

| 问题 | 最小模型 | 必须保留的条件 |
|---|---|---|
| 起飞 | `az = Tz/m − g` | Tz是世界竖直分量；车身翻转会改变喷口方向 |
| 耗气 | `ṁ = T/(Isp·g0)` | 比冲、储气质量是模型假设；随排气更新总质量 |
| 吸附 | `D = Δp·Aeff` | 仅近表面、密封有效时成立；离面后迅速衰减 |
| 斜坡抓地 | `N = D + mg cosθ`；`mg|sinθ| ≤ μN` | 静态局部平面；曲坡运动另需法向加速度 |
| 回正 | `τ = r × F`；`Iω̇ + ω×(Iω) = τ` | 机体系方程；单轴可简化为Iα=τ；回正后仍需制动 |
| 落地 | `N平均 ≈ mg − Tz平均 + mv²/(2s)` | 一维制动近似；喷口竖直力、接触点速度、悬架行程共同决定结果 |

唯一保留的直觉算例：**假设2吨、有效面积4 m²，水平倒挂临界压差约4.9 kPa**；μ=1、135°斜面则约6.94 kPa。两者均不是Tesla实测。

风扇的压差吸附与排气反作用共享能量，不能各算一份完整功率。[风扇模型与专利](fan-patents.md)

任意姿态都能回正，需要足够时间、空间、控制力矩和剩余气量。**任意低高度保证成功不可能**。训练页面展示有限样本与失败案例。静止起转并刹停的理想下界为 `t ≥ 2√(Iθ/τmax)`；向下速度v还需至少 `v²/(2a向上净)` 的制动余量。这些是必要约束，不是充分保证。

来源：[NASA推力](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/rocket-thrust/) · [比冲](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/specific-impulse/) · [OpenStax压力](https://openstax.org/books/university-physics-volume-1/pages/14-1-fluids-density-and-pressure) · [摩擦](https://openstax.org/books/university-physics-volume-1/pages/6-2-friction) · [转动](https://openstax.org/books/university-physics-volume-1/pages/10-key-equations) · [McMurtry倒置先例](https://mcmurtry.com/upside-down-driving/)。
