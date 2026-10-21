# 浏览器物理实验 · r6

MuJoCo 3.15.0 WASM 每 2 ms 积分，控制器每 20 ms 读取当前状态并更新喷口。改初态后重新计算，不用预录轨迹决定位姿。

- 整车重心离地 0.50 m、前后 50:50 为假设；`frame.pos` 就是整车重心。供气全程衰减，没有恒压平台。
- kickflip 离地后先小角速转动并积累上升动能；估算的高度和滞空时间足够后锁存 `energyReady`，用目标净力为零的力偶加速滚转，再由反馈控制降落。阀门延迟留下的实际余推仍会积分。
- 两项 kickflip 必须实际达到重心峰值 **5.0 m**，并满足原有安全接触和连续 1 秒稳定条件。5 m 是本轮任务目标，不是通用安全高度；不是要求起飞后每个时刻都高于 5 m。
- 高度奖励只来自仿真实际位置，失败后 0.40 秒尾段的反弹不再增加峰值或高度奖励。落稳但高度不足记为 `height_shortfall`。
- 允许向前滚动，没有水平速度或位置目标；仍检查竖直速度、侧滑、轮胎接触滑移、姿态与角速。硬件、质量、压力和接触参数未因 r6 增强。

```js
const engine = await loadEngine('./vendor/mujoco/mujoco.js');
const sim = await createSimulation(config, parameters, initialCase, {
  engine, xml: worldXml, policyId: 'final'
});
const frame = sim.step(1);
sim.applyImpulse([0, 2000, 0]); // 下一物理子步在整车重心施加 N·s 冲量
sim.reset(newCase, parameters, {policyId: 'untuned'});
sim.dispose();
```

初态使用米、秒、弧度；`quat` 为 w,x,y,z，`vel` 为世界坐标，`omega` 为车体坐标。穿地、超出气罐容量和恒压供气模式会报错。

`goal` 保留控制采样时刻的能量、高度、几何预测与启动阶段。`frame.geometryClearance` 来自帧末真实姿态，`metrics.minimumHeightMet` 来自真实峰值。`allocation.requestedForceWorld` 保存采样时刻的目标力，不能用帧末姿态重算；初始和关闭控制为零，旧录像缺失时显示“未记录”。

运行 `node simulation_v2/test-browser-runtime.mjs` 做双引擎和判定规则验收；运行 `node simulation_v2/benchmark_timing.mjs` 测当前最终策略耗时。100 代搜索选中第 74 代参数，20 项核验通过，包含明确标注的合成高度判定测试。标准原地与有初速 kickflip 都达到约 6.39 m 并落稳；标准倒悬回正和自由姿态样例仍失败。任务留出分别通过 28/40、28/40、0/40，自由姿态 0/80。1 ms 复算中两项 kickflip 各降至 27/40，边界案例和额外 0.5 ms 检查保留在 `audit.json`，不能声称步长无关。

最终参数在 Node 中测量了 3,248 个控制窗，控制器 p95 为 0.077 ms，完整 20 ms 仿真窗 p95 为 0.920 ms、最大 6.399 ms，本次无超时。环境、预热、全部分位数和覆盖边界见 `timing-benchmark.json`。这不是中期浏览器计时，也不能作为实时性保证。

这是简化轮胎、假设喷口布局和理想传感器实验；外观网格不参与碰撞。能量估计不是飞行轨迹，Node 计时不是浏览器或实车 WCET 保证。

公开包的安装、复核命令和历史依赖见 [复现说明](REPRODUCE.md)。历史对照所需的 r4、r5 与中断短程文件收录在 `archive-records.zip`，按复现说明解压后恢复原路径，浏览器引擎保留在同级 `vendor/mujoco/`。
