# 浏览器回正控制 v3

`controller.mjs` 是冻结 `controller.py` 的逐项移植。它每 20 ms 读取当前姿态、角速度、位置、速度、气量和实际阀门开度，输出 12 个非负喷口指令及风机指令。它没有写入位置或姿态；飞行和接触由官方 MuJoCo 3.15.0 WASM 以 2 ms 步长积分。

当前观测是模拟器真值，代表理想传感器。未加入 IMU 噪声、延迟、状态融合或真实设备通信，因此不能称为实车控制验证。

回正使用完整四元数 PD、受可达角减速度约束的角速参考和提前刹转。分配器优先保留姿态力矩，再分配竖直力，不追踪水平位置或速度大小。正立后才缓慢选择与滚动方向一致的航向。世界水平目标力严格为零；实际阀门滞后和车身转动仍可产生瞬时水平合力。

预测有两部分：姿态回正时间与滑翔时间是局部预算；刹降预测是假定姿态固定的竖直积分，最长 6 秒。后者保留 2 ms 阀响应、气量与压力更新及 20 ms 质量更新，并从当前实际阀门开度开始。它不预测今后的姿态变化或额外姿态喷气，不是完整六自由度 MPC，也不是可达保证。真实着陆标准保持 r6 原样。

浏览器入口：

```js
import {createSimulation} from './simulation_v3/browser-runtime.mjs';
import {controllerStep,createMemory} from './simulation_v3/controller.mjs';
const sim = await createSimulation(config, parameters, initialCase, {
  engine, xml, controllerStep, createMemory, policyId: 'v3_recovery'
});
const frame = sim.step(1); // advances one 20 ms control window
sim.applyImpulse([100, 0, 0]);
sim.reset(initialCase, parameters);
sim.dispose();
```

用于 `moving_recovery`。`parameters` 和 `SETTINGS` 必须对应 `manifest.json`；配置与硬件沿用冻结 r6。`browser-runtime.mjs` 相比 r6 只增加控制器输入中的实际阀门开度和喷口力，物理子步、碰撞、奖励与落地判定代码逐字相同。`world.xml` 也相同。

8 个开发初态的 1,855 个控制窗完成纯控制对照和独立原生/WASM 飞行对照：两端均为 7 成功、1 失败；位置最大差约 1.8×10⁻¹⁴ m，实际单口力约 2.2×10⁻⁹ N。它验证移植一致，不代表未见初态成功率；正式评估另见原生评估记录。

本机 Node 计时 5,035 个完整控制窗，预热不计入：控制器 p95 0.846 ms、最大 2.024 ms；完整物理窗 p95 1.351 ms、最大 2.553 ms。另测 3,000 步的最长预测，最大约 6.02 ms。Node 测时不包括浏览器 Worker 调度、渲染或传感器链路，也不能给出最坏执行时间保证。浏览器实测另存；不要把分配器或预测器单独耗时当作全链路保证。

额外用低气量、空罐和喷口失效的固定观测触发完整控制器最长预测分支，400 次测量中最慢为 8.27 ms，无超过 20 ms。记录在 `controller-stress-timing.json`；该项包含控制器与预测器，未包含物理积分和调度，仍不构成最坏时间保证。

对照与计时文件：`controller-parity.json`、`browser-verification.json`、`predictor-parity.json`、`timing-benchmark.json`。源码与报告指纹见 `BROWSER-SNAPSHOT.json`。原生 fixture 由 `export_fixtures.py` 导出；Python 使用 r6 的虚拟环境，浏览器依赖已随站点提供。

```sh
simulation_v2/.venv/bin/python simulation_v3/export_fixtures.py
node simulation_v3/test-controller.mjs
node simulation_v3/test-browser-runtime.mjs
simulation_v2/.venv/bin/python simulation_v3/export-predictor-fixtures.py
node simulation_v3/test-predictor.mjs
node simulation_v3/benchmark_timing.mjs
node simulation_v3/benchmark_predictor_stress.mjs
```

若复现目录没有站点的 `site/dist/vendor/mujoco/`，用 `MUJOCO_JS` 指向官方 3.15.0 `mujoco.js` 的文件 URL，且同目录保留 `mujoco.wasm`。无需 HiGHS。
