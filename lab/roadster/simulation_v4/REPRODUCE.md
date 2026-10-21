# 复现统一腾空试验

在完整离线资料目录中运行，保留并列的 `simulation_v3/` 与 `simulation_v4/`。v4 复用 v3 冻结的 MuJoCo 物理、压力模型、分配器和刹降预测，不能只复制一个控制文件。

已有环境：

```sh
simulation_v2/.venv/bin/python simulation_v4/run_case.py --case initial-state.json --output new-flight.json
```

新环境使用 Python 3.12：

```sh
python3.12 -m venv simulation_v4/.venv
simulation_v4/.venv/bin/pip install -r simulation_v3/baseline_r6/requirements.txt
simulation_v4/.venv/bin/python simulation_v4/run_case.py --case initial-state.json --output new-flight.json
```

初态格式如下。四元数顺序为 w,x,y,z；角速度为车体坐标的 rad/s。`goal.turns` 取 0 或 1，分别表示回正落地或回正后完整一圈再落地。

```json
{"id":"inverted","pos":[0,0,9],"quat":[0,1,0,0],"vel":[5,0,-1],"omega":[0,0,0],"mass":2000,"fuel":260,"thrust_scale":1,"goal":{"turns":0}}
```

用 `--policy initial` 或 `--policy reference` 对照训练前与人工参考。`--parameters parameters.json` 可传入按清单顺序排列的 14 个数。输出文件已存在时拒绝覆盖。输出包括全部物理帧、配置、喷口输出与控制源码哈希。

浏览器的“计算新试验”使用相同控制公式和 MuJoCo 3.15 WASM。下载本次数据会保留初态、参数、扰动、逐步记录和实测耗时。浏览器测得的耗时不是硬实时保证。

训练记录包保存所有代的候选参数、初态、终态指标、固定开发评估与检查点。需要从头重训时，在副本里另存已有 `campaign/`，再执行 `train.py`；它拒绝覆盖同名目录。`evaluate.py` 只接受已完成且哈希一致的训练，评估后再运行 `audit_steps.py` 做 1 ms 复查。留出数据不能回填为新一轮训练的独立测试集。

本轮每代 18 个候选各测 6 个初态，使用 CEM 搜索一组共享参数。第 1 代放入一个明确标注的人工参考候选。最终参数按固定开发集选择，所有候选通过同一物理世界。姿态路点、PD、分配器和刹降预测是人工设计，不能把它们说成训练自行发明。

随网站提供的浏览器核对可直接执行 `node simulation_v4/test-browser.mjs`，读取压缩的原生状态记录，独立重算 12 次飞行并写入 `browser-parity-rerun.json`。`node simulation_v4/benchmark-timing.mjs` 会在本机重新计时，写入 `timing-benchmark-rerun.json`，保留原报告。
