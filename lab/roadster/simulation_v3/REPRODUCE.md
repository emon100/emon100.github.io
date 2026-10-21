# 复现 v3 回正实验

以下命令从项目根目录运行。v3 仅修复 `moving_recovery`，不重训或覆盖 r6 的 100 代记录。控制、动力学和数据指纹见[清单](manifest.json)，结果与数值敏感性见[审计](audit.json)。

## 单次真实实验

本项目已有环境时：

```sh
simulation_v2/.venv/bin/python simulation_v3/run_case.py --output /tmp/recovery-v3.json
simulation_v2/.venv/bin/python simulation_v3/run_case.py --case /tmp/initial-case.json --output /tmp/custom-recovery-v3.json
```

省略 `--case` 时使用名义 9 m 倒置初态。初态 JSON 的 `pos` 为米、`quat` 为单位四元数 `[w,x,y,z]`、`vel` 为 m/s、`omega` 为机体系 rad/s；可另给 `mass`、`fuel`、`thrust_scale` 和 `disabled_nozzles`。可从[已冻结初态](evaluation-cases.json)取出一条完整案例，避免漏字段。加入 `--baseline` 可对同一初态运行原 r6 控制器。输出包含真实积分帧、12 口输出、接触和成败。

独立安装使用 Python 3.12，安装包需从软件源或预先准备的本地文件取得：

```sh
python3.12 -m venv simulation_v3/.venv
simulation_v3/.venv/bin/python -m pip install -r simulation_v3/baseline_r6/requirements.txt
simulation_v3/.venv/bin/python simulation_v3/test_frozen.py
simulation_v3/.venv/bin/python simulation_v3/run_case.py --output /tmp/recovery-v3.json
```

锁定依赖是 MuJoCo 3.15.0、NumPy 2.5.4、SciPy 1.18.1。新分配器不需要额外安装 `highspy`；仍需 SciPy，因为冻结的 r6 基础模块在导入时引用它。保留 `config.json`、`manifest.json`、`controller.py`、`landing_predictor.py`、`run_case.py`、`evaluate.py`、`allocator/` 与 `baseline_r6/`。单次实验不读取旧的大结果文件。

## 全量评估

先复制工作目录，保留原交付证据。在副本中把已有 `results.json`、`step-audit.json`、`step-comparison.json` 与 `evaluation/` 移入独立存档，再运行以下命令；脚本会拒绝覆盖已有汇总。若使用新环境，把解释器路径换为 `simulation_v3/.venv/bin/python`。

```sh
simulation_v2/.venv/bin/python simulation_v3/evaluate.py --prepare-only
simulation_v2/.venv/bin/python simulation_v3/evaluate.py
simulation_v2/.venv/bin/python simulation_v3/audit_steps.py
simulation_v2/.venv/bin/python simulation_v3/finish_step_audit.py
simulation_v2/.venv/bin/python simulation_v3/summarize_evaluation.py
```

全量重跑还必须保留这个相对路径：

`simulation_v2/archive/energy-coupled-r6-100-generations/results.json`

网站下载版中同一冻结文件在 `simulation_v2/results.json.gz`，先按 `simulation_v2/REPRODUCE.md` 解压为 `results.json`，再复制到上述路径。SHA256 应为 `0e842c7de451718be13be703be246deb6fc0033b2ac159f00ea26f3576e03213`。

它提供旧固定 40+80 个初态及原基线；文件 SHA 在 `evaluation-cases.json` 和结果的 `baselineReuse` 中核对。原始执行为 360 次新评估加 120 条旧基线复用；另有 240 次 1 ms 审计和仅 4 例的 0.5 ms 复查。新 seed 已在清单中登记。脚本不根据结果调参。

旧复用失败基线在首次失败时停止；新的 `record=True` 回放还保留失败后 0.4 秒。成功率和首个失败状态可比较，但失败耗气、时长、末态、峰值接触指标不能直接宣称改善。8 个开发配对案例及新 seed 两边均采用相同尾段记录规则。

原交付 `audit.json` 是一次已完成运行的冻结记录。重新执行会产生独立结果，不能沿用原耗时或结果指纹。原始逐例 JSON 保留在 `evaluation/`，下载版可按分组 ZIP 保存；不能把压缩包中的帧改写成插值轨迹。

浏览器独立 MuJoCo WASM 对照、纯控制对照和计时方法见[BROWSER.md](BROWSER.md)。本机计时不等于真实传感器与执行器链路的最坏耗时保证。

## 网站压缩下载

在线站点受展开体积上限约束，[reproduction-data.zip](reproduction-data.zip) 包含 `comparisons.json`、`browser-fixtures.json` 和 `development/002/results.json`。在 `simulation_v3/` 目录解压后再运行完整回放与浏览器一致性测试。压缩前后的字节与 SHA256 已核对；`source-index.json` 的 `compressedSources` 给出还原位置。原项目目录和完整离线包仍保留这些原文件。单次 `run_case.py` 不依赖这个数据包。
