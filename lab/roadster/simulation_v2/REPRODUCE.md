# 复现浏览器与物理试验

网站需保留 `simulation_v2/` 与同级的 `vendor/mujoco/`。使用本地静态服务打开 `lab.html`，不要直接双击 HTML。MuJoCo 的 JS、WASM 和许可证已随网站提供，浏览器计算不请求云端物理服务。

以下命令在 `simulation_v2/` 中运行，适用于 macOS/Linux。原生复核使用 Python 3.12；JS 复核使用 Node。Python 环境和依赖不打入包中，首次安装需要软件源或预先下载的安装包。

网页下载版将离线记录改为 ZIP / GZIP，以满足托管容量限制。每个条目都与压缩前文件逐字节一致，没有删减试验。先在本目录解压，恢复原相对路径；离线整包若已含裸文件，可跳过对应解压。

```sh
python3 -c "import gzip,pathlib; pathlib.Path('results.json').write_bytes(gzip.decompress(pathlib.Path('results.json.gz').read_bytes()))"
python3 -m zipfile -e training-episodes.jsonl.zip .
python3 -m zipfile -e candidate-records.jsonl.zip .
python3 -m zipfile -e archive-records.zip .
python3 -m zipfile -e development-controller-scan.json.zip .
python3 -m zipfile -e development-braking-feedforward.json.zip .
```

原始账本 SHA-256 为 `81f35d51c384ae883c865f5d5e1fc1341fd5e43b3feb0f22f976740b7e859aac`，共 19,039 条。原账本压缩与历史复现验证记录见 `browser-snapshot.json`、[public-reproduction-check.json](public-reproduction-check.json)。该复核报告保持历史原件；本轮仅重新组织下载文件，未重新执行物理评估。新增 ZIP 的逐项字节与 SHA-256 校验见 [download-packing-check.json](download-packing-check.json)。

| 下载 | 解压后位置 |
|---|---|
| [完整训练记录 GZIP](results.json.gz) | `results.json` |
| [逐代候选 ZIP](candidate-records.jsonl.zip) | `candidate-records.jsonl` |
| [逐次账本 ZIP](training-episodes.jsonl.zip) | `training-episodes.jsonl` |
| [历史依赖 ZIP](archive-records.zip) | `archive/` 原目录及全部18个文件 |
| [控制器开发扫描 ZIP](development-controller-scan.json.zip) | `development-controller-scan.json` |
| [制动前馈开发 ZIP](development-braking-feedforward.json.zip) | `development-braking-feedforward.json` |

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python test_core.py
.venv/bin/python export_parity.py
node test_controller.mjs
node test-browser-runtime.mjs
.venv/bin/python launch_timing_audit.py
.venv/bin/python test_timing_audit.py
```

这些命令会重新生成相应复核记录。双引擎测试使用本目录 `.venv/bin/python` 和网站随附的 MuJoCo WASM。`run_case.py` 的姿态输入为度，角速度输入为 rad/s；浏览器界面的角速度输入为度/s，导出初态时会换算。

训练中的快照使用 `.venv/bin/python audit_training_records.py --partial`。只有 `results.json` 已标为 `complete`，才运行完整账本审计、最终策略和步长检验：

```sh
.venv/bin/python audit_training_records.py
.venv/bin/python audit_results.py --steps
.venv/bin/python run_case.py --task moving_recovery --height 8 --vx 4 --roll 150 --pitch 20 --policy final
node benchmark_timing.mjs
```

解压 `archive-records.zip` 后，以下历史依赖恢复到原目录。ZIP 内已带 `archive/` 路径，不要额外套一层目录。

| 位置 | 文件 | 用途 |
|---|---|---|
| `archive/attitude-only-blowdown-r4/` | `core.py`、`config.json`、`results.json` | 旧起转事件与原始版本核对 |
| `archive/launch-coupled-r5/` | `core.py`、`config.json`、`results.json`、`policies.json`、`launch-comparisons.json`、`launch-timing-audit.json` | 历史回放、旧控制器诊断、跨版本初态复核 |
| `archive/energy-coupled-r6-short-run/` | `INTERRUPTION.json`、`results.json` | 中断记录与重复训练前缀核对 |
| 当前目录 | `history-timing-replays.json` | 四条历史诊断回放的缓存，来源指纹会核对 |

`reproduction-dependencies.json` 保留这些文件解压后的路径与哈希。整轮结果、逐代候选、逐次账本和检查点分片也须保留；训练中快照的对应关系见 `browser-snapshot.json`。历史 14 参数仅由历史版本处理，不能填入 r6 的 15 参数控制器。

如需重训，先复制整份目录，再在副本中运行 `.venv/bin/python train.py`。该命令会重写副本中的训练记录。不要对正在训练或作为交付证据的目录重复启动。重新测得的耗时取决于机器、系统负载和运行环境，不构成 20 ms 硬实时保证。
