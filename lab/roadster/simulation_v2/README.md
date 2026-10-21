# 真实接触仿真与训练记录

MuJoCo 3.15.0，每 2 ms 积分、20 ms 控制。r6 用 CEM 训练 15 个反馈参数，共 100 代。姿态始终由引擎计算，没有预设动画轨迹。人工开发基线与未调参初始策略分开保存；人工基线不是 CEM 的学习成果。

两项 kickflip 要求真实质心峰值至少 5 m。这是本轮工程任务目标，不是普遍安全高度。控制器从离地上升阶段缓慢滚转，同时建立竖直动量；能量与滞空预算足够后，以净力目标为零的力偶加快翻转，随后制动下降。滚转制动段有对应的角加速度前馈。`energyHeight` 是重力参照量，不能代替真实峰值；当前姿态所需的几何高度另存为 `geometryMinimumCG`。

成功还须完成动作、没有车身触地或超限冲击，并连续 1 秒至少三轮接触。每 2 ms 检查倾角 <8°、角速 <0.20 rad/s、竖直和侧向速度各 <0.30 m/s、轮胎接地点滑移 <0.50 m/s。允许向前滚动，不要求刹停或回起点。接触稳定但真实高度不足会记为 `height_shortfall`。

12 个单向喷口、4.10 × 1.72 × 1.20 m 车体、质量、惯量、260 kg 气量和所有功率均为实验假设，不是 Tesla 实车复原。质心离地 0.50 m；没有水平位置或速度反馈。气压全程按剩余气量衰减，风扇压差与轴向反作用共用功率。轮胎采用被动轮铰链和简化接触，没有真实悬架或橡胶模型。

每代完整候选和逐案例结果在 [candidate-records.jsonl.zip](candidate-records.jsonl.zip)；每次实际评估另记入 [training-episodes.jsonl.zip](training-episodes.jsonl.zip)。解压后恢复原 JSONL 文件。每 5 代的三任务真实回放按 `checkpoint-index.json` 分片加载，显示当时选中的策略代数。第 50 代是中期。训练不会因退步而删记录。 `launch-timing-audit.json` 分开记录进入滚转阶段、首次非零力矩请求、真实喷口力矩和车体转动；事件来自 20 ms 原始记录，阶段切换不等于已经开始转动。

固定开发集 12 例中 2 例也用于训练。120 例任务评估和 80 例 SO(3) 评估不参与本轮选参数，但沿用先前版本的初态，并非全新测试集。r4、r5和中断的 r6 短程记录在 [archive-records.zip](archive-records.zip)，解压恢复 `archive/` 原目录。较大的开发记录见[控制器扫描 ZIP](development-controller-scan.json.zip)与[制动前馈 ZIP](development-braking-feedforward.json.zip)。短程未提交代的计算量无法精确恢复，单独记录范围。新旧奖励不同，不能直接比较原始分数。 开发记录的完整度见 `development-evidence-index.json`；临时原型未逐轮冻结源码，不能保证精确复演，正式 100 代则有完整源码指纹和账本。

```sh
.venv/bin/python test_core.py
.venv/bin/python export_parity.py && node test_controller.mjs
.venv/bin/python run_case.py --task moving_recovery --height 8 --vx 4 --roll 150 --pitch 20 --policy final
```

`results.json`、`policies.json` 保存结果与源码指纹。重跑训练前必须先归档现有记录；运行期间不要重复启动 `train.py`。失败材料保留供视频使用，失败也不证明其他控制方法必定不可达。

依据：[MuJoCo 接触动力学](https://mujoco.readthedocs.io/en/stable/computation/index.html)。

网页下载的解压次序见 [REPRODUCE.md](REPRODUCE.md)，逐字节压缩校验见 [download-packing-check.json](download-packing-check.json)。原 `public-reproduction-check.json` 是历史复核报告；下载打包调整不代表再次物理评估。
