<h1 data-i18n-en="Roadster acceleration and braking calibration">Roadster 加速与制动校准</h1>

<p data-i18n-en="2026-10-22. Current driving version.">2026-10-22。当前驾驶版本。</p>

<h2 data-i18n-en="Public evidence and calibration target">公开资料与本次目标</h2>

<p data-i18n-en="Tesla’s October 16 configuration post lists 0–60 mph below 1.6 s for Performance, below 0.9 s for Thrust and below 0.59 s for Rocket. Performance includes fan suction; the augments add cold-gas propulsion. These are manufacturer claims for different configurations, not independent production-car measurements.">Tesla 10 月 16 日官方配置帖列出 Performance 的 0–60 mph 小于 1.6 秒，Thrust 小于 0.9 秒，Rocket 小于 0.59 秒。Performance 配有风扇吸附；后两者另有冷气推进。这些是厂方宣称，不能混为同一加速配置，也不是独立量产实测。</p>

<p><a href="https://x.com/Tesla/status/2110905588402295283" data-i18n-en="Tesla configuration announcement">Tesla 官方配置原帖</a> · <a href="https://www.tesla.com/roadster" data-i18n-en="Tesla Roadster product page">Tesla Roadster 官方页</a></p>

<p data-i18n-en="The user-requested calibration targets about 1.8 s without the fan, with fan-on performance measured separately. No published stopping distance or complete tire and wheel-torque curves for the new Roadster were found. Braking figures below are high-grip sandbox results only.">本次按用户要求将风扇关闭时的普通电驱校准到约 1.8 秒；风扇开启时另测性能。未找到新 Roadster 的公开制动距离与完整轮胎、轮端扭矩曲线，制动数字只作为本沙盘的高附着路面模拟结果。</p>

<h2 data-i18n-en="Changes under the same test conditions">相同测试条件下的变化</h2>

<p data-i18n-en="Total mass is 2140 kg including a full 400 kg gas tank. Dry high-grip flat ground, with jets disabled. Acceleration starts at the full-pedal request from rest with no rollout subtraction. Stops begin near 100 km/h and end below 0.1 m/s, excluding driver reaction time. All measurements stay inside the map; boundary resets cannot count as stopping.">总质量 2140 kg，含 400 kg 满气；干燥高附着平地，喷气关闭。计时从静止全油门请求开始，不扣除起步滑行距离。停车从约 100 km/h 开始，低于 0.1 m/s 记停，不含驾驶员反应时间；车辆始终在地图内，不能把触边复位计作停车。</p>

<table><thead><tr><th data-i18n-en="Metric">指标</th><th data-i18n-en="Before · fan off">修改前 · 风扇关</th><th data-i18n-en="Current · fan off">当前 · 风扇关</th><th data-i18n-en="Current · fan on">当前 · 风扇开</th></tr></thead><tbody>

<tr><td data-i18n-en="0–60 mph">0–60 mph</td><td>3.412 s</td><td>1.776 s</td><td>1.513 s</td></tr>

<tr><td data-i18n-en="0–100 km/h">0–100 km/h</td><td>3.534 s</td><td>1.835 s</td><td>1.571 s</td></tr>

<tr><td data-i18n-en="Approx. 100–0 km/h stopping distance">约 100–0 km/h 制动距离</td><td>45.88 m</td><td>24.32 m</td><td>21.76 m</td></tr>

<tr><td data-i18n-en="Approx. 100–0 km/h stopping time">约 100–0 km/h 制动时间</td><td>3.74 s</td><td>2.32 s</td><td>2.14 s</td></tr>

</tbody></table>

<p data-i18n-en="Each condition is one deterministic simulation. 60 mph equals 96.56064 km/h, not 60 km/h. Wheel torques and MuJoCo contacts produce acceleration and braking; vehicle speed is never prescribed. Gas remains at 400 kg throughout.">以上为每种条件的一次确定性仿真。60 mph = 96.56064 km/h，不是 60 km/h。加速和制动均由轮轴扭矩及 MuJoCo 接触计算，没有直接设置车辆速度；全程气量保持 400 kg。</p>

<p><a href="driving-dynamics-20261022-baseline.json" data-i18n-en="Baseline measurements">修改前记录</a> · <a href="driving-dynamics-20261022.json" data-i18n-en="Current measurements and source hashes">当前记录与源码哈希</a></p>

<h2 data-i18n-en="Model assumptions">建模假设</h2>

<p data-i18n-en="Tire friction changes from 0.9 to 1.8 in both contact physics and the wheel-torque limiter, with a 5% traction margin. Wheel-power, drive-force and mechanical-brake limits are calibrated to 1 MW, 46 kN and 40 kN respectively. These are simulation assumptions, not measured Tesla component specifications. Regeneration remains 6 kN / 120 kW, so lifting off and pressing the brake produce different deceleration.">轮胎摩擦系数由 0.9 调至 1.8，接触求解和轮轴扭矩限制共用该值；扭矩留 5% 附着余量。电驱轮端功率上限 1 MW、总驱动力上限 46 kN、机械制动力上限 40 kN，均为此次模拟校准值，不能视作 Tesla 的实测部件参数。再生制动仍为 6 kN / 120 kW，松油门与踩刹车的减速度不同。</p>

<p data-i18n-en="To resist higher contact loads, steering uses 300000 N·m/rad stiffness, a 24000 N·m torque limit and 1800 N·m·s/rad implicit damping. Peak uncommanded front-wheel deflection is about 0.97 degrees in the full-pedal ramp test. The test driver adapts its small pedal inputs; production control gains no speed-target or automatic slow mode.">更高轮载下，转向机构使用 300000 N·m/rad 刚度、24000 N·m 力矩上限和 1800 N·m·s/rad 隐式阻尼。全油门冲坡测试的非指令前轮偏转峰值约 0.97°。测试驾驶员按新动力调节小油门；生产控制器没有新增目标车速或自动限速模式。</p>

<h2 data-i18n-en="Regression checks and limits">回归检查与边界</h2>

<p data-i18n-en="Repeated flips, 80/100 km/h car jumps, ramp braking and reversing, free edge departures and post-landing controls pass. Eight combinations of fan setting, turn direction and initial speed each sustain full-pedal cornering for over 60 seconds without body contact or reset.">连续翻转、80 / 100 km/h 三车飞跃、坡上制动倒车、坡边自由脱离与落地后操控均通过。八种风扇开关、左右转向和初速组合，各持续满油门转向超过 60 秒，无车身触地或复位。</p>

<p data-i18n-en="On the same 200 full-gas random starts, with uniform orientation, 2–12 rad/s spin, 6–20 m CG height and 5 m/s horizontal speed, fan-off and fan-on runs each yield 179/200 safe landings. Per-case pass/fail classifications match the prior version. Acceptance still requires about one second of stable tire support without body or excessive contact. This is not a driving-incident distribution; the 99% goal remains unmet.">同一组 200 个满气随机初态，均匀姿态、角速度 2–12 rad/s、重心 6–20 m、水平速度 5 m/s，风扇关和开分别为 179/200 平稳落地，逐例成功/失败分类与修改前一致。验收仍为约一秒稳定轮胎支撑、无车身或超限碰撞；该分布并非驾驶事故分布，99% 目标仍未达到。</p>

<p data-i18n-en="All 12 full-gas altitude cases at 20/40/80 m pass with the fan both off and on. Recovery actions succeed in 40/48 cases of the original ramp distribution, but complete collision-free runs including entry impacts remain 0/48. No failures were removed and no acceptance thresholds were relaxed.">20 / 40 / 80 m 的 12 个满气高空案例，风扇开关均 12/12 通过。原 48 例坡道分布的回正动作成功为 40/48，但包含驶入冲击的全程无碰撞仍为 0/48。没有删除失败或放宽验收阈值。</p>

<p><a href="driving-recovery-validation-dynamics.json" data-i18n-en="Random recovery · fan off">随机回正 · 风扇关</a> · <a href="driving-recovery-fan-validation-dynamics.json" data-i18n-en="Random recovery · fan on">随机回正 · 风扇开</a> · <a href="driving-ramp-recovery-validation-dynamics.json" data-i18n-en="Complete ramp records">坡道全程记录</a> · <a href="driving-landing-timing-validation-dynamics.json" data-i18n-en="Altitude · fan off">高空 · 风扇关</a> · <a href="driving-landing-timing-fan-validation-dynamics.json" data-i18n-en="Altitude · fan on">高空 · 风扇开</a></p>

<p data-i18n-en="This calibration covers dry-ground 0–100 km/h acceleration and braking only. The existing load-distributed four-wheel drive approximation remains; real transmission, suspension, tire temperature, battery thermal limits and high-speed aero are not reconstructed, and top speed is not calibrated. Browser verification is not real-vehicle or physical-mobile acceptance.">本次只校准干燥场地的 0–100 km/h 加速与制动。保留现有按轮载分配的四轮驱动近似，没有复原实车传动、悬架、轮胎温度、电池热限制和高速气动，极速也未标定。浏览器验证不等于实车或实体手机验收。</p>
