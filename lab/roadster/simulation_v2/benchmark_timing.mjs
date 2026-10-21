// Instrument the unchanged r6 controller and official MuJoCo WASM runtime.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import os from 'node:os';
import {performance} from 'node:perf_hooks';
import {createSimulation,loadEngine} from './browser-runtime.mjs';
import {controllerStep} from './controller.mjs';

const bytes=await readFile(new URL('./policies.json',import.meta.url)),policies=JSON.parse(bytes);
const cfg=policies.config,params=policies.policies.final?.parameters,windowMs=cfg.physics.control_dt*1000;
assert.equal(policies.revision,'energy-coupled-r6');assert.ok(['evaluating','complete'].includes(policies.status));assert.ok(Array.isArray(params));
const sourceHashes={};
for(const file of ['core.py','controller.mjs','browser-runtime.mjs','world.xml','benchmark_timing.mjs'])sourceHashes[file]=createHash('sha256').update(await readFile(new URL(file,import.meta.url))).digest('hex');
sourceHashes['policies.json']=createHash('sha256').update(bytes).digest('hex');
assert.equal(sourceHashes['core.py'],policies.sourceSha256,'Policy must belong to the current physics revision');
assert.equal(sourceHashes['controller.mjs'],policies.controllerSha256,'Policy must belong to the current controller revision');
assert.equal(windowMs,20);

const qmul=(a,b)=>[a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];
const euler=(r,p,y)=>qmul(qmul([Math.cos(y/2),0,0,Math.sin(y/2)],[Math.cos(p/2),0,Math.sin(p/2),0]),[Math.cos(r/2),Math.sin(r/2),0,0]);
const cases=Object.keys(cfg.tasks).map(task=>({id:task,task,mass:cfg.vehicle.mass,fuel:cfg.fuel.initial_kg,thrust_scale:1,pos:[0,0,task==='moving_recovery'?9:cfg.vehicle.ground_cg_height+.005],quat:euler(task==='moving_recovery'?Math.PI:0,0,0),vel:[...cfg.tasks[task].initial_velocity],omega:[0,0,0]}));
cases.push({...cases[2],id:'free_attitude',pos:[1,-2,8],quat:euler(2.2,.6,-.4),vel:[4,-2,-2],omega:[.2,-.4,.3],mass:2100});
const warmupRounds=1,minimumMeasuredRounds=4,minimumWindowsPerCase=600,xml=await readFile(new URL('./world.xml',import.meta.url),'utf8');
const bundledEngine=new URL('../vendor/mujoco/mujoco.js',import.meta.url);
const moduleUrl=process.env.MUJOCO_JS??(existsSync(bundledEngine)?bundledEngine:new URL('../site/dist/vendor/mujoco/mujoco.js',import.meta.url)).href;
const engineStart=performance.now(),engine=await loadEngine(moduleUrl),engineLoadMs=performance.now()-engineStart;
const loadBefore=os.loadavg(),started=performance.now(),rows=[],allController=[],allStep=[];
const summarize=values=>{
  assert.ok(values.length&&values.every(v=>Number.isFinite(v)&&v>=0));
  const sorted=[...values].sort((a,b)=>a-b),q=p=>sorted[Math.max(0,Math.ceil(sorted.length*p)-1)];
  return {samples:sorted.length,p50Ms:q(.5),p95Ms:q(.95),p99Ms:q(.99),maxMs:sorted.at(-1),over20ms:sorted.filter(v=>v>windowMs).length};
};
for(const c of cases){
  let measuring=false,controllerSamples=[];
  const timedController=(...args)=>{
    if(!measuring)return controllerStep(...args);
    const start=performance.now();
    try{return controllerStep(...args);}finally{controllerSamples.push(performance.now()-start);}
  };
  const createStart=performance.now(),sim=await createSimulation(cfg,params,c,{engine,xml,policyId:'final',controllerStep:timedController}),modelCreateMs=performance.now()-createStart;
  const stepSamples=[],outcomes=[],overruns=[];let warmupSteps=0,expected,measuredRounds=minimumMeasuredRounds;
  try{
    for(let round=-warmupRounds;round<measuredRounds;round++){
      sim.reset(c,params,{policyId:'final'});measuring=round>=0;let frame=sim.snapshot(),steps=0;
      while(!frame.done){
        if(measuring){const start=performance.now();frame=sim.step(1);const elapsed=performance.now()-start;stepSamples.push(elapsed);if(elapsed>windowMs)overruns.push({round,step:steps,simulatedTime:frame.t,phase:frame.goal.phase,wallMs:elapsed});}
        else frame=sim.step(1);
        steps++;assert.ok(steps<=Math.ceil(cfg.physics.max_time/cfg.physics.control_dt)+1,'Episode did not terminate');
      }
      const fingerprint=JSON.stringify({status:frame.status,t:frame.t,pos:frame.pos,quat:frame.quat,fuel:frame.fuel,score:frame.metrics.score});
      if(expected===undefined)expected=fingerprint;else assert.equal(fingerprint,expected,'Instrumentation/repetition changed the physical outcome');
      if(measuring)outcomes.push({round,steps,status:frame.status,simulatedSeconds:frame.t,stableSeconds:frame.contact.stableSeconds});else{warmupSteps+=steps;measuredRounds=Math.max(minimumMeasuredRounds,Math.ceil(minimumWindowsPerCase/steps));}
    }
  }finally{sim.dispose();}
  assert.equal(controllerSamples.length,stepSamples.length,'Exactly one feedback update is required per control window');
  allController.push(...controllerSamples);allStep.push(...stepSamples);
  rows.push({case:c,modelCreateMs,warmupSteps,measuredRounds,controller:summarize(controllerSamples),step20ms:summarize(stepSamples),outcomes,overruns});
}
assert.ok(allStep.length>=2000,'Use thousands of measured control windows');
const report={
  schemaVersion:1,status:'measured_not_wcet',generatedAt:new Date().toISOString(),revision:policies.revision,selectedGeneration:policies.policies.final.generation,engine:'Official MuJoCo 3.15.0 WebAssembly',
  environment:{runtime:`Node ${process.version}`,platform:os.platform(),release:os.release(),architecture:os.arch(),cpuModel:os.cpus()[0]?.model??'unknown',logicalCpuCount:os.cpus().length,totalMemoryBytes:os.totalmem(),loadAverageBefore:loadBefore,loadAverageAfter:os.loadavg()},
  method:{controlWindowMs:windowMs,physicsStepMs:cfg.physics.dt*1000,physicsStepsPerControl:Math.round(cfg.physics.control_dt/cfg.physics.dt),warmupRoundsPerCase:warmupRounds,minimumMeasuredRounds,minimumWindowsPerCase,measuredRoundsRule:'max(4, ceil(600 / steps_in_warmup_episode))',warmup:'每个初态先完整运行一次，预热记录不计入百分位。每轮重置同一初态与最终参数；短失败轨迹重复至每个初态至少600个测量窗口。',controllerScope:'controllerStep 的同步调用：状态反馈、阶段切换、目标力矩、喷口分配；不包括传感器、数据传输或渲染。',stepScope:'完整 step(1)：控制器、10 次 MuJoCo 物理子步、阀门/供气/风机、接触与稳定判定、返回快照；包含计时包装开销。',excluded:'引擎加载、XML 模型创建、每轮 reset、文件读取、UI/Worker 通信与渲染另列或未测；没有实时调度、CPU绑核或强制GC。',percentiles:'nearest-rank：排序后取 ceil(p*N) 项；max 为本次观察最大值。',coverage:'三项标准任务及一个非轴对齐自由姿态；重复测时不等于更多独立物理初态。'},
  engineLoadMs,benchmarkWallSeconds:(performance.now()-started)/1000,aggregate:{controller:summarize(allController),step20ms:summarize(allStep)},cases:rows,sourceHashes,
  limits:['本结果来自当前电脑上的 Node，不能替代浏览器 Worker 实测。','只覆盖列出的轨迹和实际经过的控制分支；快速算完不代表飞行任务成功。','p99 和观察最大值不是最坏执行时间 WCET；即使本次无超时，也不能保证每个 20 ms 窗口。','控制使用模拟器完整状态，尚未测 IMU 噪声、采样延迟、状态融合、真实计算机或执行器链路。']
};
await writeFile(new URL('./timing-benchmark.json',import.meta.url),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({environment:report.environment,engineLoadMs,aggregate:report.aggregate,caseSteps:rows.map(r=>({case:r.case.id,warmup:r.warmupSteps,measured:r.step20ms.samples})),limits:report.limits},null,2));
