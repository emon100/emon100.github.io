import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import crypto from 'node:crypto';
import {createSimulation,loadEngine} from './browser-runtime.mjs';
import {controllerStep} from './controller.mjs';
import {predictVerticalStop} from './landing-predictor.mjs';
const here=new URL('.',import.meta.url),manifest=JSON.parse(fs.readFileSync(new URL('manifest.json',here))),fixtures=JSON.parse(fs.readFileSync(new URL('browser-fixtures.json',here))),cfg=fixtures.config,params=manifest.parameters,xml=fs.readFileSync(new URL('world.xml',here),'utf8');
const hash=n=>crypto.createHash('sha256').update(fs.readFileSync(new URL(n,here))).digest('hex');
assert.equal(hash('controller.py'),manifest.controllerSha256);assert.equal(hash('landing_predictor.py'),manifest.predictorSha256);assert.equal(hash('allocator/conservative_allocator.py'),manifest.allocatorSha256);
const sum=values=>{const a=[...values].sort((x,y)=>x-y),q=p=>a[Math.ceil(p*a.length)-1];return {samples:a.length,p50Ms:q(.5),p95Ms:q(.95),p99Ms:q(.99),maxMs:a.at(-1),over20ms:a.filter(x=>x>20).length};};
const loadStart=performance.now(),engine=await loadEngine(process.env.MUJOCO_JS??new URL('../site/dist/vendor/mujoco/mujoco.js',here).href),engineLoadMs=performance.now()-loadStart,started=performance.now(),rows=[],allControl=[],allStep=[],raw=[];
for(const seq of fixtures.sequences){
 let measured=false,controlTimes=[],stepTimes=[],warmupSteps=0;const wrap=(...args)=>{const start=performance.now();try{return controllerStep(...args)}finally{if(measured)controlTimes.push(performance.now()-start)}};
 const sim=await createSimulation(cfg,params,seq.case,{engine,xml,controllerStep:wrap,policyId:'v3_recovery'});let expected;
 try{
  let round=-1,rounds=1;const outcomes=[];
  for(;round<rounds;round++){
   sim.reset(seq.case,params,{policyId:'v3_recovery'});measured=round>=0;let s=sim.snapshot(),steps=0;
   while(!s.done){const start=performance.now();s=sim.step(1);if(measured)stepTimes.push(performance.now()-start);steps++;}
   const fingerprint=JSON.stringify([s.status,s.pos,s.quat,s.fuel,s.metrics.score]);if(expected===undefined)expected=fingerprint;else assert.equal(fingerprint,expected);
   if(!measured){warmupSteps=steps;rounds=Math.max(2,Math.ceil(500/steps))}else outcomes.push({round,status:s.status,steps,simulatedSeconds:s.t});
  }
  rows.push({id:seq.case.id,case:seq.case,warmupSteps,controller:sum(controlTimes),step20ms:sum(stepTimes),outcomes});raw.push({id:seq.case.id,controllerMs:controlTimes,stepMs:stepTimes});allControl.push(...controlTimes);allStep.push(...stepTimes);
 }finally{sim.dispose()}
}
// Exercise the predictor's full 6 s horizon explicitly, outside the flight benchmark.
const bench=JSON.parse(fs.readFileSync(new URL('landing-prediction-calibration.json',here))),predictorRows=[];
for(const row of bench.rows){const c=row.case,state={...c,R:[[1,0,0],[0,1,0],[0,0,1]],thrustScale:c.thrust_scale,disabledNozzles:c.disabled_nozzles??[],actualNozzleThrottle:row.initialValveThrottle};
 // Use the fixture quaternion, including the tilted calibration rows.
 const {qmat}=await import('./baseline_r6/controller.mjs');state.R=qmat(c.quat);const times=[];let last;
 for(let i=0;i<45;i++){const start=performance.now();last=predictVerticalStop(state,cfg);if(i>=5)times.push(performance.now()-start)}
 predictorRows.push({id:c.id,feasible:last.feasible,predictedSeconds:last.time,physicsIterations:Math.round(last.time/cfg.physics.dt),timing:sum(times),rawMs:times});
}
const sourceHashes=Object.fromEntries(['manifest.json','controller.py','controller.mjs','landing_predictor.py','landing-predictor.mjs','math.mjs','browser-runtime.mjs','allocator/conservative_allocator.py','allocator/conservative_allocator.mjs','allocator/canonical_allocation.mjs','world.xml','config.json','benchmark_timing.mjs'].map(n=>[n,hash(n)]));
const report={status:'measured_not_wcet',revision:manifest.revision,generatedAt:new Date().toISOString(),environment:{runtime:process.version,platform:os.platform(),release:os.release(),arch:os.arch(),cpu:os.cpus()[0]?.model,logicalCpus:os.cpus().length,loadAverage:os.loadavg()},method:{controlWindowMs:20,physicsStepMs:2,warmup:'One complete flight per case; excluded from measured percentiles.',measuredRounds:'max(2, ceil(500 / warmup windows)) per case',controllerScope:'Current-state feedback, bounded quaternion PD, exact 2ms fixed-pose braking preview when enabled, geometric allocation.',stepScope:'Controller plus 10 official MuJoCo WASM substeps, valves, gas, fan, contact criteria, snapshot.',percentiles:'nearest rank',casesMeaning:'Eight prior development cases; repeated timing rounds are not independent evaluation samples.'},engineLoadMs,wallSeconds:(performance.now()-started)/1000,aggregate:{controller:sum(allControl),step20ms:sum(allStep)},cases:rows,predictorFullHorizonChecks:predictorRows,raw,sourceHashes,limits:['Node measurements on this machine, not browser Worker scheduling or hard realtime.','Observed p99/max do not bound WCET or guarantee every 20ms deadline.','Ideal simulator state; excludes IMU noise, sensor fusion, communications and real actuator hardware.','Failures remain failures irrespective of computation speed.']};
fs.writeFileSync(new URL('timing-benchmark.json',here),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({aggregate:report.aggregate,predictor:predictorRows.map(({rawMs,...r})=>r),environment:report.environment},null,2));
