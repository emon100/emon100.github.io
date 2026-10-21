import {t as tr} from './i18n.mjs';
import {createSimulation,loadEngine} from './simulation_v2/browser-runtime.mjs';
import {controllerStep} from './simulation_v2/controller.mjs';
let engine,sim,activeRun,controlTimes=[],stepTimes=[],windowMs=20;
function summary(values){if(!values.length)return {count:0,p95:0,max:0,overBudget:0};const ordered=[...values].sort((a,b)=>a-b);return {count:values.length,p95:ordered[Math.ceil(ordered.length*.95)-1],max:ordered.at(-1),overBudget:values.filter(x=>x>windowMs).length};}
self.onmessage=async({data:m})=>{const send=data=>self.postMessage({...data,runId:m.runId,sentAt:m.sentAt});try{
 if(m.type==='start'){
  activeRun=m.runId;engine??=await loadEngine(new URL('./vendor/mujoco/mujoco.js',import.meta.url).href);sim?.dispose();controlTimes=[];stepTimes=[];windowMs=m.config.physics.control_dt*1000;
  const repaired=m.policy==='v3_recovery';if(repaired&&m.case.task!=='moving_recovery')throw Error(tr("倒置修复版仅用于任意姿态回正","The inverted-recovery fix is only for arbitrary-attitude recovery"));
  const runtime=repaired?await import('./simulation_v3/browser-runtime.mjs'):{createSimulation};
  const control=repaired?await import('./simulation_v3/controller.mjs'):{controllerStep};
  if(repaired&&(!m.settings||Object.keys(m.settings).length!==Object.keys(control.SETTINGS).length||Object.entries(control.SETTINGS).some(([k,v])=>m.settings[k]!==v)))throw Error(tr("控制说明与实际算法版本不一致，请刷新页面。","The control description and algorithm version differ. Refresh the page."));
  const measuredController=(...args)=>{const start=performance.now();const result=control.controllerStep(...args);controlTimes.push(performance.now()-start);return result};
  const next=await runtime.createSimulation(m.config,m.params,m.case,{engine,policyId:m.policy,controllerStep:measuredController,...(repaired?{createMemory:control.createMemory}:{})});if(activeRun!==m.runId){next.dispose();return}sim=next;send({type:'started',frame:sim.snapshot()});
 }else if(m.runId!==activeRun)return;
 else if(m.type==='step'){
  const frames=[];for(let i=0;i<m.count;i++){const start=performance.now(),frame=sim.step();stepTimes.push(performance.now()-start);frames.push(frame);if(frame.done)break;}
  if(frames.length)send({type:'frames',frames,timing:{controller:summary(controlTimes),step:summary(stepTimes),windowMs}});
 }else if(m.type==='impulse'){sim.applyImpulse(m.impulse);send({type:'impulse',impulse:m.impulse,t:sim.snapshot().t});}
 }catch(e){send({type:'error',message:e.message})}};
