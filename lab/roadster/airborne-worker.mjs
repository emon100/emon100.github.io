import {createSimulation,loadEngine} from './simulation_v4/browser-runtime.mjs';
import {controllerStep,createMemory} from './simulation_v4/controller.mjs';
let engine,sim,activeRun,controlTimes=[],stepTimes=[];
const summary=a=>{const s=[...a].sort((x,y)=>x-y);return {count:s.length,p95:s[Math.max(0,Math.ceil(s.length*.95)-1)]??0,max:s.at(-1)??0,overBudget:s.filter(x=>x>20).length}};
const timing=()=>({controller:summary(controlTimes),step:summary(stepTimes),windowMs:20});
self.onmessage=async({data:m})=>{const send=data=>self.postMessage({...data,runId:m.runId,sentAt:m.sentAt});try{
 if(m.type==='start'||m.type==='batch'){
  activeRun=m.runId;engine??=await loadEngine(new URL('./vendor/mujoco/mujoco.js',import.meta.url).href);sim?.dispose();controlTimes=[];stepTimes=[];
  sim=await createSimulation(m.config,m.params,m.case,{engine,policyId:'unified',createMemory,controllerStep:(...a)=>{const t=performance.now(),r=controllerStep(...a);controlTimes.push(performance.now()-t);return r}});
  if(m.type==='start'){send({type:'started',frame:sim.snapshot()});return;}
  const frames=[sim.snapshot()];while(!frames.at(-1).done){const t=performance.now();frames.push(sim.step());stepTimes.push(performance.now()-t);}
  send({type:'batch',frames,timing:timing()});sim.dispose();sim=null;
 }else if(m.runId!==activeRun)return;
 else if(m.type==='step'){const frames=[];for(let i=0;i<Math.min(4,m.count);i++){const t=performance.now(),f=sim.step();stepTimes.push(performance.now()-t);frames.push(f);if(f.done)break;}send({type:'frames',frames,timing:timing()});}
 else if(m.type==='impulse'){sim.applyImpulse(m.impulse);send({type:'impulse',t:sim.snapshot().t,impulse:m.impulse});}
 }catch(e){send({type:'error',message:e.message});}};
