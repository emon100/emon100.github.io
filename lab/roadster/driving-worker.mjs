import {loadEngine} from './simulation_v4/browser-runtime.mjs?v=20261019';
import {createDrivingSimulation} from './driving-core.mjs?v=20261022-render-refill';
let sim,input={},kick=false,timer,last=0,accumulator=0,failed=false;
async function tick(){
 const start=performance.now();
 try{accumulator=Math.min(.5,accumulator+(start-last)/1000);last=start;let frame;while(accumulator>=.02){frame=await sim.step({...input,kick});kick=false;accumulator-=.02;}if(frame)self.postMessage({type:'frame',frame});}
 catch(e){failed=true;input={};kick=false;self.postMessage({type:'error',message:e.message});return;}
 timer=setTimeout(tick,Math.max(0,20-(performance.now()-start)));
}
self.onmessage=async({data:m})=>{try{
 if(m.type==='init'){sim=await createDrivingSimulation(m.config,m.params,{engine:await loadEngine(undefined,progress=>self.postMessage({type:'loading',progress})),xml:m.xml,...m.settings});self.postMessage({type:'ready',frame:sim.snapshot()});last=performance.now();timer=setTimeout(tick,20);}
 else if(m.type==='input')input=m.value;
 else if(m.type==='settings'&&sim)self.postMessage({type:'frame',frame:sim.setOptions(m.value)});
 else if(m.type==='kick')kick=true;
 else if(m.type==='recenter'&&sim){self.postMessage({type:'frame',frame:sim.recenter()});if(failed){failed=false;last=performance.now();accumulator=0;timer=setTimeout(tick,20);}}
 }catch(e){self.postMessage({type:'error',message:e.message});}};
