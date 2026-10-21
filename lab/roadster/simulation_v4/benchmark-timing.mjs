import fs from 'node:fs';
import os from 'node:os';
import {performance} from 'node:perf_hooks';
import {controllerStep} from './controller.mjs';
import {createSimulation,loadEngine} from './browser-runtime.mjs';
const manifest=JSON.parse(fs.readFileSync(new URL('./manifest.json',import.meta.url))),cfg=JSON.parse(fs.readFileSync(new URL('./config.json',import.meta.url))),xml=fs.readFileSync(new URL('./world.xml',import.meta.url),'utf8'),engine=await loadEngine(new URL('../vendor/mujoco/mujoco.js',import.meta.url).href),control=[],steps=[],cases=[];
if(manifest.status!=='training_complete')throw Error('Parameters must be frozen');
for(const {case:c} of manifest.demos){const sim=await createSimulation(cfg,manifest.final,c,{engine,xml,controllerStep:(...a)=>{const t=performance.now(),r=controllerStep(...a);control.push(performance.now()-t);return r;}});let f=sim.snapshot();while(!f.done){const t=performance.now();f=sim.step();steps.push(performance.now()-t);}cases.push({id:c.id,status:f.status,success:f.success});sim.dispose();}
const summarize=a=>{const s=[...a].sort((x,y)=>x-y);return {count:s.length,p95:s[Math.ceil(s.length*.95)-1],max:s.at(-1),over20ms:s.filter(x=>x>20).length}};
const result={revision:manifest.revision,controllerSha256:manifest.controllerSha256,timestamp:new Date().toISOString(),cpu:os.cpus()[0].model,engine:'MuJoCo 3.15.0 WASM',scope:'Six fixed demonstration cases in Node. Includes failure cases; excludes WebGL, communication and loading. Not a worst-case execution-time or hard real-time guarantee.',controller:summarize(control),controlAndPhysics:summarize(steps),cases};fs.writeFileSync(new URL('./timing-benchmark-rerun.json',import.meta.url),JSON.stringify(result,null,2));console.log(JSON.stringify(result,null,2));
