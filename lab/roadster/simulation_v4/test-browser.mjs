import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import assert from 'node:assert/strict';
import {controllerStep,createMemory} from './controller.mjs';
import {createSimulation,loadEngine} from './browser-runtime.mjs';
const data=JSON.parse(gunzipSync(fs.readFileSync(new URL('./browser-fixtures.json.gz',import.meta.url)))),cfg=data.config,issues=[],checks=[],maximum={};
const compare=(key,a,b,tolerance,id)=>{const x=Array.isArray(a)?a.flat(Infinity):[a],y=Array.isArray(b)?b.flat(Infinity):[b];assert.equal(x.length,y.length);const delta=Math.max(...x.map((v,i)=>Math.abs(v-y[i])));maximum[key]=Math.max(maximum[key]??0,delta);if(!(delta<=tolerance))issues.push({key,id,delta,tolerance});};
const manifest=JSON.parse(fs.readFileSync(new URL('./manifest.json',import.meta.url)));const campaign={status:manifest.status,selectedParameters:manifest.final};if(campaign.status==='training_complete')assert(data.rows.some(r=>r.label==='final'&&JSON.stringify(r.parameters)===JSON.stringify(campaign.selectedParameters)),'Final parameter fixtures missing');
const engine=await loadEngine(new URL('../vendor/mujoco/mujoco.js',import.meta.url).href),xml=fs.readFileSync(new URL('./world.xml',import.meta.url),'utf8');
for(const row of data.rows){const c=row.result.case,id=c.id+'/'+row.label;let memory=createMemory(c,cfg);
 for(let i=0;i<row.steps.length;i++){const s=row.steps[i],r=controllerStep({...s.state,contact:s.contact},memory,row.parameters,cfg);memory=r.memory;compare('force',r.desiredForceWorld,s.force,.002,id+'/'+i);compare('torque',r.desiredTorqueBody,s.torque,.002,id+'/'+i);compare('pathRemaining',r.goal.pathRemaining,s.goal.pathRemaining,1e-7,id+'/'+i);compare('spinMeasured',r.goal.spinMeasured,s.goal.spinMeasured,1e-7,id+'/'+i);assert.deepEqual(r.desiredForceWorld.slice(0,2),[0,0]);}
 const sim=await createSimulation(cfg,row.parameters,c,{engine,xml});let f=sim.snapshot(),i=0;while(!f.done){f=sim.step();i++;const target=row.result.frames[i];if(!target){issues.push({id,key:'frame_count',actual:i});break;}for(const key of ['pos','quat','vel','omega'])compare('trajectory.'+key,f[key],target[key],3e-5,id+'/'+i);compare('trajectory.fuel',f.fuel,target.fuel,3e-5,id+'/'+i);assert.deepEqual(f.allocation.requestedForceWorld.slice(0,2),[0,0]);}
 if(f.status!==row.result.status||f.success!==row.result.success||i!==row.result.frames.length-1)issues.push({id,key:'terminal',actual:f.status,expected:row.result.status,steps:i,expectedSteps:row.result.frames.length-1});checks.push({id,status:f.status,success:f.success,windows:i});sim.dispose();console.log(id,f.status,i);
}
const report={status:issues.length?'differences_require_review':'passed',checks,maxDifferences:maximum,issues};fs.writeFileSync(new URL('./browser-parity-rerun.json',import.meta.url),JSON.stringify(report,null,2));console.log(JSON.stringify({...report,issues:issues.slice(0,8)},null,2));assert.equal(issues.length,0);
