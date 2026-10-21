import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import {createSimulation,loadEngine} from './browser-runtime.mjs';
const here=new URL('.',import.meta.url),f=JSON.parse(fs.readFileSync(new URL('browser-fixtures.json',here))),manifest=JSON.parse(fs.readFileSync(new URL('manifest.json',here)));
const engine=await loadEngine(process.env.MUJOCO_JS??new URL('../site/dist/vendor/mujoco/mujoco.js',here).href),xml=fs.readFileSync(new URL('world.xml',here),'utf8');
assert.equal(xml,fs.readFileSync(new URL('baseline_r6/world.xml',here),'utf8'));
const original=fs.readFileSync(new URL('../simulation_v2/browser-runtime.mjs',here),'utf8'),runtime=fs.readFileSync(new URL('browser-runtime.mjs',here),'utf8');
assert.equal(runtime.replace(',actualNozzleThrottle:[...throttle],actualNozzleForces:[...forces]',''),original,'Only actuator observations differ in the runtime; physics and landing tests remain unchanged');
const checks=[],issues=[],max={};let windows=0;
const observe=(key,a,b,tolerance,id)=>{const aa=Array.isArray(a)?a.flat(Infinity):[a],bb=Array.isArray(b)?b.flat(Infinity):[b];assert.equal(aa.length,bb.length);const delta=Math.max(...aa.map((v,i)=>Math.abs(v-bb[i])));max[key]=Math.max(max[key]??0,delta);if(!(delta<=tolerance))issues.push({id,key,delta,tolerance});return delta;};
for(const seq of f.sequences){
 const sim=await createSimulation(f.config,f.parameters,seq.case,{engine,xml,policyId:'v3_recovery'});let s=sim.snapshot(),index=0;const local={};
 try{
  assert.deepEqual(s.allocation.requestedForceWorld,[0,0,0]);
  while(!s.done){
   s=sim.step(1);windows++;index++;assert.deepEqual(s.allocation.requestedForceWorld.slice(0,2),[0,0]);
   const next=seq.steps[index];
   if(next){
    for(const key of ['pos','quat','vel','omega','wheelRates'])local[key]=Math.max(local[key]??0,observe(key,s[key],next.state[key],2e-5,`${seq.case.id}/${index}`));
    observe('fuel',s.fuel,next.state.fuel,2e-5,seq.case.id);observe('actualNozzleThrottle',s.nozzleThrottle,next.state.actualNozzleThrottle,1e-5,seq.case.id);observe('actualNozzleForces',s.nozzleForces,next.state.actualNozzleForces,2e-3,seq.case.id);
    if(s.contact.wheelCount!==next.contact.wheelCount)issues.push({id:seq.case.id,index,key:'wheelCount',js:s.contact.wheelCount,native:next.contact.wheelCount});
   }
  }
  if(index!==seq.steps.length||s.status!==seq.status||s.success!==seq.success)issues.push({id:seq.case.id,index,expectedSteps:seq.steps.length,status:s.status,expectedStatus:seq.status,success:s.success});
  for(const key of ['pos','quat','vel','omega','wheelRates'])observe(`final.${key}`,s[key],seq.finalState[key],2e-5,seq.case.id);
  for(const key of ['score','fuelUsed','fuelLeft','duration','turnProgress','turnAngle','peakHeight','maxContactClosingSpeed','maxTotalContactForce','stableSeconds','finalTilt','finalLongitudinalSpeed','finalLateralSpeed','finalVerticalSpeed','peakJetHorizontalForce','finalAngularSpeed','horizontalDisplacement','jetHorizontalImpulse','fanHorizontalImpulse'])observe(`metrics.${key}`,s.metrics[key],seq.metrics[key],key==='maxTotalContactForce'?1e-1:1e-4,seq.case.id);
  checks.push({id:seq.case.id,status:s.status,success:s.success,windows:index,maximumTrajectoryDifferences:local,metrics:s.metrics});
 }finally{sim.dispose();}
}
const sourceHashes=Object.fromEntries(['controller.py','controller.mjs','landing_predictor.py','landing-predictor.mjs','math.mjs','browser-runtime.mjs','world.xml','config.json','browser-fixtures.json','allocator/conservative_allocator.py','allocator/conservative_allocator.mjs'].map(n=>[n,crypto.createHash('sha256').update(fs.readFileSync(new URL(n,here))).digest('hex')]));
const report={revision:manifest.revision,status:issues.length?'differences_require_review':'passed',meaning:'Native and official MuJoCo 3.15.0 WASM run the same cases independently; successful landing is not assumed.',timestamp:new Date().toISOString(),engineVersion:engine.mj_versionString(),windows,checks,maxDifferences:max,issues,sourceHashes};fs.writeFileSync(new URL('browser-verification.json',here),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({...report,checks:checks.map(({metrics,...c})=>c),issues:issues.slice(0,20)},null,2));assert.equal(issues.length,0);
