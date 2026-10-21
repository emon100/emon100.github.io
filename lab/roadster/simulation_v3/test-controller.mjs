import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import {controllerStep,createMemory,SETTINGS} from './controller.mjs';
const here=new URL('.',import.meta.url),f=JSON.parse(fs.readFileSync(new URL('browser-fixtures.json',here))),manifest=JSON.parse(fs.readFileSync(new URL('manifest.json',here)));
assert.deepEqual(SETTINGS,manifest.settings);assert.deepEqual(f.settings,SETTINGS);
const names={initial_xy:'initialXY',goal_q:'goalQuat',previous_roll:'previousRoll',turn_angle:'turnAngle',initial_roll:'initialRoll',recovery_height:'recoveryHeight',initial_height:'initialHeight',peak_height:'peakHeight',energy_ready:'energyReady',turn_complete:'turnComplete',was_airborne:'wasAirborne',heading_goal:'headingGoal'};
const memory=m=>Object.fromEntries(Object.entries(m).map(([k,v])=>[names[k]??k,v]));
const maxima={},worst={},issues=[];
function compare(a,b,path,id){
 if(typeof b==='number'){const delta=Math.abs(a-b);if(!Number.isFinite(a))issues.push({id,path,a,b});if(delta>(maxima[path]??0)){maxima[path]=delta;worst[path]=id;}return;}
 if(Array.isArray(b)){assert.ok(Array.isArray(a),`${id}/${path} missing array`);assert.equal(a.length,b.length);b.forEach((v,i)=>compare(a[i],v,path,id));return;}
 if(b&&typeof b==='object'){for(const [k,v] of Object.entries(b))compare(a?.[k],v,`${path}.${k}`,id);return;}
 if(a!==b)issues.push({id,path,a,b});
}
let count=0,forecasts=0,maximumHorizontal=0;
for(const seq of f.sequences){
 let sequential=createMemory(seq.case,f.config);
 compare(sequential,memory(seq.steps[0].memoryBefore),'initialMemory',seq.case.id);
 for(let i=0;i<seq.steps.length;i++){
  const step=seq.steps[i],id=`${seq.case.id}/${i}`,state={...step.state,contact:step.contact},m={...memory(step.memoryBefore),task:seq.case.task},out=controllerStep(state,m,f.parameters,f.config),chain=controllerStep(state,sequential,f.parameters,f.config);sequential=chain.memory;
  for(const [key,expected] of [['forceWorld',step.forceWorld],['torqueBody',step.torqueBody],['fanCommand',step.fanCommand],['goal',step.goal],['error',step.error],['allocation',step.allocation],['memory',memory(step.memoryAfter)]])compare(out[key],expected,key,id);
  compare(chain.memory,memory(step.memoryAfter),'sequentialMemory',id);compare(chain.nozzleThrottle,out.nozzleThrottle,'sequentialThrottle',id);
  compare(out.nozzleThrottle,step.nozzleThrottle,'throttle',id);
  maximumHorizontal=Math.max(maximumHorizontal,Math.hypot(...out.desiredForceWorld.slice(0,2)));if(out.goal.verticalStoppingForecast)forecasts++;count++;
 }
}
const tolerances={forceWorld:1e-5,torqueBody:1e-5,fanCommand:1e-12,throttle:1e-7,sequentialThrottle:1e-7};
for(const [path,value] of Object.entries(maxima)){let tolerance=tolerances[path]??(path.startsWith('allocation.')?1e-4:1e-5);if(path==='allocation.feasibilityCalls'||path==='allocation.solverIterations')tolerance=0;if(value>tolerance)issues.push({path,value,tolerance,id:worst[path]});}
assert.equal(maximumHorizontal,0);
const hashes=Object.fromEntries(['controller.mjs','landing-predictor.mjs','math.mjs','allocator/conservative_allocator.mjs','allocator/canonical_allocation.mjs','browser-fixtures.json'].map(n=>[n,crypto.createHash('sha256').update(fs.readFileSync(new URL(n,here))).digest('hex')]));
const report={revision:manifest.revision,status:issues.length?'differences_require_review':'passed',count,forecasts,maximumHorizontalForceTarget:maximumHorizontal,maxima,worst,issues,sourceHashes:hashes,nativeControllerSha:f.controllerSha256};fs.writeFileSync(new URL('controller-parity.json',here),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({status:report.status,count,forecasts,issues,maxima},null,2));assert.equal(issues.length,0);
