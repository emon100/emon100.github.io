// Timing branch coverage only: low-fuel fixed observations are not flight results.
import fs from 'node:fs';
import os from 'node:os';
import crypto from 'node:crypto';
import {controllerStep,createMemory} from './controller.mjs';
const here=new URL('.',import.meta.url),f=JSON.parse(fs.readFileSync(new URL('predictor-fixtures.json',here))),manifest=JSON.parse(fs.readFileSync(new URL('manifest.json',here))),rows=[];
for(const row of f.fixtures){
 const state={...row.state,pos:[0,0,10],omega:[0,0,0],contact:{wheelCount:0,bodyContact:false}},c={...state,task:'moving_recovery'},m=createMemory(c,f.config),times=[];let out;
 for(let i=0;i<45;i++){const start=performance.now();out=controllerStep(state,m,manifest.parameters,f.config);if(i>=5)times.push(performance.now()-start);}
 const ordered=[...times].sort((a,b)=>a-b),q=p=>ordered[Math.ceil(p*ordered.length)-1];rows.push({id:row.id,verticalMode:out.goal.verticalMode,forecastFeasible:out.goal.verticalStoppingForecast?.feasible??null,forecastSubsteps:Math.round((out.goal.verticalStoppingForecast?.time??0)/f.config.physics.dt),samples:times.length,warmup:5,p50Ms:q(.5),p95Ms:q(.95),p99Ms:q(.99),maxMs:ordered.at(-1),over20ms:times.filter(x=>x>20).length,rawMs:times});
}
const report={revision:manifest.revision,status:'measured_not_wcet',purpose:'Full controller timing in calibration and empty/failed-jet states, including 3000-step preview; no physical flight classification.',environment:{node:process.version,cpu:os.cpus()[0]?.model},maximumObservedMs:Math.max(...rows.map(r=>r.maxMs)),cases:rows,sourceHashes:Object.fromEntries(['controller.mjs','landing-predictor.mjs','math.mjs','allocator/conservative_allocator.mjs','allocator/canonical_allocation.mjs','predictor-fixtures.json'].map(n=>[n,crypto.createHash('sha256').update(fs.readFileSync(new URL(n,here))).digest('hex')])),limits:['No sensor, browser scheduling, physics or rendering in these stress timings.','Observed maxima are not WCET.']};fs.writeFileSync(new URL('controller-stress-timing.json',here),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify({...report,cases:rows.map(({rawMs,...r})=>r)},null,2));
