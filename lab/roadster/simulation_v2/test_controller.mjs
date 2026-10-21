import fs from 'node:fs';
import assert from 'node:assert/strict';
import {controllerStep,nozzleLayout,allocateFixedNozzles} from './controller.mjs';
const {config,vectors}=JSON.parse(fs.readFileSync(new URL('./controller-parity.json',import.meta.url)));
let maximum=0;
function close(a,b,path=''){
 if(typeof b==='number'){assert.ok(Number.isFinite(a),path);const error=Math.abs(a-b);maximum=Math.max(maximum,error);assert.ok(error<2e-5,`${path}: ${a} != ${b}`)}
 else if(Array.isArray(b)){assert.equal(a.length,b.length,path);b.forEach((v,i)=>close(a[i],v,`${path}[${i}]`))}
 else if(b&&typeof b==='object'){for(const k of Object.keys(b))close(a[k],b[k],`${path}.${k}`)}
 else assert.equal(a,b,path);
}
for(const [i,v] of vectors.entries())close(controllerStep(v.state,v.memory,v.params,config),v.expected,`vector${i}`);
const nozzles=nozzleLayout(config),limits=nozzles.map(n=>n.maxForce);
for(const v of vectors){const f=allocateFixedNozzles(v.expected.requestedWrenchBody,limits,config);assert.ok(f.every((n,i)=>n>=0&&n<=limits[i]));assert.ok(f.reduce((a,b)=>a+b,0)<=config.actuators.total_force_max+1e-6)}
console.log(JSON.stringify({vectors:vectors.length,maximumAbsoluteError:maximum,nonnegativeBounds:'passed'}));
