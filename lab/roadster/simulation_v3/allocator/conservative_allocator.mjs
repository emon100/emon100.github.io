import {canonicalize} from './canonical_allocation.mjs';
export const BISECTION_STEPS=40;
export function createAllocator(matrix){
  return function allocate(parts,limits,cap){
    if(parts.length!==3||parts.some(x=>x.length!==6)||limits.length!==12||matrix.length!==6||matrix.some(x=>x.length!==12)||![...parts.flat(),...matrix.flat(),...limits,cap].every(Number.isFinite)||limits.some(x=>x<0)||cap<0)throw new Error('Invalid allocation inputs');
    canonicalize(Array(6).fill(0),limits,matrix);
    let scales=[0,0,0],best=Array(12).fill(0),calls=0;const iterations=[];
    const target=sc=>Array.from({length:6},(_,i)=>sc.reduce((sum,v,j)=>sum+v*parts[j][i],0));
    const feasible=sc=>{calls++;try{const f=canonicalize(target(sc),limits,matrix);return f.reduce((s,v)=>s+v,0)<=cap?f:null}catch(error){if(['LP wrench outside vertical feasible interval','Canonical force outside nozzle bounds','Canonical wrench changed'].includes(error.message))return null;throw error}};
    for(let priority=0;priority<3;priority++){
      let candidate=[...scales];candidate[priority]=1;let forces=feasible(candidate);
      if(forces){scales=candidate;best=forces;iterations.push(0);continue}
      let low=0,high=1;
      for(let i=0;i<BISECTION_STEPS;i++){candidate=[...scales];candidate[priority]=(low+high)/2;forces=feasible(candidate);if(forces){low=candidate[priority];best=forces}else high=candidate[priority]}
      scales[priority]=low;iterations.push(BISECTION_STEPS);
    }
    const achieved=matrix.map(row=>row.reduce((s,v,j)=>s+v*best[j],0)),requested=target(scales),total=best.reduce((s,v)=>s+v,0);
    if(Math.max(...achieved.map((v,i)=>Math.abs(v-requested[i])))>1e-5||best.some((v,i)=>v<0||v>limits[i])||total>cap)throw new Error('Analytic allocation certificate failed');
    return {forces:best,tiltScale:scales[0],forceScale:scales[1],yawScale:scales[2],achievedWrench:achieved,totalForce:total,iterations:[...iterations,0],feasibilityCalls:calls,method:'conservative_analytic_priority',globallyOptimal:false,solverRetries:0};
  };
}
