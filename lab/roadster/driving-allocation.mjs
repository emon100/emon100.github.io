import {RECOVERY_PARAMETERS} from './driving-recovery-parameters.mjs?v=20261022-fan-assist';
import {drivingNozzles} from './driving-thrusters.mjs?v=20261022-rocket10';
import {delivery,unitInertia} from './simulation_v4/controller.mjs';
import {wrenchMatrix,mv} from './simulation_v3/math.mjs';

// Project onto the individual nozzle bounds and shared flow limit; never rescale an infeasible command afterward.
function project(values,upper){
 let out=values.map((v,i)=>Math.max(0,Math.min(upper[i],v)));
 if(out.reduce((a,b)=>a+b,0)<=1)return out;
 let low=Math.min(...values.map((v,i)=>v-upper[i])),high=Math.max(...values);
 for(let k=0;k<36;k++){const mid=(low+high)/2,sum=values.reduce((s,v,i)=>s+Math.max(0,Math.min(upper[i],v-mid)),0);if(sum>1)low=mid;else high=mid;}
 return values.map((v,i)=>Math.max(0,Math.min(upper[i],v-high)));
}
export function createDrivingAllocator(s,cfg){
 const tuning=cfg.recoveryParameters??RECOVERY_PARAMETERS,nozzles=drivingNozzles(cfg),A=wrenchMatrix(nozzles),disabled=new Set(s.disabledNozzles??[]),pressure=delivery(s.fuel,cfg).pressureScale;
 const limits=nozzles.map(n=>disabled.has(n.id)?0:n.maxForce*pressure*(s.thrustScale??1)),cap=cfg.actuators.total_force_max*pressure*(s.thrustScale??1),J=unitInertia(cfg).map(v=>v*s.mass);
 if(![...limits,cap,s.mass,...J,...s.R.flat()].every(Number.isFinite)||limits.some(v=>v<0)||cap<0||s.mass<=0||J.some(v=>v<=0))throw Error('Invalid driving allocation state');
 const lift=A[0].map((_,j)=>s.R[2].reduce((sum,v,i)=>sum+v*A[i][j],0)),rows=[A[3],A[4],lift,A[5]],weights=[tuning.tiltWeight/J[0],tuning.tiltWeight/J[1],1/s.mass,tuning.yawWeight/J[2]];
 const scale=Math.max(cap,1),upper=limits.map(v=>v/scale),B=rows.map((row,i)=>row.map(v=>v*weights[i]*scale)),ridge=1e-6,gasCost=tuning.gasCost;
 const lipschitz=B.flat().reduce((sum,v)=>sum+v*v,ridge);
 function allocate(torque,forceZ){
  const requested=[torque[0],torque[1],forceZ,torque[2]];if(!requested.every(Number.isFinite))throw Error('Invalid driving wrench');
  const target=requested.map((v,i)=>v*weights[i]);let x=Array(nozzles.length).fill(0),y=[...x],momentum=1,iterations=0;
  if(cap>0)for(;iterations<240;iterations++){
   const error=B.map((row,i)=>row.reduce((sum,v,j)=>sum+v*y[j],0)-target[i]);
   const next=project(y.map((v,j)=>v-(B.reduce((sum,row,i)=>sum+row[j]*error[i],0)+ridge*v+gasCost)/lipschitz),upper);
   const change=Math.max(...next.map((v,j)=>Math.abs(v-x[j]))),nextMomentum=(1+Math.sqrt(1+4*momentum*momentum))/2;
   y=next.map((v,j)=>v+(momentum-1)/nextMomentum*(v-x[j]));x=next;momentum=nextMomentum;
   if(change<1e-9&&iterations>30)break;
  }
  const forces=x.map(v=>v*scale),achievedWrenchBody=mv(A,forces),forceWorld=mv(s.R,achievedWrenchBody.slice(0,3));
  if(forces.some((v,i)=>!Number.isFinite(v)||v<0||v>limits[i]+1e-6)||forces.reduce((a,b)=>a+b,0)>cap+1e-6)throw Error('Driving allocation exceeded actuator limits');
  const residual=rows.map((row,i)=>row.reduce((sum,v,j)=>sum+v*forces[j],0)-requested[i]);
  return {forces,nozzleThrottle:forces.map((v,i)=>limits[i]>0?v/limits[i]:0),achievedWrenchBody,forceWorld,controlledResidual:residual,weightedResidual:Math.hypot(...residual.map((v,i)=>v*weights[i])),iterations,allocator:'bounded_projected_least_squares',globallyOptimal:false,priority:'weighted_tilt_lift_yaw_with_gas_cost',horizontalForceControlled:false};
 }
 return {nozzles,A,limits,cap,lift,allocate};
}
