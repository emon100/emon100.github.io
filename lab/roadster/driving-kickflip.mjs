import {controllerStep} from './simulation_v4/controller.mjs';
import {createDrivingAllocator} from './driving-allocation.mjs?v=20261022-fan-assist';
import {recoveryStep} from './driving-recovery.mjs?v=20261022-fan-assist';

export function drivingKickflipStep(s,m,params,cfg){
 if(m.spinSatisfied&&m.requiredSpin>0)return recoveryStep(s,m,params,cfg);
 // shortcut: The frozen stunt planner still estimates legacy capacity; retune its path if ten-nozzle stunt checks fail.
 const valves=Array(12).fill(0);for(let i=0;i<4;i++)valves[i*2]=s.nozzleThrottle?.[i]??0;
 const result=controllerStep({...s,actualNozzleThrottle:valves},m,params,cfg);
 const allocation=createDrivingAllocator(s,cfg).allocate(result.desiredTorqueBody,result.desiredForceWorld[2]);
 result.nozzleThrottle=allocation.nozzleThrottle;result.allocation=allocation;
 result.goal={...result.goal,controllerFamily:'driving_rocket10_kickflip',horizontalForceControlled:false,horizontalForceTarget:null,allocatedForceWorld:allocation.forceWorld};
 return result;
}
