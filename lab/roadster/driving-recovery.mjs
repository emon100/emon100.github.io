import {controllerStep as flightStep,createMemory,unitInertia,nozzleLayout,delivery} from './simulation_v4/controller.mjs';
import {clamp,norm,mul,sub,cross,tmv,mv,rotationError,wrenchMatrix} from './simulation_v3/math.mjs';
import {createAllocator} from './simulation_v3/allocator/conservative_allocator.mjs';

export function recoveryStep(s,m,params,cfg){
 // A collision invalidates the old waypoint path; recovery has no prescribed spin to preserve.
 const headingTarget=createMemory({...s,goal:{turns:0}},cfg).target;
 const fresh=createMemory({...s,vel:[0,0,s.vel[2]],goal:{turns:0}},cfg);
 Object.assign(m,{segments:fresh.segments,pathFloors:fresh.pathFloors,target:fresh.target,index:0,progress:0,completedArc:0,totalArc:fresh.totalArc,released:false});
 // shortcut: v4 predicts the flat floor; close ramp exits need terrain-aware stopping prediction.
 const result=flightStep(s,m,params,cfg),J=mul(unitInertia(cfg),s.mass),tiltRate=Math.hypot(s.omega[0],s.omega[1]),tilt=Math.acos(clamp(s.R[2][2],-1,1));
 m.braking=tiltRate>6||Boolean(m.braking&&tiltRate>2);
 const up=s.R.map(r=>r[2]),axis=cross(up,[0,0,1]),length=norm(axis);
 const error=length>1e-6?mul(tmv(s.R,axis),tilt/length):[tilt,0,0];
 const target=mul(error,Math.min(4,4*tilt)/Math.max(tilt,1e-9));
 if(m.braking)target.fill(0);
 // Heading can wait while the wheels are being brought underneath the car.
 const yawError=rotationError(s.quat,headingTarget)[2];
 // Either travel heading is valid; a smooth target avoids reversing a saturated yaw command at 90 degrees.
 if(!m.braking&&tilt<.35)target[2]=Math.sin(2*yawError);
 const gyro=cross(s.omega,s.omega.map((v,i)=>v*J[i]));
 const torque=s.omega.map((v,i)=>J[i]*8*(target[i]-v)+(m.braking?0:gyro[i]));
 const nozzles=nozzleLayout(cfg),A=wrenchMatrix(nozzles),pressure=delivery(s.fuel,cfg).pressureScale,disabled=new Set(s.disabledNozzles??[]),limits=nozzles.map(n=>disabled.has(n.id)?0:n.maxForce*pressure*(s.thrustScale??1));
 const parts=[[0,0,0,torque[0],torque[1],0],[...tmv(s.R,result.desiredForceWorld),0,0,0],[0,0,0,0,0,torque[2]]];
 const allocated=createAllocator(A)(parts,limits,cfg.actuators.total_force_max*pressure*(s.thrustScale??1));
 result.nozzleThrottle=allocated.forces.map((f,i)=>limits[i]>0?f/limits[i]:0);
 result.desiredTorqueBody=torque;
 const requested=[...tmv(s.R,result.desiredForceWorld),...torque],achieved=mv(A,allocated.forces),residual=sub(requested,achieved);
 result.requestedWrenchBody=requested;
 result.allocation={requestedWrenchBody:requested,achievedWrenchBody:achieved,residual,normalizedResidual:norm(residual)/Math.max(norm(requested),1),momentScale:allocated.tiltScale,forceScale:allocated.forceScale,yawScale:allocated.yawScale,allocator:'conservative_analytic_priority',globallyOptimal:false,feasibilityCalls:allocated.feasibilityCalls,solverIterations:allocated.iterations,priority:'tilt_then_lift_then_yaw'};
 m.goal=result.goal={...result.goal,headingTargetQuat:headingTarget,brakingSpin:m.braking,controllerFamily:'driving_recovery'};
 return result;
}
