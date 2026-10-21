/* Port of frozen controller.py. Engineered recovery repair, not additional training.
   Pose/velocity come from the current plant state. No pose integration happens here. */
export * from './baseline_r6/controller.mjs';
import {PARAMETER_NAMES,qmat,unitInertia,nozzleLayout,delivery,fanModel,allocateFixedNozzles,geometryMinimumCG,controllerStep as baselineStep,createControllerMemory as baselineMemory} from './baseline_r6/controller.mjs';
import {clamp,dot,norm,add,sub,mul,cross,mv,tmv,mm,axisQ,wrap,rotationError,wrenchMatrix} from './math.mjs';
import {predictVerticalStop} from './landing-predictor.mjs';
import {createAllocator} from './allocator/conservative_allocator.mjs';
export const SETTINGS=Object.freeze({rate_cap:4,capacity_fraction:.55,acceleration_cap:15,brake_cap:12,early_rate:.15,landing_reserve:.25,boost_speed_cap:10,support_ratio:1.4,descent_speed:4.5,maximum_descent_speed:8,descent_acceleration_cap:4,vertical_gain:5.5,touchdown_speed:.12,ground_margin:.2,heading_slew:.6,gyro_feedforward:true,prediction_overlay:true});
export function createControllerMemory(task,initialCase,cfg){const m=baselineMemory(task,initialCase,cfg);return {...m,released:false,headingGoal:m.yaw};}
export const createMemory=(initialCase,cfg)=>createControllerMemory(initialCase.task,initialCase,cfg);

export function localCapacity(s,cfg,axis){
  const nozzles=nozzleLayout(cfg),A=wrenchMatrix(nozzles),pressure=delivery(s.fuel,cfg).pressureScale,disabled=new Set(s.disabledNozzles??[]),limits=nozzles.map(n=>disabled.has(n.id)?0:n.maxForce*pressure*(s.thrustScale??1)),J=mul(unitInertia(cfg),s.mass),bodyAxis=tmv(s.R,axis),result={};
  for(const [name,sign] of [['accelerate',1],['brake',-1]]){const target=J.map((v,i)=>sign*v*bodyAxis[i]*1e6),force=allocateFixedNozzles([0,0,0,...target],limits,cfg),alpha=mv(s.R,mv(A,force).slice(3).map((v,i)=>v/J[i]));result[name]=Math.max(0,dot(alpha,mul(axis,sign)));}
  const verticalForces=allocateFixedNozzles([...tmv(s.R,[0,0,1e6]),0,0,0],limits,cfg),vertical=mv(A,verticalForces);result.vertical=Math.max(0,mv(s.R,vertical.slice(0,3))[2]);result.verticalFlowPerLift=verticalForces.reduce((sum,x)=>sum+x,0)/Math.max(result.vertical,1e-9);return result;
}
export function rotationTime(angle,speed,cap,accel,brake){
  if(accel<1e-6||brake<1e-6)return 99;
  let delay=0;if(speed<0){delay=-speed/accel;angle+=speed*speed/(2*accel);speed=0;}
  const stopping=speed*speed/(2*brake);if(stopping>angle+1e-12)return delay+speed/brake+rotationTime(stopping-angle,0,cap,accel,brake);
  if(speed>cap){delay+=(speed-cap)/brake;angle-=(speed*speed-cap*cap)/(2*brake);speed=cap;}
  const aa=Math.max(0,(cap*cap-speed*speed)/(2*accel)),bb=cap*cap/(2*brake);
  if(aa+bb<=angle)return delay+(cap-speed)/accel+(angle-aa-bb)/Math.max(cap,.01)+cap/brake;
  const peak=Math.sqrt(Math.max(0,(2*angle*accel*brake+brake*speed*speed)/(accel+brake)));return delay+Math.max(0,peak-speed)/accel+peak/brake;
}
export function landingGasBudget(s,cfg){
  const dry=s.mass-s.fuel,nozzles=nozzleLayout(cfg),disabled=new Set(s.disabledNozzles??[]),limits=nozzles.map(n=>disabled.has(n.id)?0:n.maxForce*(s.thrustScale??1)),A=wrenchMatrix(nozzles),force0=mv(A,allocateFixedNozzles([0,0,1e6,0,0,0],limits,cfg))[2],gravity=cfg.physics.gravity;let lo=0,hi=cfg.fuel.initial_kg;
  for(let i=0;i<32;i++){const mid=(lo+hi)/2,pressure=delivery(mid,cfg).pressureScale;if(force0*pressure>=1.25*(dry+mid)*gravity)hi=mid;else lo=mid;}
  const {pressureScale:pressure,effectiveIsp:isp}=delivery(s.fuel,cfg),reachable=force0>=1.25*(dry+cfg.fuel.initial_kg)*gravity,usable=reachable?Math.max(0,s.fuel-hi):0;
  return {forceMarginReachable:reachable,uprightForceAtCurrentPressure:force0*pressure,uprightLiftWeightRatio:force0*pressure/(s.mass*gravity),minimumFuelForUprightForceMargin:reachable?hi:null,availableGasAboveMargin:usable,equivalentHoverTime:usable*isp/Math.max(s.mass,1),marginWeightRatio:1.25,notice:'Approximate mass/pressure budget; not a guarantee of attitude-dependent landing authority.'};
}
export function controllerStep(state,memory,params,cfg){
  if(memory.task!=='moving_recovery')return baselineStep(state,memory,params,cfg);
  const c=SETTINGS,p=Array.isArray(params)?Object.fromEntries(PARAMETER_NAMES.map((n,i)=>[n,params[i]])):params,R=state.R??qmat(state.quat),s={...state,R},J=mul(unitInertia(cfg),s.mass),tilt=Math.acos(clamp(R[2][2],-1,1)),rate=norm(s.omega),dt=cfg.physics.control_dt,gravity=cfg.physics.gravity,prior=structuredClone(memory);
  if(prior.turnComplete&&norm(s.vel.slice(0,2))>.5){const heading=Math.atan2(s.vel[1],s.vel[0]),choices=[heading,wrap(heading+Math.PI)],target=Math.abs(wrap(choices[0]-prior.headingGoal))<=Math.abs(wrap(choices[1]-prior.headingGoal))?choices[0]:choices[1];prior.headingGoal+=clamp(wrap(target-prior.headingGoal),-c.heading_slew*dt,c.heading_slew*dt);}
  prior.goalQuat=axisQ([0,0,1],prior.headingGoal);
  const baseline=baselineStep(s,prior,params,cfg),m=baseline.memory,error=rotationError(s.quat,m.goalQuat),angle=norm(error),axisBody=mul(error,1/Math.max(angle,1e-12)),axisWorld=mv(R,axisBody),toward=dot(s.omega,axisBody),caps=localCapacity(s,cfg,angle>1e-8?axisWorld:R.map(row=>row[0])),accel=Math.min(c.acceleration_cap,c.capacity_fraction*caps.accelerate),brake=Math.min(c.brake_cap,c.capacity_fraction*caps.brake),delay=dt+cfg.actuators.lag_seconds,stopAngle=Math.max(0,toward)**2/(2*Math.max(brake,.01))+Math.max(0,toward)*delay,turnTime=rotationTime(angle,toward,c.rate_cap,accel,brake);
  const floor=(angle>1e-8?Math.max(...Array.from({length:9},(_,i)=>geometryMinimumCG(mm(qmat(axisQ(axisWorld,angle*i/8)),R),cfg))):geometryMinimumCG(R,cfg))+.1,z=s.pos[2],vz=s.vel[2],disc=vz*vz+2*gravity*(z-floor),coast=disc>=0?Math.max(0,(vz+Math.sqrt(disc))/gravity):0,needed=turnTime+c.landing_reserve,gap=z-geometryMinimumCG(R,cfg);
  if(coast>=needed)m.released=true;
  const boost=!m.released&&angle>.6&&caps.vertical>c.support_ratio*s.mass*gravity;
  let commandRate=Math.min(c.rate_cap,p.recovery_attitude_gain/p.attitude_rate_gain*angle,Math.sqrt(2*Math.max(brake,0)*angle));if(boost)commandRate=Math.min(commandRate,c.early_rate);
  const desiredOmega=mul(axisBody,commandRate);let alpha=mul(sub(desiredOmega,s.omega),p.attitude_rate_gain),along=dot(alpha,axisBody);const guard=toward>.2&&stopAngle>angle;if(guard)along=Math.min(along,-brake);along=clamp(along,-brake,accel);alpha=add(alpha,mul(axisBody,along-dot(alpha,axisBody)));
  const budget=landingGasBudget(s,cfg),net=caps.vertical/s.mass-gravity;let targetVz=null,mode='support_during_recovery',forceZ=s.mass*gravity,forecast=null;
  if(boost){targetVz=Math.min(c.boost_speed_cap,Math.max(2,(floor-z+.5*gravity*needed*needed)/Math.max(needed,.1)+.5));forceZ=s.mass*(gravity+c.vertical_gain*(targetVz-vz));mode='build_recovery_time';}
  else if(tilt<30*Math.PI/180){
    const cap=Math.min(c.maximum_descent_speed,Math.max(c.descent_speed,gap/Math.max(.3,.65*budget.equivalentHoverTime))),reserve=c.ground_margin+Math.max(0,-vz)*2*delay,decel=Math.min(c.descent_acceleration_cap,Math.max(.1,.5*net)),available=Math.max(0,gap-reserve);targetVz=-Math.min(cap,Math.sqrt(2*decel*available)+c.touchdown_speed);const ff=available>.02&&targetVz>-cap&&targetVz<-.2&&vz<-.2?decel:0;forceZ=s.mass*(gravity+c.vertical_gain*(targetVz-vz)+ff);mode='002_descent_law';
    if(c.prediction_overlay&&tilt<8*Math.PI/180&&rate<.4&&gap>1.5){forecast=predictVerticalStop(s,cfg,{fraction:.85,touchdownSpeed:c.touchdown_speed});const margin=.05+Math.max(0,-vz)*dt;if(forecast.feasible&&gap>forecast.distance+margin){forceZ=0;mode='calibrated_prediction_coast';}else{forceZ=.85*caps.vertical;mode='calibrated_prediction_brake';}}
  }else if(m.released&&coast>=needed){forceZ=0;mode='coast_during_recovery';}
  if(m.phase==='settle'){forceZ=0;alpha=mul(alpha,.2);mode='settle';}
  const P=J.map((v,i)=>v*p.recovery_attitude_gain*error[i]),D=J.map((v,i)=>-v*p.attitude_rate_gain*s.omega[i]),gyro=c.gyro_feedforward?cross(s.omega,s.omega.map((v,i)=>J[i]*v)):[0,0,0],torque=add(alpha.map((v,i)=>J[i]*v),gyro),correction=sub(sub(sub(torque,P),D),gyro),force=[0,0,Math.max(0,forceZ)],fanCommand=m.phase==='settle'?p.landing_fan:0;
  const goal={...baseline.goal,feedbackStructure:'full_quaternion_PD_with_rate_envelope',PDtermsMeaning:'P and D are original torque references; governorCorrection is the bounded-rate/braking modification.',PDGainUnits:{P:'Nm/rad',D:'Nm*s/rad'},proportionalTorqueGains:mul(J,p.recovery_attitude_gain),dampingTorqueGains:mul(J,p.attitude_rate_gain),PDProportionalTorque:P,PDDampingTorque:D,PDIntegralTorque:[0,0,0],gyroscopicTorque:gyro,gyroFeedforward:c.gyro_feedforward,governorCorrectionTorque:correction,attitudeTorqueRequest:torque,outerBodyRateReference:desiredOmega,innerBodyRateError:sub(desiredOmega,s.omega),targetYaw:m.headingGoal,uprightRecovered:m.turnComplete,headingSettled:angle<.08&&rate<.2,horizontalForceTarget:[0,0],targetVerticalSpeed:targetVz,verticalMode:mode,recoveryBudget:{attitudeAngle:angle,angularStoppingAngle:stopAngle,angularBrakeMargin:angle-stopAngle,angularBrakeGuard:guard,availableAngularDeceleration:brake,...budget},recoveryPreview:{estimatedOrientationTime:turnTime,availableCoastTime:coast,requiredTime:needed,pathGeometryFloor:floor,notice:'Local constant-acceleration orientation budget; not full 6DoF MPC.'},verticalStoppingForecast:forecast};
  const fan=fanModel(s.pos,R,fanCommand,cfg),requestedWrenchBody=[...tmv(R,force),...sub(torque,tmv(R,fan.torqueWorld))],nozzles=nozzleLayout(cfg),A=wrenchMatrix(nozzles),pressure=delivery(s.fuel,cfg).pressureScale,disabled=new Set(s.disabledNozzles??[]),limits=nozzles.map(n=>disabled.has(n.id)?0:n.maxForce*pressure*(s.thrustScale??1)),parts=[[0,0,0,...requestedWrenchBody.slice(3)],[...requestedWrenchBody.slice(0,3),0,0,0],Array(6).fill(0)],allocated=createAllocator(A)(parts,limits,cfg.actuators.total_force_max*pressure*(s.thrustScale??1)),achieved=mv(A,allocated.forces),residual=sub(requestedWrenchBody,achieved);
  const allocation={requestedWrenchBody,achievedWrenchBody:achieved,residual,normalizedResidual:norm(residual)/Math.max(norm(requestedWrenchBody),1),momentScale:allocated.tiltScale,forceScale:allocated.forceScale,reservedZeroScale:allocated.yawScale,allocator:'conservative_analytic_priority',globallyOptimal:false,feasibilityCalls:allocated.feasibilityCalls,solverIterations:allocated.iterations,priority:'full_recovery_moment_then_vertical_force; heading target slews only after upright recovery'};
  return {nozzleThrottle:allocated.forces.map((f,i)=>limits[i]>0?f/limits[i]:0),fanCommand,desiredForceWorld:force,desiredTorqueBody:torque,forceWorld:force,torqueBody:torque,requestedWrenchBody,allocation,goal,error:baseline.error,memory:m};
}
