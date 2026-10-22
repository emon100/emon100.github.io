import {RECOVERY_PARAMETERS} from './driving-recovery-parameters.mjs?v=20261022-fan-assist';
import {allocateRecoveryFan,fanExhaust,DRIVING_FAN} from './driving-fan.mjs?v=20261022-fan-assist';
import {createMemory,unitInertia,delivery,geometryMinimumCG,PARAMETER_NAMES} from './simulation_v4/controller.mjs';
import {clamp,norm,mul,sub,cross,tmv,mv,dot,rotationError} from './simulation_v3/math.mjs';
import {createDrivingAllocator} from './driving-allocation.mjs?v=20261022-fan-assist';

function predictCoastStop(s,cfg,nozzles,A,command,brake){
 const dt=cfg.physics.dt,window=Math.round(cfg.physics.control_dt/dt),response=1-Math.exp(-dt/cfg.actuators.lag_seconds),g=cfg.physics.gravity,disabled=new Set(s.disabledNozzles??[]);
 const fmax=nozzles.map(n=>disabled.has(n.id)?0:n.maxForce*(s.thrustScale??1)),lift=A[0].map((_,j)=>s.R[2].reduce((sum,v,i)=>sum+v*A[i][j],0)),dry=s.mass-s.fuel;
 const fanLift=fanExhaust(DRIVING_FAN.maxElectricalPower,s,cfg).forceWorld[2],fanDecay=Math.exp(-dt/DRIVING_FAN.responseSeconds);
 let fanPower=s.airFan?.power??0;
 let fuel=s.fuel,mass=s.mass,velocity=s.vel[2],distance=0,coastDistance=0,u=[...(s.actualNozzleThrottle??Array(nozzles.length).fill(0))],peak=null,feasible=false,elapsed=0;
 // Keep the current attitude-control gas cost in the full-burn preview.
 for(let i=0;i<window+Math.round(6/dt);i++){
  if(i===window)coastDistance=distance;
  if(i>=window&&velocity>=-.08){feasible=true;break;}
  if(i%window===0)mass=dry+fuel;
  const target=i<window?command:brake;
  u=u.map((v,j)=>v+(target[j]-v)*response);
  const {pressureScale:pressure,effectiveIsp:isp}=delivery(fuel,cfg),cap=Math.min(cfg.actuators.total_force_max*pressure*(s.thrustScale??1),fuel*isp*g/dt);
  let forces=u.map((v,j)=>v*fmax[j]*pressure),total=forces.reduce((a,b)=>a+b,0);
  if(total>cap)forces=mul(forces,cap/Math.max(total,1e-12));
  fuel=Math.max(0,fuel-forces.reduce((a,b)=>a+b,0)*dt/(isp*g));
  fanPower*=fanDecay;
  const acceleration=(forces.reduce((sum,f,j)=>sum+f*lift[j],0)+fanLift*(fanPower/DRIVING_FAN.maxElectricalPower)**(2/3))/mass-g;
  velocity+=acceleration*dt;distance-=velocity*dt;elapsed=(i+1)*dt;
  if(i>=window&&acceleration>0&&(!peak||velocity>peak.velocity))peak={velocity,distance,fuel,time:elapsed};
  if(peak&&elapsed-peak.time>cfg.physics.control_dt)break;
 }
 if(velocity>=-.08)feasible=true;
 const end=feasible?{velocity,distance,fuel,time:elapsed}:peak??{velocity,distance,fuel,time:elapsed};
 return {feasible,distance:end.distance,time:end.time,gasUsed:s.fuel-end.fuel,endFuel:end.fuel,predictedFinalVz:end.velocity,peakBraking:!feasible&&!!peak,coastDistance,coastSeconds:cfg.physics.control_dt,dt,assumption:'Fixed attitude and current attitude torque allocation: one coast control period, then maximum available axial lift with valve lag, gas pressure decay and residual fan exhaust spinning down.'};
}

export function recoveryStep(s,m,params,cfg){
 s={...s,actualNozzleThrottle:s.actualNozzleThrottle??s.nozzleThrottle};
 const p=Array.isArray(params)?Object.fromEntries(PARAMETER_NAMES.map((key,i)=>[key,params[i]])):{...params};
 const tuning=cfg.recoveryParameters??RECOVERY_PARAMETERS;
 const J=mul(unitInertia(cfg),s.mass),tiltRate=Math.hypot(s.omega[0],s.omega[1]),tilt=Math.acos(clamp(s.R[2][2],-1,1)),up=s.R.map(r=>r[2]);
 const headingTarget=createMemory({...s,goal:{turns:0}},cfg).target,axis=cross(up,[0,0,1]),length=norm(axis);
 const error=length>1e-6?mul(tmv(s.R,axis),tilt/length):[tilt,0,0],target=mul(error,Math.min(tuning.rateCap,tuning.tiltGain*tilt)/Math.max(tilt,1e-9));
 m.braking=tiltRate>tuning.brakeEnter||Boolean(m.braking&&tiltRate>tuning.brakeExit);if(m.braking)target.fill(0);
 if(!m.braking&&tilt<.35)target[2]=clamp(2*rotationError(s.quat,headingTarget)[2],-2,2);
 const gyro=cross(s.omega,s.omega.map((v,i)=>v*J[i])),torque=s.omega.map((v,i)=>J[i]*tuning.rateGain*(target[i]-v)+(m.braking?0:gyro[i]));
 const {nozzles,A,limits,cap,lift,allocate}=createDrivingAllocator(s,cfg);
 if(s.actualNozzleThrottle&&s.actualNozzleThrottle.length!==nozzles.length)throw Error('Recovery valve state must match the ten-nozzle layout');
 // shortcut: The stopping target is flat ground; nearby ramp sides require a terrain intercept predictor.
 const gap=s.pos[2]-geometryMinimumCG(s.R,cfg),g=cfg.physics.gravity,delay=cfg.physics.control_dt+cfg.actuators.lag_seconds;
 const maximum=Math.min(lift.reduce((sum,v,i)=>sum+Math.max(0,v)*limits[i],0),cap*Math.max(0,...lift)),net=maximum/s.mass-g;
 const decel=Math.min(p.descent_accel,Math.max(.1,.5*net)),available=Math.max(0,gap-.04-Math.max(0,-s.vel[2])*2*delay);
 const targetVz=-Math.min(8,Math.sqrt(2*decel*available)+.08),ff=available>.02&&targetVz>-8&&targetVz<-.2&&s.vel[2]<-.2?decel:0;
 let fanAllowed=true;
 let forceZ=Math.max(0,s.mass*(g+p.vertical_gain*(targetVz-s.vel[2])+ff)),phase='landing',forecast=null;
 if(tilt<2*Math.PI/180&&tiltRate<.1&&(gap>.25||s.vel[2]<-.5)){
  fanAllowed=false;
  const stoppedFan={...s,airFan:{...s.airFan,enabled:false}},coastCommand=allocateRecoveryFan(stoppedFan,cfg,torque,0,allocate),brakeCommand=allocateRecoveryFan(stoppedFan,cfg,torque,maximum,allocate);
  forecast=predictCoastStop(s,cfg,nozzles,A,coastCommand.nozzleThrottle,brakeCommand.nozzleThrottle);
  const coast=(forecast.feasible||forecast.peakBraking)&&gap>forecast.distance+.15;
  forceZ=coast?0:maximum;phase=coast?'predicted_coast':'predicted_brake';
 }
 const allocation=allocateRecoveryFan({...s,airFan:{...s.airFan,enabled:fanAllowed&&s.airFan?.enabled}},cfg,torque,forceZ,allocate),requested=[...tmv(s.R,[0,0,forceZ]),...torque];
 allocation.requestedWrenchBody=requested;allocation.residual=sub(requested,allocation.achievedWrenchBody);allocation.normalizedResidual=norm(allocation.residual)/Math.max(1,norm(requested));
 if(m.elapsed>0&&m.spinAxis)m.turnAngle+=dot(s.omegaWorld,m.spinAxis)*cfg.physics.control_dt;
 m.elapsed=(m.elapsed??0)+cfg.physics.control_dt;m.peakHeight=Math.max(m.peakHeight??s.pos[2],s.pos[2]);
 if(!m.requiredSpin)m.spinSatisfied=true;
 m.turnComplete=tilt<12*Math.PI/180&&norm(s.omega)<.65&&m.spinSatisfied;
 m.goal={phase,targetQuat:headingTarget,headingTargetQuat:headingTarget,targetVerticalSpeed:targetVz,verticalStoppingForecast:forecast,horizontalForceControlled:false,horizontalForceTarget:null,allocatedForceWorld:allocation.forceWorld,brakingSpin:m.braking,turnComplete:m.turnComplete,requiredSpin:m.requiredSpin??0,spinSatisfied:m.spinSatisfied,controllerFamily:'driving_rocket10_recovery'};
 m.error={tilt,angularSpeed:norm(s.omega),remaining:tilt};
 return {nozzleThrottle:allocation.nozzleThrottle,fanCommand:0,fanPowerFraction:allocation.fanPowerFraction,desiredForceWorld:[0,0,forceZ],desiredTorqueBody:torque,requestedWrenchBody:requested,allocation,goal:m.goal,error:m.error,memory:m};
}
