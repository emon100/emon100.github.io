import {DRIVING_FAN,fanExhaust} from './driving-fan.mjs?v=20261022-fan-assist';
import {RECOVERY_PARAMETERS} from './driving-recovery-parameters.mjs?v=20261022-fan-assist';
import {createSimulation} from './simulation_v4/browser-runtime.mjs?v=20261021-gas-proof';
import {createMemory,fanModel} from './simulation_v4/controller.mjs';
import {recoveryStep} from './driving-recovery.mjs?v=20261022-fan-assist';
import {drivingKickflipStep} from './driving-kickflip.mjs?v=20261022-fan-assist';
import {drivingNozzles,drivingNozzleXML} from './driving-thrusters.mjs?v=20261022-rocket10';
import {wheelDrive} from './driving-powertrain.mjs?v=20261021-gas-proof';
import {MAP,RAMPS,SPAWN,drivingXML,probeRoad} from './driving-terrain.mjs?v=20261022-rocket10';
export {MAP,RAMPS,SPAWN,rampPose,rampMeshData} from './driving-terrain.mjs?v=20261022-rocket10';
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),mix=(a,b,dt,tau)=>a+(b-a)*(1-Math.exp(-dt/tau)),dot=(a,b)=>a.reduce((v,x,i)=>v+x*b[i],0),norm=v=>Math.hypot(...v);

export function updateKickGround(state,frame,gravity,dt){
 const weight=frame.mass*gravity,loads=frame.contact.averageWheelLoads,total=loads.reduce((a,b)=>a+b,0);
 // Last-substep contact counts chatter even when the tires carry the car all control period.
 if(frame.contact.onRamp||frame.R[2][2]<=.96||frame.geometryClearance>.04||Math.abs(frame.vel[2])>.35){state.ready=false;state.seconds=0;return;}
 const supported=loads.filter(force=>force>weight*.01).length>=(state.ready?2:3)&&total>weight*.6;
 if(supported===state.ready)state.seconds=0;
 else{state.seconds+=dt;if(state.seconds+1e-9>=(supported ? .12 : .18)){state.ready=supported;state.seconds=0;}}
}

export function drivingDownforce(s,cfg,enabled){
 const result={enabled,active:false,force:0,power:0,forceWorld:[0,0,0],torqueWorld:[0,0,0],supportedPads:0,onRamp:false};if(!enabled)return result;
 const up=s.R.map(r=>r[2]),weight=(s.mass??cfg.vehicle.mass)*cfg.physics.gravity,tilt=Math.sqrt(Math.max(0,1-up[2]**2)),wanted=Math.max(6500,weight*(1+tilt-up[2]));
 // shortcut: hypothetical 180 kW sealed fan with 4 cm skirts; real ducts, seals and tires need calibration.
 const fanCfg={...cfg,fan:{...cfg.fan,electrical_power_w:180000,pressure_limit_pa:20000}};
 for(const x of [-cfg.vehicle.axle_half,cfg.vehicle.axle_half])for(const y of [-cfg.vehicle.track_half,cfg.vehicle.track_half]){
  const arm=s.R.map(row=>dot(row,[x,y,cfg.vehicle.wheel_z])),origin=s.pos.map((v,i)=>v+arm[i]),hit=probeRoad(origin,up,.45);if(!hit)continue;
  const gap=Math.max(0,hit.distance-(cfg.vehicle.wheel_z-cfg.fan.seal_body_z)-.04),fan=fanModel([0,0,gap-cfg.fan.seal_body_z],[[1,0,0],[0,1,0],[0,0,1]],1,fanCfg),force=Math.min(wanted,fan.pressure*cfg.fan.effective_floor_area)/4;
  if(force<=1)continue;const f=up.map(v=>-v*force);result.force+=force;result.power+=fan.pressurePower*(force*4/(fan.pressure*cfg.fan.effective_floor_area))**1.5/4;result.supportedPads++;result.onRamp||=hit.onRamp;
  for(let i=0;i<3;i++){result.forceWorld[i]+=f[i];result.torqueWorld[i]+=arm[(i+1)%3]*f[(i+2)%3]-arm[(i+2)%3]*f[(i+1)%3];}
 }
 result.active=result.force>1;return result;
}
export async function createDrivingSimulation(cfg,params,{engine,xml,assist=true,downforce=false,gasCapacity=400,recoveryParameters=RECOVERY_PARAMETERS}={}){
 if(Object.keys(RECOVERY_PARAMETERS).some(k=>!Number.isFinite(recoveryParameters[k])||recoveryParameters[k]<=0)||recoveryParameters.brakeExit>=recoveryParameters.brakeEnter)throw Error('Invalid recovery parameters');
 if(!Number.isFinite(gasCapacity)||gasCapacity<=0||gasCapacity>600)throw Error('Invalid gas capacity');
 // Preserve dry mass; the larger hypothetical tank adds gas mass, not free impulse.
 cfg={...cfg,recoveryParameters:{...recoveryParameters},vehicle:{...cfg.vehicle,mass:cfg.vehicle.mass+gasCapacity-cfg.fuel.initial_kg},fuel:{...cfg.fuel,initial_kg:gasCapacity}};
 const nozzles=drivingNozzles(cfg),dt=cfg.physics.control_dt,capacity=cfg.fuel.initial_kg,zero=Array(nozzles.length).fill(0),flightCfg={...cfg,fan:{...cfg.fan,electrical_power_w:0}},kickGround={ready:false,seconds:0};
 let fanPower=0,fanTarget=0,fanEnergy=0,fanPeak=0,fanSeconds=0,fanImpulse=[0,0,0],airFan=null;
 let physical,f,input={},steering=0,driveCommand=0,braking=false,gear='D',mode='ground',action=null,lastAction=null,actionId=0,resetSerial=0,resetReason='initial',refillAt=null,refillFrom=capacity,airTime=0,contactTime=0,overturnedTime=0,takeoff=null,joints,heading=0,suction={enabled:downforce,active:false,force:0,power:0},powertrain={motorPower:0,regenPower:0,brakePower:0,hold:true};
 const baseCase=pos=>({id:'driving',task:'moving_recovery',pos,quat:[1,0,0,0],vel:[0,0,0],omega:[0,0,0],goal:{turns:0}});
 function driveForces({mj,model,data,state:s,contact}){
  joints??={body:mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY.value,'car'),spin:[0,1,2,3].map(i=>mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT.value,`wheelspin${i}`)),steer:[2,3].map(i=>mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT.value,`steer${i}`))};
  const radius=Math.abs(steering)>.001?2*cfg.vehicle.axle_half/Math.tan(steering):1e6;
  // Rack stiffness counters tire scrub; joint damping is integrated implicitly by MuJoCo.
  for(let k=0;k<2;k++){const j=joints.steer[k],y=(k===0?-1:1)*cfg.vehicle.track_half,target=Math.abs(steering)<.001?0:Math.atan(2*cfg.vehicle.axle_half/(radius-y)),dof=model.jnt_dofadr[j],angle=data.qpos[model.jnt_qposadr[j]];data.qfrc_applied[dof]+=clamp((target-angle)*60000,-6000,6000);}
  // Wheel torques only: ground contact supplies traction and steering; airborne pose is never prescribed.
  // Reattach the fan after stable tire contact; suction during final descent would increase impact speed.
  fanPower+=(fanTarget*DRIVING_FAN.maxElectricalPower-fanPower)*(1-Math.exp(-cfg.physics.dt/DRIVING_FAN.responseSeconds));
  airFan=fanExhaust(fanPower,{...s,massProperties:{bodyIpos:Array.from(model.body_ipos.slice(joints.body*3,joints.body*3+3))}},cfg);
  fanEnergy+=fanPower*cfg.physics.dt;fanPeak=Math.max(fanPeak,fanPower);if(fanPower>100)fanSeconds+=cfg.physics.dt;
  for(let i=0;i<3;i++)fanImpulse[i]+=airFan.forceWorld[i]*cfg.physics.dt;
  suction=drivingDownforce(s,cfg,fanPower<1&&downforce&&(!action||(action.handedOffAt!==null&&action.roadContactSeconds>.15)));suction.enabled=downforce;
  powertrain=wheelDrive(s,contact,{pedal:driveCommand,brake:braking,attached:suction.supportedPads>=3||contact.wheelCount>=2},cfg);
  for(const [i,j] of joints.spin.entries())data.qfrc_applied[model.jnt_dofadr[j]]+=powertrain.torques[i];
  for(let i=0;i<3;i++){data.xfrc_applied[joints.body*6+i]+=suction.forceWorld[i]+powertrain.drag[i]+airFan.forceWorld[i];data.xfrc_applied[joints.body*6+i+3]+=suction.torqueWorld[i]+airFan.torqueWorld[i];}
 }
 physical=await createSimulation(cfg,params,baseCase([...SPAWN]),{engine,xml:drivingXML(drivingNozzleXML(xml,cfg)),nozzles,driving:true,applyForces:driveForces});f=physical.snapshot();
 const speed=()=>dot(f.vel,f.R.map(r=>r[0]));
 function gasState(){const elapsed=refillAt===null?null:f.t-refillAt,phase=action?(action.handedOffAt!==null?'settling':assist?'using':'coasting'):elapsed===null?'ready':elapsed<0?'waiting':'refilling';return {phase,capacity,fraction:f.fuel/capacity,secondsRemaining:elapsed===null?0:Math.max(0,elapsed<0?-elapsed:2-elapsed),fillProgress:elapsed===null?1:clamp(elapsed/2,0,1)};}
 function canKick(){return assist&&!action&&refillAt===null&&f.fuel>=capacity-1e-6&&kickGround.ready;}
 function snapshot(){return {...f,airFan:{...airFan,enabled:downforce&&assist,power:fanPower,command:fanTarget,energyJ:fanEnergy,peakPower:fanPeak,activeSeconds:fanSeconds,impulseWorld:[...fanImpulse]},mode,braking,needsReset:overturnedTime>=4,speed:norm(f.vel)*3.6,longitudinalSpeed:speed(),heading,steeringAngle:(f.wheelSteering[2]+f.wheelSteering[3])/2,yawRate:f.omegaWorld[2],gear,assist,powertrain,downforce:suction,kick:action?.kind==='kickflip'&&action.handedOffAt===null,canKick:canKick(),kickDegrees:action?.kind==='kickflip'?clamp(action.memory.turnAngle*180/Math.PI,0,360):0,gas:gasState(),lastAction,resetSerial,resetReason,airborne:mode==='air',recoveryActive:!!action&&assist&&action.handedOffAt===null,landingPhase:action?.handedOffAt!=null?'grounded':action?'airborne':'none'};}
 function reset(id){fanPower=fanTarget=fanEnergy=fanPeak=fanSeconds=0;fanImpulse=[0,0,0];airFan=null;kickGround.ready=false;kickGround.seconds=0;const r=RAMPS.find(r=>r.id===id),c=typeof id==='object'?{...baseCase([...SPAWN]),...id}:baseCase(r?[r.x-15,r.y,.5]:[...SPAWN]);f=physical.reset(c,params);heading=Math.atan2(f.R[1][0],f.R[0][0]);steering=driveCommand=airTime=contactTime=overturnedTime=0;powertrain={motorPower:0,regenPower:0,brakePower:0,hold:true};braking=false;gear='D';mode='ground';action=lastAction=takeoff=null;refillAt=null;resetReason='test-initial-state';resetSerial++;return snapshot();}
 function recenter(reason='manual'){fanPower=fanTarget=fanEnergy=fanPeak=fanSeconds=0;fanImpulse=[0,0,0];airFan=null;kickGround.ready=false;kickGround.seconds=0;const fuel=f.fuel;f=physical.reset({...baseCase([...SPAWN]),fuel,mass:cfg.vehicle.mass-capacity+fuel},params);action=null;mode='ground';steering=driveCommand=heading=airTime=contactTime=overturnedTime=0;powertrain={motorPower:0,regenPower:0,brakePower:0,hold:true};braking=false;gear='D';refillFrom=fuel;refillAt=fuel>=capacity-1e-6?null:f.t+5;lastAction=takeoff=null;suction=drivingDownforce(f,cfg,downforce);resetReason=reason;resetSerial++;return snapshot();}
 function begin(kind){takeoff={...baseCase(f.pos),quat:[...f.quat],vel:[...f.vel],omega:[...f.omega],fuel:f.fuel,goal:{turns:kind==='kickflip'?1:0}};action={id:++actionId,kind,startedAt:f.t,memory:createMemory(takeoff,cfg),peakHeight:f.pos[2],airborne:false,handedOffAt:null,roadContactSeconds:0};refillAt=null;physical.clearContactMetrics();}
 function finish(success,status){lastAction={id:action.id,kind:action.kind,success,status,completedAt:f.t,spin:action.memory.turnAngle,measuredSpin:action.memory.spinMeasured,metrics:{...f.metrics,fanEnergyJ:fanEnergy,fanPeakPower:fanPeak,fanActiveSeconds:fanSeconds,fanImpulseWorld:[...fanImpulse],groundContactSeconds:action.roadContactSeconds,requiredSpin:action.memory.requiredSpin,measuredSpin:action.memory.spinMeasured,turnAngle:action.memory.turnAngle,turnComplete:action.memory.turnComplete,peakHeight:action.peakHeight,finalTilt:Math.acos(clamp(f.R[2][2],-1,1))}};action=null;refillFrom=f.fuel;refillAt=f.t+5;physical.clearContactMetrics();}
 async function step(next={}){
  input=next;const v=speed(),throttle=Number.isFinite(input.throttle)?clamp(input.throttle,-1,1):0,steer=Number.isFinite(input.steer)?clamp(input.steer,-1,1):0;
  braking=Boolean(input.brake)||(throttle!==0&&v*throttle<-.25);driveCommand=mix(driveCommand,braking?0:throttle,dt,.16);
  // shortcut: A dry-tire acceleration budget bounds steering including sideslip; calibrate a tire/ESC model for real handling.
  const lateralBudget=.75*cfg.physics.gravity*(1+Math.min(.35,suction.force/(f.mass*cfg.physics.gravity))),maxSteer=Math.min(.68,Math.atan(2*cfg.vehicle.axle_half*lateralBudget/Math.max(1,dot(f.vel,f.vel))));
  steering=mix(steering,steer*maxSteer,dt,.18);
  if(v<-.2)gear='R';else if(v>.2||throttle>0)gear='D';
  if(input.kick&&canKick())begin('kickflip');
  if(!action&&f.contact.wheelCount===0&&airTime>.06&&f.geometryClearance>.2)begin('recovery');
  if(action&&action.airborne&&action.handedOffAt===null){
   const normal=f.contact.surfaceNormal,aligned=normal?dot(f.R.map(r=>r[2]),normal):f.R[2][2],normalSpeed=normal?dot(f.vel,normal):f.vel[2],rotationSlow=Math.hypot(f.omega[0],f.omega[1])<.65,spinDone=action.memory.requiredSpin===0||action.memory.spinSatisfied;
   const touching=f.contact.wheelCount>=2&&aligned>.96&&Math.abs(normalSpeed)<1.2,nearFloor=f.R[2][2]>.985&&f.geometryClearance<.008&&f.vel[2]>-.45&&f.vel[2]<.25;
   // Keep recovery feedback until tire contact; a near-floor cutoff creates an uncontrolled final drop.
   if(rotationSlow&&(touching||(action.kind!=='recovery'&&spinDone&&nearFloor)))action.handedOffAt=f.t;
  }
  let control=null;if(action&&assist&&action.handedOffAt===null){control=(action.kind==='recovery'?recoveryStep:drivingKickflipStep)({...f,airFan:{enabled:downforce&&assist,power:fanPower}},action.memory,params,flightCfg);action.memory=control.memory;}
  fanTarget=control?.fanPowerFraction??0;
  physical.advance(control?.nozzleThrottle??zero,0,{turnComplete:action?.memory.turnComplete??false,wasAirborne:action?.airborne??false});f=physical.snapshot();
  const loaded=(f.contact.averageWheelLoads??[]).reduce((sum,n)=>sum+n,0)>f.mass*cfg.physics.gravity*.03;
  airTime=f.contact.wheelCount===0&&!loaded&&suction.supportedPads<3?airTime+dt:0;contactTime=f.contact.wheelCount>0||loaded||f.contact.bodyContact?contactTime+dt:0;
  mode=airTime>.04?'air':f.contact.onRamp||suction.onRamp?'ramp':'ground';if(mode!=='air'&&Math.hypot(f.R[0][0],f.R[1][0])>.08)heading=Math.atan2(f.R[1][0],f.R[0][0]);
  const supportedRamp=f.contact.onRamp&&f.contact.surfaceNormal&&dot(f.R.map(r=>r[2]),f.contact.surfaceNormal)>.85;
  overturnedTime=f.R[2][2]<.5&&!supportedRamp?overturnedTime+dt:0;
  if(action){action.airborne||=airTime>.06;action.peakHeight=Math.max(action.peakHeight,f.pos[2]);const m=f.metrics;
   const normal=f.contact.surfaceNormal,roadTilt=normal?Math.acos(clamp(dot(f.R.map(r=>r[2]),normal),-1,1)):Math.PI,roadSpeed=normal?Math.abs(dot(f.vel,normal)):Infinity;
   const loads=f.contact.averageWheelLoads??f.contact.wheelLoads,supported=loads.filter(v=>v>f.mass*cfg.physics.gravity*.02).length>=2&&loads.reduce((a,b)=>a+b,0)>f.mass*cfg.physics.gravity*.65&&roadTilt<.18&&roadSpeed<.35&&Math.hypot(f.omega[0],f.omega[1])<.35;
   action.roadContactSeconds=supported?action.roadContactSeconds+dt:0;
   if(action.handedOffAt!==null&&airTime>.12&&f.geometryClearance>.35){action.handedOffAt=null;action.kind='recovery';action.memory=createMemory({...takeoff,pos:f.pos,quat:f.quat,vel:f.vel,omega:f.omega,goal:{turns:0}},cfg);}
   if(action.airborne&&action.roadContactSeconds>=1-1e-8&&!m.bodyContact&&!m.hardContact&&(action.memory.requiredSpin===0||action.memory.spinSatisfied))finish(true,'success');
   else if(action.airborne&&action.roadContactSeconds>=1-1e-8&&!action.memory.spinSatisfied)finish(false,'turn_incomplete');
   else if(action.airborne&&contactTime>.5&&(m.bodyContact||m.hardContact))finish(false,m.bodyContact?'body_contact':'hard_contact');
   else if(f.t-action.startedAt>=12)finish(false,'not_stable');
  }
  if(!action&&refillAt!==null&&mode!=='air'){const elapsed=f.t-refillAt;if(elapsed<0)refillFrom=f.fuel;if(elapsed>=0){physical.setFuel(refillFrom+(capacity-refillFrom)*clamp(elapsed/2,0,1));f=physical.snapshot();if(elapsed>=2-1e-8)refillAt=null;}}
  updateKickGround(kickGround,f,cfg.physics.gravity,dt);
  if(Math.abs(f.pos[0])>MAP.halfX-MAP.edgeMargin||Math.abs(f.pos[1])>MAP.halfY-MAP.edgeMargin)recenter('boundary');
  return snapshot();
 }
 function setOptions(next){
  if('assist' in next){if(typeof next.assist!=='boolean')throw Error('Invalid driving mode');const reenable=!assist&&next.assist;assist=next.assist;if(reenable&&action&&action.handedOffAt===null){action.kind='recovery';action.memory=createMemory({...takeoff,pos:f.pos,quat:f.quat,vel:f.vel,omega:f.omega,goal:{turns:0}},cfg);}}
  if('downforce' in next){if(typeof next.downforce!=='boolean')throw Error('Invalid downforce setting');downforce=next.downforce;suction={...suction,enabled:downforce};}
  return snapshot();
 }
 return {step,snapshot,reset,recenter,setOptions,get takeoff(){return takeoff},dispose(){physical.dispose()}};
}
