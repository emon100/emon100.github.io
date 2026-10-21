import {PARAMETER_NAMES,qmat,nozzleLayout,unitInertia,massProperties,geometryMinimumCG,delivery,fanModel,allocateFixedNozzles,controllerStep as defaultController,createControllerMemory} from './controller.mjs';
export {qmat,nozzleLayout,unitInertia,delivery,fanModel,allocateFixedNozzles} from './controller.mjs';
// MuJoCo is the sole pose integrator. Controller, valves, gas and fan models mirror core.py.
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = v => Math.hypot(...v);
const dot = (a,b) => a.reduce((s,x,i)=>s+x*b[i],0);
const add = (a,b) => a.map((x,i)=>x+b[i]);
const sub = (a,b) => a.map((x,i)=>x-b[i]);
const mul = (a,k) => a.map(x=>x*k);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const rotate = (R,v) => R.map(row=>dot(row,v));
const unrotate = (R,v) => [0,1,2].map(j=>R.reduce((s,row,i)=>s+row[j]*v[i],0));
const copy = x => structuredClone(x);

export function initialClearance(testCase,cfg) {
  return testCase.pos[2]-geometryMinimumCG(qmat(testCase.quat),cfg);
}
function validateCase(c,cfg) {
  if(!cfg.tasks[c.task])throw new Error('Unknown task');
  for(const [key,n] of [['pos',3],['quat',4],['vel',3],['omega',3]])if(!Array.isArray(c[key])||c[key].length!==n||!c[key].every(Number.isFinite))throw new Error(`Invalid ${key}`);
  if(Math.abs(norm(c.quat)-1)>1e-6)throw new Error('Quaternion must have unit norm');
  if(initialClearance(c,cfg)<-.001)throw new Error('Initial geometry intersects the floor');
  const mass=c.mass??cfg.vehicle.mass,fuel=c.fuel??cfg.fuel.initial_kg,scale=c.thrust_scale??1;
  if(!Number.isFinite(mass)||!Number.isFinite(fuel)||mass<=fuel+80||fuel<0||fuel>cfg.fuel.initial_kg)throw new Error('Invalid mass or fuel');
  massProperties(cfg,mass);massProperties(cfg,mass-fuel);
  if(!Number.isFinite(scale)||scale<0||scale>2)throw new Error('Invalid thrust scale');
  if(c.delivery_mode&&c.delivery_mode!=='unregulated_blowdown')throw new Error('Only unregulated blowdown is supported');
  if(c.disabled_nozzles&&(!Array.isArray(c.disabled_nozzles)||c.disabled_nozzles.some(id=>!nozzleLayout(cfg).some(n=>n.id===id))))throw new Error('Unknown disabled nozzle');
}

export async function loadEngine(moduleUrl=new URL('./vendor/mujoco/mujoco.js',globalThis.document?.baseURI??import.meta.url).href) {
  const {default:load}=await import(moduleUrl);
  const engine=await load();
  if(engine.mj_versionString()!=='3.15.0')throw new Error('The experiment requires MuJoCo 3.15.0');
  return engine;
}

export async function createSimulation(config,parameters,testCase,options={}) {
  const cfg=copy(config),mj=options.engine??await loadEngine(options.moduleUrl);
  const xml=options.xml??await fetch(new URL('./world.xml',import.meta.url)).then(r=>{if(!r.ok)throw new Error(`Model HTTP ${r.status}`);return r.text();});
  const controller=options.controllerStep??defaultController;
  const createMemory=options.createMemory??((c,cfg)=>createControllerMemory(c.task,c,cfg));
  const model=mj.MjModel.from_xml_string(xml),data=new mj.MjData(model);
  const obj=mj.mjtObj,body=mj.mj_name2id(model,obj.mjOBJ_BODY.value,'car'),floor=mj.mj_name2id(model,obj.mjOBJ_GEOM.value,'floor');
  if(body<0||floor<0||model.nu!==12||model.nv!==10){data.delete();model.delete();throw new Error('Model must contain car, floor, four wheel hinges and twelve nozzles');}
  const wheelIndex=new Map([0,1,2,3].map(i=>[mj.mj_name2id(model,obj.mjOBJ_GEOM.value,`wheel${i}`),i])),wheelIds=new Set(wheelIndex.keys());
  const wheelPositions=[];for(const x of [-cfg.vehicle.axle_half,cfg.vehicle.axle_half])for(const y of [-cfg.vehicle.track_half,cfg.vehicle.track_half])wheelPositions.push([x,y,cfg.vehicle.wheel_z]);
  const bodyIds=new Set(cfg.vehicle.collision_boxes.map(b=>mj.mj_name2id(model,obj.mjOBJ_GEOM.value,b.name)));
  const stateSpec=mj.mjtState.mjSTATE_INTEGRATION.value;
  const saved=new mj.DoubleBuffer(mj.mj_stateSize(model,stateSpec));
  const velocity=new mj.DoubleBuffer(6),contactForce=new mj.DoubleBuffer(6);
  const nozzles=nozzleLayout(cfg),fmax=nozzles.map(n=>n.maxForce);
  const dt=cfg.physics.dt,controlDt=cfg.physics.control_dt,substeps=Math.round(controlDt/dt),response=1-Math.exp(-dt/cfg.actuators.lag_seconds);
  if(!(dt>0&&substeps>0&&Math.abs(substeps*dt-controlDt)<1e-10)){saved.delete();velocity.delete();contactForce.delete();data.delete();model.delete();throw new Error('Control period must be a positive integer number of physics steps');}
  let disposed=false,c,params,memory,fuel,dryMass,disabled,thrustScale,mode,throttle,forces,fan,contact,allocation,goal,error;
  let fuelUsed,pressureScale,effectiveIsp,peakClosing,peakForce,bodyContact,hardContact,stable,lastStable,status,success,failedAt,score,lastProgress,lastHeightProgress,peakHeight,initialTilt,turnBest,pendingImpulse,done,policyId,lastRewardInstant,lastRewardComponents,jetForceWorld,jetHorizontalImpulse,fanHorizontalImpulse,peakJetHorizontalForce;
  const assertLive=()=>{if(disposed)throw new Error('Simulation disposed');};
  function updateMass(){
    const mp=massProperties(cfg,dryMass+fuel);model.body_mass[body]=mp.bodyMass;
    model.body_ipos.set(mp.bodyIpos,body*3);model.body_inertia.set(mp.bodyInertia,body*3);
    mj.mj_getState(model,data,saved,stateSpec);mj.mj_setConst(model,data);mj.mj_setState(model,data,Array.from(saved.GetView()),stateSpec);
  }
  function state(){
    const flat=Array.from(data.xmat.slice(body*9,body*9+9)),R=[flat.slice(0,3),flat.slice(3,6),flat.slice(6,9)];
    mj.mj_objectVelocity(model,data,obj.mjOBJ_XBODY.value,body,velocity,0);
    const v=Array.from(velocity.GetView()),omegaWorld=v.slice(0,3);
    return {pos:Array.from(data.qpos.slice(0,3)),quat:Array.from(data.qpos.slice(3,7)),vel:v.slice(3),omega:unrotate(R,omegaWorld),omegaWorld,R,mass:dryMass+fuel,wheelRates:Array.from(data.qvel.slice(6,10))};
  }
  const limitsAt=pressure=>fmax.map((f,i)=>disabled.has(nozzles[i].id)?0:f*pressure*thrustScale);
  function advance(command,fanCommand=0,stabilityContext={}){
    assertLive();if(command.length!==12||!command.every(Number.isFinite)||!Number.isFinite(fanCommand))throw new Error('Invalid actuator command');
    updateMass();mj.mj_forward(model,data);
    let maximumClosing=0,maximumForce=0,maximumTotal=0,touchedBody=false,wheelCount=0,minimumWheelCount=4,maximumSlip=0;
    for(let k=0;k<substeps;k++){
      const s=state();throttle=throttle.map((v,i)=>v+(clamp(command[i],0,1)-v)*response);
      const {pressureScale:pressure,effectiveIsp:isp}=delivery(fuel,cfg,mode),limits=limitsAt(pressure);
      forces=throttle.map((v,i)=>v*limits[i]);const cap=Math.min(cfg.actuators.total_force_max*pressure*thrustScale,fuel*isp*cfg.physics.gravity/dt),total=forces.reduce((a,b)=>a+b,0);
      if(total>cap)forces=mul(forces,cap/Math.max(total,1e-12));
      data.ctrl.set(forces);pressureScale=pressure;effectiveIsp=isp;
      const consumed=forces.reduce((a,b)=>a+b,0)*dt/(isp*cfg.physics.gravity);fuel=Math.max(0,fuel-consumed);fuelUsed+=consumed;
      fan=fanModel(s.pos,s.R,fanCommand,cfg);const fanForce=add(fan.suctionForce,fan.axialForce),fanTorque=sub(fan.torqueWorld,cross(rotate(s.R,Array.from(model.body_ipos.slice(body*3,body*3+3))),fanForce));
      data.xfrc_applied.fill(0);data.xfrc_applied.set([...fanForce,...fanTorque],body*6);
      const jetBody=[0,0,0];nozzles.forEach((n,i)=>n.direction.forEach((v,j)=>jetBody[j]+=v*forces[i]));jetForceWorld=rotate(s.R,jetBody);peakJetHorizontalForce=Math.max(peakJetHorizontalForce,norm(jetForceWorld.slice(0,2)));for(let i=0;i<2;i++){jetHorizontalImpulse[i]+=jetForceWorld[i]*dt;fanHorizontalImpulse[i]+=fan.axialForce[i]*dt;}
      const impulseForce=mul(pendingImpulse,1/dt),arm=sub(Array.from(data.subtree_com.slice(body*3,body*3+3)),s.pos);
      data.qfrc_applied.fill(0);data.qfrc_applied.set([...impulseForce,...unrotate(s.R,cross(arm,impulseForce))],0);pendingImpulse=[0,0,0];
      mj.mj_step(model,data);
      const touched=new Set();let totalForce=0,stepSlip=0;const contacts=data.contact;
      try{for(let i=0;i<data.ncon;i++){
        const con=contacts.get(i);try{
          if(con.geom1!==floor&&con.geom2!==floor)continue;const other=con.geom1===floor?con.geom2:con.geom1;
          mj.mj_contactForce(model,data,i,contactForce);const normal=Math.max(0,contactForce.GetView()[0]);totalForce+=normal;maximumForce=Math.max(maximumForce,normal);
          const pv=add(s.vel,cross(s.omegaWorld,sub(Array.from(con.pos),s.pos))),closing=Math.max(0,-pv[2]);maximumClosing=Math.max(maximumClosing,closing);
          if(wheelIds.has(other)&&normal>1){touched.add(other);const wi=wheelIndex.get(other),center=add(s.pos,rotate(s.R,wheelPositions[wi])),wheelVelocity=add(pv,cross(mul(s.R.map(r=>r[1]),s.wheelRates[wi]),sub(Array.from(con.pos),center)));stepSlip=Math.max(stepSlip,norm(wheelVelocity.slice(0,2)));}if(bodyIds.has(other)&&normal>1)touchedBody=true;
        }finally{con?.delete();}
      }}finally{contacts.delete();}
      maximumSlip=Math.max(maximumSlip,stepSlip);wheelCount=touched.size;minimumWheelCount=Math.min(minimumWheelCount,wheelCount);maximumTotal=Math.max(maximumTotal,totalForce);
      mj.mj_forward(model,data);const post=state(),l=cfg.landing;
      hardContact||=maximumClosing>l.max_contact_closing_speed||totalForce>post.mass*cfg.physics.gravity*l.max_normal_force_weight_ratio;
      const statesOK=[s,post].every(st=>Math.acos(clamp(st.R[2][2],-1,1))<l.max_tilt_deg*Math.PI/180&&Math.abs(st.vel[2])<l.max_vertical_speed&&Math.abs(unrotate(st.R,st.vel)[1])<l.max_lateral_speed&&norm(st.omega)<l.max_angular_speed);
      lastStable=Boolean(wheelCount>=l.minimum_wheel_contacts&&stepSlip<l.max_wheel_slip_speed&&statesOK&&stabilityContext.turnComplete&&stabilityContext.wasAirborne&&!hardContact&&!touchedBody&&!bodyContact);
      stable=lastStable?stable+dt:0;
    }
    data.qfrc_applied.fill(0);mj.mj_forward(model,data);
    peakClosing=Math.max(peakClosing,maximumClosing);peakForce=Math.max(peakForce,maximumTotal);bodyContact||=touchedBody;
    contact={count:data.ncon,wheelCount,bodyContact:touchedBody,maxNormalForce:maximumForce,totalNormalForce:maximumTotal,maxClosingSpeed:maximumClosing,minimumWheelCount,maxWheelSlipSpeed:maximumSlip,stableSeconds:stable};
    return state();
  }
  function snapshot(){
    assertLive();const s=state();return {...s,t:Math.round(data.time*1e6)/1e6,controlObservationTime:Math.round(Math.max(0,data.time-controlDt)*1e6)/1e6,assemblyCom:Array.from(data.subtree_com.slice(body*3,body*3+3)),massProperties:{bodyMass:model.body_mass[body],bodyIpos:Array.from(model.body_ipos.slice(body*3,body*3+3)),bodyInertia:Array.from(model.body_inertia.slice(body*3,body*3+3))},jetForceWorld:[...jetForceWorld],jetHorizontalImpulse:[...jetHorizontalImpulse],fanHorizontalImpulse:[...fanHorizontalImpulse],geometryMinimumCG:geometryMinimumCG(s.R,cfg),geometryClearance:s.pos[2]-geometryMinimumCG(s.R,cfg),contact:copy(contact),nozzleThrottle:[...throttle],nozzleForces:[...forces],fuel,gasLeft:fuel,fuelUsed,pressureScale,effectiveIsp,fan:copy(fan),goal:copy(goal),error:copy(error),allocation:copy(allocation),status,success,done,policyId,reward:{instant:lastRewardInstant,total:score,components:copy(lastRewardComponents)},metrics:{score,fuelUsed,fuelLeft:fuel,duration:data.time,turnProgress:turnBest,peakHeight,minimumApexCG:cfg.tasks[c.task].minimum_peak_cg_height??0,minimumHeightMet:peakHeight+1e-9>=(cfg.tasks[c.task].minimum_peak_cg_height??0),maxContactClosingSpeed:peakClosing,maxTotalContactForce:peakForce,stableSeconds:stable,bodyContact,hardContact,turnAngle:(memory.turnAngle??0)-(memory.initialRoll??0),turnComplete:memory.turnComplete??false,finalTilt:Math.acos(clamp(s.R[2][2],-1,1)),finalSpeed:norm(s.vel),finalLongitudinalSpeed:unrotate(s.R,s.vel)[0],finalLateralSpeed:unrotate(s.R,s.vel)[1],finalVerticalSpeed:s.vel[2],peakJetHorizontalForce,jetHorizontalImpulse:[...jetHorizontalImpulse],fanHorizontalImpulse:[...fanHorizontalImpulse],finalAngularSpeed:norm(s.omega),horizontalDisplacement:norm(sub(s.pos.slice(0,2),c.pos.slice(0,2)))},nozzles:nozzles.map((n,i)=>({...n,positionWorld:add(s.pos,rotate(s.R,n.position)),directionWorld:rotate(s.R,n.direction),force:forces[i],throttle:throttle[i]}))};
  }
  function reset(nextCase,nextParameters=params,nextOptions={}){
    assertLive();const next=copy(nextCase);next.omega??=[0,0,0];validateCase(next,cfg);
    const values=Array.isArray(nextParameters)?nextParameters:PARAMETER_NAMES.map(k=>nextParameters?.[k]);
    if(values.length!==PARAMETER_NAMES.length||!values.every(Number.isFinite))throw new Error('Invalid controller parameters');
    params=copy(nextParameters);c=next;policyId=nextOptions.policyId??options.policyId??'final';mj.mj_resetData(model,data);
    fuel=c.fuel??cfg.fuel.initial_kg;dryMass=(c.mass??cfg.vehicle.mass)-fuel;disabled=new Set(c.disabled_nozzles??[]);thrustScale=c.thrust_scale??1;mode=c.delivery_mode??cfg.fuel.delivery_mode;
    updateMass();data.qpos.set([...c.pos,...c.quat],0);data.qvel.set([...c.vel,...c.omega],0);data.qvel.fill(unrotate(qmat(c.quat),c.vel)[0]/cfg.vehicle.wheel_radii[0],6,10);
    throttle=Array(12).fill(0);forces=Array(12).fill(0);fuelUsed=0;({pressureScale,effectiveIsp}=delivery(fuel,cfg,mode));peakJetHorizontalForce=0;jetForceWorld=[0,0,0];jetHorizontalImpulse=[0,0];fanHorizontalImpulse=[0,0];fan=fanModel(c.pos,qmat(c.quat),0,cfg);
    contact={count:0,wheelCount:0,bodyContact:false,maxNormalForce:0,totalNormalForce:0,maxClosingSpeed:0,stableSeconds:0};allocation={requestedWrenchBody:Array(6).fill(0),requestedForceWorld:[0,0,0],achievedWrenchBody:Array(6).fill(0),residual:Array(6).fill(0),normalizedResidual:0};
    peakClosing=peakForce=stable=score=lastProgress=lastHeightProgress=turnBest=0;bodyContact=hardContact=lastStable=success=done=false;status='running';failedAt=null;goal={};error={};lastRewardInstant=0;lastRewardComponents={};pendingImpulse=[0,0,0];peakHeight=c.pos[2];initialTilt=Math.acos(clamp(qmat(c.quat)[2][2],-1,1));
    memory=createMemory(c,cfg);mj.mj_forward(model,data);return snapshot();
  }
  function step(steps=1){
    assertLive();if(!Number.isInteger(steps)||steps<0||steps>10000)throw new Error('Invalid control step count');
    for(let k=0;k<steps&&!done;k++){
      const s=state();if([...s.pos,...s.quat,...s.vel,...s.omega].some(v=>!Number.isFinite(v))){status='diverged';done=true;break;}
      const result=controller({...s,contact,fuel,thrustScale,deliveryMode:mode,disabledNozzles:[...disabled]},memory,params,cfg);memory=result.memory??memory;goal=copy(result.goal??memory.goal??{});error=copy(result.error??memory.error??{});
      let fanCommand=result.fanCommand??0,command;
      if(policyId==='no_control'){command=Array(12).fill(0);fanCommand=0;allocation.requestedForceWorld=[0,0,0];}
      else{command=result.nozzleThrottle;allocation={...copy(result.allocation),requestedForceWorld:[...result.desiredForceWorld]};}
      const turnAngle=memory.turn_angle??memory.turnAngle??goal.turnAngle??0,initialRoll=memory.initial_roll??memory.initialRoll??0,turnComplete=memory.turn_complete??memory.turnComplete??goal.turnComplete??false,wasAirborne=memory.was_airborne??memory.wasAirborne??false;
      const end=advance(command,fanCommand,{task:c.task,initialXY:c.pos.slice(0,2),turnComplete,wasAirborne}),l=cfg.landing,tilt=Math.acos(clamp(end.R[2][2],-1,1)),rate=norm(end.omega);
      if(failedAt===null)peakHeight=Math.max(peakHeight,end.pos[2]);hardContact||=contact.maxClosingSpeed>l.max_contact_closing_speed||contact.totalNormalForce>end.mass*cfg.physics.gravity*l.max_normal_force_weight_ratio;
      const stableNow=lastStable;
      const progress=c.task!=='moving_recovery'?Math.min(1,Math.max(0,(turnAngle-initialRoll)/(2*Math.PI))):Math.max(0,(initialTilt-tilt)/Math.max(initialTilt,.1));turnBest=Math.max(turnBest,progress);
      const minimumApex=cfg.tasks[c.task].minimum_peak_cg_height??0,heightProgress=minimumApex>0?clamp((peakHeight-c.pos[2])/Math.max(minimumApex-c.pos[2],.01),0,1):0;
      lastRewardComponents={heightProgress:120*(heightProgress-lastHeightProgress),taskProgress:100*(progress-lastProgress),fuel:-.03*forces.reduce((a,b)=>a+b,0)*controlDt/(effectiveIsp*cfg.physics.gravity),angularMotion:-.008*controlDt*rate,time:-.2*controlDt,stableContact:stableNow&&turnComplete?4*controlDt:0};lastRewardInstant=Object.values(lastRewardComponents).reduce((a,b)=>a+b,0);score+=lastRewardInstant;lastProgress=progress;lastHeightProgress=heightProgress;
      if(failedAt===null){
        if(bodyContact){status='body_contact';lastRewardComponents.terminal=-220;failedAt=data.time;}
        else if(hardContact){status='hard_contact';lastRewardComponents.terminal=-150-20*contact.maxClosingSpeed-Math.min(200,contact.totalNormalForce/(end.mass*cfg.physics.gravity));failedAt=data.time;}
        else if(norm(end.pos.slice(0,2))>180||end.pos[2]>40){status='out_of_bounds';lastRewardComponents.terminal=-100;failedAt=data.time;}
        else if(stable+1e-10>=l.stable_seconds&&turnComplete&&wasAirborne){
          if(peakHeight+1e-9>=minimumApex){status='success';success=true;lastRewardComponents.terminal=500;done=true;}
          else{status='height_shortfall';lastRewardComponents.terminal=-100;failedAt=data.time;}
        }
      }
      if('terminal' in lastRewardComponents){lastRewardInstant+=lastRewardComponents.terminal;score+=lastRewardComponents.terminal;}
      if(failedAt!==null&&data.time-failedAt>=.4)done=true;
      if(policyId==='no_control'&&stable>=1.2&&!wasAirborne){status='no_takeoff';done=true;}
      if(data.time>=cfg.physics.max_time-1e-9&&!done){status=fuel<.01?'fuel_exhausted':!wasAirborne?'no_takeoff':!turnComplete?'turn_incomplete':'not_stable';done=true;}
    }
    return snapshot();
  }
  function applyImpulse(impulse){assertLive();if(!Array.isArray(impulse)||impulse.length!==3||!impulse.every(Number.isFinite)||norm(impulse)>1e6)throw new Error('Invalid impulse');if(done)throw new Error('Reset a finished test before applying an impulse');pendingImpulse=add(pendingImpulse,impulse);}
  function dispose(){if(disposed)return;disposed=true;saved.delete();velocity.delete();contactForce.delete();data.delete();model.delete();}
  try{reset(testCase,parameters);}catch(e){dispose();throw e;}
  return {step,snapshot,applyImpulse,reset,dispose,advance,config:copy(cfg),engineVersion:mj.mj_versionString()};
}
