// Fixed attitude preview: same 2 ms valve/gas loop and 20 ms mass updates as Python.
import {nozzleLayout,delivery,allocateFixedNozzles} from './baseline_r6/controller.mjs';
import {mv,tmv,mul,wrenchMatrix} from './math.mjs';

export function targetUpwardThrottles(state,cfg,fraction=.85){
  const nozzles=nozzleLayout(cfg),A=wrenchMatrix(nozzles),disabled=new Set(state.disabledNozzles??[]),fmax=nozzles.map(n=>disabled.has(n.id)?0:n.maxForce*(state.thrustScale??1));
  const atUnit=allocateFixedNozzles([...tmv(state.R,[0,0,1e6]),0,0,0],fmax,cfg),unitLift=mv(state.R,mv(A,atUnit).slice(0,3))[2];
  return {target:atUnit.map((f,i)=>fmax[i]>0?f*fraction/fmax[i]:0),fmax,A,unitLift};
}
export function predictVerticalStop(state,cfg,{fraction=.85,touchdownSpeed=.2,horizon=6}={}){
  const dt=cfg.physics.dt,window=Math.round(cfg.physics.control_dt/dt),gravity=cfg.physics.gravity,dry=state.mass-state.fuel,initialFuel=state.fuel,R=state.R;
  const {target,fmax,A,unitLift}=targetUpwardThrottles(state,cfg,fraction),response=1-Math.exp(-dt/cfg.actuators.lag_seconds);
  let velocity=state.vel[2],fuel=state.fuel,u=[...(state.actualNozzleThrottle??Array(12).fill(0))],distance=0,elapsed=0,mass=dry+fuel,forceZ=0;
  if(velocity>=-touchdownSpeed){const pressure=delivery(fuel,cfg).pressureScale,ratio=unitLift*pressure/(mass*gravity);return {feasible:true,distance:0,time:0,gasUsed:0,endFuel:fuel,endPressure:pressure,maximumZeroMomentLiftWeightRatio:ratio,targetLiftWeightRatio:fraction*ratio,endLiftWeightRatioMeaning:'legacy full zero-moment capacity, not actual thrust',endLiftWeightRatio:ratio,dt,massUpdateInterval:cfg.physics.control_dt};}
  for(let step=0;step<Math.round(horizon/dt);step++){
    if(step%window===0)mass=dry+fuel;
    u=u.map((v,i)=>v+(target[i]-v)*response);
    const {pressureScale:pressure,effectiveIsp:isp}=delivery(fuel,cfg),cap=Math.min(cfg.actuators.total_force_max*pressure*(state.thrustScale??1),fuel*isp*gravity/dt);
    let forces=u.map((v,i)=>v*fmax[i]*pressure),total=forces.reduce((sum,x)=>sum+x,0);
    if(total>cap)forces=mul(forces,cap/Math.max(total,1e-12));
    fuel=Math.max(0,fuel-forces.reduce((sum,x)=>sum+x,0)*dt/(isp*gravity));forceZ=mv(R,mv(A,forces).slice(0,3))[2];velocity+=(forceZ/mass-gravity)*dt;distance-=velocity*dt;elapsed=(step+1)*dt;
    if(velocity>=-touchdownSpeed)break;
  }
  const pressure=delivery(fuel,cfg).pressureScale,ratio=unitLift*pressure/Math.max((dry+fuel)*gravity,1);
  return {feasible:velocity>=-touchdownSpeed,distance,time:elapsed,gasUsed:initialFuel-fuel,endFuel:fuel,endPressure:pressure,maximumZeroMomentLiftWeightRatio:ratio,targetLiftWeightRatio:fraction*ratio,lastPredictedActualLiftWeightRatio:forceZ/(mass*gravity),endLiftWeightRatioMeaning:'legacy full zero-moment capacity, not actual thrust',endLiftWeightRatio:ratio,predictedFinalVz:velocity,dt,massUpdateInterval:cfg.physics.control_dt,assumption:'Fixed attitude, fixed zero-moment upward throttle target. Exact valve/flow/pressure and 20ms mass schedule; does not predict changing attitude or additional future attitude commands.'};
}
