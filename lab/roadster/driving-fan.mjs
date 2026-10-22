import {cross,mv} from './simulation_v3/math.mjs';

// shortcut: Shared 180 kW and 150 ms response are assumptions; replace after fan/duct calibration.
export const DRIVING_FAN={maxElectricalPower:180000,responseSeconds:.15};
export function fanExhaust(power,s,cfg){
 if(!Number.isFinite(power)||power<0||power>DRIVING_FAN.maxElectricalPower+1e-6)throw Error('Invalid fan electrical power');
 const f=cfg.fan,angle=f.exhaust_up_tilt_deg*Math.PI/180,thrust=(power*f.efficiency*Math.sqrt(2*f.air_density*f.disk_area))**(2/3);
 const forceBody=[thrust*Math.cos(angle),0,-thrust*Math.sin(angle)],center=s.massProperties?.bodyIpos??[0,0,0],arm=f.position.map((v,i)=>v-center[i]),torqueBody=cross(arm,forceBody);
 return {power,forceBody,torqueBody,forceWorld:mv(s.R,forceBody),torqueWorld:mv(s.R,torqueBody)};
}
export function allocateRecoveryFan(s,cfg,torque,forceZ,allocate){
 const power=s.airFan?.power??0,decay=Math.exp(-cfg.physics.control_dt/(2*DRIVING_FAN.responseSeconds));
 function proposal(command){
  const fan=fanExhaust(power*decay+command*DRIVING_FAN.maxElectricalPower*(1-decay),s,cfg),jets=allocate(torque.map((v,i)=>v-fan.torqueBody[i]),forceZ-fan.forceWorld[2]);
  return {...jets,fan,fanPowerFraction:command,combinedForceWorld:jets.forceWorld.map((v,i)=>v+fan.forceWorld[i]),combinedWrenchBody:jets.achievedWrenchBody.map((v,i)=>v+(i<3?fan.forceBody[i]:fan.torqueBody[i-3]))};
 }
 const coast=proposal(0),full=fanExhaust(DRIVING_FAN.maxElectricalPower,s,cfg),forward=coast.forces.slice(4,8).reduce((a,b)=>a+b,0);
 // Tail-down pitch and existing rearward-jet demand must both need this fixed exhaust direction.
 const eligible=s.airFan?.enabled&&s.geometryClearance>1&&torque[1]<-50&&forward>100&&full.torqueBody[1]<0;
 if(!eligible)return coast;
 const command=Math.min(1,-torque[1]/-full.torqueBody[1],forward/full.forceBody[0])**1.5,candidate=proposal(command);
 const gas=a=>a.forces.reduce((sum,v)=>sum+v,0);
 return gas(candidate)<gas(coast)-1&&candidate.weightedResidual<=coast.weightedResidual+.02?candidate:coast;
}
