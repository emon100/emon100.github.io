const clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0);
// Calibrated sandbox assumptions; Tesla publishes acceleration targets, not tire or brake curves.
export const DRIVE={wheelRadius:.32,tireGrip:1.8,peakForce:46000,peakPower:1000000,regenForce:6000,regenPower:120000,brakeForce:40000,reverseSpeed:6,dragArea:.65,rollingResistance:.012};
export function wheelDrive(s,contact,{pedal=0,brake=false,attached=false},cfg){
 const v=dot(s.vel,s.R.map(row=>row[0])),loads=contact.averageWheelLoads??[0,0,0,0],loadSum=loads.reduce((a,b)=>a+b,0),p=Math.abs(pedal),r=DRIVE.wheelRadius;
 const hill=attached?s.mass*cfg.physics.gravity*s.R[2][0]:0,hold=p<.01&&Math.abs(v)<.25;
 const motor=Math.sign(pedal)*DRIVE.peakForce*p**1.5*(pedal<0?clamp((DRIVE.reverseSpeed+1+v)/2,0,1):1);
 const regen=-clamp(v/.6,-1,1)*Math.min(DRIVE.regenForce,DRIVE.regenPower/Math.max(1,Math.abs(v)))*(1-p)**2;
 const local=s.R[0].map((_,k)=>s.R.reduce((n,row,j)=>n+row[k]*s.vel[j],0));
 let motorPower=0,regenPower=0,brakePower=0;
 const torques=s.wheelRates.map((rate,i)=>{
  const load=Math.max(0,loads[i]),share=loadSum>1?load/loadSum:0,delta=s.wheelSteering[i]??0,x=(i<2?-1:1)*cfg.vehicle.axle_half,y=(i%2?1:-1)*cfg.vehicle.track_half;
  const rolling=(local[0]-s.omega[2]*y)*Math.cos(delta)+(local[1]+s.omega[2]*x)*Math.sin(delta),slip=rate*r-rolling;
  const requested=(brake||hold)?hill*r*share-clamp(rate*240,-DRIVE.brakeForce*r*share,DRIVE.brakeForce*r*share):(hill+motor+regen)*r*share;
  const powerLimit=(brake||hold)?Infinity:((requested*rate>=0?DRIVE.peakPower:DRIVE.regenPower)*share/Math.max(5,Math.abs(rate)));
  let torque=clamp(requested,-Math.min(DRIVE.peakForce*r*share,powerLimit),Math.min(DRIVE.peakForce*r*share,powerLimit));
  // Use measured tire loads, not fan suction. MuJoCo resolves the combined lateral/longitudinal friction cone.
  const grip=DRIVE.tireGrip*load*r*.95;torque=clamp(torque,-grip,grip);
  if(Math.abs(rolling)>.5&&Math.sign(torque)*slip>.6)torque*=clamp(1-(Math.sign(torque)*slip-.6)/2,0,1);
  const power=torque*rate;if(power>0)motorPower+=power;else if(brake||hold)brakePower-=power;else regenPower-=power;
  return torque-Math.tanh(rate*r/.3)*DRIVE.rollingResistance*load*r;
 });
 const speed=Math.hypot(...s.vel),drag=s.vel.map(v=>-.5*1.225*DRIVE.dragArea*speed*v);
 return {torques,drag,motorPower,regenPower,brakePower,hold};
}
