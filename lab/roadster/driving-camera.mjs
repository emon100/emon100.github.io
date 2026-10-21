const clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),mix=(a,b,t)=>a+(b-a)*t,angle=(a,b)=>Math.atan2(Math.sin(b-a),Math.cos(b-a));
function rotation(a,b,t){let d=a.reduce((sum,v,i)=>sum+v*b[i],0),sign=d<0?-1:1;d=Math.min(1,Math.abs(d));if(d>.9995){const q=a.map((v,i)=>mix(v,b[i]*sign,t)),n=Math.hypot(...q);return q.map(v=>v/n);}const theta=Math.acos(d),divisor=Math.sin(theta);return a.map((v,i)=>(v*Math.sin((1-t)*theta)+b[i]*sign*Math.sin(t*theta))/divisor);}
export function createChaseFollow(){
 let target=null,yaw=0,roadHeading=0;
 return {reset(){target=null;},update(f,dt){
  const upright=1-2*(f.quat[1]**2+f.quat[2]**2),speed=Math.hypot(f.vel[0],f.vel[1]),grounded=f.mode!=='air'&&upright>.55;
  if(!target||grounded)roadHeading=f.heading??0;
  let desired=roadHeading;
  if(grounded&&speed>1.5){const direction=f.gear==='R'?-1:1,travel=Math.atan2(f.vel[1]*direction,f.vel[0]*direction);desired+=clamp(angle(roadHeading,travel),-.9,.9)*clamp((speed-1.5)/5,0,.65);}
  const aim=f.pos.map((v,i)=>i===2?v-.4:v+clamp(f.vel[i]*.06,-1,1));
  if(!target){target=[...aim];yaw=desired;}else{for(let i=0;i<3;i++)target[i]=mix(target[i],aim[i],1-Math.exp(-dt/(i===2?.16:.06)));yaw+=angle(yaw,desired)*(1-Math.exp(-dt/.28));}
  return {cameraTarget:[...target],cameraHeading:yaw};
 }};
}
export function createDrivingView(){
 let frames=[],playhead=0,lastTime=null;const chase=createChaseFollow();
 return {push(f,now){const last=frames.at(-1);if(!last||f.resetSerial!==last.resetSerial||f.t<last.t){frames=[];playhead=f.t-.04;lastTime=now;chase.reset();}if(frames.at(-1)?.t===f.t)frames[frames.length-1]=f;else frames.push(f);if(frames.length>10)frames.shift();},
  sample(now){if(!frames.length)return null;const elapsed=Math.max(0,(now-lastTime)/1000),dt=Math.min(.05,elapsed),latest=frames.at(-1);lastTime=now;
   if(elapsed>.25||latest.t-playhead>.18){playhead=latest.t-.04;chase.reset();}else playhead=Math.min(latest.t,playhead+elapsed);
   while(frames.length>2&&frames[1].t<=playhead)frames.shift();const a=frames[0],b=frames[1]??a,t=b.t===a.t?1:clamp((playhead-a.t)/(b.t-a.t),0,1),f={...latest,t:mix(a.t,b.t,t),quat:rotation(a.quat,b.quat,t),heading:(a.heading??0)+angle(a.heading??0,b.heading??0)*t};
   for(const key of ['pos','vel','wheelAngles','wheelSteering'])if(a[key]&&b[key])f[key]=a[key].map((v,i)=>mix(v,b[key][i],t));
   return {...f,...chase.update(f,dt)};
  }
 };
}
