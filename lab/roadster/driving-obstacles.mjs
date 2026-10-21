// Static jump targets. X/Cybertruck envelopes follow Tesla owner manuals; Semi's 7.8 × 2.5 × 3.7 m envelope is assumed.
export const JUMP_LANE={y:16,start:-230,end:220,takeoff:-17,landingStart:50,halfWidth:5};
export const PARKED_VEHICLES=[
 {id:'modelx',label:'MODEL X',x:4,y:15.4715,yaw:Math.PI/2,length:5.057,width:1.999,height:1.68,axles:[-1.4245,1.5405],wheelRadius:.38,wheelHalfWidth:.13,wheelTrack:.86,
  hulls:[{material:'white',smooth:true,sections:[[-2.5285,.76,.38,.82],[-2.30,.94,.30,1.02],[-1.5,.9995,.30,1.08],[.6,.9995,.30,1.06],[1.8,.94,.34,.93],[2.35,.80,.40,.80],[2.5285,.69,.45,.67]]},
   {material:'glass',smooth:true,sections:[[-2.12,.72,.92,1.06],[-1.65,.80,.95,1.42],[-.9,.80,.95,1.67],[.25,.79,.95,1.68],[.9,.76,.95,1.48],[1.55,.83,.95,1.04]]}]},
 {id:'semi',label:'TESLA SEMI',x:10,y:14.1,yaw:Math.PI/2,length:7.8,width:2.5,height:3.7,axles:[-2.95,-1.55,2.65],wheelRadius:.51,wheelHalfWidth:.19,wheelTrack:1.06,
  hulls:[{material:'dark',sections:[[-3.9,1.13,.48,1.03],[3.9,1.13,.48,1.03]]},
   {material:'white',smooth:true,sections:[[-.65,1.24,.66,3.55],[.20,1.25,.60,3.7],[1.75,1.24,.55,3.7],[3.05,1.12,.55,3.15],[3.9,.96,.55,1.6]]}]},
 {id:'cybertruck',label:'CYBERTRUCK',x:16,y:15.15855,yaw:Math.PI/2,length:5.6829,width:2.0316,height:1.794,axles:[-1.6718,1.9632],wheelRadius:.43,wheelHalfWidth:.145,wheelTrack:.9,
  hulls:[{material:'steel',sections:[[-2.84145,1.0158,.38,1.10],[-.7,1.0158,.27,1.60],[.35,1.0158,.27,1.794],[2.84145,.96,.42,1.04]]}]}
];

// The same closed hulls feed the renderer and MuJoCo, so roofs cannot become invisible collision boxes.
export function vehicleHull({sections}){
 const vertices=[],faces=[],n=8;
 for(const [x,w,b,t] of sections){const bevel=Math.min(.08,(t-b)*.2);for(const [y,z] of [[-w*.86,b],[w*.86,b],[w,b+bevel],[w,t-bevel],[w*.86,t],[-w*.86,t],[-w,t-bevel],[-w,b+bevel]])vertices.push(x,y,z);}
 for(let s=0;s<sections.length-1;s++)for(let i=0;i<n;i++){const a=s*n+i,b=s*n+(i+1)%n,c=b+n,d=a+n;faces.push(a,b,c,a,c,d);}
 for(let i=1;i<n-1;i++){faces.push(0,i+1,i);const end=(sections.length-1)*n;faces.push(end,end+i,end+i+1);}
 return {vertices,faces};
}
export function parkedVehiclesXML(){
 const assets=[],geoms=[];
 for(const car of PARKED_VEHICLES){
  car.hulls.forEach((h,i)=>{const name=`parked_${car.id}_${i}`,{vertices,faces}=vehicleHull(h);assets.push(`<mesh name="${name}" vertex="${vertices.join(' ')}" face="${faces.join(' ')}"/>`);geoms.push(`<geom name="${name}" type="mesh" mesh="${name}" pos="${car.x} ${car.y} 0" quat="${Math.cos(car.yaw/2)} 0 0 ${Math.sin(car.yaw/2)}"/>`);});
  car.axles.forEach((x,i)=>{for(const side of [-1,1]){const y=side*car.wheelTrack,wx=car.x+x*Math.cos(car.yaw)-y*Math.sin(car.yaw),wy=car.y+x*Math.sin(car.yaw)+y*Math.cos(car.yaw);geoms.push(`<geom name="parked_${car.id}_wheel_${i}_${side===1?'l':'r'}" type="ellipsoid" pos="${wx} ${wy} ${car.wheelRadius}" quat="${Math.cos(car.yaw/2)} 0 0 ${Math.sin(car.yaw/2)}" size="${car.wheelRadius} ${car.wheelHalfWidth} ${car.wheelRadius}"/>`);}});
 }
 return {assets:`<asset>${assets.join('')}</asset>`,geoms:geoms.join('')};
}
