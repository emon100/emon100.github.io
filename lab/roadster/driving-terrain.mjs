import {DRIVE} from './driving-powertrain.mjs?v=20261022-dynamics';
import {parkedVehiclesXML} from './driving-obstacles.mjs?v=20261022-rocket10';
import {qmat,qmul} from './simulation_v4/controller.mjs';
export const MAP={halfX:270,halfY:216,edgeMargin:2.2};
export const RAMP_THICKNESS=.6;
export const RAMPS=[
 {id:'clockwise',label:'顺时针',x:40,y:-54,approach:30,length:18,bank:.8,color:0xe5b77b},
 {id:'counterclockwise',label:'逆时针',x:40,y:54,approach:30,length:18,bank:-.8,color:0x8acbde},
 {id:'vertical',label:'90°',x:-64,y:-54,approach:42,length:6*Math.PI/2+8,angle:Math.PI/2,color:0xc9e78a},
 {id:'ceiling',label:'170°',x:14,y:0,approach:62,radius:9,length:9*170*Math.PI/180+8,angle:170*Math.PI/180,color:0xc3a9ed}
];
const runway=RAMPS.find(r=>r.id==='ceiling');
export const SPAWN=[runway.x-runway.approach+5,runway.y,.5];
const axis=(v,a)=>[Math.cos(a/2),...v.map(x=>x*Math.sin(a/2))];
export function rampPose(r,s){
 const radius=r.radius??6,u=Math.max(0,Math.min(1,s/(r.angle?radius*r.angle:r.length))),theta=r.angle?u*r.angle:Math.atan(8*u/18),roll=r.bank?u*u*r.bank:0,quat=qmul(axis([0,1,0],-theta),axis([1,0,0],roll)),R=qmat(quat),tail=r.angle?Math.max(0,Math.min(s,r.length)-radius*r.angle):0;
 const surface=r.angle?[r.x+radius*Math.sin(theta)+tail*Math.cos(theta),r.y,radius*(1-Math.cos(theta))+tail*Math.sin(theta)]:[r.x+s,r.y,4*u*u];return {surface,quat,normal:R.map(row=>row[2])};
}
export const RAMP_PANELS=RAMPS.flatMap(r=>Array.from({length:80},(_,i)=>{const a=rampPose(r,r.length*i/80),b=rampPose(r,r.length*(i+1)/80),p=rampPose(r,r.length*(i+.5)/80),R=qmat(p.quat);return {...p,id:`ramp_${r.id}_${i}`,ramp:r,forward:R.map(row=>row[0]),side:R.map(row=>row[1]),halfLength:Math.hypot(...a.surface.map((v,k)=>b.surface[k]-v))*.56};}));
export function rampMeshData(r){
 const vertices=[],indices=[],quad=(a,b,c,d)=>indices.push(a,b,c,a,c,d);
 for(let i=0;i<=80;i++){const p=rampPose(r,r.length*i/80),R=qmat(p.quat);for(const depth of [0,RAMP_THICKNESS])for(const side of [-1,1])vertices.push(...p.surface.map((v,k)=>v+R[k][1]*side*3-p.normal[k]*depth));if(i<80){const a=i*4,b=a+4;quad(a,b,b+1,a+1);quad(a+2,a+3,b+3,b+2);quad(a,a+2,b+2,b);quad(a+1,b+1,b+3,a+3);}}
 quad(0,1,3,2);quad(320,322,323,321);return {vertices,indices};
}
const dot=(a,b)=>a.reduce((v,x,i)=>v+x*b[i],0);
// Short underside probes share the collision panels; they cannot attach a car across open air.
export function probeRoad(origin,up,maxDistance){
 let hit=null,nearest=maxDistance;
 if(up[2]>.85){const distance=origin[2]/up[2];if(distance>=0&&distance<nearest){nearest=distance;hit={distance,normal:[0,0,1],onRamp:false};}}
 for(const p of RAMP_PANELS){if(Math.abs(origin[1]-p.ramp.y)>5||Math.abs(origin[0]-p.surface[0])>5||Math.abs(origin[2]-p.surface[2])>3)continue;const aligned=dot(up,p.normal);if(aligned<.85)continue;const delta=origin.map((v,i)=>v-p.surface[i]),distance=dot(delta,p.normal)/aligned;if(distance<0||distance>=nearest)continue;const at=delta.map((v,i)=>v-up[i]*distance);if(Math.abs(dot(at,p.forward))>p.halfLength||Math.abs(dot(at,p.side))>3)continue;nearest=distance;hit={distance,normal:p.normal,onRamp:true};}
 return hit;
}
export function drivingXML(xml){
 const pieces=RAMP_PANELS.map(p=>`<geom name="${p.id}" type="box" pos="${p.surface.map((v,k)=>v-p.normal[k]*RAMP_THICKNESS/2).join(' ')}" quat="${p.quat.join(' ')}" size="${p.halfLength} 3 ${RAMP_THICKNESS/2}"/>`);
 const parked=parkedVehiclesXML();
 let model=xml.replace('<worldbody>',parked.assets+'<worldbody>'+pieces.join('')+parked.geoms);
 // Apply the same dry-tire assumption to contact physics and wheel-torque limiting.
 model=model.replace(/<geom name="wheel([0-3])"/g,`<geom name="wheel$1" friction="${DRIVE.tireGrip} 0.005 0"`);
 for(const i of [2,3])model=model.replace(new RegExp(`<body name="wheelbody${i}" pos="([^"]+)">([\\s\\S]*?)</body>`),(_,pos,content)=>`<body name="steeringhub${i}" pos="${pos}"><inertial pos="0 0 0" mass="1" diaginertia=".01 .01 .01"/><joint name="steer${i}" type="hinge" axis="0 0 1" damping="1800" armature=".5" limited="true" range="-.8 .8"/><body name="wheelbody${i}">${content.replace('mass="20"','mass="19"')}</body></body>`);
 return model.replace('</worldbody>','</worldbody><contact><exclude body1="car" body2="wheelbody2"/><exclude body1="car" body2="wheelbody3"/></contact>');
}
