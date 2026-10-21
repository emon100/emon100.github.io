import * as k from '../kit/iso-kit.mjs';
const pi=Math.PI, clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export const sceneNames=['弯坡入口','过垂直弯坡接触','倒悬释放','姿态回正','坡道着陆','平地起跳','越过车列','Semi 上空倒转','滚转回正','越车后着陆'];
const lerp=(a,b,t)=>a+(b-a)*t;
const progress=p=>clamp(Number(p)||0,0,1);
function sceneNumber(value){const n=Number(value);return Number.isNaN(n)?1:clamp(Math.round(n),1,10);}
const TRACK_R=72,TRACK_X=-70,TRACK_Z=73,WHEEL_R=7,AXLE_X=19;
const TRACK_ORIGIN=Math.sqrt((TRACK_R-WHEEL_R)**2-AXLE_X**2);
// Editorial teaching poses, not measured telemetry. Roll is about the 3D longitudinal axis.
function rampPose(a){return {x:TRACK_X+TRACK_ORIGIN*Math.sin(a),z:TRACK_Z-TRACK_ORIGIN*Math.cos(a),pitch:a,roll:0};}
export function poseAt(scene,p){
 scene=sceneNumber(scene);p=progress(p);const t=p*p*(3-2*p);
 if(scene===1)return rampPose(lerp(.1,1.25,t));
 if(scene===2)return rampPose(lerp(1.25,2.75,t));
 if(scene===3){const start=rampPose(2.75);return {x:lerp(start.x,-67,t),z:lerp(start.z,80,t),pitch:lerp(2.75,pi,t),roll:0};}
 if(scene===4)return {x:lerp(-67,-110,t),z:lerp(80,65,t),pitch:pi,roll:pi*t};
 if(scene===5)return {x:lerp(-110,-140,t),z:lerp(65,8,t),pitch:pi,roll:pi};
 const q=scene-6;
 const x=lerp([-180,-135,100,135,155][q],[-135,100,135,155,175][q],t);
 const z=lerp([8,75,108,108,65][q],[75,108,108,65,8][q],t);
 const roll=lerp([0,0,pi*.65,pi*1.35,2*pi][q],[0,pi*.65,pi*1.35,2*pi,2*pi][q],t);
 return {x,z,pitch:0,roll};
}
export function transform(v,o){const [x,y,z]=v,c=Math.cos(o.roll),s=Math.sin(o.roll),yy=y*c-z*s,zz=y*s+z*c,cp=Math.cos(o.pitch),sp=Math.sin(o.pitch);return [o.x+x*cp-zz*sp,yy,x*sp+zz*cp+o.z];}
function normal(a,b,c){const u=b.map((x,i)=>x-a[i]),v=c.map((x,i)=>x-a[i]);return [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];}
const mean=p=>p[0].map((_,i)=>p.reduce((s,v)=>s+v[i],0)/p.length);
function box(x,y,z,w,d,h){const v=[[x,y,z],[x+w,y,z],[x+w,y+d,z],[x,y+d,z],[x,y,z+h],[x+w,y,z+h],[x+w,y+d,z+h],[x,y+d,z+h]];return [[0,3,2,1],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[4,5,6,7]].map(f=>f.map(i=>v[i]));}
function prism(ring,y,d){const a=ring.map(([x,z])=>[x,y,z]),b=ring.map(([x,z])=>[x,y+d,z]);return [a,b.toReversed(),...a.map((v,i)=>[v,b[i],b[(i+1)%a.length],a[(i+1)%a.length]])];}
const ringXZ=(x,z,r,steps)=>Array.from({length:steps},(_,i)=>[x+r*Math.cos(i*2*pi/steps),z+r*Math.sin(i*2*pi/steps)]);
function wheel(x,y,z=0,r=7,depth=4,steps=24){return prism(ringXZ(x,z,r,steps),y,depth);}
function cylinderBetween(a,b,r,steps=16){
 const delta=b.map((v,i)=>v-a[i]),length=Math.hypot(...delta);if(length<1e-6||r<=0)return [];
 const u=delta.map(v=>v/length),seed=Math.abs(u[0])<.9?[1,0,0]:[0,1,0];
 const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
 const v0=cross(u,seed),vl=Math.hypot(...v0),v=v0.map(n=>n/vl),w=cross(v,u);
 const ring=c=>Array.from({length:steps},(_,i)=>c.map((n,j)=>n+r*(Math.cos(i*2*pi/steps)*v[j]+Math.sin(i*2*pi/steps)*w[j])));
 const start=ring(a),end=ring(b);return [start,end.toReversed(),...start.map((v,i)=>[v,end[i],end[(i+1)%steps],start[(i+1)%steps]])];
}
function carParts(){const a=[];const B=(...p)=>a.push({faces:box(...p.slice(0,6)),tone:p[6]||'hi'});
 B(-26,-11,-2,54,22,5,'mid'); B(-29,-12,0,58,24,5); B(13,-12,5,16,24,3); B(-28,-12,5,12,24,6);
 B(-26,-13,-1,52,2,6,'mid');B(-26,11,-1,52,2,6,'mid');
 a.push({faces:prism([[-15,5],[12,5],[6,16],[-9,16]],-9,18),tone:'hi'});
 B(-8,-8,16,13,16,.7,'glass');
 for(const y of [-9.3,9])a.push({faces:prism([[-13,7],[10,7],[5,14],[-8,14]],y,.3),tone:'glass'});
 B(-30,-14,9,3,28,2);B(28,-13,1,2,26,1,'mid');for(const y of [-8,7])B(-28,y,6,2,1,3,'mid');
 for(const y of [-9,5]){B(28,y,5,1,4,1,'lit');B(-28,y,9,1,4,1,'mid');}
 for(const x of [-19,19])for(const y of [-15,11]){
  a.push({faces:wheel(x,y),tone:'rubber'});
  const outside=y<0?y-.4:y+4;a.push({faces:wheel(x,outside,0,4.8,.4,16),tone:'mid'});
  B(x-1,outside-.1,-1,2,.7,2,'hi');B(x-3,y+1,-3,6,2,6,'lo');
 }
 for(const x of [-26,23])for(const y of [-12,9]){B(x,y,-4,4,3,2,'mid');B(x+.8,y+.6,-4.5,2.4,1.8,.5,'rubber');}
 return a;
}
const CAR=carParts();
function referenceCar(x,y){const a=[];const B=(...p)=>a.push({faces:box(...p.slice(0,6)),tone:p[6]||'mid'});
 B(x+2,y-11,5,31,22,3,'lo');B(x,y-12,8,35,24,7,'mid');
 a.push({faces:prism([[x+7,15],[x+28,15],[x+24,23],[x+13,23]],y-9,18),tone:'mid'});
 for(const yy of [y-9.35,y+9])a.push({faces:prism([[x+11,17],[x+25,17],[x+23,21],[x+14,21]],yy,.35),tone:'glass'});
 B(x-1,y-11,9,2,22,2,'lo');B(x+34,y-11,9,2,22,2,'lo');
 B(x+35,y-8,11,.6,4,2,'hi');B(x+35,y+4,11,.6,4,2,'hi');
 for(const wx of [x+7,x+28]){
  B(wx-1,y-12,5.4,2,24,1.2,'lo');
  for(const side of [-1,1]){const wy=y+(side<0?-15:12),outside=side<0?wy-.35:wy+3;
   a.push({faces:wheel(wx,wy,6,5,3,20),tone:'rubber'});a.push({faces:wheel(wx,outside,6,2.5,.35,12),tone:'mid'});
  }
 }
 return a;
}
function semiParts(){const a=[];const B=(...p)=>a.push({faces:box(...p.slice(0,6)),tone:p[6]||'mid'});
 B(103,-14,8,44,28,4,'lo');B(100,-17,12,50,34,27,'mid');B(97,-9,8,7,18,4,'lo');
 B(87,-17,8,13,34,23,'lo');B(88,-16,31,11,32,1,'mid');B(86.6,-15,22,.6,30,7,'glass');
 B(92,-17.35,22,5,.35,6,'glass');B(92,17,22,5,.35,6,'glass');B(149,-16,8,2,32,3,'lo');
 for(const wx of [94,132,144]){
  B(wx-1.2,-17,6.4,2.4,34,1.6,'lo');
  for(const side of [-1,1]){const wy=side<0?-20:17,outside=side<0?wy-.4:wy+3;
   a.push({faces:wheel(wx,wy,7,6,3,20),tone:'rubber'});a.push({faces:wheel(wx,outside,7,3,.4,12),tone:'mid'});
  }
 }
 return a;
}
function controlParts(scene,p){
 if(scene!==4&&scene!==9)return [];p=progress(p);const a=[];
 const B=(x,y,z,w,d,h,tone='mid')=>a.push({faces:box(x,y,z,w,d,h),tone});
 for(const side of [-1,1]){
  B(-3,side<0?-15:13,-2,6,2,4);
  for(const dir of [-1,1])a.push({faces:cylinderBetween([0,side*14,dir*1.2],[0,side*14,dir*3.5],1.3,12),tone:'mid'});
 }
 const half=p<.5?0:1,local=p*2-half,envelope=Math.sin(pi*local),sign=half? -1:1;
 if(envelope>.001)for(const side of [-1,1]){
  const dir=-side*sign;
  for(let i=0;i<3;i++){
   const start=3.5+i*4+(i?local*2:0),length=2.6*envelope,radius=1.05*envelope;
   a.push({faces:cylinderBetween([0,side*14,dir*start],[0,side*14,dir*(start+length)],radius,16),tone:'gas'});
  }
 }
 return a;
}
export function mount(container,{scene=1,theme='dark',onReadout=()=>{}}={}){
 scene=sceneNumber(scene);const hadIso=container.classList.contains('iso'),oldTheme=container.dataset.theme;
 container.classList.add('iso');container.dataset.theme=theme==='light'?'light':'dark';
 const P=k.fitProjection([[-230,-68,-15],[210,68,0],[0,0,180]],640,370,{pad:30,azimuth:140,elevation:28});
 const solids=[],add=(x,y,z,w,d,h,tone='mid')=>solids.push({faces:box(x,y,z,w,d,h),tone});
 add(-230,-65,-9,440,130,9,'lo');for(const x of [-212,192])for(const y of [-50,38])add(x,y,-15,12,12,6,'lo');
 add(-222,-40,0,424,80,1,'mid');for(const y of [-58,54])add(-220,y,0,420,4,2,'mid');
 for(let i=0;i<20;i++)add(-210+i*20,54,2,.8,i%5===0?4:2,.3,'hi');
 for(const x of [-215,-75,65,195])for(const y of [-50,49])add(x,y,0,2,2,.5,'hi');
 const floorCount=solids.length;
 if(scene<=5){
  for(let i=0;i<28;i++){const a=i*pi/30,b=(i+1)*pi/30;const ring=[[-70+72*Math.sin(a),1+72*(1-Math.cos(a))],[-70+72*Math.sin(b),1+72*(1-Math.cos(b))],[-70+76*Math.sin(b),1+72-76*Math.cos(b)],[-70+76*Math.sin(a),1+72-76*Math.cos(a)]];solids.push({faces:prism(ring,-22,44),tone:'mid'});}
  for(const [x,z] of [[-50,12],[-2,65],[-15,120]])for(const y of [-27,23]){add(x,y,1,4,4,z,'lo');add(x-5,y-3,0,14,10,1,'mid');}
 }else{
  for(const x of [-100,-50,0,50])solids.push(...referenceCar(x,0));
  solids.push(...semiParts());
 }
 container.innerHTML=`<svg viewBox="0 0 640 370" role="img" aria-label="${sceneNames[scene-1]}，微缩场景示意，不是现场轨迹测量；车辆尺寸与冷气体位置均为教学重构"><g data-world></g></svg>`;
 const svg=container.querySelector('svg'),layer=svg.querySelector('[data-world]');
 const c=k.cameraOf(P),viewer=[c.cosA*c.cosE,c.sinA*c.cosE,c.sinE];
 const palettes={hi:['#777c81','#94999e','#b9bdc1','#e4e7e8'],mid:['#292d32','#363b41','#4c5259','#646b72'],lo:['#171b20','#23282f','#30363e','#3e4650'],glass:['#111820','#1b2731','#2a3a45','#3b4d58'],rubber:['#101214','#161a1e','#20262b','#30363b'],lit:['#b7bfbe','#d8dfdf','#e7eeee','#ffffff'],gas:['#3a4147','#59636a','#7c878d','#a7b0b4']};
 const light={hi:['#a6abad','#bbc0c1','#d5d8d8','#f0f1ef'],mid:['#858b8f','#a1a7aa','#b7bdbf','#d0d5d5'],lo:['#a4aaad','#bbc1c3','#d2d7d7','#e5e8e6'],glass:['#33414a','#435866','#596c77','#72818a'],rubber:palettes.rubber,lit:['#aeb4b6','#c3cbcc','#e7eeee','#fff'],gas:['#6c767c','#899398','#a8b0b4','#c3c9cb']};
 function project(parts,o){const list=[];for(const part of parts)for(const face0 of part.faces){const face=o?face0.map(v=>transform(v,o)):face0,n=normal(...face.slice(0,3)),len=Math.hypot(...n);if(len===0||n.reduce((s,v,i)=>s+v*viewer[i],0)<=0)continue;const unit=n.map(v=>v/len),shade=clamp(Math.round((unit[2]*.7+unit[0]*-.3+unit[1]*-.4+1)*1.5),0,3);list.push({d:k.pathOf(face.map(v=>k.iso(v,P)),true),depth:k.depthOf(mean(face),P),tone:part.tone,shade});}return list;}
 const floorFaces=project(solids.slice(0,floorCount)).sort((a,b)=>a.depth-b.depth),fixed=project(solids.slice(floorCount)),report=typeof onReadout==='function'?onReadout:()=>{};let dead=false;
 function setProgress(p){if(dead)return;const pose=poseAt(scene,p),u=progress(p),faces=[...floorFaces,...[...fixed,...project(CAR,pose),...project(controlParts(scene,u),pose)].sort((a,b)=>a.depth-b.depth)],pal=container.dataset.theme==='light'?light:palettes;
  layer.innerHTML=faces.map(f=>`<path d="${f.d}" fill="${(pal[f.tone]||pal.hi)[f.shade]}" stroke="${container.dataset.theme==='light'?'#5b636a':'#858e98'}" stroke-width=".45" stroke-linejoin="round"/>`).join('');
  let note='';if(scene===2)note=' · 过垂直接触≠180°水平倒悬';if(scene===3)note=' · 释放';if(scene===8)note=' · Semi上方，位置示意';if(scene===4||scene===9)note=` · 冷气体${u<.5?'回正':'制动'}力偶，非喷口复原`;
  report(`示意，非实测 · 教学角 俯仰 ${Math.round(pose.pitch*180/pi)}° / 滚转 ${Math.round(pose.roll*180/pi)}°${note}`);
 }
 setProgress(.5);return {setProgress,destroy(){if(dead)return;dead=true;svg.remove();if(!hadIso)container.classList.remove('iso');if(oldTheme===undefined)delete container.dataset.theme;else container.dataset.theme=oldTheme;}};
}
