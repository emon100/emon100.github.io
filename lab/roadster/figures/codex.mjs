import { t } from '../i18n.mjs';
import * as k from '../kit/iso-kit.mjs';
const pi=Math.PI, clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export const sceneNames=[t('弯坡入口', 'Curved-ramp entry'),t('过垂直吸附', 'Past-vertical adhesion'),t('倒悬脱离', 'Inverted release'),t('姿态回正', 'Attitude recovery'),t('坡道着陆', 'Ramp landing'),t('平地起跳', 'Level-ground takeoff'),t('越过车列', 'Over the car row'),t('Semi 上空倒转', 'Inverted above Semi'),t('滚转回正', 'Roll recovery'),t('越车后着陆', 'Landing after the car row')];
const lerp=(a,b,t)=>a+(b-a)*t;
// These are editorial poses, not a reconstruction of Tesla's telemetry.
export function poseAt(scene,p){
 p=clamp(Number(p)||0,0,1); const t=p*p*(3-2*p), r=72;
 if(scene===1){const a=lerp(.1,1.25,t);return {x:-70+(r-7)*Math.sin(a),z:1+r-(r-7)*Math.cos(a),pitch:a,roll:0};}
 if(scene===2){const a=lerp(1.25,2.75,t);return {x:-70+(r-7)*Math.sin(a),z:1+r-(r-7)*Math.cos(a),pitch:a,roll:0};}
 if(scene===3)return {x:lerp(-42,-70,t),z:lerp(147,145,t),pitch:lerp(2.75,pi,t),roll:0};
 if(scene===4)return {x:lerp(-70,-110,t),z:lerp(145,65,t),pitch:pi,roll:pi*t};
 if(scene===5)return {x:lerp(-110,-140,t),z:lerp(65,8,t),pitch:pi,roll:pi};
 const q=scene-6;
 const x=lerp([-180,-135,-65,20,110][q],[-135,-65,20,110,160][q],t);
 const z=lerp([8,75,108,108,65][q],[75,108,108,65,8][q],t);
 const roll=lerp([0,0,pi*.65,pi*1.35,2*pi][q],[0,pi*.65,pi*1.35,2*pi,2*pi][q],t);
 return {x,z,pitch:0,roll};
}
function transform(v,o){const [x,y,z]=v,c=Math.cos(o.roll),s=Math.sin(o.roll),yy=y*c-z*s,zz=y*s+z*c,cp=Math.cos(o.pitch),sp=Math.sin(o.pitch);return [o.x+x*cp-zz*sp,yy,x*sp+zz*cp+o.z];}
function normal(a,b,c){const u=b.map((x,i)=>x-a[i]),v=c.map((x,i)=>x-a[i]);return [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];}
const mean=p=>p[0].map((_,i)=>p.reduce((s,v)=>s+v[i],0)/p.length);
function box(x,y,z,w,d,h){const v=[[x,y,z],[x+w,y,z],[x+w,y+d,z],[x,y+d,z],[x,y,z+h],[x+w,y,z+h],[x+w,y+d,z+h],[x,y+d,z+h]];return [[0,3,2,1],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[4,5,6,7]].map(f=>f.map(i=>v[i]));}
function prism(ring,y,d){const a=ring.map(([x,z])=>[x,y,z]),b=ring.map(([x,z])=>[x,y+d,z]);return [a,b.toReversed(),...a.map((v,i)=>[v,b[i],b[(i+1)%a.length],a[(i+1)%a.length]])];}
function wheel(x,y){return prism(Array.from({length:12},(_,i)=>[x+7*Math.cos(i*pi/6),7*Math.sin(i*pi/6)]),y,4);}
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
 const outside=y<0?y-.4:y+4; a.push({faces:prism(Array.from({length:8},(_,i)=>[x+4.8*Math.cos(i*pi/4),4.8*Math.sin(i*pi/4)]),outside,.4),tone:'mid'});
 B(x-1,outside-.1,-1,2,.7,2,'hi');B(x-3,y+1,-3,6,2,6,'lo');
 }
 for(const x of [-26,23])for(const y of [-12,9]){B(x,y,-4,4,3,2,'mid');B(x+.8,y+.6,-4.5,2.4,1.8,.5,'rubber');}
 return a;
}
const CAR=carParts();
export function mount(container,{scene=1,theme='dark',onReadout=()=>{}}={}){
 scene=clamp(Math.round(scene),1,10);container.classList.add('iso');container.dataset.theme=theme;
 const P=k.fitProjection([[-230,-68,-15],[210,68,0],[0,0,180]],640,370,{pad:30,azimuth:140,elevation:28});
 const floor={x:-230,y:-65,w:440,d:130,r:8}, solids=[];
 const add=(x,y,z,w,d,h,tone='mid')=>solids.push({faces:box(x,y,z,w,d,h),tone});
 add(-230,-65,-9,440,130,9,'lo');for(const x of [-212,192])for(const y of [-50,38])add(x,y,-15,12,12,6,'lo');
 add(-222,-40,0,424,80,1,'mid');for(const y of [-58,54])add(-220,y,0,420,4,2,'mid');
 for(let i=0;i<20;i++)add(-210+i*20,54,2,.8,i%5===0?4:2,.3,'hi');
 for(const x of [-215,-75,65,195])for(const y of [-50,49])add(x,y,0,2,2,.5,'hi');
 const floorCount=solids.length;
 if(scene<=5){
  for(let i=0;i<28;i++){const a=i*pi/30,b=(i+1)*pi/30;const ring=[[-70+72*Math.sin(a),1+72*(1-Math.cos(a))],[-70+72*Math.sin(b),1+72*(1-Math.cos(b))],[-70+76*Math.sin(b),1+72-76*Math.cos(b)],[-70+76*Math.sin(a),1+72-76*Math.cos(a)]];solids.push({faces:prism(ring,-22,44),tone:'mid'});}
  for(const [x,z] of [[-50,12],[-2,65],[-15,120]])for(const y of [-27,23]){add(x,y,1,4,4,z,'lo');add(x-5,y-3,0,14,10,1,'mid');}
 }else{
  [-100,-50,0,50].forEach((x,i)=>{add(x,-12,1,35,24,14,'lo');add(x+8,-10,15,18,20,9,'mid');for(const y of [-15,12])for(const xx of [x+3,x+25])add(xx,y,1,7,3,7,'lo');});
  add(100,-17,5,50,34,34,'mid');add(87,-17,5,13,34,27,'lo');add(89,-16,23,1,32,7,'glass');
 }
 container.innerHTML=t(`<svg viewBox="0 0 640 370" role="img" aria-label="${sceneNames[scene-1]}，微缩场景示意，不是现场轨迹测量"><g data-world></g></svg>`, `<svg viewBox="0 0 640 370" role="img" aria-label="${sceneNames[scene-1]}, miniature schematic, not a measured on-site trajectory"><g data-world></g></svg>`);
 const layer=container.querySelector('[data-world]');
 const c=k.cameraOf(P),viewer=[c.cosA*c.cosE,c.sinA*c.cosE,c.sinE];
 const palettes={hi:['#777c81','#94999e','#b9bdc1','#e4e7e8'],mid:['#292d32','#363b41','#4c5259','#646b72'],lo:['#171b20','#23282f','#30363e','#3e4650'],glass:['#111820','#1b2731','#2a3a45','#3b4d58'],rubber:['#101214','#161a1e','#20262b','#30363b'],lit:['#b7bfbe','#d8dfdf','#e7eeee','#ffffff']};
 const light={hi:['#a6abad','#bbc0c1','#d5d8d8','#f0f1ef'],mid:['#858b8f','#a1a7aa','#b7bdbf','#d0d5d5'],lo:['#a4aaad','#bbc1c3','#d2d7d7','#e5e8e6'],glass:['#33414a','#435866','#596c77','#72818a'],rubber:palettes.rubber,lit:['#aeb4b6','#c3cbcc','#e7eeee','#fff']};
 function project(parts,o){const list=[];for(const part of parts)for(const face0 of part.faces){const face=o?face0.map(v=>transform(v,o)):face0,n=normal(...face.slice(0,3)),len=Math.hypot(...n);if(len===0||n.reduce((s,v,i)=>s+v*viewer[i],0)<=0)continue;const unit=n.map(v=>v/len),shade=clamp(Math.round((unit[2]*.7+unit[0]*-.3+unit[1]*-.4+1)*1.5),0,3);list.push({d:k.pathOf(face.map(v=>k.iso(v,P)),true),depth:k.depthOf(mean(face),P),tone:part.tone,shade});}return list;}
 const floorFaces=project(solids.slice(0,floorCount)).sort((a,b)=>a.depth-b.depth),fixed=project(solids.slice(floorCount));let dead=false;
 function setProgress(p){if(dead)return;const pose=poseAt(scene,p),faces=[...floorFaces,...[...fixed,...project(CAR,pose)].sort((a,b)=>a.depth-b.depth)],pal=container.dataset.theme==='light'?light:palettes;
 layer.innerHTML=faces.map(f=>`<path d="${f.d}" fill="${(pal[f.tone]||pal.hi)[f.shade]}" stroke="${container.dataset.theme==='light'?'#5b636a':'#858e98'}" stroke-width=".45" stroke-linejoin="round"/>`).join('');
 onReadout(t(`示意姿态 · 俯仰 ${Math.round(pose.pitch*180/pi)}° / 滚转 ${Math.round(pose.roll*180/pi)}°`, `Schematic attitude · Pitch ${Math.round(pose.pitch*180/pi)}° / Roll ${Math.round(pose.roll*180/pi)}°`));
 }
 setProgress(.5);return {setProgress,destroy(){dead=true;container.replaceChildren();}};
}
