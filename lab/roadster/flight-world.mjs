import {t as tr} from './i18n.mjs';
import * as T from './vendor/three.module.min.js';
import {roadsterModel} from './roadster-model.mjs';
import {createNightRange} from './night-range.mjs?v=20261021-range';
import {createJetSmoke} from './jet-smoke.mjs?v=20261021-night';

const V=(x=0,y=0,z=0)=>new T.Vector3(x,y,z);
const mat=(color,extra={})=>new T.MeshStandardMaterial({color,roughness:.65,metalness:.12,...extra});
function box(parent,size,pos,color){const m=new T.Mesh(new T.BoxGeometry(...size),mat(color));m.position.set(...pos);parent.add(m);return m;}
function line(parent,points,color,opacity=1){const m=new T.Line(new T.BufferGeometry().setFromPoints(points.map(p=>V(...p))),new T.LineBasicMaterial({color,transparent:opacity<1,opacity}));parent.add(m);return m;}
function label(parent,text,pos,scale=1){const c=document.createElement('canvas');c.width=512;c.height=64;const x=c.getContext('2d');x.font='24px sans-serif';x.fillStyle='#a5b4bb';x.textAlign='center';x.fillText(text,256,40);const texture=new T.CanvasTexture(c),s=new T.Sprite(new T.SpriteMaterial({map:texture,transparent:true,depthWrite:false}));s.position.set(...pos);s.scale.set(6.4*scale,.8*scale,1);parent.add(s);}
export function vehicle(size,reference=false){
  if(!reference)return roadsterModel(size);
  const [L,W,H]=size,g=new T.Group();
  box(g,[L,W*.88,H*.6],[0,0,-H*.2],0x4c5960);box(g,[L*.52,W*.86,H*.4],[-L*.05,0,H*.3],0x67757b);
  box(g,[.018,W*.72,H*.23],[L*.211,0,H*.27],0x263b45);
  for(const s of [-1,1])box(g,[L*.34,.015,H*.23],[-L*.04,s*W*.432,H*.27],0x354a53);
  const radius=Math.min(H*.22,.53),width=W*.13;
  for(const x of [-L*.30,L*.30])for(const y of [-1,1]){const p=[x,y*(W/2-width/2-.002),-H/2+radius];const wheel=new T.Mesh(new T.CylinderGeometry(radius,radius,width,24),mat(0x10171a));wheel.position.set(...p);g.add(wheel);const hub=new T.Mesh(new T.CylinderGeometry(radius*.55,radius*.55,width+.002,18),mat(0x7d8d92));hub.position.copy(wheel.position);g.add(hub);}
  return g;
}
function disposeGroup(g){g.traverse(o=>{o.geometry?.dispose();for(const m of [].concat(o.material||[])){m.map?.dispose();m.dispose()}});g.clear();}

export function createWorld(container,cfg,onFallback){
  const scene=new T.Scene();scene.background=new T.Color(0x151b20);
  const content=new T.Group();scene.add(content);
  const camera=new T.OrthographicCamera(-20,20,14,-14,.1,250);camera.up.set(0,0,1);
  let renderer=null,fallback=null;
  try{const canvas=document.createElement('canvas'),context=canvas.getContext('webgl2',{antialias:true,alpha:false});if(!context)throw Error('WebGL unavailable');renderer=new T.WebGLRenderer({canvas,context,antialias:true,alpha:false});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=T.SRGBColorSpace;container.append(renderer.domElement)}catch{fallback=document.createElement('canvas');container.append(fallback);onFallback?.();}
  const night=renderer?createNightRange(scene,renderer):null,smoke=createJetSmoke(cfg.actuator.virtual_nozzles);scene.add(smoke.object);
  let car,arrows=[],plumes=[],path,task,run,frame,showForces=true,view='overview',zoom=1,azimuth=-.98,elevation=.48;
  const centre=V(),size=[cfg.vehicle.length,cfg.vehicle.width,cfg.vehicle.height];
  function setCamera(){
    if(!run)return;
    const vertical=task!=='flat_kickflip';const points=run.frames;const xmax=Math.max(...points.map(f=>f.pos[0]),cfg.scenarios[task].distance||0),zmax=Math.max(...points.map(f=>f.pos[2]));
    centre.set(vertical?0:xmax/2,0,Math.max(3,zmax/2));let span=Math.max(vertical?25:34,zmax*1.4)*zoom;
    if(view==='close'&&frame){centre.fromArray(frame.pos);span=10*zoom;}
    const aspect=container.clientWidth/Math.max(1,container.clientHeight);camera.left=-span/2;camera.right=span/2;camera.top=span/2/aspect;camera.bottom=-span/2/aspect;
    // Fit the vertical scene on narrow screens without changing any physical dimensions.
    if(view!=='close'&&camera.top<zmax*.7){camera.top=zmax*.7;camera.bottom=-camera.top;camera.right=camera.top*aspect;camera.left=-camera.right;}
    const angle=view==='close'&&task==='inverted_release'?-.15:elevation;
    const offset=view==='side'?V(0,-60,.01):view==='top'?V(.01,0,60):V(Math.cos(azimuth)*Math.cos(angle)*60,Math.sin(azimuth)*Math.cos(angle)*60,Math.sin(angle)*60);
    camera.position.copy(centre).add(offset);camera.lookAt(centre);camera.updateProjectionMatrix();smoke.setPixelScale(container.clientHeight*(renderer?.getPixelRatio()??1)/(camera.top-camera.bottom));
  }
  function build(taskName,rollout){
    disposeGroup(content);task=taskName;run=rollout;frame=run.frames[0];arrows=[];plumes=[];zoom=1;smoke.reset();
    for(let x=-10;x<=40;x+=10)label(content,`${x} m`,[x,-7,.05],.55);
    const target=run.environment?.targetDistance??cfg.scenarios[task].distance??0;line(content,[[target-2.5,-2.5,.02],[target+2.5,-2.5,.02],[target+2.5,2.5,.02],[target-2.5,2.5,.02],[target-2.5,-2.5,.02]],0xc9e78a);
    label(content,tr("着陆区域","Landing region"),[target,4,.1],.7);
    for(const obstacle of run.environment?.obstacles??cfg.scenarios[task].obstacles){const ref=vehicle(obstacle.size,true);ref.position.set(...obstacle.center);ref.rotation.z=obstacle.heading;content.add(ref);label(content,obstacle.name,[ref.position.x,-4.5,.1],.6);}
    if(task==='inverted_release'){
      const h=run.environment?.ceilingHeight??run.frames[0].pos[2]+cfg.vehicle.height/2,half=cfg.scenarios[task].ramp_half_size;
      const platform=box(content,[half[0]*2,half[1]*2,.15],[0,0,h+.075],0x55676c);platform.material.transparent=true;platform.material.opacity=.2;platform.material.depthWrite=false;
      const rim=new T.LineSegments(new T.EdgesGeometry(platform.geometry),new T.LineBasicMaterial({color:0x6f888f}));rim.position.copy(platform.position);content.add(rim);
      for(const x of [-half[0],half[0]])for(const y of [-half[1],half[1]])box(content,[.12,.12,h],[x,y,h/2],0x3d5159);
      label(content,`${tr("半透明平台 · ","Translucent platform · ")}${h.toFixed(1)} m`,[0,half[1]+1.5,h],.85);
    }
    path=new T.Line(new T.BufferGeometry().setFromPoints(run.frames.map(f=>V(...f.pos))),new T.LineDashedMaterial({color:run.success?0xc9e78a:0xc48a79,dashSize:.3,gapSize:.22,transparent:true,opacity:.65}));path.computeLineDistances();content.add(path);
    car=vehicle(size);car.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true}});content.add(car);
    const bounds=new T.LineSegments(new T.EdgesGeometry(new T.BoxGeometry(...size)),new T.LineBasicMaterial({color:0x819697,transparent:true,opacity:.24}));car.add(bounds);
    for(const nozzle of cfg.actuator.virtual_nozzles){const plume=new T.ArrowHelper(V(...nozzle.direction).negate(),V(...nozzle.position),.01,0xd9a86b,.13,.06);car.add(plume);plumes.push(plume);}
    for(const color of [0xf4c674,0xc9e78a,0x75d4d6,0xed8880]){const a=new T.ArrowHelper(V(0,0,1),V(),1,color,.22,.10);content.add(a);arrows.push(a)}
    setCamera();draw(frame);
  }
  function fallbackDraw(){
    const c=fallback,x=c.getContext('2d'),w=c.width,h=c.height;x.fillStyle='#151b20';x.fillRect(0,0,w,h);
    const vertical=task!=='flat_kickflip',minX=vertical?-7:-5,maxX=vertical?7:Math.max(cfg.scenarios[task].distance,frame.pos[0])+5,maxZ=Math.max(...run.frames.map(f=>f.pos[2]))+4;
    const scale=Math.min((w-30)/(maxX-minX),(h-40)/maxZ),ox=(w-(maxX-minX)*scale)/2-minX*scale,oy=h-25,pt=(p)=>[ox+p[0]*scale,oy-p[2]*scale];
    x.strokeStyle='#35454d';x.lineWidth=1;for(let z=0;z<maxZ;z+=2){x.beginPath();x.moveTo(0,oy-z*scale);x.lineTo(w,oy-z*scale);x.stroke()}
    x.strokeStyle=run.success?'#c9e78a':'#c48a79';x.setLineDash([4,5]);x.beginPath();run.frames.forEach((f,i)=>{const p=pt(f.pos);i?x.lineTo(...p):x.moveTo(...p)});x.stroke();x.setLineDash([]);
    for(const o of run.environment?.obstacles??cfg.scenarios[task].obstacles){const extent=Math.abs(Math.cos(o.heading))*o.size[0]/2+Math.abs(Math.sin(o.heading))*o.size[1]/2;x.fillStyle='#65767c';x.fillRect(ox+(o.center[0]-extent)*scale,oy-(o.center[2]+o.size[2]/2)*scale,extent*2*scale,o.size[2]*scale)}
    if(task==='inverted_release'){const height=run.environment?.ceilingHeight??run.frames[0].pos[2]+size[2]/2,half=cfg.scenarios[task].ramp_half_size[0];x.strokeStyle='#6f888f';x.beginPath();x.moveTo(ox-half*scale,oy-height*scale);x.lineTo(ox+half*scale,oy-height*scale);x.stroke();}
    const q=new T.Quaternion(frame.quat[1],frame.quat[2],frame.quat[3],frame.quat[0]),corners=[];for(const a of [-1,1])for(const b of [-1,1])for(const d of [-1,1])corners.push(V(a*size[0]/2,b*size[1]/2,d*size[2]/2).applyQuaternion(q).add(V(...frame.pos)));
    x.strokeStyle='#e5e7dc';x.lineWidth=2;for(let i=0;i<8;i++)for(const bit of [1,2,4])if((i^bit)>i){x.beginPath();x.moveTo(...pt(corners[i].toArray()));x.lineTo(...pt(corners[i^bit].toArray()));x.stroke()}
    x.fillStyle='#a5b4bb';x.font='12px sans-serif';x.fillText(tr("侧投影 · 2 m 网格 · 实际积分轨迹","Side projection · 2 m grid · integrated trajectory"),12,20);
  }
  function draw(f){
    frame=f;if(!run)return;
    car.position.set(...f.pos);car.quaternion.set(f.quat[1],f.quat[2],f.quat[3],f.quat[0]);
    const forces=[f.thrust,f.fanSuctionForce||[0,0,0],f.fanAxialForce||[0,0,0],[0,0,-(f.mass||cfg.vehicle.mass)*cfg.gravity]];
    arrows.forEach((a,i)=>{const v=V(...forces[i]),length=v.length()*.000065;a.visible=showForces&&length>.025;a.position.copy(car.position);if(length>.025){a.setDirection(v.normalize());a.setLength(Math.min(length,7),Math.min(.27,length*.25),Math.min(.12,length*.12));}});
    plumes.forEach((a,i)=>{const force=f.nozzleForces?.[i]||0;a.visible=showForces&&force>80;a.setLength(Math.min(2.4,.1+force*.00006),.13,.06)});
    if(view==='close')setCamera();if(renderer){smoke.update(f);night.update(camera,f.t);renderer.render(scene,camera)}else fallbackDraw();
  }
  function resize(){const w=container.clientWidth,h=container.clientHeight;if(!w||!h)return;if(renderer)renderer.setSize(w,h,false);else{fallback.width=w;fallback.height=h}setCamera();if(frame)draw(frame)}
  const observer=new ResizeObserver(resize);observer.observe(container);
  let drag=null;
  container.addEventListener('pointerdown',e=>{if(e.pointerType==='touch')return;drag=[e.clientX,e.clientY,azimuth,elevation];container.setPointerCapture(e.pointerId)});
  container.addEventListener('pointermove',e=>{if(!drag)return;view='overview';azimuth=drag[2]-(e.clientX-drag[0])*.008;elevation=Math.max(.08,Math.min(1.45,drag[3]+(e.clientY-drag[1])*.007));setCamera();if(frame)draw(frame)});
  container.addEventListener('pointerup',()=>drag=null);container.addEventListener('pointercancel',()=>drag=null);
  container.addEventListener('wheel',e=>{if(!e.ctrlKey)return;e.preventDefault();zoom=Math.max(.45,Math.min(2.2,zoom*(e.deltaY>0?1.08:.92)));setCamera();if(frame)draw(frame)},{passive:false});
  return {setRun:build,draw,setCamera(v){view=v;zoom=1;setCamera();if(frame)draw(frame)},setForces(v){showForces=v;if(frame)draw(frame)},destroy(){observer.disconnect();disposeGroup(content);night?.dispose();if(night)disposeGroup(night.object);smoke.dispose();renderer?.dispose();container.replaceChildren()},getMode:()=>renderer?'webgl':'2d',getView:()=>view};
}
