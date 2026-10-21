import {t as tr} from './i18n.mjs';
import * as T from './vendor/three.module.min.js';
import {PARKED_VEHICLES,JUMP_LANE,vehicleHull} from './driving-obstacles.mjs?v=20261022-jump-final';

export function createParkedVehicles(){
 const group=new T.Group();group.name='三车飞越测试区';
 const paint=new T.MeshPhysicalMaterial({color:0xcbd0d2,metalness:.35,roughness:.27,clearcoat:.8}),glass=new T.MeshPhysicalMaterial({color:0x0a1420,metalness:.5,roughness:.17,clearcoat:1}),steel=new T.MeshStandardMaterial({color:0x9ca4aa,metalness:.85,roughness:.32,flatShading:true}),dark=new T.MeshStandardMaterial({color:0x161a1e,roughness:.7}),rubber=new T.MeshStandardMaterial({color:0x080a0d,roughness:.92}),rim=new T.MeshStandardMaterial({color:0x6e757b,metalness:.85,roughness:.24}),whiteLight=new T.MeshStandardMaterial({color:0xc9e5f4,emissive:0xc9e5f4,emissiveIntensity:2}),redLight=new T.MeshStandardMaterial({color:0xaf1821,emissive:0xdd1826,emissiveIntensity:1.2});
 const materials={white:paint,glass,steel,dark};
 const mesh=(parent,g,m,pos=[0,0,0])=>{const o=new T.Mesh(g,m);o.position.set(...pos);o.castShadow=true;o.receiveShadow=true;parent.add(o);return o;};
 const box=(parent,size,pos,mat)=>mesh(parent,new T.BoxGeometry(...size),mat,pos);
 function panel(parent,points,mat){const g=new T.BufferGeometry().setFromPoints(points.map(p=>new T.Vector3(...p))),indices=[];for(let i=1;i<points.length-1;i++)indices.push(0,i,i+1);g.setIndex(indices);g.computeVertexNormals();const m=mat.clone();m.side=T.DoubleSide;return mesh(parent,g,m);}
 function line(parent,points,color=0x252b30){const g=new T.BufferGeometry().setFromPoints(points.map(p=>new T.Vector3(...p)));parent.add(new T.Line(g,new T.LineBasicMaterial({color})));}
 function sign(text,x,y,z,width=5){const canvas=document.createElement('canvas');canvas.width=768;canvas.height=96;const ctx=canvas.getContext('2d');ctx.font='500 32px system-ui';ctx.textAlign='center';ctx.lineWidth=5;ctx.strokeStyle='#06090ddd';ctx.strokeText(text,384,62,750);ctx.fillStyle='#f1f3f5';ctx.fillText(text,384,62,750);const sprite=new T.Sprite(new T.SpriteMaterial({map:new T.CanvasTexture(canvas),transparent:true,depthWrite:false}));sprite.position.set(x,y,z);sprite.scale.set(width,width/8,1);group.add(sprite);return sprite;}
 for(const car of PARKED_VEHICLES){
  const body=new T.Group();body.name=car.label;body.position.set(car.x,car.y,0);body.rotation.z=car.yaw;group.add(body);
  for(const h of car.hulls){const {vertices,faces}=vehicleHull(h),g=new T.BufferGeometry();g.setAttribute('position',new T.Float32BufferAttribute(vertices,3));g.setIndex(faces);g.computeVertexNormals();const m=materials[h.material].clone();m.flatShading=!h.smooth;mesh(body,g,m);}
  for(const x of car.axles)for(const side of [-1,1]){
   const y=side*car.wheelTrack,r=car.wheelRadius,w=car.wheelHalfWidth,z=r;
   mesh(body,new T.CylinderGeometry(r*1.1,r*1.1,.025,40),dark,[x,side*(car.width/2+.003),z]);
   mesh(body,new T.CylinderGeometry(r,r,w*2,40),rubber,[x,y,z]);
   mesh(body,new T.CylinderGeometry(r*.63,r*.63,.022,40),dark,[x,y+side*(w+.012),z]);
   for(let i=0;i<7;i++){const a=i*Math.PI*2/7,spoke=box(body,[r*.09,.026,r*1.16],[x,y+side*(w+.028),z],rim);spoke.rotation.y=a;}
   mesh(body,new T.CylinderGeometry(r*.16,r*.16,.035,24),rim,[x,y+side*(w+.04),z]);
  }
  if(car.id==='modelx'){
   for(const side of [-1,1]){
    line(body,[[-2,.75*side,1.1],[-1.6,.8*side,1.45],[-.9,.8*side,1.685],[.25,.79*side,1.695],[.9,.76*side,1.5],[1.55,.83*side,1.05]],0xbfc6cb);
    panel(body,[[-.25,side*.807,1.67],[-.16,side*.811,1.67],[.15,side*.97,1.04],[.04,side*.97,1.04]],dark);
    line(body,[[-1.45,side*1.003,.43],[-1.5,side*1.003,1.04],[-.9,side*.806,1.67]],0x454b50);
    line(body,[[.36,side*1.003,.41],[.46,side*1.003,1.05],[.25,side*.795,1.68]],0x454b50);
    box(body,[.22,.02,.035],[-.8,side*1.006,1.00],rim);box(body,[.22,.02,.035],[.68,side*1.006,.99],rim);
    box(body,[.3,.18,.12],[1.22,side*1.06,1.1],paint);
    box(body,[.09,.40,.065],[2.38,side*.60,.79],whiteLight);box(body,[.06,.43,.075],[-2.40,side*.59,.87],redLight);
   }
   box(body,[.025,.65,.075],[2.54,0,.56],dark);
  }else if(car.id==='semi'){
   for(const side of [-1,1]){
    panel(body,[[.25,side*1.256,2.2],[.25,side*1.256,3.32],[1.72,side*1.247,3.34],[1.72,side*1.247,2.19]],glass);
    panel(body,[[1.75,side*1.247,2.19],[1.75,side*1.247,3.32],[2.82,side*1.148,2.92],[3.05,side*1.128,2.18]],glass);
    line(body,[[.2,side*1.255,.85],[.2,side*1.255,2.2],[1.2,side*1.247,2.2]],0x515962);
    box(body,[.65,.10,.12],[.5,side*1.3,.85],dark);box(body,[.15,.25,.35],[2.55,side*1.31,2.4],dark);
    box(body,[.035,.32,.28],[3.91,side*.71,1.12],whiteLight);box(body,[.06,.35,.12],[-3.91,side*.75,.82],redLight);
   }
   for(const [a,b] of [[[1.98,1.12,3.61],[3.05,.96,3.16]],[[3.05,.96,3.16],[3.5,.89,2.34]]])panel(body,[[a[0],-a[1],a[2]],[a[0],a[1],a[2]],[b[0],b[1],b[2]],[b[0],-b[1],b[2]]],glass);
   box(body,[.03,1.45,.28],[3.92,0,.70],dark);
   const hitch=mesh(body,new T.CylinderGeometry(.65,.65,.10,32),dark,[-2.0,0,1.09]);hitch.rotation.x=Math.PI/2;
  }else{
   for(const side of [-1,1]){
    panel(body,[[-1.09,side*1.019,1.15],[-.68,side*1.019,1.53],[.28,side*1.019,1.70],[.28,side*1.019,1.15]],glass);
    panel(body,[[.35,side*1.019,1.72],[1.83,side*.986,1.17],[.35,side*1.019,1.15]],glass);
    line(body,[[.05,side*1.022,.43],[.05,side*1.022,1.64]],0x41474d);line(body,[[-1.1,side*1.022,.45],[-1.1,side*1.022,1.38]],0x41474d);
    box(body,[.26,.16,.10],[1.5,side*1.08,1.15],steel);
   }
   panel(body,[[.38,-.85,1.80],[.38,.85,1.80],[2.34,.82,1.20],[2.34,-.82,1.20]],glass);
   panel(body,[[-2.73,-.79,1.12],[-2.73,.79,1.12],[-.75,.79,1.60],[-.75,-.79,1.60]],dark);
   for(let x=-2.68;x<-.78;x+=.16)line(body,[[x,-.79,1.12+(x+2.73)*.242],[x,.79,1.12+(x+2.73)*.242]],0x3c4248);
   box(body,[.035,1.83,.045],[2.85,0,1.025],whiteLight);box(body,[.035,1.93,.045],[-2.85,0,1.10],redLight);
  }
  sign(car.label,car.x,22,.7,3.4);
 }
 const lane=new T.Mesh(new T.PlaneGeometry(JUMP_LANE.end-JUMP_LANE.start,JUMP_LANE.halfWidth*2),new T.MeshStandardMaterial({color:0x34383d,roughness:.9}));lane.receiveShadow=true;lane.position.set((JUMP_LANE.start+JUMP_LANE.end)/2,JUMP_LANE.y,.006);group.add(lane);
 const marking=new T.MeshBasicMaterial({color:0xd8d8d8});
 for(const y of [-1,1].map(s=>JUMP_LANE.y+s*JUMP_LANE.halfWidth))for(let x=JUMP_LANE.start;x<JUMP_LANE.end;x+=12)box(group,[5,.1,.008],[x+2.5,y,.013],marking);
 for(const x of [-200,-140,-80,-25]){box(group,[3,.12,.008],[x-2,JUMP_LANE.y,.016],marking);panel(group,[[x+1,JUMP_LANE.y,.02],[x-1,JUMP_LANE.y+.7,.02],[x-1,JUMP_LANE.y-.7,.02]],marking).castShadow=false;}
 box(group,[.18,JUMP_LANE.halfWidth*2,.008],[JUMP_LANE.takeoff,JUMP_LANE.y,.017],new T.MeshBasicMaterial({color:0xe82127}));
 sign(tr('三车飞越区  ←','CAR JUMP  ←'),-25,8,1.1,8);sign(tr('三车飞越区  →','CAR JUMP  →'),-55,24,1.5,8);sign(tr('80 km/h · 在红线按 K','80 km/h · Press K at red line'),JUMP_LANE.takeoff,25,1.4,8);sign(tr('落地区 · 继续直行','LANDING · Keep straight'),85,25,1.2,8);
 return group;
}
