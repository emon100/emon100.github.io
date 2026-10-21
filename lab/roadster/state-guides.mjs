import {t as tr} from './i18n.mjs';
import * as T from './vendor/three.module.min.js';

const colors=[0xff947d,0x95dfaa,0x8dbdff],axes=[[1,0,0],[0,1,0],[0,0,1]];
export function rotationArc(axis,rate){
 const u=new T.Vector3(...axes[(axis+1)%3]),v=new T.Vector3(...axes[(axis+2)%3]);
 return Array.from({length:33},(_,i)=>u.clone().multiplyScalar(2.15*Math.cos(rate*.5*i/32)).addScaledVector(v,2.15*Math.sin(rate*.5*i/32)));
}

export function createStateGuides(container){
 const object=new T.Group(),body=new T.Group();object.add(body);object.visible=false;
 const overlay=document.createElement('div');overlay.className='state-guide-labels';const svg=document.createElementNS('http://www.w3.org/2000/svg','svg');overlay.append(svg);container.append(overlay);const labels=[];
 function label(color){
  const element=document.createElement('span'),line=document.createElementNS(svg.namespaceURI,'line');element.className='state-guide-label';element.style.color=new T.Color(color).getStyle();line.setAttribute('stroke',element.style.color);svg.append(line);overlay.append(element);
  const item={sprite:{position:new T.Vector3(),visible:true},element,line,set(text){element.textContent=text}};labels.push(item);return item;
 }
 function arrow(parent,color){const a=new T.ArrowHelper(new T.Vector3(1,0,0),new T.Vector3(),1,color,.16,.09);a.line.material.depthTest=a.cone.material.depthTest=false;a.renderOrder=3;parent.add(a);return a;}
 const basis=axes.map((axis,i)=>{const a=arrow(body,colors[i]);a.setDirection(new T.Vector3(...axis));a.setLength(1.35,.15,.08);return a;});
 const bodyLabels=colors.map(label),velocity=arrow(object,0x7ce8ee),velocityLabel=label(0x7ce8ee),components=colors.map(c=>arrow(object,c)),componentLabels=colors.map(label);
 const rotations=axes.map((_,i)=>{const line=new T.Line(new T.BufferGeometry(),new T.LineBasicMaterial({color:colors[i],depthTest:false}));body.add(line);return {line,head:arrow(body,colors[i]),label:label(colors[i])}});
 const center=new T.Mesh(new T.SphereGeometry(.09,12,8),new T.MeshBasicMaterial({color:0xffde85,depthTest:false}));center.renderOrder=6;object.add(center);
 const centerLabel=label(0xffde85),heightLabel=label(0xffde85),height=new T.Line(new T.BufferGeometry(),new T.LineDashedMaterial({color:0xffde85,dashSize:.15,gapSize:.1,depthTest:false}));object.add(height);
 const groundAxes=axes.map((axis,i)=>{const a=arrow(object,colors[i]);a.setDirection(new T.Vector3(...axis));a.setLength(1.35,.15,.08);return a}),groundLabels=colors.map(label);
 let snapshot=null,view=null;
 function update(c){
  object.visible=true;const p=new T.Vector3(...c.pos),q=new T.Quaternion(c.quat[1],c.quat[2],c.quat[3],c.quat[0]);body.position.copy(p);body.quaternion.copy(q);center.position.copy(p);
  centerLabel.set('CG · '+c.mass.toFixed(0)+' kg');centerLabel.sprite.position.copy(p).add(new T.Vector3(0,0,.45));
  const ground=new T.Vector3(p.x-3,p.y-2,.035);groundAxes.forEach((a,i)=>{a.position.copy(ground);groundLabels[i].set(tr("世界 +","World +")+['X','Y','Z'][i]);groundLabels[i].sprite.position.copy(ground).addScaledVector(new T.Vector3(...axes[i]),1.7)});
  bodyLabels.forEach((l,i)=>{l.set([tr("车体 X","Body X"),tr("车体 Y","Body Y"),tr("车体 Z","Body Z")][i]);l.sprite.position.copy(new T.Vector3(...axes[i]).multiplyScalar(1.75).applyQuaternion(q)).add(p)});
  const v=new T.Vector3(...c.vel),speed=v.length();velocity.visible=velocityLabel.sprite.visible=speed>1e-8;if(speed>1e-8){velocity.position.copy(p);velocity.setDirection(v.clone().normalize());velocity.setLength(speed*.28,.2,.11);velocityLabel.set(tr("合速度 ","Resultant velocity ")+speed.toFixed(2)+' m/s');velocityLabel.sprite.position.copy(p).addScaledVector(v,.28).add(new T.Vector3(0,0,.45));}
  components.forEach((a,i)=>{const value=c.vel[i],l=componentLabels[i];a.visible=l.sprite.visible=Math.abs(value)>1e-8;if(a.visible){a.position.copy(p);a.setDirection(new T.Vector3(...axes[i]).multiplyScalar(Math.sign(value)));a.setLength(Math.abs(value)*.28,.12,.07);l.set(tr("世界 ","World ")+['X','Y','Z'][i]+' '+value.toFixed(2)+' m/s');l.sprite.position.copy(p).addScaledVector(new T.Vector3(...axes[i]),value*.28).add(new T.Vector3(0,0,-.35));}});
  rotations.forEach((r,i)=>{const rate=c.omega[i];r.line.visible=r.head.visible=r.label.sprite.visible=Math.abs(rate)>1e-8;if(!r.line.visible)return;const points=rotationArc(i,rate);r.line.geometry.dispose();r.line.geometry=new T.BufferGeometry().setFromPoints(points);r.head.position.copy(points.at(-1));r.head.setDirection(points.at(-1).clone().sub(points.at(-2)).normalize());r.head.setLength(.18,.18,.1);r.label.set('ω'+['x','y','z'][i]+' '+(rate*180/Math.PI).toFixed(1)+'°/s');r.label.sprite.position.copy(points[16]).applyQuaternion(q).add(p).add(new T.Vector3(0,0,.3));});
  height.geometry.dispose();height.geometry=new T.BufferGeometry().setFromPoints([new T.Vector3(p.x,p.y,0),p]);height.computeLineDistances();heightLabel.set(tr("重心高 ","CoM height ")+p.z.toFixed(2)+' m');heightLabel.sprite.position.set(p.x+.8,p.y,p.z*.5);
  const tip=p.clone().addScaledVector(v,.28),bounds=new T.Box3().setFromPoints([p.clone().addScalar(3),p.clone().addScalar(-3),tip.clone().addScalar(1),new T.Vector3(p.x-4,p.y-3,0)]);view={center:bounds.getCenter(new T.Vector3()),radius:bounds.getSize(new T.Vector3()).length()/2};
  snapshot={pos:[...c.pos],quat:[...c.quat],velocity:[...c.vel],omega:[...c.omega],bodyAxes:axes.map(a=>new T.Vector3(...a).applyQuaternion(q).toArray()),velocityTip:p.clone().addScaledVector(v,.28).toArray(),height:c.pos[2]};
 }
 return {object,update,view:()=>view,layout(camera){
  overlay.hidden=!object.visible;if(!object.visible)return;
  const width=container.clientWidth,height=container.clientHeight,used=[];
  const projected=labels.filter(l=>{l.element.hidden=!l.sprite.visible;l.line.style.display=l.sprite.visible?'':'none';return l.sprite.visible}).map(l=>({l,p:l.sprite.position.clone().project(camera)})).sort((a,b)=>a.p.y-b.p.y);
  for(const {l,p} of projected){const x=(p.x+1)*width/2,y=(1-p.y)*height/2,w=l.element.offsetWidth,h=22;let chosen=null;
   for(const dy of [0,-26,26,-52,52,-78,78,-104,104]){for(const dx of [10,-w-10]){const left=Math.max(5,Math.min(width-w-5,x+dx)),top=Math.max(32,Math.min(height-h-5,y+dy-h/2));if(!used.some(b=>left<b.x+b.w+5&&left+w+5>b.x&&top<b.y+b.h+3&&top+h+3>b.y)){chosen={x:left,y:top,w,h};break}}if(chosen)break}
   chosen??={x:Math.max(5,Math.min(width-w-5,x+10)),y:Math.max(32,Math.min(height-h-5,y-h/2)),w,h};used.push(chosen);l.element.style.transform=`translate(${chosen.x}px,${chosen.y}px)`;l.line.setAttribute('x1',x);l.line.setAttribute('y1',y);l.line.setAttribute('x2',Math.max(chosen.x,Math.min(chosen.x+w,x)));l.line.setAttribute('y2',chosen.y+h/2);
  }
 },stats:()=>object.visible?snapshot:null};
}
