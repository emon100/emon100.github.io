import {t as tr} from './i18n.mjs';
import * as T from './vendor/three.module.min.js';
import {loadMeshyModelFactory} from './meshy-model.mjs';
import {createJetSmoke} from './jet-smoke.mjs?v=20261021-night';
import {createNightRange} from './night-range.mjs?v=20261021-range';
import {suctionModel,thrustModel,rollModel,aeroModel,G} from './physics.mjs';

export const CAR_LENGTH_METERS=4.1;
export const GROUND_CG_METERS=.5;
export const RAMP_RADIUS_METERS=6.5;
const PI=Math.PI,clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),mix=(a,b,t)=>a+(b-a)*t,smooth=t=>t*t*(3-2*t);
const v3=a=>new T.Vector3(...a),qArray=q=>[q.w,q.x,q.y,q.z];
const qAxis=(axis,angle)=>new T.Quaternion().setFromAxisAngle(v3(axis),angle);
const finite=(x,name)=>{if(!Number.isFinite(x))throw new TypeError(name+tr(" 必须是有限数值"," must be a finite number"));return x};
const sceneNames=[tr("弯坡入口","Curved ramp entry"),tr("过垂直贴坡","Ramp contact past vertical"),tr("倒置脱离坡顶","Inverted release from ramp top"),tr("空中滚转回正","Airborne roll recovery"),tr("坡道演示着陆","Ramp demonstration landing"),tr("平地喷流起跳","Jet liftoff from level ground"),tr("爬升越过车列","Climbing above the car row"),tr("车列上方倒转","Inverted above the cars"),tr("回正并下降","Recovering upright and descending"),tr("车列后着陆","Landing beyond the car row")];
const sceneLabels=[tr("倒车进入弯坡","Reversing onto the curved ramp"),tr("过垂直：贴面位置示意","Past vertical: surface-contact position schematic"),tr("倒置离面，密封吸附终止","Inverted release ends sealed suction"),tr("绕车身纵轴滚转","Roll about the vehicle longitudinal axis"),tr("地面着陆位置示意","Schematic ground landing position"),tr("上升时开始小角度滚转","Small roll begins during ascent"),tr("车列并排，飞越轴垂直于车头朝向","Cars are side by side; the flight axis is perpendicular to their heading"),tr("进度中点倒置；净空是假设","Inverted at mid-progress; clearance is assumed"),tr("回正不等于已消除下降速度","Recovering upright does not eliminate descent velocity"),tr("喷流和接地点均为教学示意","Jets and contact points are teaching illustrations")];
const BENCH_INPUTS={suction:['m','A','mu','theta','deltaP'],thrust:['m','Tz','Isp','seconds'],roll:['I','seconds','angle','progress'],aero:['velocity','rho','S','CL','fan']};
const SOLVE={suction:suctionModel,thrust:thrustModel,roll:rollModel,aero:aeroModel};
const PRIMARY={suction:'theta',thrust:'Tz',roll:'progress',aero:'velocity'};
const mounts=new WeakMap();

export function parkedLayout(){
 const widths=[1.85,1.85,2.5,1.85,1.85],gap=1,total=widths.reduce((a,b)=>a+b,0)+gap*(widths.length-1);let edge=-total/2;
 return widths.map((width,i)=>{const x=edge+width/2;edge+=width+gap;return {id:i,kind:i===2?'truck':'car',position:[x,0,0],yaw:-PI/2,length:i===2?7:4.5,width,height:i===2?3.6:1.45}});
}
export function shotState(scene,progress,{rampOffset=.65}={}){
 if(!Number.isInteger(scene)||scene<1||scene>10)throw new RangeError(tr("镜头必须是1至10","Shot must be between 1 and 10"));
 const p=clamp(finite(progress,tr("进度","Progress")),0,1),t=smooth(p),R=RAMP_RADIUS_METERS,top=2*R-rampOffset;let position,quaternion,roll=0,rampAngle=null;
 if(scene<=2){
  let a;if(scene===1&&p<.5){const u=smooth(p*2);position=[mix(-6,0,u),0,mix(.5,rampOffset,u)];a=0}else{a=scene===1?mix(0,40*PI/180,smooth((p-.5)*2)):mix(40*PI/180,PI,t);position=[(R-rampOffset)*Math.sin(a),0,R-(R-rampOffset)*Math.cos(a)]}
  quaternion=qArray(qAxis([0,1,0],-a).multiply(qAxis([0,0,1],PI)));rampAngle=a;
 }else if(scene===3){position=[mix(0,-4,t),0,top];roll=PI;quaternion=qArray(qAxis([1,0,0],roll))}
 else if(scene===4){position=[mix(-4,-12,t),0,mix(top,4.3,t)];roll=mix(PI,2*PI,t);quaternion=qArray(qAxis([1,0,0],roll))}
 else if(scene===5){const land=smooth(Math.min(1,p/.82));position=[mix(-12,-18,t),0,mix(4.3,.5,land)];roll=2*PI;quaternion=qArray(qAxis([1,0,0],roll))}
 else{const i=scene-6,x=[-12,-9,-3,4,9.5,12],z=[.5,4.3,7,7,3.4,.5],r=[0,.15*PI,.65*PI,1.35*PI,2*PI,2*PI],land=scene===10?smooth(Math.min(1,p/.82)):t;position=[mix(x[i],x[i+1],t),0,mix(z[i],z[i+1],land)];roll=mix(r[i],r[i+1],t);quaternion=qArray(qAxis([1,0,0],roll))}
 const airborne=scene>=3&&!(scene===5&&p>=.82)&&!(scene===10&&p>=.82),fraction=airborne?.22+.28*Math.sin(PI*p)**2:0;
 return {scene,progress:p,position,quaternion,roll,rampAngle,airborne,effectFraction:fraction,title:sceneNames[scene-1],description:sceneLabels[scene-1],evidence:'editorial_concept_not_telemetry',units:'meters',carLength:CAR_LENGTH_METERS};
}
export function benchState(model,value={}){
 if(!SOLVE[model])throw new RangeError(tr("未知物理问题","Unknown physics problem"));
 if(typeof value==='number')value={[PRIMARY[model]]:value};if(!value||typeof value!=='object')throw new TypeError(tr("参数需要对象或数值","Parameters must be an object or number"));
 const inputs=Object.fromEntries(BENCH_INPUTS[model].filter(k=>value[k]!==undefined).map(k=>[k,value[k]]));
 const solved=SOLVE[model](inputs);let position=[0,0,.784],q=new T.Quaternion(),forceVectors=[],description;
 if(model==='suction'){
  q=qAxis([0,1,0],-solved.theta*PI/180);const normal=new T.Vector3(0,0,1).applyQuaternion(q),tangent=new T.Vector3(1,0,0).applyQuaternion(q);position=new T.Vector3(0,0,4).addScaledVector(normal,.5).toArray();
  forceVectors=[{origin:position,vector:normal.clone().multiplyScalar(-solved.D).toArray(),color:0x72dbe5,label:tr("压差吸附","Pressure suction")},{origin:position,vector:[0,0,-solved.m*G],color:0xe7b67d,label:tr("重力","Gravity")},{origin:position,vector:normal.clone().multiplyScalar(Math.max(0,solved.N)).toArray(),color:0xb5d69b,label:tr("法向支撑","Normal support")},{origin:position,vector:tangent.multiplyScalar(Math.min(solved.along,solved.mu*Math.max(0,solved.N))).toArray(),color:0xcad4db,label:tr("可用摩擦","Available friction")}];
  description=(!solved.contact?tr("无法维持接触","Cannot maintain contact"):!solved.held?tr("摩擦不足，会沿面滑移","Insufficient friction; slides along the surface"):tr("静力条件满足","Static conditions satisfied"))+tr(" · 图中姿态由台架约束，不模拟脱落"," · Pose is constrained by the bench; detachment is not simulated");
 }else if(model==='thrust'){
  forceVectors=[{origin:[.7,0,.95],vector:[0,0,solved.thrust],color:0x72dbe5,label:tr("推力","Thrust")},{origin:[-.7,0,.95],vector:[0,0,-solved.m*G],color:0xe7b67d,label:tr("重力","Gravity")}];description=`${tr("理想离地净加速度 ","Ideal airborne net acceleration ")}${solved.acceleration.toFixed(2)}${tr(" m/s² · 车固定在台架上"," m/s² · Vehicle is fixed to the bench")}`;
 }else if(model==='roll'){
  position=[0,0,2.6];q=qAxis([1,0,0],PI-solved.rotation);const force=-solved.torque/1.2;
  for(const side of [-1,1])forceVectors.push({origin:new T.Vector3(0,side*.6,0).applyQuaternion(q).add(v3(position)).toArray(),vector:new T.Vector3(0,0,side*force).applyQuaternion(q).toArray(),color:solved.torque<0?0xe7b67d:0x72dbe5,label:tr("理想力偶","Ideal force couple")});
  description=`${solved.progress===1?tr("停止","Stopped"):solved.progress<.5?tr("加速翻转","Accelerating the flip"):tr("反向刹转","Reverse braking")}${tr(" · 已转 "," · Rotated ")}${Math.round(solved.rotation*180/PI)}° · τ ${(solved.torque/1000).toFixed(2)} kN·m`;
 }else{
  forceVectors=[{origin:[-1.2,0,1.1],vector:[0,0,-solved.active],color:0x72dbe5,label:tr("假设主动贡献","Assumed active contribution")},{origin:[1.2,0,1.1],vector:[0,0,-solved.passive],color:0xb5d69b,label:tr("假设翼片贡献","Assumed wing contribution")}];description=`${tr("独立假设贡献：风机 ","Separate assumed contributions: fan ")}${(solved.active/1000).toFixed(1)}${tr(" + 翼片 "," + wing ")}${(solved.passive/1000).toFixed(2)} kN`;
 }
 return {model,inputs,solved,position,quaternion:qArray(q),forceVectors,description,evidence:'algebraic_teaching_model_not_vehicle_simulation',units:'meters',carLength:CAR_LENGTH_METERS};
}
function material(color,roughness=.66,metalness=.2){return new T.MeshStandardMaterial({color,roughness,metalness})}
function box(parent,size,pos,mat){const m=new T.Mesh(new T.BoxGeometry(...size),mat);m.position.set(...pos);m.castShadow=true;m.receiveShadow=true;parent.add(m);return m}
function rod(parent,a,b,r,mat){const start=v3(a),end=v3(b),delta=end.clone().sub(start);const mesh=new T.Mesh(new T.CylinderGeometry(r,r,delta.length(),12),mat);mesh.position.copy(start.add(end).multiplyScalar(.5));mesh.quaternion.setFromUnitVectors(new T.Vector3(0,1,0),delta.normalize());mesh.castShadow=true;parent.add(mesh);return mesh}
function polyPrism(parent,profile,width,mat){const shape=new T.Shape(profile.map(([x,z])=>new T.Vector2(x,z))),g=new T.ExtrudeGeometry(shape,{depth:width,bevelEnabled:true,bevelThickness:.025,bevelSize:.025,bevelSegments:2,steps:1});g.rotateX(PI/2);g.translate(0,width/2,0);const m=new T.Mesh(g,mat);m.castShadow=true;parent.add(m);return m}
function buildParked(parent,mat){
 for(const c of parkedLayout()){const g=new T.Group();g.position.set(...c.position);g.rotation.z=c.yaw;parent.add(g);const L=c.length,W=c.width;
  if(c.kind==='truck'){box(g,[L,W,.4],[0,0,.7],mat.structure);polyPrism(g,[[.5,.8],[L/2,.8],[L/2,2.5],[L/2-.8,3.6],[.5,3.6]],W,mat.obstacle);box(g,[.055,W*.86,.8],[L/2+.03,0,2.3],mat.glass);box(g,[1.7,W*.8,.13],[-2,0,1.02],mat.structure)}
  else{polyPrism(g,[[-L/2,.35],[L/2,.35],[L/2,.65],[L/2-.5,.85],[-L/2+.15,.85]],W,mat.obstacle);polyPrism(g,[[-1.3,.84],[1.0,.84],[.55,1.43],[-.85,1.43]],W*.78,mat.glass)}
  for(const x of c.kind==='truck'?[-2.6,-1.3,2.5]:[-1.42,1.42])for(const y of [-W/2,W/2]){const wheel=new T.Mesh(new T.CylinderGeometry(c.kind==='truck'?.47:.31,c.kind==='truck'?.47:.31,.19,20),mat.rubber);wheel.position.set(x,y,c.kind==='truck'?.47:.31);g.add(wheel)}
  // Parking strips are physical inlays, not vehicle-length distance assumptions.
  box(parent,[c.width+.55,5.8,.012],[c.position[0],0,.008],mat.parking);
 }
}
function rampPoint(a,R,y=0){return [R*Math.sin(a),y,RAMP_RADIUS_METERS-R*Math.cos(a)]}
function buildRamp(parent,mat){
 const R=RAMP_RADIUS_METERS,thick=.2,width=3.5,segments=80,vertices=[],indices=[];
 for(let i=0;i<=segments;i++){const a=i*PI/segments;for(const rr of [R,R+thick])for(const y of [-width/2,width/2])vertices.push(...rampPoint(a,rr,y))}
 for(let i=0;i<segments;i++){const a=i*4,b=(i+1)*4;for(const [j,k]of [[0,1],[2,3],[0,2],[1,3]])indices.push(a+j,b+j,b+k,a+j,b+k,a+k)}indices.push(0,2,3,0,3,1,segments*4,segments*4+1,segments*4+3,segments*4,segments*4+3,segments*4+2);
 const geo=new T.BufferGeometry();geo.setAttribute('position',new T.Float32BufferAttribute(vertices,3));geo.setIndex(indices);geo.computeVertexNormals();const skin=new T.Mesh(geo,mat.ramp);skin.castShadow=true;skin.receiveShadow=true;parent.add(skin);
 for(const y of [-width/2-.04,width/2+.04]){const pts=Array.from({length:81},(_,i)=>v3(rampPoint(i*PI/80,R+.06,y)));const rail=new T.Mesh(new T.TubeGeometry(new T.CatmullRomCurve3(pts),100,.055,8,false),mat.structure);parent.add(rail)}
 for(const a of [.3,.7,1.1,1.5,1.9,2.3,2.7,3.05])rod(parent,rampPoint(a,R+.24,-1.85),rampPoint(a,R+.24,1.85),.07,mat.structure);
 for(const y of [-2.15,2.15]){box(parent,[.32,.32,R],[R+.6,y,R/2],mat.structure);rod(parent,[R+.6,y,R],rampPoint(PI/2,R+.25,y),.13,mat.structure);rod(parent,[R+.6,y,.3],rampPoint(.6,R+.25,y),.10,mat.structure);box(parent,[1,.9,.16],[R+.6,y,.08],mat.structure)}
}
function buildBench(parent,mat){
 const sets={};for(const name of Object.keys(SOLVE)){sets[name]=new T.Group();parent.add(sets[name]);sets[name].visible=false}
 for(const name of ['thrust','aero']){const g=sets[name];box(g,[6.5,3.4,.26],[0,0,.13],mat.structure);box(g,[5.8,2.9,.024],[0,0,.272],mat.base);for(const x of [-2.8,2.8])for(const y of [-1.3,1.3])box(g,[.3,.3,.18],[x,y,.09],mat.rubber)}
 const pivot=new T.Group();pivot.position.z=4;sets.suction.add(pivot);box(pivot,[6,3.1,.18],[0,0,-.09],mat.ramp);for(const x of [-2.8,2.8])box(pivot,[.08,3.3,.1],[x,0,-.16],mat.structure);
 for(const y of [-2.15,2.15]){box(sets.suction,[.32,.32,3.8],[0,y,1.9],mat.structure);box(sets.suction,[2,.85,.16],[0,y,.08],mat.structure);rod(sets.suction,[0,y,4],[0,Math.sign(y)*1.58,4],.12,mat.structure)}
 for(const x of [-2.65,2.65]){box(sets.roll,[.28,2.4,.18],[x,0,.09],mat.structure);box(sets.roll,[.22,.22,2.5],[x,0,1.25],mat.structure);rod(sets.roll,[x,0,2.6],[Math.sign(x)*2.08,0,2.6],.11,mat.structure);const ring=new T.Mesh(new T.TorusGeometry(1.25,.045,8,72),mat.structure);ring.rotation.y=PI/2;ring.position.set(x,0,2.6);sets.roll.add(ring)}
 const air=[];for(const y of [-2,2])for(const z of [.7,1.6,2.4]){const a=new T.ArrowHelper(new T.Vector3(-1,0,0),new T.Vector3(3,y,z),1.6,0x86b8c9,.18,.09);sets.aero.add(a);air.push(a)}
 return {sets,pivot,air};
}
export function measureModelGeometry(skin){
 skin.updateMatrixWorld(true);const b=new T.Box3().setFromObject(skin),size=b.getSize(new T.Vector3());let rampOffset=.5;const p=new T.Vector3();skin.traverse(o=>{if(!o.isMesh)return;const a=o.geometry.attributes.position;for(let i=0;i<a.count;i++){p.fromBufferAttribute(a,i).applyMatrix4(o.matrixWorld);const localZ=p.z-GROUND_CG_METERS;rampOffset=Math.max(rampOffset,RAMP_RADIUS_METERS-localZ-Math.sqrt(Math.max(0,RAMP_RADIUS_METERS**2-p.x**2)))}});
 return {length:size.x,width:size.y,height:size.z,rampOffset:rampOffset+.012};
}
function disposeTree(root){const geometry=new Set(),materials=new Set(),textures=new Set();root.traverse(o=>{if(o.geometry)geometry.add(o.geometry);for(const m of Array.isArray(o.material)?o.material:[o.material])if(m){materials.add(m);for(const t of Object.values(m))if(t?.isTexture)textures.add(t)}});geometry.forEach(x=>x.dispose());materials.forEach(x=>x.dispose());textures.forEach(x=>x.dispose())}
function themeValue(value){return value==='light'?'light':'dark'}
async function mount(container,{kind,scene=1,model='suction',value={},theme='dark',onReadout=()=>{}}){
 if(!container||typeof container.replaceChildren!=='function')throw new TypeError(tr("需要可挂载的DOM容器","A mountable DOM container is required"));
 const token={};mounts.get(container)?.destroy?.();mounts.set(container,token);const factory=await loadMeshyModelFactory();if(mounts.get(container)!==token)throw new DOMException(tr("场景已被替换","Scene was replaced"),'AbortError');
 const skin=factory(),info=measureModelGeometry(skin),world=new T.Scene(),camera=new T.PerspectiveCamera(28,1,.05,450);camera.up.set(0,0,1);
 const canvas=document.createElement('canvas'),context=canvas.getContext('webgl2',{antialias:true,alpha:false,preserveDrawingBuffer:true});if(!context){disposeTree(skin);throw Error(tr("此设备无法显示最新三维车模","This device cannot display the latest 3D vehicle model"))}
 const renderer=new T.WebGLRenderer({canvas,context,antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio||1,1.5));renderer.toneMapping=T.ACESFilmicToneMapping;renderer.toneMappingExposure=1.15;renderer.shadowMap.enabled=true;renderer.shadowMap.type=T.PCFSoftShadowMap;
 const mat={base:material(0x253941),structure:material(0x52616a,.42,.55),ramp:material(0x647780,.7),obstacle:material(0x7d8990,.5),glass:material(0x1c303c,.22,.55),rubber:material(0x141a1e,.9),parking:material(0x2c424b,.9)};mat.ramp.side=T.DoubleSide;mat.ramp.shadowSide=T.BackSide;
 const night=createNightRange(world,renderer);mat.ramp.color.set(0x737b81);mat.ramp.metalness=.78;mat.ramp.roughness=.34;
 const body=new T.Group();skin.position.z-=GROUND_CG_METERS;body.add(skin);world.add(body);const stage=new T.Group();world.add(stage);let fixtures;
 if(kind==='shot'){if(scene<=5)buildRamp(stage,mat);else buildParked(stage,mat)}else fixtures=buildBench(stage,mat);
 const forceArrows=Array.from({length:4},()=>{const a=new T.ArrowHelper(new T.Vector3(0,0,1),new T.Vector3(),1,0x72dbe5,.18,.09);world.add(a);a.visible=false;return a});
 // Dimensionless editorial effect strengths, not measured or simulated nozzle forces.
 const visualJets=[];for(const x of [-1.4,1.4])for(const y of [-.64,.64])for(const sign of [1,-1])visualJets.push({position:[x,y,sign===1?-.36:.52],direction:[0,0,sign],maxForce:1});
 const smoke=createJetSmoke(visualJets,{capacity:240});world.add(smoke.object);
 const badge=document.createElement('p');badge.className='latest-scene-badge';badge.textContent=kind==='shot'?tr("最新着色 Meshy · 原片动作概念重建","Latest colored Meshy · conceptual video-maneuver reconstruction"):tr("最新着色 Meshy · 公式驱动试验台","Latest colored Meshy · equation-driven test bench");
 const note=document.createElement('p');note.className='latest-scene-note';note.textContent=kind==='shot'?tr("4.1 m 长度假设 · 原比例 · 夜间场景 · 非实测路径","Assumed 4.1 m length · original proportions · night scene · unmeasured path"):tr("4.1 m 长度假设 · 力箭头仅示方向与相对强弱","Assumed 4.1 m length · arrows show direction and relative magnitude only");
 const controls=document.createElement('div');controls.className='latest-scene-camera';controls.setAttribute('aria-label',tr("三维视角","3D view"));for(const [id,label,text]of [['out',tr("拉远视角","Zoom out"),'−'],['in',tr("拉近视角","Zoom in"),'＋'],['reset',tr("复位视角","Reset view"),tr("复位","Reset")]]){const b=document.createElement('button');b.type='button';b.dataset.camera=id;b.setAttribute('aria-label',label);b.textContent=text;controls.append(b)}
 const hint=document.createElement('p');hint.className='latest-scene-hint';hint.textContent=tr("拖动查看 · 方向键旋转 · ＋/−缩放 · Home复位","Drag to inspect · arrow keys rotate · +/− zoom · Home resets");
 canvas.className='latest-scene-canvas';canvas.tabIndex=0;canvas.setAttribute('role','img');canvas.setAttribute('aria-label',tr("可旋转三维教学场景。方向键旋转，正负键缩放，Home复位。不是实测运动轨迹。","Rotatable 3D teaching scene. Arrow keys rotate, +/− zoom, Home resets. Not a measured trajectory."));container.classList.add('latest-scene');container.replaceChildren(canvas,badge,note,controls,hint);
 let dead=false,raf=0,visible=true,progress=.5,state,azimuth=-.85,elevation=.38,zoom=1,drag=null,currentTheme=themeValue(theme),sizeValid=false,smokeDirty=true;
 const reduced=globalThis.matchMedia?.('(prefers-reduced-motion: reduce)');
 const cameraDefaults=()=>{azimuth=kind==='shot'&&scene<=5?-2.2:-.85;elevation=kind==='bench'?.28:.18;zoom=1};cameraDefaults();
 function render(force=false){if(dead||!state||!sizeValid||(!force&&(!visible||document.hidden)))return;if(smokeDirty){updateSmoke();smokeDirty=false}const focus=v3(state.position);let distance;
  if(kind==='shot'){focus.z=Math.max(1.2,focus.z-.3);distance=scene===2?13.5:scene===7||scene===8?25:10.5;if(scene===7||scene===8){focus.x=mix(focus.x,0,.3);focus.z=3.7}}
  else{focus.set(0,0,model==='suction'?3.7:model==='roll'?2.2:1.1);distance=model==='suction'?12.3:10}
  // Preserve vertical and horizontal framing on a narrow phone canvas.
  distance*=1.3*Math.max(1,1.05/Math.max(camera.aspect,.3))*zoom;camera.position.copy(focus).add(new T.Vector3(Math.cos(azimuth)*Math.cos(elevation),Math.sin(azimuth)*Math.cos(elevation),Math.sin(elevation)).multiplyScalar(distance));camera.lookAt(focus);night.update(camera,progress*8);renderer.render(world,camera);
 }
 function requestRender(){if(dead||raf||!visible||document.hidden)return;raf=requestAnimationFrame(()=>{raf=0;render()})}
 function resize(){if(dead)return;const w=container.clientWidth,h=container.clientHeight;if(w<1||h<1){sizeValid=false;return}sizeValid=true;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();smoke.setPixelScale(h*renderer.getPixelRatio()/(2*Math.tan(camera.fov*PI/360)));requestRender()}
 function setTheme(next){if(dead)return;currentTheme=themeValue(next);container.dataset.theme=currentTheme;requestRender()}
 function updateSmoke(){smoke.reset();const q=new T.Quaternion(state.quaternion[1],state.quaternion[2],state.quaternion[3],state.quaternion[0]),up=new T.Vector3(0,0,1).applyQuaternion(q),strength=kind==='shot'?state.effectFraction:model==='thrust'?Math.min(.65,state.solved.thrust/60000):0,forces=visualJets.map(n=>Math.max(0,n.direction[2]*up.z)*strength);
  smoke.setEnabled(strength>.01);if(strength<=.01)return;const f={pos:state.position,quat:state.quaternion,vel:[0,0,0],nozzleForces:forces};const steps=reduced?.matches?4:6;for(let i=0;i<=steps;i++)smoke.update({...f,t:i*.04});
 }
 function draw(){if(dead)return;body.position.set(...state.position);body.quaternion.set(state.quaternion[1],state.quaternion[2],state.quaternion[3],state.quaternion[0]);forceArrows.forEach(a=>a.visible=false);
  if(kind==='bench'){
   for(const [name,g]of Object.entries(fixtures.sets))g.visible=name===model;fixtures.pivot.quaternion.copy(qAxis([0,1,0],-(state.solved.theta??0)*PI/180));fixtures.air.forEach(a=>{a.visible=(state.solved.velocity??0)>0;a.setLength(.4+Math.min(2.4,(state.solved.velocity??0)/25),.2,.09)});
   state.forceVectors.forEach((f,i)=>{const length=Math.hypot(...f.vector),arrow=forceArrows[i];arrow.visible=length>1e-7;if(arrow.visible){arrow.position.set(...f.origin);arrow.setDirection(v3(f.vector).normalize());arrow.setColor(f.color);arrow.setLength(.2+Math.min(2.3,Math.sqrt(length/10000)),.18,.09)}});
  }
  smokeDirty=true;const text=(kind==='shot'?`${state.title} · ${Math.round(progress*100)}% · ${state.description}`:state.description)+tr("。教学示意，非Tesla实测或仿真结果。",". Teaching illustration, not Tesla measurements or simulation results.");canvas.setAttribute('aria-label',text+tr(" 方向键旋转，正负键缩放，Home复位。"," Arrow keys rotate, +/− zoom, Home resets."));onReadout(text,state);requestRender();
 }
 function setProgress(next){if(dead)return;progress=clamp(finite(next,tr("进度","Progress")),0,1);if(kind==='shot')state=shotState(scene,progress,{rampOffset:info.rampOffset});else if(model==='roll')state=benchState(model,{...state.inputs,progress});else return;draw()}
 function setState(nextModel,nextValue={}){if(dead)return;if(kind!=='bench')throw Error(tr("镜头展示不接受公式参数","Shot displays do not accept equation parameters"));const next=benchState(nextModel,nextValue);model=nextModel;state=next;draw()}
 const reset=()=>{cameraDefaults();requestRender()};const zoomBy=f=>{zoom=clamp(zoom*f,.65,2.4);requestRender()};
 function pointerDown(e){if(e.button!==0&&e.pointerType==='mouse')return;canvas.focus({preventScroll:true});drag={id:e.pointerId,x:e.clientX,y:e.clientY,az:azimuth,el:elevation,touch:e.pointerType==='touch',active:e.pointerType!=='touch'};if(drag.active)canvas.setPointerCapture(e.pointerId)}
 function pointerMove(e){if(!drag||drag.id!==e.pointerId)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(!drag.active){if(Math.abs(dx)<8||Math.abs(dx)<Math.abs(dy))return;drag.active=true;canvas.setPointerCapture(e.pointerId)}azimuth=drag.az-dx*.007;elevation=clamp(drag.el+dy*.005,.06,1.35);requestRender()}
 const pointerEnd=()=>drag=null;
 function wheel(e){if(document.activeElement!==canvas)return;e.preventDefault();zoomBy(Math.exp(e.deltaY*.001))}
 function key(e){let handled=true;switch(e.key){case'ArrowLeft':azimuth+=.1;break;case'ArrowRight':azimuth-=.1;break;case'ArrowUp':elevation=clamp(elevation+.08,.06,1.35);break;case'ArrowDown':elevation=clamp(elevation-.08,.06,1.35);break;case'+':case'=':zoomBy(.9);break;case'-':zoomBy(1.1);break;case'Home':reset();break;default:handled=false}if(handled){e.preventDefault();requestRender()}}
 const listeners=[];function listen(el,type,fn,options){el.addEventListener(type,fn,options);listeners.push(()=>el.removeEventListener(type,fn,options))}
 listen(canvas,'pointerdown',pointerDown);listen(canvas,'pointermove',pointerMove);listen(canvas,'pointerup',pointerEnd);listen(canvas,'pointercancel',pointerEnd);listen(canvas,'lostpointercapture',pointerEnd);listen(canvas,'wheel',wheel,{passive:false});listen(canvas,'keydown',key);listen(controls,'click',e=>{const id=e.target.closest('button')?.dataset.camera;if(id==='in')zoomBy(.85);if(id==='out')zoomBy(1.15);if(id==='reset')reset()});listen(document,'visibilitychange',()=>{if(document.hidden){cancelAnimationFrame(raf);raf=0}else requestRender()});if(reduced?.addEventListener)listen(reduced,'change',()=>{smokeDirty=true;requestRender()});
 const ro=globalThis.ResizeObserver?new ResizeObserver(resize):null;ro?.observe(container);if(!ro)listen(globalThis,'resize',resize);const io=globalThis.IntersectionObserver?new IntersectionObserver(entries=>{visible=entries.at(-1).isIntersecting;if(visible)requestRender();else{cancelAnimationFrame(raf);raf=0}}):null;io?.observe(container);
 async function exportPNG(){if(dead)throw Error(tr("场景已关闭","Scene is closed"));resize();if(!sizeValid)throw Error(tr("场景没有可导出的尺寸","Scene has no exportable dimensions"));render(true);const out=document.createElement('canvas');out.width=canvas.width;out.height=canvas.height;const ctx=out.getContext('2d');ctx.drawImage(canvas,0,0);const pad=Math.max(12,Math.round(out.width*.018)),height=Math.max(48,Math.round(out.height*.13));ctx.fillStyle=currentTheme==='light'?'rgba(235,240,239,.94)':'rgba(18,32,41,.94)';ctx.fillRect(0,out.height-height,out.width,height);ctx.fillStyle=currentTheme==='light'?'#24373c':'#d4e2e5';ctx.font=`${Math.max(11,Math.round(out.width*.017))}px system-ui,sans-serif`;ctx.fillText(tr("最新着色 Meshy · 4.1 m 长度假设","Latest colored Meshy · assumed 4.1 m length"),pad,out.height-height*.59);ctx.fillText(kind==='shot'?tr("动作与路径为教学示意，非实测轨迹","Maneuver and path are teaching illustrations, not measured trajectories"):tr("公式参数示意，非Tesla实测或动态仿真结果","Equation parameter illustration, not Tesla measurements or dynamic simulation"),pad,out.height-height*.23);return new Promise((resolve,reject)=>out.toBlob(blob=>blob?resolve(blob):reject(Error(tr("PNG导出失败","PNG export failed"))),'image/png'))}
 function destroy(){if(dead)return;dead=true;cancelAnimationFrame(raf);ro?.disconnect();io?.disconnect();listeners.forEach(f=>f());disposeTree(world);night.dispose();renderer.dispose();renderer.forceContextLoss();if(mounts.get(container)===api){mounts.delete(container);container.replaceChildren();container.classList.remove('latest-scene')}}
 const api={setProgress,setTheme,setState,exportPNG,destroy,getState:()=>state,geometry:info,appearance:'meshy-colored',resetView:reset};mounts.set(container,api);setTheme(theme);if(kind==='shot')setProgress(.5);else setState(model,value);resize();return api;
}
export async function mountShot(container,options={}){shotState(options.scene??1,.5);return mount(container,{...options,kind:'shot'})}
export async function mountBench(container,options={}){benchState(options.model??'suction',options.value??{});return mount(container,{...options,kind:'bench'})}
