import * as T from './vendor/three.module.min.js';
import {MAP} from './driving-terrain.mjs?v=20261022-rocket10';

export const PERIMETER_LIGHTS=[...[-5,-3,-1,1,3,5].flatMap(x=>[-1,1].map(side=>[x*MAP.halfX/6,side*(MAP.halfY-8)])),...[-3,-1,1,3].flatMap(y=>[-1,1].map(side=>[side*(MAP.halfX-8),y*MAP.halfY*5/24]))];

export function createRangeKeyLight(driving=false){
 const light=new T.DirectionalLight(0xfff5e9,2.25),extent=driving?Math.hypot(MAP.halfX,MAP.halfY)+24:60;
 light.castShadow=true;light.shadow.mapSize.set(driving?4096:1024,driving?4096:1024);
 light.position.set(-28,-38,48).normalize().multiplyScalar(extent*3);
 Object.assign(light.shadow.camera,{left:-extent,right:extent,top:extent,bottom:-extent,near:extent,far:extent*5});
 light.shadow.camera.updateProjectionMatrix();
 light.shadow.normalBias=driving?.08:.025;light.shadow.bias=-.00004;return light;
}

// Scenery and lighting only. Nothing here enters the physics world.
export function createNightRange(scene,renderer,{driving=false}={}){
 const range=new T.Group();range.name='夜间工业测试场';scene.add(range);
 scene.background=new T.Color(0x070b12);scene.fog=new T.FogExp2(0x101721,driving?.004:.006);
 renderer.toneMapping=T.ACESFilmicToneMapping;renderer.toneMappingExposure=1.08;
 renderer.shadowMap.enabled=true;renderer.shadowMap.type=T.PCFSoftShadowMap;
 const mesh=(geometry,material,position)=>{const o=new T.Mesh(geometry,material);o.position.set(...position);range.add(o);return o};
 const steel=new T.MeshStandardMaterial({color:0x49515a,metalness:.72,roughness:.38});
 const concrete=new T.MeshStandardMaterial({color:0x262a30,roughness:.88,metalness:.06});
 const pixels=new Uint8Array(128*128*4);let seed=731;
 for(let i=0;i<pixels.length;i+=4){seed=(Math.imul(seed,1664525)+1013904223)>>>0;const n=116+(seed>>>27);pixels.set([n,n,n,255],i)}
 const grain=new T.DataTexture(pixels,128,128);grain.wrapS=grain.wrapT=T.RepeatWrapping;grain.repeat.set(90,90);grain.needsUpdate=true;
 concrete.bumpMap=grain;concrete.bumpScale=.018;
 const floor=mesh(new T.PlaneGeometry(1000,1000),concrete,[0,0,-.035]);floor.name='测试场混凝土地面';floor.receiveShadow=true;
 const seams=[];for(let x=-MAP.halfX;x<=MAP.halfX;x+=12)seams.push(x,-MAP.halfY,-.032,x,MAP.halfY,-.032);for(let y=-MAP.halfY;y<=MAP.halfY;y+=12)seams.push(-MAP.halfX,y,-.032,MAP.halfX,y,-.032);const joints=new T.BufferGeometry();joints.setAttribute('position',new T.Float32BufferAttribute(seams,3));range.add(new T.LineSegments(joints,new T.LineBasicMaterial({color:0x161a20,transparent:true,opacity:.28})));
 const hemisphere=new T.HemisphereLight(0xaebfd7,0x36302b,.5);range.add(hemisphere);
 const key=createRangeKeyLight(driving),shadowSize=Math.min(key.shadow.mapSize.x,renderer.capabilities.maxTextureSize,globalThis.matchMedia?.('(pointer: coarse)').matches?2048:4096);key.shadow.mapSize.set(shadowSize,shadowSize);
 range.add(key,key.target);const fill=new T.DirectionalLight(0xc3d6ef,.65);fill.position.set(18,30,24);range.add(fill);
 const r=driving?132:35,lamps=[],ray=new T.Vector3();
 const glowCanvas=document.createElement('canvas');glowCanvas.width=glowCanvas.height=128;const ctx=glowCanvas.getContext('2d'),gradient=ctx.createRadialGradient(64,64,0,64,64,64);
 gradient.addColorStop(0,'rgba(255,248,226,.9)');gradient.addColorStop(.06,'rgba(255,246,220,.65)');gradient.addColorStop(.2,'rgba(226,228,225,.12)');gradient.addColorStop(1,'rgba(205,222,245,0)');ctx.fillStyle=gradient;ctx.fillRect(0,0,128,128);
 ctx.fillStyle='rgba(245,239,219,.1)';ctx.fillRect(6,63,116,2);ctx.fillRect(63,6,2,116);const glow=new T.CanvasTexture(glowCanvas);
 if(driving){
  const poles=new T.InstancedMesh(new T.CylinderGeometry(.09,.16,9,8),steel,PERIMETER_LIGHTS.length),heads=new T.InstancedMesh(new T.BoxGeometry(1.3,.6,.22),new T.MeshBasicMaterial({color:0xffdfad,toneMapped:false}),PERIMETER_LIGHTS.length),pools=new T.InstancedMesh(new T.PlaneGeometry(24,24),new T.MeshBasicMaterial({map:glow,color:0xe0c69c,transparent:true,opacity:.22,blending:T.AdditiveBlending,depthWrite:false}),PERIMETER_LIGHTS.length),pose=new T.Object3D();
  poles.name='场地周边稀疏灯杆';heads.name='场地周边灯头';pools.name='灯下柔光';range.add(poles,heads,pools);
  PERIMETER_LIGHTS.forEach(([x,y],i)=>{
   pose.position.set(x,y,4.5);pose.rotation.set(Math.PI/2,0,0);pose.updateMatrix();poles.setMatrixAt(i,pose.matrix);
   pose.position.z=9;pose.rotation.set(0,0,0);pose.updateMatrix();heads.setMatrixAt(i,pose.matrix);
   pose.position.z=-.025;pose.updateMatrix();pools.setMatrixAt(i,pose.matrix);
   const flare=new T.Sprite(new T.SpriteMaterial({map:glow,color:i%4===0?0xb8d4ff:0xffd7a2,transparent:true,blending:T.AdditiveBlending,depthWrite:false,toneMapped:false,fog:false,opacity:.8}));flare.name='周边灯光晕';flare.position.set(x,y,9);flare.scale.setScalar(8);range.add(flare);
  });
 }
 for(const [x,y,z] of (driving?[[MAP.halfX+45,-46,32],[MAP.halfX+55,54,36],[-MAP.halfX-25,-60,30]]:[[-.45*r,-.82*r,.65*r],[.65*r,.82*r,.55*r],[1.05*r,-.82*r,.6*r]])){
  mesh(new T.CylinderGeometry(.11,.24,z,10),steel,[x,y,z/2]).rotation.x=Math.PI/2;
  mesh(new T.BoxGeometry(2.5,2.5,.8),concrete,[x,y,.4]);
  const head=mesh(new T.BoxGeometry(3,.5,1.2),steel,[x,y,z]);head.up.set(0,0,1);head.lookAt(0,0,3);
  const lamp=new T.Mesh(new T.PlaneGeometry(2.7,.8),new T.MeshBasicMaterial({color:0xfff5db,toneMapped:false}));lamp.position.z=.61;head.add(lamp);
  const flare=new T.Sprite(new T.SpriteMaterial({map:glow,transparent:true,blending:T.AdditiveBlending,depthWrite:false,toneMapped:false,opacity:.65}));flare.position.set(x,y,z);flare.scale.setScalar(driving?12:6);range.add(flare);lamps.push({flare,normal:new T.Vector3(-x,-y,3-z).normalize()});
  const spot=new T.SpotLight(0xfff4e8,driving?26000:3800,r*3,.65,.8,2);spot.position.set(x,y,z);spot.target.position.set(driving?14:0,0,0);range.add(spot,spot.target);
 }
 const studio=new T.Scene();studio.background=new T.Color(0x151b25);
 for(const [position,w,h,intensity] of [[[-5,-8,7],10,2,5],[[7,4,9],6,3,2],[[0,0,12],8,1,3]]){const panel=new T.Mesh(new T.PlaneGeometry(w,h),new T.MeshBasicMaterial({color:new T.Color(intensity,intensity*.96,intensity*.87),side:T.DoubleSide}));panel.position.set(...position);panel.lookAt(0,0,0);studio.add(panel)}
 const pmrem=new T.PMREMGenerator(renderer),env=pmrem.fromScene(studio,.03);scene.environment=env.texture;scene.environmentIntensity=.5;pmrem.dispose();studio.traverse(o=>{o.geometry?.dispose();o.material?.dispose()});
 const sky=new T.Mesh(new T.SphereGeometry(440,24,12),new T.ShaderMaterial({side:T.BackSide,depthWrite:false,fog:false,toneMapped:false,
  vertexShader:'varying vec3 direction;void main(){direction=position;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);gl_Position.z=gl_Position.w;}',
  fragmentShader:`varying vec3 direction;
   float hash(vec2 p){return fract(sin(dot(p,vec2(127.1,311.7)))*43758.5453);}
   float noise(vec2 p){vec2 i=floor(p),f=fract(p);f=f*f*(3.0-2.0*f);return mix(mix(hash(i),hash(i+vec2(1,0)),f.x),mix(hash(i+vec2(0,1)),hash(i+vec2(1,1)),f.x),f.y);}
   void main(){
    vec3 d=normalize(direction);float h=max(d.z,0.0),az=atan(d.y,d.x)/6.2831853+.5;
    vec3 color=mix(vec3(.043,.056,.079),vec3(.008,.013,.025),pow(h,.4));
    // Blurred distant lights set the atmosphere; fixed perimeter lamps provide motion parallax.
    float block=floor(az*240.0),height=.018+.045*hash(vec2(block,8.0));
    vec2 facade=vec2(az*240.0,d.z*75.0),cell=floor(facade),window=fract(facade)-.5;
    float skyline=smoothstep(-.004,.005,d.z)*(1.0-smoothstep(height-.01,height+.01,d.z));
    float lights=(exp(-dot(window,window)*24.0)+.15*exp(-dot(window,window)*5.0))*step(.60,hash(cell+2.0))*skyline;
    color+=mix(vec3(.62,.34,.13),vec3(.22,.39,.63),step(.72,hash(cell+7.0)))*lights;
    color+=vec3(.035,.026,.019)*exp(-abs(d.z-.008)*40.0);
    vec2 starGrid=vec2(az,asin(clamp(d.z,-1.0,1.0))/3.1415927+.5)*vec2(360.0,180.0),starCell=floor(starGrid);
    vec2 offset=vec2(hash(starCell+1.7),hash(starCell+9.2))*.5+.25;
    vec2 star=(fract(starGrid)-offset)/max(fwidth(starGrid),vec2(.015));
    color+=vec3(.50,.57,.7)*exp(-dot(star,star)*1.4)*step(.993,hash(starCell))*smoothstep(.04,.12,d.z);
    float moonDistance=length(d-normalize(vec3(1.0,.25,.25))),moon=1.0-smoothstep(.011,.013,moonDistance);
    vec2 lunar=(d.yz-normalize(vec3(1.0,.25,.25)).yz)*400.0;
    color+=vec3(.8,.85,.92)*moon*(.9-.2*noise(lunar)-.07*noise(lunar*2.7));
    color+=vec3(.08,.11,.17)*exp(-moonDistance*30.0);
    gl_FragColor=vec4(color,1.0);
   }`
 }));sky.name='远景灯光与稀疏星空';sky.renderOrder=-100;sky.frustumCulled=false;range.add(sky);
 const reduced=globalThis.matchMedia?.('(prefers-reduced-motion: reduce)');
 function update(camera,time=0,{handheld=false}={}){
  sky.position.copy(camera.position);
  // Driving shadows cover the complete map and never follow the camera.
  if(!driving){camera.getWorldDirection(ray);const target=camera.position.clone().addScaledVector(ray,15);target.z=Math.max(0,target.z);key.target.position.copy(target);key.position.copy(target).add(new T.Vector3(-28,-38,48).normalize().multiplyScalar(180));}
  if(handheld&&!reduced?.matches){camera.position.x+=Math.sin(time*1.7)*.012;camera.position.z+=Math.sin(time*2.3+.4)*.009;camera.rotateZ(Math.sin(time*1.3)*.00045)}
  for(const {flare,normal} of lamps){ray.copy(camera.position).sub(flare.position).normalize();flare.material.opacity=.6*Math.max(0,ray.dot(normal))**2;}
 }
 function dispose(){env.dispose();range.traverse(o=>{o.shadow?.dispose()});scene.environment=null;}
 return {object:range,floor,update,dispose};
}
