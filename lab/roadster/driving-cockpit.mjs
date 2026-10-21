import * as T from './vendor/three.module.min.js';

// Top Gear JdkIOhFmbKI, 34:30–37:10: low dark dash, rectangular yoke, ivory grips.
// shortcut: proportions are inferred from video, replace with measured cabin geometry when available.
export function createDrivingCockpit(){
 const scene=new T.Scene(),camera=new T.PerspectiveCamera(55,1,.05,10);
 const shell=new T.Group(),wheel=new T.Group(),frame=new T.Group();scene.add(shell,wheel,frame);
 const leather=new T.MeshStandardMaterial({color:0x202329,roughness:.91});
 const weavePixels=new Uint8Array(32*32*4);
 for(let y=0;y<32;y++)for(let x=0;x<32;x++){const horizontal=((x>>3)+(y>>3))%2===0,strand=(horizontal?y:x)%8,shade=170+Math.round(45*Math.sin(strand*Math.PI/8));weavePixels.set([shade,shade,shade,255],(y*32+x)*4);}
 const weave=new T.DataTexture(weavePixels,32,32);weave.wrapS=weave.wrapT=T.RepeatWrapping;weave.repeat.set(24,24);weave.needsUpdate=true;
 const carbon=new T.MeshStandardMaterial({color:0x181c22,map:weave,roughness:.45,metalness:.32});
 const edge=new T.MeshStandardMaterial({color:0x525c65,roughness:.34,metalness:.72});
 const ivory=new T.MeshStandardMaterial({color:0xd9d7cb,roughness:.7});
 const black=new T.MeshStandardMaterial({color:0x080a0e,roughness:.56});
 const seam=new T.MeshStandardMaterial({color:0x7b807e,roughness:.9});
 const ambient=new T.HemisphereLight(0xdbe7f4,0x15151b,2.1);scene.add(ambient);
 const key=new T.DirectionalLight(0xe8e7dc,2.6);key.position.set(-2,3,2);scene.add(key);
 const fill=new T.DirectionalLight(0x879db9,.75);fill.position.set(3,.4,0);scene.add(fill);

 function box(parent,size,pos,material){const m=new T.Mesh(new T.BoxGeometry(...size),material);m.position.set(...pos);parent.add(m);return m;}
 function panel(parent,points,depth,pos,material,bevel=.008){
  const shape=new T.Shape();points.forEach(([x,y],i)=>i?shape.lineTo(x,y):shape.moveTo(x,y));shape.closePath();
  const geometry=new T.ExtrudeGeometry(shape,{depth,bevelEnabled:bevel>0,bevelThickness:bevel,bevelSize:bevel,bevelSegments:2,steps:1,curveSegments:6});
  const mesh=new T.Mesh(geometry,material);mesh.position.set(...pos);parent.add(mesh);return mesh;
 }
 function tube(parent,points,radius,material){
  const path=new T.CatmullRomCurve3(points.map(p=>new T.Vector3(...p)),false,'centripetal');
  const mesh=new T.Mesh(new T.TubeGeometry(path,Math.max(4,points.length*4),radius,6,false),material);parent.add(mesh);return mesh;
 }
 // A broad, tapered dashboard with a very low windshield sill.
 const dash=panel(shell,[[-1.9,-.62],[2.3,-.62],[2.3,-.44],[1.5,-.33],[.6,-.305],[-.6,-.305],[-1.9,-.39]],.74,[0,0,-1.75],leather,.018);
 dash.rotation.x=-.065;
 tube(shell,[[-1.85,-.336,-1.72],[-.7,-.291,-1.7],[.5,-.29,-1.7],[1.6,-.32,-1.7],[2.2,-.4,-1.7]],.006,edge);
 box(shell,[3.9,.025,.13],[.2,-.465,-1.03],black);
 tube(shell,[[-1.85,-.46,-1.01],[-.5,-.442,-1.01],[.6,-.444,-1.01],[2.2,-.47,-1.01]],.0035,seam);
 panel(shell,[[-1.8,-1.1],[2.2,-1.1],[2.2,-.57],[.5,-.47],[-.6,-.48],[-1.8,-.57]],.10,[0,0,-1.02],carbon);
 // Passenger-side fold and pale door trim are visible only at the edge of the driver's view.
 panel(shell,[[.36,-1.1],[.69,-1.1],[.58,-.5],[.45,-.44],[.37,-.49]],.08,[0,0,-.94],carbon);
 panel(shell,[[-1.5,-1.1],[-.72,-1.1],[-.85,-.57],[-1.2,-.47],[-1.5,-.5]],.065,[0,0,-.82],ivory,.012);
 tube(shell,[[-1.35,-.54,-.72],[-1.0,-.59,-.72],[-.87,-.76,-.72]],.012,black);

 // The thin windscreen surround leaves the horizon and ramp entry unobstructed.
 for(const side of [-1,1]){
  const pillar=new T.Group();frame.add(pillar);
  tube(pillar,[[side*1.26,-.34,-1.5],[side*1.12,.07,-1.35],[side*.73,.54,-1.08]],.035,carbon);
  tube(pillar,[[side*1.235,-.34,-1.5],[side*1.095,.07,-1.35],[side*.705,.54,-1.08]],.006,edge);
 }
 tube(frame,[[-.74,.54,-1.08],[0,.56,-1.1],[.74,.54,-1.08]],.035,carbon);

 wheel.name='cockpit-yoke';wheel.position.set(0,-.27,-.86);wheel.rotation.x=-.14;
 box(wheel,[.115,.09,.25],[0,-.025,-.15],black);
 // Rounded, chamfered central pad and open shoulders; no conventional circular rim.
 panel(wheel,[[-.145,.076],[-.106,.103],[.106,.103],[.145,.076],[.128,-.065],[.087,-.095],[-.087,-.095],[-.128,-.065]],.045,[0,0,0],edge,.009);
 panel(wheel,[[-.136,.071],[-.101,.094],[.101,.094],[.136,.071],[.12,-.061],[.083,-.085],[-.083,-.085],[-.12,-.061]],.046,[0,0,.008],carbon,.009);
 for(const side of [-1,1]){
  const grip=panel(wheel,[[-.034,-.115],[.027,-.109],[.04,.096],[.021,.128],[-.03,.119],[-.045,.077]],.058,[side*.214,0,.015],ivory,.014);
  grip.scale.set(side*.72,.96,1);
  tube(wheel,[[side*.14,.044,.006],[side*.183,.102,-.018],[side*.232,.139,-.024],[side*.26,.12,-.018],[side*.265,-.076,.004],[side*.224,-.125,.012]],.014,carbon);
  tube(wheel,[[side*.264,.09,.014],[side*.262,-.075,.025],[side*.232,-.115,.034]],.0027,edge);
  box(wheel,[.04,.025,.012],[side*.171,.03,.045],black);
 }
 // Small simulation display integrated into the yoke. It is not a claim about Tesla's UI.
 const canvas=document.createElement('canvas');canvas.width=512;canvas.height=192;
 const ctx=canvas.getContext('2d'),texture=new T.CanvasTexture(canvas);texture.colorSpace=T.SRGBColorSpace;
 const screen=new T.Mesh(new T.PlaneGeometry(.217,.081),new T.MeshBasicMaterial({map:texture,toneMapped:false}));screen.position.set(0,.026,.065);wheel.add(screen);
 tube(wheel,[[-.065,-.045,.065],[0,-.051,.069],[.065,-.045,.065]],.0015,seam);
 let lastDisplay='',visible=false,lastDrawCalls=0;
 function update(f){
  wheel.rotation.z=(f.steeringAngle??0)*4;
  const speed=Math.round(Math.abs(f.speed??0)),gear=f.gear??'D',gas=Math.round((f.gas?.fraction??1)*100),active=!!f.recoveryActive;
  const text=`${speed}|${gear}|${gas}|${active}`;if(text===lastDisplay)return;lastDisplay=text;
  ctx.fillStyle='#080c10';ctx.fillRect(0,0,512,192);
  ctx.textAlign='left';ctx.fillStyle='#adc4bc';ctx.font='19px system-ui';ctx.fillText('ROADSTER  /  SIM',22,29);
  ctx.fillStyle='#eaf2ef';ctx.font='300 82px system-ui';ctx.fillText(String(speed),20,119);
  ctx.fillStyle='#a8b8b9';ctx.font='19px system-ui';ctx.fillText('km/h',180,116);
  ctx.textAlign='right';ctx.font='38px system-ui';ctx.fillStyle='#cee0b0';ctx.fillText(gear,482,87);
  ctx.font='18px system-ui';ctx.fillStyle=active?'#d7cfab':'#98aaa9';ctx.fillText(active?'JET ACTIVE':`GAS ${gas}%`,482,122);
  ctx.fillStyle='#26343b';ctx.fillRect(22,155,460,3);ctx.fillStyle='#c7d9b2';ctx.fillRect(22,155,460*Math.min(1,speed/240),3);texture.needsUpdate=true;
 }
 function resize(aspect){
  camera.aspect=aspect;camera.updateProjectionMatrix();
  frame.scale.x=aspect/(16/9);
  const scale=Math.min(1,aspect/1.25);wheel.scale.setScalar(.72*scale);
  wheel.position.y=-.27-(1-scale)*.055;
 }
 function render(renderer){
  const autoClear=renderer.autoClear;renderer.autoClear=false;renderer.clearDepth();
  try{renderer.render(scene,camera);lastDrawCalls=renderer.info.render.calls;}finally{renderer.autoClear=autoClear;}
 }
 function dispose(){
  const geometries=new Set(),materials=new Set();scene.traverse(o=>{if(o.geometry)geometries.add(o.geometry);if(o.material)materials.add(o.material);});
  geometries.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());texture.dispose();weave.dispose();
 }
 update({});
 return {scene,camera,wheel,update,resize,render,dispose,setVisible(value){visible=value;},stats(){let triangles=0,meshes=0;scene.traverse(o=>{if(o.isMesh){meshes++;triangles+=(o.geometry.index?.count??o.geometry.attributes.position.count)/3;}});return {visible,meshes,triangles,drawCalls:lastDrawCalls,steeringRotation:wheel.rotation.z,display:lastDisplay,textureSize:[512,192]};}};
}
