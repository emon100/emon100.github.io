import * as T from './vendor/three.module.min.js';

// Outer proportions are assumptions; the shell follows official views, wheels follow the flying prototype.
export function roadsterModel([L,W,H]) {
  const model=new T.Group();model.name='Rocket 外观研究模型';
  const materials={
    paint:new T.MeshPhysicalMaterial({color:0xdde1df,metalness:.48,roughness:.29,clearcoat:.8,clearcoatRoughness:.2,side:T.DoubleSide}),
    carbon:new T.MeshStandardMaterial({color:0x202a2d,metalness:.28,roughness:.62,side:T.DoubleSide}),
    rubber:new T.MeshStandardMaterial({color:0x14191b,roughness:.93}),
    metal:new T.MeshStandardMaterial({color:0x9ba7aa,metalness:.82,roughness:.29}),
    wheel:new T.MeshStandardMaterial({color:0xa8b2b5,metalness:.57,roughness:.4}),
    glass:new T.MeshPhysicalMaterial({color:0x272423,metalness:.4,roughness:.15,clearcoat:1,side:T.DoubleSide}),
    cavity:new T.MeshStandardMaterial({color:0x080e11,roughness:.9,side:T.DoubleSide}),
    red:new T.MeshStandardMaterial({color:0xc82f25,emissive:0xc72c22,emissiveIntensity:.5,roughness:.3}),
    white:new T.MeshStandardMaterial({color:0xf5f1de,emissive:0xe9e6ce,emissiveIntensity:.6}),
    amber:new T.MeshStandardMaterial({color:0xd9782d,emissive:0xae4218,emissiveIntensity:.2}),
  };
  const vec=a=>new T.Vector3(...a),p=(x,y,z)=>[x*L,y*W,z*H];
  function mesh(geometry,material,name,pos=[0,0,0],parent=model){const m=new T.Mesh(geometry,materials[material]);m.name=name;m.position.set(...pos);parent.add(m);return m;}
  function block(size,pos,material,name){return mesh(new T.BoxGeometry(...size),material,name,pos)}
  function panel(points,material,name){const g=new T.BufferGeometry(),indices=[];for(let i=1;i<points.length-1;i++)indices.push(0,i,i+1);g.setAttribute('position',new T.Float32BufferAttribute(points.flat(),3));g.setIndex(indices);g.computeVertexNormals();return mesh(g,material,name)}
  function tube(points,r,material,name){const curve=new T.CatmullRomCurve3(points.map(vec));return mesh(new T.TubeGeometry(curve,Math.max(8,points.length*6),r,6,false),material,name)}
  function torus(radius,thickness,pos,axis,material,name){const m=mesh(new T.TorusGeometry(radius,thickness,8,64),material,name,pos);if(axis==='x')m.rotation.y=Math.PI/2;if(axis==='y')m.rotation.x=Math.PI/2;return m;}
  function cylinder(radius,depth,pos,axis,material,name,open=false){const m=mesh(new T.CylinderGeometry(radius,radius,depth,64,1,open),material,name,pos);if(axis==='x')m.rotation.z=Math.PI/2;if(axis==='z')m.rotation.x=Math.PI/2;return m;}

  const R=H*.28,wheelZ=-H/2+R,archR=R+H*.025,axles=[-.30*L,.30*L];
  // Each side is a real cut-out profile. No body box passes through the visible tyres.
  const flank=new T.Shape();flank.moveTo(-.497*L,-.34*H);
  for(const x of axles){flank.lineTo(x-archR-.006,-.34*H);flank.lineTo(x-archR,wheelZ);flank.absarc(x,wheelZ,archR,Math.PI,0,true);flank.lineTo(x+archR+.006,-.34*H)}
  flank.lineTo(.497*L,-.30*H);flank.lineTo(.495*L,-.08*H);flank.quadraticCurveTo(.38*L,.13*H,.28*L,.16*H);flank.quadraticCurveTo(.06*L,.14*H,-.18*L,.19*H);flank.quadraticCurveTo(-.37*L,.25*H,-.497*L,.19*H);flank.closePath();
  const flankGeo=new T.ExtrudeGeometry(flank,{depth:W*.04,bevelEnabled:true,bevelSize:.003,bevelThickness:.003,bevelSegments:2,curveSegments:32});flankGeo.rotateX(Math.PI/2);
  for(const sign of [-1,1]){const f=mesh(flankGeo.clone(),'paint',sign>0?'左翼子板与真实轮拱':'右翼子板与真实轮拱',[0,sign*W*.477,0]);f.scale.y=sign;
    for(const x of axles){const arc=[];for(let i=0;i<=32;i++){const a=i*Math.PI/32;arc.push([x+archR*Math.cos(a),sign*W*.479,wheelZ+archR*Math.sin(a)])}tube(arc,.006,'carbon','轮拱内缘');}
    panel([p(-.49,sign*.326,.19),p(-.35,sign*.477,.22),p(-.18,sign*.477,.19),p(-.20,sign*.32,.19)],'paint','后肩折面');
    panel([p(.20,sign*.34,.12),p(.28,sign*.477,.16),p(.495,sign*.477,-.08),p(.495,sign*.33,-.08)],'paint','前翼折面');
    panel([p(-.20,sign*.32,.19),p(-.20,sign*.477,.19),p(.20,sign*.477,.145),p(.20,sign*.34,.12)],'paint','座舱外侧肩面');
    // Door seam follows the long tapered side, rather than drawing a box on it.
    tube([p(.15,sign*.480,.145),p(.13,sign*.480,-.08),p(.11,sign*.480,-.335)],.0022,'cavity','前门缝');
    tube([p(-.17,sign*.480,.188),p(-.19,sign*.480,-.08),p(-.21,sign*.480,-.335)],.0022,'cavity','后门缝');
    block([L*.053,.006,H*.013],p(-.13,sign*.482,.135),'metal','嵌平门把手');
    block([L*.058,.009,H*.018],p(.19,sign*.481,.03),'cavity','侧向灯槽');
    block([L*.025,.011,H*.012],p(.20,sign*.484,.03),'amber','前侧标识灯');
    block([L*.52,W*.06,H*.062],p(0,sign*.44,-.383),'carbon','原型车外露侧裙');
  }
  // The centre tub is narrower than the tyre inner faces and stays below the glasshouse.
  block([L*.72,W*.60,H*.32],p(-.07,0,-.18),'carbon','中央底盘包络');
  panel([p(.20,-.34,.12),p(.20,.34,.12),p(.495,.33,-.08),p(.495,-.33,-.08)],'paint','梯形前盖');
  panel([p(-.49,-.326,.19),p(-.49,.326,.19),p(-.20,.32,.19),p(-.20,-.32,.19)],'paint','后甲板');
  panel([p(.497,-.33,-.08),p(.497,.33,-.08),p(.489,.35,-.21),p(.489,-.35,-.21)],'paint','前鼻梁');
  panel([p(.490,-.35,-.21),p(.490,.35,-.21),p(.486,.39,-.335),p(.486,-.39,-.335)],'cavity','前下部进气开口');
  block([L*.037,W*.86,H*.027],p(.479,0,-.36),'carbon','前分流板');
  block([.008,W*.66,.009],p(.499,0,-.086),'white','贯穿前灯细线');
  for(const sign of [-1,1])panel([p(.43,sign*.36,.01),p(.36,sign*.466,.09),p(.425,sign*.453,-.015)],'white','折角前灯');
  for(const y of [-.26,-.13,0,.13,.26])block([.012,.012,H*.09],p(.49,y,-.274),'carbon','前进气口分隔');

  // A continuous, curved canopy is shared by the windscreen and side glass.
  const nodes=[[-.30,.225,.205,.19],[-.23,.30,.39,.18],[-.12,.32,.477,.16],[-.015,.33,.486,.13],[.09,.342,.35,.12],[.205,.34,.125,.12]];
  const roofCurve=new T.CatmullRomCurve3(nodes.map(([x,w,z])=>vec([x*L,w*W,z*H]))),beltCurve=new T.CatmullRomCurve3(nodes.map(([x,w,z,b])=>vec([x*L,w*W,b*H])));
  const positions=[],uv=[],indices=[],nx=48,ny=28;
  for(let i=0;i<=nx;i++){const roof=roofCurve.getPoint(i/nx),belt=beltCurve.getPoint(i/nx);for(let j=0;j<=ny;j++){const u=j/ny*2-1;positions.push(roof.x,u*roof.y,belt.z+(roof.z-belt.z)*Math.pow(Math.cos(u*Math.PI/2),.45));uv.push(i/nx,j/ny)}}
  for(let i=0;i<nx;i++)for(let j=0;j<ny;j++){const a=i*(ny+1)+j,b=a+ny+1;indices.push(a,b,a+1,b,b+1,a+1)}
  const glass=new T.BufferGeometry();glass.setAttribute('position',new T.Float32BufferAttribute(positions,3));glass.setAttribute('uv',new T.Float32BufferAttribute(uv,2));glass.setIndex(indices);glass.computeVertexNormals();mesh(glass,'glass','连续曲面风挡与座舱玻璃');
  for(const sign of [-1,1]){
    const edge=[];for(let i=0;i<=40;i++){const v=beltCurve.getPoint(i/40);edge.push([v.x,sign*v.y,v.z])}tube(edge,.008,'paint','座舱下缘');
    const pillar=[];for(let i=0;i<=16;i++){const t=.94-.39*i/16,u=sign*(1-.33*i/16),roof=roofCurve.getPoint(t),belt=beltCurve.getPoint(t);pillar.push([roof.x,u*roof.y,belt.z+(roof.z-belt.z)*Math.pow(Math.cos(u*Math.PI/2),.45)+.003])}tube(pillar,.009,'paint','A柱与上框');
    const rail=[];for(let i=0;i<=20;i++){const t=.55-.5*i/20,u=sign*(i<15?.67:.67+.33*(i-15)/5),roof=roofCurve.getPoint(t),belt=beltCurve.getPoint(t);rail.push([roof.x,u*roof.y,belt.z+(roof.z-belt.z)*Math.pow(Math.cos(u*Math.PI/2),.45)+.003])}tube(rail,.009,'paint','弧形座舱侧框');
    const divider=[];for(let i=0;i<=12;i++){const t=.29,u=sign*(1-.34*i/12),roof=roofCurve.getPoint(t),belt=beltCurve.getPoint(t);divider.push([roof.x,u*roof.y,belt.z+(roof.z-belt.z)*Math.pow(Math.cos(u*Math.PI/2),.45)+.002])}tube(divider,.003,'cavity','侧窗分隔线');
  }

  // Actual flying-car frames show closed discs, unlike the showroom car's open-spoke wheels.
  const tyreWidth=W*.128;
  for(const x of axles)for(const sign of [-1,1]){
    const y=sign*(W/2-tyreWidth/2-.012);cylinder(R,tyreWidth,[x,y,wheelZ],'y','rubber','原型轮胎');
    const outer=sign*(W/2-.005),rimR=R*.84;
    cylinder(rimR,.007,[x,outer-sign*.001,wheelZ],'y','wheel','封闭圆盘轮罩');
    torus(R*.94,.006,[x,outer-sign*.009,wheelZ],'y','rubber','胎肩圆边');
    torus(rimR,.006,[x,outer-sign*.001,wheelZ],'y','metal','轮罩外环');
    torus(rimR*.88,.003,[x,outer+sign*.001,wheelZ],'y','carbon','轮罩内环');
    cylinder(R*.095,.001,[x,outer+sign*.0035,wheelZ],'y','cavity','黑色轮心');
    for(let i=0;i<12;i++){const a=i*Math.PI/6;block([R*.07,.001,R*.018],[x+rimR*.94*Math.cos(a),outer+sign*.003,wheelZ+rimR*.94*Math.sin(a)],'carbon','轮罩外缘短槽').rotation.y=-a;}
    // A few tread grooves remain visible on the tyre shoulder at oblique angles.
    for(let i=0;i<3;i++)torus(R*.99,.0014,[x,y+tyreWidth*(i-1)*.22,wheelZ],'y','carbon','胎面环纹');
  }

  const rearFace=new T.Shape();rearFace.moveTo(-W*.415,-H*.43);rearFace.lineTo(W*.415,-H*.43);rearFace.lineTo(W*.415,H*.14);rearFace.lineTo(-W*.415,H*.14);rearFace.closePath();
  for(const y of [-.31,-.105,.105,.31])for(const [z,r]of[[-.255*H,W*.089+.003],[.035*H,W*.043+.003]]){const hole=new T.Path();hole.absarc(y*W,z,r,0,Math.PI*2,true);rearFace.holes.push(hole)}
  const rearGeo=new T.ExtrudeGeometry(rearFace,{depth:.008,bevelEnabled:false,curveSegments:48});rearGeo.applyMatrix4(new T.Matrix4().set(0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1));mesh(rearGeo,'carbon','八孔尾部设备舱边框',[-.490*L+.006,0,0]);
  block([.014,W*.91,H*.021],p(-.497,0,.177),'red','贯穿尾灯');
  block([.018,W*.17,H*.072],p(-.496,0,.10),'metal','尾部牌照板');
  const fanRadius=W*.089;
  for(const y of [-.31,-.105,.105,.31]){
    const fy=y*W,fc=-.255*H,front=-.490*L;
    cylinder(fanRadius,.16,[front+.07,fy,fc],'x','carbon','大涵道风扇外筒',true);
    cylinder(fanRadius*.91,.006,[front+.145,fy,fc],'x','cavity','涵道内部暗腔');
    torus(fanRadius,.007,[front,fy,fc],'x','metal','涵道口缘');
    torus(fanRadius*.82,.003,[front+.035,fy,fc],'x','carbon','涵道内环');
    const hub=mesh(new T.ConeGeometry(fanRadius*.30,.105,32),'carbon','风扇中心导流体',[front+.055,fy,fc]);hub.rotation.z=Math.PI/2;
    for(const a of [0,2*Math.PI/3,4*Math.PI/3])tube([[front+.07,fy+fanRadius*.25*Math.cos(a),fc+fanRadius*.25*Math.sin(a)],[front+.08,fy+fanRadius*.85*Math.cos(a),fc+fanRadius*.85*Math.sin(a)]],.004,'carbon','风扇内部支撑示意');
    const r=W*.043,zc=.035*H;
    cylinder(r,.11,[front+.045,fy,zc],'x','metal','小型冷气喷口',true);
    cylinder(r*.88,.004,[front+.096,fy,zc],'x','cavity','喷口内壁暗面');
    torus(r,.0035,[front-.005,fy,zc],'x','metal','喷口薄口缘');
  }
  for(const y of [-.415,-.207,0,.207,.415])panel([p(-.495,y,-.45),p(-.495,y,-.10),p(-.40,y,-.19),p(-.36,y,-.46)],'carbon','后扩散器竖向导板');
  panel([p(-.495,-.43,-.45),p(-.495,.43,-.45),p(-.35,.39,-.46),p(-.35,-.39,-.46)],'carbon','扩散器底缘');
  // Wing geometry is visible evidence; deployment angle is illustrative.
  for(const y of [-.27,.27]){
    tube([p(-.39,y,.20),p(-.40,y,.29),p(-.43,y,.34)],.011,'carbon','后翼双支架');
    cylinder(.016,.05,p(-.40,y,.29),'y','metal','后翼铰接轴');
  }
  panel([p(-.475,-.47,.34),p(-.475,.47,.34),p(-.382,.46,.32),p(-.382,-.46,.32)],'carbon','整宽后翼主面');
  panel([p(-.475,-.47,.34),p(-.475,.47,.34),p(-.475,.47,.36),p(-.475,-.47,.36)],'paint','后翼尾缘');

  block([L*.77,W*.72,.026],p(-.015,0,-.395),'carbon','底部设备板');
  const sealL=L*.78,sealW=W*.727,sealH=H*.11;
  for(const y of [-sealW/2,sealW/2])block([sealL,.018,sealH],[0,y,-H/2+sealH/2],'carbon','柔性密封裙边');
  for(const x of [-sealL/2,sealL/2])block([.018,sealW,sealH],[x,0,-H/2+sealH/2],'carbon','前后密封边');
  for(const x of [-.29,.29])for(const y of [-.27,.27]){
    block([.14,.14,.03],p(x,y,-.405),'metal','原型底部喷口安装座');
    cylinder(.045,.065,p(x,y,-.448),'z','carbon','朝下喷口示意',true);
    torus(.045,.004,p(x,y,-.474),'z','metal','底部喷口缘');
  }
  for(const y of [-.22,.22])tube([p(.23,y,-.42),p(0,y,-.42),p(-.31,y,-.42)],.009,'metal','外露管线示意');
  model.userData={type:'source-informed exterior reconstruction',dimensions:[L,W,H],prototypeWheels:'disc-like appearance in moving footage; stationary construction remains unverified',unknowns:'underbody routing, internal fan supports and exact nozzle dimensions are illustrative'};
  return model;
}
