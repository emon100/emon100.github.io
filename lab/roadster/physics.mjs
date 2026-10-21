import {t as tr,getLocale} from './i18n.mjs';
import * as k from './kit/iso-kit.mjs';

export const G = 9.81;
const PI = Math.PI;
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
function valid(values, positive = [], nonnegative = []) {
  for (const [name, value] of Object.entries(values)) {
    if (typeof value !== 'number' || !Number.isFinite(value)) throw new TypeError(`${name}${tr(" 必须是有限数值"," must be a finite number")}`);
  }
  for (const name of positive) if (values[name] <= 0) throw new RangeError(`${name}${tr(" 必须大于零"," must be greater than zero")}`);
  for (const name of nonnegative) if (values[name] < 0) throw new RangeError(`${name}${tr(" 不能小于零"," must not be negative")}`);
}
export function suctionModel({ m = 2000, A = 4, mu = 1, theta = 135, deltaP = 8 } = {}) {
  valid({ m, A, mu, theta, deltaP }, ['m', 'A', 'mu'], ['deltaP']);
  if (theta < 0 || theta > 180) throw new RangeError(tr("theta 必须介于 0 与 180 度","theta must be between 0 and 180 degrees"));
  const angle = theta * PI / 180, D = deltaP * 1000 * A, N = D + m * G * Math.cos(angle);
  const along = m * G * Math.abs(Math.sin(angle));
  const requiredD = m * G * Math.max(0, Math.abs(Math.sin(angle)) / mu - Math.cos(angle));
  const frictionMargin = Math.max(0, N) * mu - along;
  return { m, A, mu, theta, deltaP, D, N, along, frictionMargin, requiredD,
    requiredPressure: requiredD / A / 1000, contact: N >= -1e-8,
    held: N >= -1e-8 && frictionMargin >= -1e-8 };
}
export function thrustModel({ m = 2000, Tz = 29.43, Isp = 60, seconds = 2 } = {}) {
  valid({ m, Tz, Isp, seconds }, ['m', 'Isp'], ['Tz', 'seconds']);
  const thrust = Tz * 1000, flow = thrust / (Isp * G);
  return { m, Tz, Isp, seconds, thrust, acceleration: thrust / m - G, flow, gas: flow * seconds, ratio: thrust / (m * G) };
}
export function rollModel({ I = 906.7, seconds = 1.5, angle = PI, progress = .5 } = {}) {
  valid({ I, seconds, angle, progress }, ['I', 'seconds', 'angle']);
  if (progress < 0 || progress > 1) throw new RangeError(tr("progress 必须介于 0 与 1","progress must be between 0 and 1"));
  const t = progress * seconds, half = seconds / 2, alpha = 4 * angle / seconds ** 2;
  const rotation = t <= half ? .5 * alpha * t ** 2 : angle - .5 * alpha * (seconds - t) ** 2;
  const omega = alpha * Math.min(t, seconds - t);
  const torque = progress === 1 ? 0 : I * alpha * (progress < .5 ? 1 : -1);
  return { I, seconds, angle, progress, t, alpha, rotation, omega, torque, peakTorque: I * alpha };
}
export function aeroModel({ velocity = 35, rho = 1.225, S = 2, CL = .6, fan = 5 } = {}) {
  valid({ velocity, rho, S, CL, fan }, ['rho', 'S'], ['velocity', 'CL', 'fan']);
  const passive = .5 * rho * velocity ** 2 * S * CL;
  return { velocity, rho, S, CL, fan, passive, active: fan * 1000, total: passive + fan * 1000 };
}

const models = {
  suction: { title: tr("旋转吸附试验台","Rotating suction test bench"), solve: suctionModel, defaults: { m: 2000, A: 4, mu: 1, theta: 135, deltaP: 8 },
    fields: [['m',tr("车重","Vehicle mass"),1000,3000,100,'kg'],['A',tr("有效吸附面积","Effective suction area"),2,6,.1,'m²'],['mu',tr("静摩擦系数","Static friction coefficient"),.3,1.5,.05,''],['theta',tr("坡面角度","Ramp angle"),0,180,1,'°'],['deltaP',tr("车底压差","Underbody pressure difference"),0,15,.1,'kPa']],
    note: tr("0° 为普通地面，90° 为竖墙，180° 为倒挂。压差表针与坡面角度均跟随当前参数。","0° is level ground, 90° a vertical wall and 180° inverted. The pressure needle and ramp angle follow the current parameters."),
    boundary: tr("教学假设，非 Tesla 实测。只算静止的局部平面；不含曲率、漏气、轮胎载荷敏感性和安全余量。临界接触不代表仍有足够抓地力。","Teaching assumptions, not Tesla measurements. Static local plane only; excludes curvature, leakage, tire load sensitivity and safety margins. Marginal contact does not guarantee grip.") },
  thrust: { title: tr("冷气推力测量台","Cold-gas thrust test bench"), solve: thrustModel, defaults: { m: 2000, Tz: 29.43, Isp: 60, seconds: 2 },
    fields: [['m',tr("车重","Vehicle mass"),1000,3000,100,'kg'],['Tz',tr("向上推力","Upward thrust"),0,60,.01,'kN'],['Isp',tr("假设有效比冲","Assumed effective specific impulse"),40,80,1,'s'],['seconds',tr("喷气持续时间","Jet duration"),0,4,.1,'s']],
    note: tr("表针读向上推力；试验台固定住喷嘴。净加速度是离地后的理想值，不是台架的实际运动。","The needle shows upward thrust; the nozzle is fixed to the bench. Net acceleration is an ideal airborne value, not actual bench motion."),
    boundary: tr("教学假设，非 Tesla 实测。推力恒定、忽略空气阻力、质量近似不变。耗气占比较大时，需更新质量与气罐热力过程。60 s 比冲不是 Tesla 的已知规格。","Teaching assumptions, not Tesla measurements. Constant thrust, no drag and nearly constant mass. Significant gas use requires updated mass and tank thermodynamics. 60 s specific impulse is not a known Tesla specification.") },
  roll: { title: tr("翻转惯量试验台","Flip inertia test bench"), solve: rollModel, defaults: { I: 906.7, seconds: 1.5, angle: PI, progress: .5 },
    fields: [['I',tr("转动惯量","Moment of inertia"),300,1600,.1,'kg·m²'],['seconds',tr("完成半圈的时间","Half-turn duration"),.8,3,.1,'s'],['progress',tr("过程进度","Progress"),0,1,.01,'']],
    note: tr("拖动进度：前半程加速，后半程施加反向力矩。试件从倒置转到水平，角速度最终回到零。","Drag the progress slider: accelerate in the first half and apply opposite torque in the second. The specimen rotates from inverted to level and ends with zero angular velocity."),
    boundary: tr("教学假设，非 Tesla 实测。固定单轴、恒定惯量、理想瞬时换向。台架表示力矩的作用，不表示真实喷口、传感器或阀门时序。","Teaching assumptions, not Tesla measurements. Fixed axis, constant inertia and instantaneous reversal. This bench illustrates torque, not actual nozzle, sensor or valve timing.") },
  aero: { title: tr("风机与翼片测力台","Fan and wing force test bench"), solve: aeroModel, defaults: { velocity: 35, rho: 1.225, S: 2, CL: .6, fan: 5 },
    fields: [['velocity',tr("相对空气速度","Relative airspeed"),0,70,1,'m/s']],
    note: tr("左表的主动风机贡献固定在 5 kN；右表的翼片下压力随速度平方增加。两者是教学中的独立贡献。","The left gauge fixes the fan contribution at 5 kN; the right shows wing downforce rising with speed squared. These are separate teaching assumptions."),
    boundary: tr("机制重建，不拟合 Tesla 概念图。固定 ρ = 1.225 kg/m³、S = 2 m²、|CL| = 0.6、风机 5 kN，全部是假设；实际存在模式切换、载荷上限及相互影响。","Mechanism reconstruction, not a fit to Tesla concept art. Fixed ρ = 1.225 kg/m³, S = 2 m², |CL| = 0.6 and fan force 5 kN are all assumptions. Real systems have mode changes, load limits and interactions.") },
};

// Figure parts: foot pads, feet, bedplate, rail clamps, rails, stops, screws,
// ruler, instrument cases, bezel strips, pointers, pivot pedestals and test modules.
const palettes={
  dark:{lo:['#20282d','#273238','#313d44','#3c4951'],mid:['#2f3942','#414d57','#56636e','#697781'],hi:['#56636c','#78858d','#9ca7ad','#c0c7cb'],lit:['#fff','#fff','#fff','#fff']},
  light:{lo:['#a3adae','#bbc5c5','#d0d7d7','#e3e8e7'],mid:['#6e7d85','#899aa2','#aab9c0','#c5d0d5'],hi:['#a2adb4','#bdc6cc','#dae0e3','#f4f6f5'],lit:['#18252c','#18252c','#18252c','#18252c']}
};
const colorVars=palette=>Object.entries(palette).flatMap(([tone,colors])=>colors.map((color,i)=>`--bench-${tone}-${i}:${color}`)).join(';');
const figureColors=`.physics-figure{${colorVars(palettes.dark)}}.iso[data-theme="light"] .physics-figure{${colorVars(palettes.light)}}`;

function boxFaces(x,y,z,w,d,h) {
  const v=[[x,y,z],[x+w,y,z],[x+w,y+d,z],[x,y+d,z],[x,y,z+h],[x+w,y,z+h],[x+w,y+d,z+h],[x,y+d,z+h]];
  return [[0,3,2,1],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[4,5,6,7]].map(f=>f.map(i=>v[i]));
}
function turn(v, pose) {
  if (!pose) return v;
  const [x,y,z]=v,c=Math.cos(pose.angle||0),s=Math.sin(pose.angle||0);
  return [pose.x+x*c-z*s,pose.y+y,pose.z+x*s+z*c];
}
function normal(face) {
  const u=face[1].map((v,i)=>v-face[0][i]),v=face[2].map((n,i)=>n-face[0][i]);
  return [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
}
function mean(face) { return face[0].map((_,i)=>face.reduce((s,p)=>s+p[i],0)/face.length); }
function makeBench(model, value) {
  const parts=[];let layer=0;
  const add=(x,y,z,w,d,h,tone='mid',pose=null)=>parts.push({ faces:boxFaces(x,y,z,w,d,h).map(f=>f.map(v=>turn(v,pose))),tone,layer });
  const screw=(x,y,z,pose=null)=>add(x-1.2,y-1.2,z,2.4,2.4,.7,'hi',pose);
  for(const x of [-129,118])for(const y of [-56,45]){add(x-3,y-3,-18,17,17,2,'lo');add(x,y,-16,11,11,6,'mid');}
  add(-145,-72,-10,290,144,10,'lo');
  add(-139,-66,0,278,132,1,'lo');layer=1;
  for(const x of [-133,130])for(const y of [-60,57])screw(x,y,1);
  for(const y of [-55,46]){
    add(-120,y,1,240,5,3);
    for(const x of [-121,116]){add(x,y-3,1,6,11,6);screw(x+3,y+2.5,7);}
  }
  add(-125,57,1,250,6,.7,'mid');
  for(let n=0;n<=40;n++)add(-121+n*6,57,1.7,.55,n%5===0?4:2,.2,n%5===0?'hi':'lo');
  add(-132,-65,1,45,11,.8,'mid');
  for(const x of [-128,-91])screw(x,-60,1.8);
  function gauge(x,y,share) {
    const previousLayer=layer;layer=2;
    add(x-23,y-19,1,46,38,7,'mid');layer=3;add(x-20,y-16,8,40,32,.6,'lo');layer=4;
    for(const [dx,dy,w,d] of [[-22,-18,44,2],[-22,16,44,2],[-22,-16,2,32],[20,-16,2,32]])add(x+dx,y+dy,8,w,d,1,'hi');
    layer=5;for(let n=0;n<13;n++){
      const a=(-150+n*25)*PI/180,r=13;
      add(x+Math.sin(a)*r-.4,y-Math.cos(a)*r-.4,8.7,.8,.8,.3,n%3===0?'hi':'mid');
    }
    const a=(-150+300*clamp(share,0,1))*PI/180,tip=[x+Math.sin(a)*12,y-Math.cos(a)*12,9.1];
    parts.push({faces:[[[x-.6*Math.cos(a),y-.6*Math.sin(a),9.1],[x+.6*Math.cos(a),y+.6*Math.sin(a),9.1],tip]],tone:'lit',twoSided:true,layer:6});
    layer=7;add(x-1,y-1,9.1,2,2,.6,'hi');
    for(const dx of [-18,18])for(const dy of [-14,14])screw(x+dx,y+dy,8.7);layer=previousLayer;
  }
  function pivots() {
    for(const y of [-44,36]){
      add(7,y-4,1,36,16,4);for(const x of [12,37])screw(x,y+3,5);
      add(17,y,5,16,8,70);add(13,y-1,68,24,10,13,'hi');
      add(21,y<0?y+8:y-8,70,8,8,7,'mid');
      add(21,y<0?y-2:y+8,71,8,2,5,'mid');
    }
  }
  if(model==='suction'){
    gauge(-91,21,value.deltaP/15);pivots();
    const pose={x:25,y:0,z:74,angle:-value.theta*PI/180};
    add(-56,-32,-4,112,64,4,'mid',pose);
    for(const y of [-32,29])add(-56,y,0,112,3,2,'hi',pose);
    for(const x of [-50,50])for(const y of [-26,26])screw(x,y,0,pose);
    add(-25,-20,0,45,40,2,'lo',pose);add(-24,-19,2,43,38,12,'hi',pose);
    add(-25,-20,14,45,40,1,'hi',pose);
    add(-18,-14,15,30,28,12,'mid',pose);add(-17,-13,27,28,26,1,'hi',pose);
    for(const x of [-21,15])for(const y of [-16,16])screw(x,y,15,pose);
    add(19,-6,3,9,12,6,'mid',pose);add(28,-13,0,22,26,16,'mid',pose);
    add(27,-14,16,24,28,1,'hi',pose);
    for(let n=0;n<5;n++)add(30+n*4,-10,17,1.4,20,.4,'lo',pose);
  }else if(model==='roll'){
    gauge(-91,21,value.rotation/PI);pivots();
    const pose={x:25,y:0,z:74,angle:value.rotation-PI};
    add(-22,-32,-5,44,64,10,'hi',pose);add(-19,-31,5,38,62,1,'mid',pose);
    add(-14,-23,6,28,46,12,'hi',pose);add(-11,-20,18,22,40,1,'hi',pose);
    for(const x of [-20,17])for(const y of [-28,25])screw(x,y,6,pose);
    for(const x of [-35,22])for(const y of [-21,13]){
      add(x,y,-3,13,8,6,'mid',pose);add(x+2,y+1,-4,9,6,1,'lo',pose);
    }
    add(-3,-36,-3,6,4,6,'mid',pose);add(-3,32,-3,6,4,6,'mid',pose);
  }else if(model==='thrust'){
    gauge(-91,21,value.Tz/60);
    add(-109,-44,1,36,37,54,'mid');add(-110,-45,55,38,39,2,'hi');
    for(let n=0;n<5;n++)add(-105+n*6,-39,57,2,24,.4,'lo');
    for(const y of [-32,26]){
      add(-20,y-4,1,90,14,4,'mid');for(const x of [-13,60])screw(x,y+3,5);
      add(7,y,5,6,6,84,'hi');add(43,y,5,6,6,84,'hi');
      add(3,y-2,89,50,10,5,'mid');
      for(const x of [5,45])screw(x,y+2,94);
      add(4,y-1,51,12,8,12,'mid');add(40,y-1,51,12,8,12,'mid');
    }
    add(-13,-27,56,78,54,8,'hi');add(-8,-22,64,68,44,1,'hi');
    add(15,-12,36,22,24,20,'mid');add(19,-9,30,14,18,6,'hi');
    add(16,-12,25,20,24,5,'mid');add(14,-14,23,24,28,2,'hi');
    add(18,-10,22.8,16,20,.2,'lo');
    for(const x of [-6,56])for(const y of [-20,20])screw(x,y,65);
    add(11,-17,1,30,34,3,'lo');add(15,-13,4,22,26,2,'mid');
  }else{
    gauge(-90,27,value.active/6000);gauge(87,27,value.passive/4000);
    add(-108,-43,1,54,43,5,'mid');add(-100,-35,6,38,27,12,'hi');
    add(-102,-37,18,42,31,1,'hi');
    for(let n=0;n<6;n++)add(-98+n*6,-32,19,2,21,.5,'lo');
    for(const x of [-104,-58])for(const y of [-39,-4])screw(x,y,6);
    for(const x of [25,83]){add(x,-25,1,15,20,4);add(x+5,-20,5,5,10,32,'hi');}
    add(16,-35,37,96,40,3,'hi');add(16,-35,40,96,4,3,'mid');
    add(18,-31,40,92,32,1,'hi');
    for(const x of [21,106])for(const y of [-28,-4])screw(x,y,41);
    add(-31,-35,1,23,36,37,'mid');add(-32,-36,38,25,38,2,'hi');
    for(let n=0;n<5;n++)add(-30,-31+n*6,40,21,1.5,.3,'lo');
  }
  return parts;
}
const fitPoints=Object.entries(models).flatMap(([model,spec])=>{
  const poses=model==='suction'?[0,45,90,135,180].map(theta=>({theta})):model==='roll'?[0,.25,.5,.75,1].map(progress=>({progress})):[{}];
  return poses.flatMap(values=>makeBench(model,spec.solve(values)).flatMap(part=>part.faces.flat()));
});
const P=k.fitProjection(fitPoints,640,360,{pad:20,azimuth:30,elevation:22});
const camera=k.cameraOf(P),viewer=[camera.cosA*camera.cosE,camera.sinA*camera.cosE,camera.sinE];
export function renderExperiment(model, values) {
  const spec=models[model];if(!spec)throw new RangeError(tr("未知物理模型","Unknown physics model"));
  const value=spec.solve(values),parts=makeBench(model,value),faces=[];
  for(const part of parts)for(const face of part.faces){
    const n=normal(face),len=Math.hypot(...n);if(!len)continue;
    const facing=n.reduce((s,v,i)=>s+v*viewer[i],0);if(facing<=0&&!part.twoSided)continue;
    const unit=n.map(v=>v/len),shade=clamp(Math.round((unit[2]*.7-unit[0]*.3-unit[1]*.4+1)*1.5),0,3);
    faces.push({path:k.pathOf(face.map(v=>k.iso(v,P)),true),depth:k.depthOf(mean(face),P),tone:part.tone,layer:part.layer,shade,top:unit[2]>.92});
  }
  faces.sort((a,b)=>a.layer-b.layer||a.depth-b.depth);
  const art=faces.map(f=>`<path d="${f.path}" fill="var(--bench-${f.tone}-${f.top?3:f.shade})" stroke="var(--anatomy-${f.tone})" stroke-width=".5" stroke-linejoin="round"/>`).join('');
  return `<svg class="physics-figure" viewBox="0 0 640 360" role="img" aria-label="${spec.title}${tr("；结构与尺寸为教学示意\" data-parts=\"","; structure and dimensions are teaching illustrations\" data-parts=\"")}${parts.length}"><style>${figureColors}</style><path d="${k.haloOf({x:-145,y:-72,w:290,d:144,r:6},-18,4,P)}" class="iso-halo"/>${art}</svg>`;
}

const number=(v,d=2)=>v.toLocaleString(getLocale(),{maximumFractionDigits:d,minimumFractionDigits:d});
let mountCount=0;
export function mountPhysics(stage, controls, readout, equation, boundary, initial='suction', visual=null) {
  if(![stage,controls,readout,equation,boundary].every(el=>el&&typeof el.replaceChildren==='function'))throw new TypeError(tr("需要五个有效 DOM 节点","Five valid DOM nodes are required"));
  const id=++mountCount,states=Object.fromEntries(Object.entries(models).map(([name,spec])=>[name,{...spec.defaults}]));
  let selected,dead=false;
  readout.setAttribute('aria-live','polite');
  function draw(){
    const spec=models[selected],state=states[selected],v=spec.solve(state);
    if(visual)visual.setState(selected,state);else stage.innerHTML=renderExperiment(selected,state);
    if(selected==='suction'){
      const status=!v.contact?tr("会脱离","Contact lost"):!v.held?tr("会沿坡滑落","Slides down the ramp"):Math.abs(v.N)<1e-7||Math.abs(v.frictionMargin)<1e-7?tr("临界平衡","Marginal equilibrium"):tr("可保持静止","Can remain stationary");
      readout.textContent=`${status} · ${number(v.requiredPressure,3)}${tr(" kPa 起"," kPa minimum")}`;
      equation.textContent=`D = ΔpA = ${number(v.D/1000)} kN；N = D + mg cosθ = ${number(v.N/1000)}${tr(" kN；沿面摩擦余量 "," kN；friction margin along the surface ")}${number(v.frictionMargin/1000)} kN`;
    }else if(selected==='thrust'){
      readout.textContent=`${tr("净加速度 ","Net acceleration ")}${number(v.acceleration)} m/s²`;
      equation.textContent=`${tr("az = Tz/m − g；推重比 ","az = Tz/m − g; thrust-to-weight ratio ")}${number(v.ratio)}；ṁ = T/(Isp·g) = ${number(v.flow)} kg/s；${number(v.seconds,1)}${tr(" s 耗气 "," s gas use ")}${number(v.gas)} kg`;
    }else if(selected==='roll'){
      readout.textContent=`${tr("转过 ","Rotated ")}${number(v.rotation*180/PI,1)}° · ${v.progress===1?tr("已停止","Stopped"):v.progress<.5?tr("加速","Accelerating"):tr("制动","Braking")}`;
      equation.textContent=`|τ|max = I·4π/t² = ${number(v.peakTorque/1000)}${tr(" kN·m；当前 τ = "," kN·m；current τ = ")}${number(v.torque/1000)} kN·m；ω = ${number(v.omega)} rad/s`;
    }else{
      readout.textContent=`${tr("总下压力 ","Total downforce ")}${number(v.total/1000)} kN`;
      equation.textContent=`${tr("风机 ","Fan ")}${number(v.active/1000)}${tr(" kN ＋ 翼片 ½ρv²S|CL| = "," kN ＋ wing ½ρv²S|CL| = ")}${number(v.passive/1000)} kN；${number(v.velocity*3.6,0)} km/h`;
    }
    boundary.textContent=spec.boundary;
    for(const [key,, , , ,unit] of spec.fields){const output=controls.querySelector(`[data-value="${key}"]`);if(output)output.textContent=key==='progress'?`${Math.round(state[key]*100)}%`:`${number(state[key],key==='m'?0:key==='theta'?0:key==='I'?1:2)} ${unit}`;}
  }
  function onInput(event){
    if(dead||event.target.tagName!=='INPUT')return;
    const key=event.target.dataset.key;if(!models[selected].fields.some(f=>f[0]===key))return;
    const value=event.target.valueAsNumber;if(!Number.isFinite(value))return;
    states[selected][key]=value;draw();
  }
  controls.addEventListener('input',onInput);
  function select(model){
    if(dead)return;if(!models[model])throw new RangeError(tr("未知物理模型","Unknown physics model"));selected=model;
    const spec=models[model];
    controls.innerHTML=`<h3>${spec.title}</h3><p>${spec.note}</p>`+spec.fields.map(([key,label,min,max,step,unit])=>`<label for="physics-${id}-${key}">${label}<output data-value="${key}" for="physics-${id}-${key}"></output></label><input id="physics-${id}-${key}" data-key="${key}" type="range" min="${min}" max="${max}" step="${step}" value="${states[model][key]}" aria-label="${label}${unit?`${tr("，单位 ",", unit ")}${unit}`:''}">`).join('');
    draw();
  }
  select(initial);
  return {select,getState:()=>({model:selected,value:{...states[selected]}}),destroy(){dead=true;visual?.destroy();controls.removeEventListener('input',onInput);for(const el of [stage,controls,readout,equation,boundary])el.replaceChildren();}};
}
