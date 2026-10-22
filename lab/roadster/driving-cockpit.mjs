import {t as tr} from './i18n.mjs';
import {qmat} from './simulation_v2/controller.mjs';
import {tmv} from './simulation_v3/math.mjs';

// Extend six pixels beneath the bezel so the supplied PNG leaves no transparent seams.
export function cockpitScreenMatrix(width,height){
 const [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]=[[94,58],[632,48],[625,371],[53,317]].map(([x,y])=>[x/701,y/429]);
 const dx1=x1-x2,dx2=x3-x2,dx3=x0-x1+x2-x3,dy1=y1-y2,dy2=y3-y2,dy3=y0-y1+y2-y3;
 const denominator=dx1*dy2-dx2*dy1,g=(dx3*dy2-dx2*dy3)/denominator,h=(dx1*dy3-dx3*dy1)/denominator;
 return [x1-x0+g*x1,(y1-y0+g*y1)*height/width,0,g/width,(x3-x0+h*x3)*width/height,y3-y0+h*y3,0,h/height,0,0,1,0,x0*width,y0*height,0,1];
}

export function createCockpitTelemetry(gravity=9.81){
 let previous=null,acceleration=null;
 return f=>{
  const R=qmat(f.quat),dt=previous?f.t-previous.t:0;
  if(!previous||f.resetSerial!==previous.resetSerial||dt<0||dt>.25)acceleration=null;
  else if(dt>1e-6){
   const next=f.vel.map((v,i)=>(v-previous.vel[i])/dt),blend=1-Math.exp(-dt/.12);
   acceleration=acceleration?next.map((v,i)=>acceleration[i]+(v-acceleration[i])*blend):next;
  }
  if(!previous||dt!==0||f.resetSerial!==previous.resetSerial)previous={t:f.t,vel:[...f.vel],resetSerial:f.resetSerial};
  const force=acceleration?tmv(R,acceleration.map((v,i)=>(v+(i===2?gravity:0))/gravity)):null;
  return {speed:Math.round(Math.abs(f.speed??0)),gear:f.gear??'D',roll:Math.atan2(R[2][1],R[2][2])*180/Math.PI,pitch:Math.atan2(R[2][0],Math.hypot(R[0][0],R[1][0]))*180/Math.PI,g:force?Math.hypot(...force):null,longitudinal:force?.[0]??null,lateral:force?.[1]??null};
 };
}

export function createDrivingCockpit(container,gravity){
 const element=document.createElement('div');element.className='driving-cockpit';element.hidden=true;
 const asset=name=>new URL(`./images/cockpit/${name}.png`,import.meta.url).href;
 element.innerHTML=`
  <div class="cockpit-cabin" aria-hidden="true"><img src="${asset('cabin')}" alt="" draggable="false"></div>
  <div class="cockpit-console" role="group" aria-label="${tr('中控屏','Center display')}">
   <div class="cockpit-screen"><div class="cockpit-screen-glow"></div><div class="cockpit-screen-road"></div>
    <div class="cockpit-gears" aria-label="${tr('挡位','Gear')}"><span>P</span><span>R</span><span>N</span><span>D</span></div>
    <span class="cockpit-screen-caption">ROADSTER<span>${tr('驾驶沙盘','DRIVING LAB')}</span></span>
   </div><img class="cockpit-screen-frame" src="${asset('display')}" alt="" draggable="false">
  </div>
  <div class="cockpit-yoke" aria-hidden="true"><div class="cockpit-yoke-crop"><img src="${asset('yoke')}" alt="" draggable="false"></div></div>
  <section class="cockpit-hud" aria-label="${tr('驾驶抬头显示','Driving head-up display')}">
   <div class="cockpit-attitude" role="img" aria-label="${tr('车身姿态','Vehicle attitude')}">
    <svg viewBox="0 0 100 100" aria-hidden="true"><defs><clipPath id="cockpit-attitude-clip"><circle cx="50" cy="50" r="37"/></clipPath></defs>
     <g clip-path="url(#cockpit-attitude-clip)"><g class="cockpit-horizon-roll"><g class="cockpit-horizon-pitch">
      <path class="cockpit-sky" d="M-300-300H400V50H-300Z"/><path class="cockpit-ground" d="M-300 50H400V400H-300Z"/>
      <path d="M-100 50H200M34 34H66M42 18H58M34 66H66M42 82H58"/>
     </g></g></g><circle cx="50" cy="50" r="38" class="cockpit-dial"/>
     <path d="M50 6V12M28 12L31 18M72 12L69 18M12 28L18 31M88 28L82 31" class="cockpit-ticks"/>
     <path d="M23 50H40L45 55M77 50H60L55 55M50 46V51" class="cockpit-wings"/>
    </svg><small class="cockpit-angles"></small>
   </div>
   <div class="cockpit-speed"><strong>0</strong><span>km/h</span></div>
   <div class="cockpit-g" title="${tr('重力补偿后的车身过载，静止约 1 G，自由落体约 0 G','Body load with gravity compensation: about 1 G at rest, 0 G in free fall')}">
    <div><strong class="cockpit-g-total">—</strong><span> G</span></div>
    <small>${tr('纵向','LONG')} <b class="cockpit-g-long">—</b></small><small>${tr('横向','LAT')} <b class="cockpit-g-lat">—</b></small>
   </div>
  </section>`;
 container.append(element);
 const $=s=>element.querySelector(s),screen=$('.cockpit-screen'),display=$('.cockpit-console'),wheel=$('.cockpit-yoke-crop'),speed=$('.cockpit-speed strong'),total=$('.cockpit-g-total'),long=$('.cockpit-g-long'),lat=$('.cockpit-g-lat'),attitude=$('.cockpit-attitude'),roll=$('.cockpit-horizon-roll'),pitch=$('.cockpit-horizon-pitch'),angles=$('.cockpit-angles'),gears=[...element.querySelectorAll('.cockpit-gears span')],sample=createCockpitTelemetry(gravity);
 let telemetry=null,steeringRotation=0;
 const signed=v=>v===null?'—':`${v<-.05?'−':'+'}${Math.abs(v).toFixed(1)}`;
 function update(f){
  telemetry=f.cockpitTelemetry??sample(f);if(element.hidden)return;
  steeringRotation=-(f.steeringAngle??0)*4;wheel.style.transform=`rotate(${steeringRotation}rad)`;
  speed.textContent=telemetry.speed;total.textContent=telemetry.g===null?'—':telemetry.g.toFixed(1);long.textContent=signed(telemetry.longitudinal);lat.textContent=signed(telemetry.lateral);
  roll.setAttribute('transform',`rotate(${-telemetry.roll} 50 50)`);pitch.setAttribute('transform',`translate(0 ${telemetry.pitch*1.6})`);
  const degrees=`${tr('俯仰','PITCH')} ${Math.round(telemetry.pitch)}° · ${tr('横滚','ROLL')} ${Math.round(telemetry.roll)}°`;
  angles.textContent=degrees;attitude.setAttribute('aria-label',degrees);
  for(const gear of gears){const active=gear.textContent===telemetry.gear;gear.classList.toggle('is-active',active);if(active)gear.setAttribute('aria-current','true');else gear.removeAttribute('aria-current');}
 }
 function resize(w,h){
  // shortcut: stretch the supplied center-view cabin around the left seat; replace with a driver-eye render if supplied.
  const cabinWidth=Math.max(w,h*1.25);
  element.style.setProperty('--cabin-width',`${cabinWidth}px`);
  element.style.setProperty('--yoke-width',`${Math.min(cabinWidth*.49,w*.88,h*.94)}px`);
  element.classList.toggle('is-portrait',w/h<1);
  const {width,height}=display.getBoundingClientRect();
  if(width&&height)screen.style.transform=`matrix3d(${cockpitScreenMatrix(width,height).join(',')})`;
 }
 return {update,resize,setVisible(value){element.hidden=!value;},dispose(){element.remove();},stats(){return {visible:!element.hidden,mode:'image-mask',steeringRotation,telemetry,assets:[...element.querySelectorAll('img')].map(img=>({loaded:img.complete&&img.naturalWidth>0,src:img.src}))};}};
}
