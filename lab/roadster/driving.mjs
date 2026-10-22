import {t as tr} from './i18n.mjs';
import {createLoadingProgress} from './loading.mjs';
import {assetFraction} from './asset-progress.mjs';
import {createParkedVehicles} from './parked-vehicles.mjs?v=20261022-render-refill';
import * as T from './vendor/three.module.min.js';
import {bindDrivingControls} from './driving-input.mjs?v=20261021-gas-proof';
import {createDrivingView} from './driving-camera.mjs?v=20261021-gas-proof';
import {createCockpitTelemetry} from './driving-cockpit.mjs?v=20261023-screen-fill';
import {createDrivingAudio} from './driving-audio.mjs?v=20261021-gas-proof';
import {createWorld} from './lab-world.mjs?v=20261023-screen-fill';
import {rigDrivingWheels} from './driving-model.mjs?v=20261021-gas-proof';
import {loadMeshyModelFactory} from './meshy-model.mjs';
import {drivingNozzles} from './driving-thrusters.mjs?v=20261022-rocket10';
import {MAP,RAMPS,rampPose,rampMeshData} from './driving-core.mjs?v=20261022-dynamics';
const $=s=>document.querySelector(s),drivingView=createDrivingView(),audio=createDrivingAudio();let driveInput={},worker,world,frame,dirty=false,physicsError=false,view='chase',settings={assist:true,downforce:false,sound:true};
try{const saved=JSON.parse(localStorage.getItem('roadster-driving-settings'));for(const key of ['assist','downforce','sound'])if(typeof saved?.[key]==='boolean')settings[key]=saved[key];localStorage.setItem('roadster-driving-settings',JSON.stringify(settings));}catch{}
audio.setEnabled(settings.sound);
function updateSoundButton(){$('#sound').setAttribute('aria-pressed',String(settings.sound));$('#sound-state').textContent=settings.sound?tr('开','On'):tr('关','Off');$('#sound').setAttribute('aria-label',settings.sound?tr('关闭车辆声音','Mute vehicle sound'):tr('开启车辆声音','Enable vehicle sound'));}
function setSettings(value){settings={...settings,...value};if('sound' in value){audio.setEnabled(value.sound);if(value.sound)audio.unlock();updateSoundButton();}worker?.postMessage({type:'settings',value});try{localStorage.setItem('roadster-driving-settings',JSON.stringify(settings));}catch{}stage.focus();}

function environment(){
 const group=new T.Group();
 const perimeter=[[-MAP.halfX,-MAP.halfY,.03],[MAP.halfX,-MAP.halfY,.03],[MAP.halfX,MAP.halfY,.03],[-MAP.halfX,MAP.halfY,.03]].map(p=>new T.Vector3(...p));group.add(new T.LineLoop(new T.BufferGeometry().setFromPoints(perimeter),new T.LineBasicMaterial({color:0x8c8675})));
 for(const x of [-MAP.halfX,MAP.halfX])for(let y=-MAP.halfY;y<=MAP.halfY;y+=8){const post=new T.Mesh(new T.BoxGeometry(.14,.3,.65),new T.MeshStandardMaterial({color:0xa89570}));post.position.set(x,y,.325);group.add(post);}
 for(const r of RAMPS){
  const {vertices,indices}=rampMeshData(r);
  const geometry=new T.BufferGeometry();geometry.setAttribute('position',new T.Float32BufferAttribute(vertices,3));geometry.setIndex(indices);geometry.computeVertexNormals();const ramp=new T.Mesh(geometry,new T.MeshStandardMaterial({color:0x737b81,metalness:.78,roughness:.34,side:T.DoubleSide,shadowSide:T.BackSide}));ramp.castShadow=true;ramp.receiveShadow=true;group.add(ramp);
  for(const side of [-1,1]){const edge=[];for(let i=0;i<=80;i++){const p=rampPose(r,r.length*i/80),q=new T.Quaternion(p.quat[1],p.quat[2],p.quat[3],p.quat[0]);edge.push(new T.Vector3(0,side*3,.035).applyQuaternion(q).add(new T.Vector3(...p.surface)));}group.add(new T.Line(new T.BufferGeometry().setFromPoints(edge),new T.LineBasicMaterial({color:0xc6b58b})));}
  for(let i=4;i<80;i+=4){const p=rampPose(r,r.length*i/80),q=new T.Quaternion(p.quat[1],p.quat[2],p.quat[3],p.quat[0]),points=[-2.96,2.96].map(y=>new T.Vector3(0,y,.008).applyQuaternion(q).add(new T.Vector3(...p.surface)));group.add(new T.Line(new T.BufferGeometry().setFromPoints(points),new T.LineBasicMaterial({color:0x3b4146,transparent:true,opacity:.5})));}
  const approach=r.approach??14,lane=new T.Mesh(new T.PlaneGeometry(approach,6),new T.MeshStandardMaterial({color:0x303337,roughness:.93}));lane.receiveShadow=true;lane.position.set(r.x-approach/2,r.y,.008);group.add(lane);
  for(let x=r.x-approach+2;x<r.x;x+=4){const stripe=new T.Mesh(new T.PlaneGeometry(1.3,.09),new T.MeshBasicMaterial({color:0xc6b58b}));stripe.position.set(x,r.y,.02);group.add(stripe);}
  const canvas=document.createElement('canvas');canvas.width=256;canvas.height=80;const ctx=canvas.getContext('2d');ctx.fillStyle='#16191ddd';ctx.fillRect(0,0,256,80);ctx.fillStyle='#e8dec7';ctx.font='28px system-ui';ctx.textAlign='center';ctx.fillText(r.id==='clockwise'?tr('顺时针','Clockwise'):r.id==='counterclockwise'?tr('逆时针','Counterclockwise'):r.label,128,51,246);const sign=new T.Sprite(new T.SpriteMaterial({map:new T.CanvasTexture(canvas)}));sign.position.set(r.x+1,r.y+4,1.3);sign.scale.set(3,1,1);group.add(sign);
 }
 return group;
}
function status(){if(physicsError)return tr('物理计算出错，请点击复位重试','Physics error. Select Reset to try again.');if(frame.needsReset)return tr('车辆长时间未回正，请点击复位或按 R','Vehicle overturned. Select Reset or press R.');if(frame.resetReason==='boundary'&&frame.t<1.5)return tr('已到边界 · 已复位','Boundary reached · Reset');if(frame.kick)return `KICKFLIP ${Math.round(frame.kickDegrees)}°`;return '';}
function render(){
 const speed=frame.speed,fraction=frame.gas.fraction,gas=frame.gas;
 $('#mode-label').textContent=tr('喷气辅助','Jet assist');$('.assist-badge').dataset.enabled=String(frame.assist);$('#assist-state').textContent=!frame.assist?tr('已关闭','Off'):frame.recoveryActive?tr('已开启 · 介入','On · Active'):tr('已开启 · 待命','On · Standby');
 document.querySelectorAll('[data-assist]').forEach(b=>b.setAttribute('aria-pressed',String((b.dataset.assist==='true')===frame.assist)));$('#downforce').setAttribute('aria-pressed',String(frame.downforce.enabled));$('#downforce-state').textContent=frame.downforce.enabled?tr('开','On'):tr('关','Off');$('#downforce-output').textContent=frame.airFan?.power>500?tr('尾部排流','Tail exhaust'):frame.downforce.active?(frame.downforce.force/1000).toFixed(1)+' kN':frame.downforce.enabled?tr('待贴地','Awaiting ground contact'):tr('已关闭','Off');$('#drive-speed').textContent=Math.round(speed);$('#drive-gear').textContent=frame.gear;$('#gas-percent').textContent=Math.round(fraction*100)+'%';$('#gas-fill').style.transform=`scaleX(${fraction})`;
 $('#drive-state').textContent=status();$('#drive-state').hidden=!$('#drive-state').textContent;$('#gas-state').textContent=gas.phase==='waiting'?tr('{seconds} s 后补气','Refill in {seconds} s',{seconds:gas.secondsRemaining.toFixed(1)}):gas.phase==='refilling'?tr('补气中 · {seconds} s','Refilling · {seconds} s',{seconds:gas.secondsRemaining.toFixed(1)}):gas.phase==='using'?tr('供气中','Jets firing'):gas.phase==='settling'?tr('喷气停止','Jets stopped'):gas.phase==='coasting'?tr('喷气关闭','Jets off'):tr('气量充足','Gas ready');$('#hud-gas').dataset.phase=gas.phase;
 $('#kick').disabled=!frame.canKick;$('#kick-label').textContent=!frame.assist?tr('需开启喷气辅助','Enable jet assist'):frame.gas.phase==='settling'?tr('准备中','Preparing'):frame.kick?tr('翻转中','Flipping'):gas.phase==='waiting'||gas.phase==='refilling'?tr('补气中','Refilling'):'KICKFLIP';
}
function clearInput(){inputControls.clear();}
function queueKick(){if(frame?.canKick)worker.postMessage({type:'kick'});$('#drive-world').focus();}
function tick(now){if(frame&&!document.hidden){if(dirty){render();dirty=false;}const display=drivingView.sample(now);if(display)world.setFrame(display);audio.update(frame,driveInput,view);}requestAnimationFrame(tick);}
function setView(value){view=value;world.setCameraMode(view);document.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.view===view)));$('#drive-world').focus();}
const viewport=$('#drive-viewport'),inputControls=bindDrivingControls(viewport,value=>{driveInput=value;worker?.postMessage({type:'input',value});});let savedOverflow='',fullPending=false;
function isFullscreen(){return document.fullscreenElement===viewport||viewport.classList.contains('is-fullscreen');}
function fullscreenState(){const active=isFullscreen();$('#fullscreen').setAttribute('aria-pressed',String(active));$('#fullscreen').setAttribute('aria-label',active?tr('退出全屏','Exit fullscreen'):tr('进入全屏','Enter fullscreen'));$('#full-label').textContent=active?tr('退出','Exit'):tr('全屏','Fullscreen');if(active)$('#drive-world').focus();}
async function enterFullscreen(){if(isFullscreen()||fullPending)return;fullPending=true;try{if(!viewport.requestFullscreen)throw Error('Fullscreen unavailable');await viewport.requestFullscreen();}catch{savedOverflow=document.body.style.overflow;document.body.style.overflow='hidden';viewport.classList.add('is-fullscreen');}finally{fullPending=false;fullscreenState();}}
async function exitFullscreen(){if(document.fullscreenElement===viewport)await document.exitFullscreen();if(viewport.classList.contains('is-fullscreen')){viewport.classList.remove('is-fullscreen');document.body.style.overflow=savedOverflow;}fullscreenState();}
const toggleFullscreen=()=>isFullscreen()?exitFullscreen():enterFullscreen();
$('#fullscreen').onclick=toggleFullscreen;document.addEventListener('fullscreenchange',fullscreenState);
let tap;const stage=$('#drive-world');stage.addEventListener('pointerdown',e=>{if(e.target.closest('button'))return;if(tap){tap.moved=true;return;}tap={id:e.pointerId,x:e.clientX,y:e.clientY,moved:false};});stage.addEventListener('pointermove',e=>{if(tap&&Math.hypot(e.clientX-tap.x,e.clientY-tap.y)>6)tap.moved=true;});stage.addEventListener('pointerup',e=>{if(tap?.id!==e.pointerId)return;const open=!tap.moved;tap=null;if(open&&!isFullscreen())enterFullscreen();});for(const type of ['pointercancel','lostpointercapture'])stage.addEventListener(type,()=>tap=null);for(const type of ['contextmenu','selectstart'])viewport.addEventListener(type,()=>tap=null);
const headerObserver=new ResizeObserver(()=>document.documentElement.style.setProperty('--game-header',document.querySelector('.site-header').offsetHeight+'px'));headerObserver.observe(document.querySelector('.site-header'));
const loading=createLoadingProgress($('#drive-load'));let modelProgress=0,configProgress=0;
const load=async(path,type)=>{const response=await fetch(path);if(!response.ok)throw Error(`HTTP ${response.status}: ${path}`);return response[type]();};
try{
 const [cfg,manifest,xml,factory]=await Promise.all([Promise.all([load('simulation_v4/config.json','json'),load('simulation_v4/manifest.json','json'),load('simulation_v4/world.xml','text')]).then(values=>{configProgress=.05;loading.update(configProgress+modelProgress,tr('下载车辆模型…','Downloading vehicle model…'));return values;}),loadMeshyModelFactory(progress=>{modelProgress=.5*assetFraction(progress);loading.update(configProgress+modelProgress,progress.phase==='prepare'?tr('准备车辆材质…','Preparing vehicle materials…'):tr('下载车辆模型…','Downloading vehicle model…'),progress);})]).then(([config,factory])=>[...config,factory]);
 loading.update(.55,tr('准备场景…','Preparing scene…'));
 const cockpitTelemetry=createCockpitTelemetry(cfg.physics.gravity);
 world=createWorld(stage,cfg,()=>rigDrivingWheels(factory()),{driving:true,nozzles:drivingNozzles(cfg)});world.addObject(environment());world.addObject(createParkedVehicles());worker=new Worker(new URL('./driving-worker.mjs?v=20261022-dynamics',import.meta.url),{type:'module'});
 await new Promise((resolve,reject)=>{worker.onerror=e=>reject(Error(e.message));worker.onmessage=({data:m})=>{if(m.type==='loading'){loading.update(.55+.4*assetFraction(m.progress),m.progress.phase==='download'?tr('下载物理引擎…','Downloading physics engine…'):tr('初始化物理引擎…','Initializing physics engine…'),m.progress);return;}if(m.type==='error'){physicsError=true;dirty=true;clearInput();$('#drive-state').hidden=false;$('#drive-state').textContent=tr('物理计算出错，请点击复位重试','Physics error. Select Reset to try again.');if(!frame)reject(Error(m.message));return;}if(!physicsError||m.frame.resetSerial!==frame?.resetSerial)physicsError=false;frame={...m.frame,cockpitTelemetry:cockpitTelemetry(m.frame)};drivingView.push(frame,performance.now());dirty=true;if(m.type==='ready')resolve();};worker.postMessage({type:'init',config:cfg,params:manifest.final,xml,settings});});
 document.querySelectorAll('[data-assist]').forEach(b=>b.onclick=()=>setSettings({assist:b.dataset.assist==='true'}));$('#downforce').onclick=()=>setSettings({downforce:!frame.downforce.enabled});
 $('#sound').onclick=()=>setSettings({sound:!settings.sound});updateSoundButton();viewport.addEventListener('pointerdown',()=>audio.unlock());document.addEventListener('keydown',()=>audio.unlock());$('#kick').onclick=queueKick;$('#recenter').onclick=()=>worker.postMessage({type:'recenter'});document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>setView(b.dataset.view));
 document.addEventListener('keydown',e=>{if(e.metaKey||e.ctrlKey||e.altKey)return;if(e.code==='Escape'){clearInput();if(isFullscreen())exitFullscreen();return;}if(e.target.closest('input,select,textarea,[contenteditable=true]'))return;if(['KeyW','KeyA','KeyS','KeyD','ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Space'].includes(e.code)){e.preventDefault();inputControls.pressKey(e.code);}if(e.code==='KeyK'&&!e.repeat)queueKick();if(e.code==='KeyR'&&!e.repeat)worker.postMessage({type:'recenter'});if(e.code==='KeyF'&&!e.repeat){e.preventDefault();toggleFullscreen();}});
 document.addEventListener('keyup',e=>inputControls.releaseKey(e.code));window.addEventListener('blur',()=>{clearInput();audio.setActive(false);});window.addEventListener('focus',()=>audio.setActive(!document.hidden));document.addEventListener('visibilitychange',()=>{audio.setActive(!document.hidden);if(document.hidden)clearInput();});window.addEventListener('pagehide',()=>{inputControls.dispose();worker.terminate();audio.dispose();world.dispose();});

 loading.update(.98,tr('准备驾驶画面…','Preparing driving view…'));render();world.setFrame(drivingView.sample(performance.now())??frame);await new Promise(requestAnimationFrame);requestAnimationFrame(tick);loading.finish();stage.focus({preventScroll:true});window.drivingLab={world,worker,audio,get input(){return {...driveInput}},get frame(){return frame},recenter(){worker.postMessage({type:'recenter'});}};
}catch(e){worker?.terminate();loading.fail();console.error(e);}
