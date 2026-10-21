import {t as tr} from '../i18n.mjs';
import {createSimulation} from '../simulation_v4/browser-runtime.mjs?v=20261019';
import {nozzleLayout,qmat,unitInertia} from '../simulation_v4/controller.mjs';
export const MODEL='typesafe/jev-1.14';
export const LEVELS={off:0,p10:.1,p20:.2,p30:.3,p40:.4,p55:.55,p75:.75,full:1};
export const PRESETS=[
 {id:'level',label:tr('正立下降 · 3 m','Upright descent · 3 m'),pos:[0,0,3],quat:[1,0,0,0],vel:[0,0,-1],omega:[0,0,0]},
 {id:'banked',label:tr('侧倾 20° · 6 m','20° tilt · 6 m'),pos:[0,0,6],quat:[Math.cos(Math.PI/18),Math.sin(Math.PI/18),0,0],vel:[2,0,-1],omega:[.3,0,0]},
 {id:'inverted',label:tr('倒置释放 · 9 m','Inverted release · 9 m'),pos:[0,0,9],quat:[0,1,0,0],vel:[3,0,0],omega:[0,0,0]}
];
export const caseFor=id=>({...structuredClone(PRESETS.find(p=>p.id===id)??PRESETS[0]),task:'moving_recovery',goal:{turns:0},fuel:260,mass:2000});
export function tenNozzles(cfg){return [...nozzleLayout(cfg).slice(0,8),...[-1,1].map((side,i)=>({id:`J${i+9<10?'0':''}${i+9}`,position:[0,side*.7,0],direction:[1,0,0],maxForce:10000,role:'yaw',evidence:'hypothetical_10_nozzle_layout'}))];}
export function tenNozzleXML(xml,cfg){const jets=tenNozzles(cfg);return xml.replace(/<site name="J\d+"[^>]*\/>/g,'').replace('</body></worldbody>',jets.map(n=>`<site name="${n.id}" pos="${n.position.join(' ')}" size=".025"/>`).join('')+'</body></worldbody>').replace(/<actuator>[\s\S]*?<\/actuator>/,`<actuator>${jets.map(n=>`<general name="${n.id}" site="${n.id}" gear="${n.direction.join(' ')} 0 0 0" ctrllimited="true" ctrlrange="0 ${n.maxForce}"/>`).join('')}</actuator>`);}
export function parseDecision(response){
 if(!response?.answers||Object.keys(response.answers).length!==10)throw Error(tr('JEV 必须返回完整的 10 个喷口决策，实验已停止。','JEV must return decisions for all 10 nozzles. The experiment has stopped.'));
 return Array.from({length:10},(_,i)=>{const id='J'+String(i+1).padStart(2,'0'),a=response.answers[id];if(a?.type!=='choice'||!Object.hasOwn(LEVELS,a.choice))throw Error(tr('{id} 决策无效，实验已停止。','Invalid decision for {id}. The experiment has stopped.',{id}));return LEVELS[a.choice];});
}
const round=x=>Array.isArray(x)?x.map(round):typeof x==='number'?Math.round(x*10000)/10000:x;
export function decisionRequest(frame,cfg,windowSeconds){
 const R=qmat(frame.quat),nozzles=tenNozzles(cfg),J=unitInertia(cfg).map(x=>x*frame.mass);
 return {model:MODEL,state:{
  task:'Directly control all ten fixed cold-gas nozzles to land this car upright, gently, and with low angular and lateral speed. These ten answers are simultaneous actuator commands. No other stabilization controller exists. Minimize fuel. Do not keep hovering. Turn jets off after safe wheel contact.',
  axes:'Body +X forward, +Y left, +Z roof. World +Z up. Right-handed rotations; roll is about body X. Quaternions are [w,x,y,z]. Nozzle direction is force ON CAR, not exhaust.',
  physics:{gravity:9.81,mass_kg:round(frame.mass),inertia_kg_m2:round(J),command_duration_s:windowSeconds,valve_lag_s:.025,pressure_scale:round(frame.pressureScale),total_force_limit_N:round(90000*frame.pressureScale),ground_cg_height_m:.5},
  measured:{time_s:frame.t,position_m:round(frame.pos),quaternion:round(frame.quat),body_to_world:round(R),velocity_world_m_s:round(frame.vel),angular_velocity_body_rad_s:round(frame.omega),roll_deg:round(Math.atan2(R[2][1],R[2][2])*180/Math.PI),pitch_deg:round(Math.asin(Math.max(-1,Math.min(1,-R[2][0])))*180/Math.PI),clearance_m:round(frame.geometryClearance),gas_kg:round(frame.fuel),wheel_contacts:frame.contact.wheelCount,previous_actual_throttles:round(frame.nozzleThrottle)},
  nozzles:nozzles.map(n=>({...n,maxForce:round(n.maxForce*frame.pressureScale),torque_per_newton:[n.position[1]*n.direction[2]-n.position[2]*n.direction[1],n.position[2]*n.direction[0]-n.position[0]*n.direction[2],n.position[0]*n.direction[1]-n.position[1]*n.direction[0]]})),
  goal:'Touchdown closing speed <=1.6 m/s, tilt <8 degrees, angular speed <0.2 rad/s, at least 3 wheel contacts stable for 1 second. Use the nozzle moment arms to counter rotation and brake angular momentum before level. Coordinate paired corner jets; opposite jets at one corner must not fight. Upward world force brakes descent; body-up jets on an inverted car push toward the ground. Account for valve lag and decreasing pressure.'
 },questions:Object.fromEntries(nozzles.map(n=>[n.id,{type:'choice',instructions:`Choose the throttle of ${n.id} for the next ${windowSeconds} simulated seconds as part of the coordinated ten-nozzle command. Base the choice on the measured physical state and landing goal.`,criteria:Object.fromEntries(Object.entries(LEVELS).map(([k,v])=>[k,`${v*100}% of this nozzle's available thrust`]))}]))};
}
export async function createJevSimulation(cfg,params,c,options){
 let command=Array(10).fill(0);
 const sim=await createSimulation(cfg,params,c,{...options,nozzles:tenNozzles(cfg),xml:tenNozzleXML(options.xml,cfg),policyId:'jev_direct',createMemory:()=>({turnComplete:true,wasAirborne:true}),controllerStep:(_s,m)=>({nozzleThrottle:command,fanCommand:0,desiredForceWorld:[0,0,0],allocation:{source:'JEV ten direct throttle choices'},goal:{phase:'jev_direct'},memory:m})});
 return {...sim,setCommand(value){if(!Array.isArray(value)||value.length!==10||!value.every(v=>Number.isFinite(v)&&v>=0&&v<=1))throw Error('Invalid ten-nozzle command');command=[...value];}};
}
export function compactFrame(f){return Object.fromEntries(['t','pos','quat','vel','omega','mass','fuel','pressureScale','nozzleThrottle','nozzleForces','contact','geometryClearance','status','success','done','metrics'].map(k=>[k,f[k]]));}
