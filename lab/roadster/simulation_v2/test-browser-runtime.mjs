import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
import {readFile,writeFile} from 'node:fs/promises';
import {existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {createSimulation,loadEngine,allocateFixedNozzles,nozzleLayout,fanModel,qmat} from './browser-runtime.mjs';
import {PARAMETER_NAMES,controllerStep,createControllerMemory,wheelInertiaOffset,unitInertia} from './controller.mjs';

const here=fileURLToPath(new URL('.',import.meta.url));
const native=spawnSync(`${here}.venv/bin/python`,[`${here}export-browser-fixtures.py`],{encoding:'utf8',maxBuffer:8e6});
assert.equal(native.status,0,native.stderr);
const refs=JSON.parse(native.stdout),cfg=refs.config;
const bundledEngine=new URL('../vendor/mujoco/mujoco.js',import.meta.url);
const localEngine=new URL('../site/dist/vendor/mujoco/mujoco.js',import.meta.url);
const engine=await loadEngine(process.env.MUJOCO_JS??(existsSync(bundledEngine)?bundledEngine.href:localEngine.href));
const checks=[];
const started=performance.now();
function close(actual,expected,tolerance,label){
  const a=Array.isArray(actual)?actual.flat(Infinity):[actual],b=Array.isArray(expected)?expected.flat(Infinity):[expected];
  assert.equal(a.length,b.length,label);
  const delta=Math.max(...a.map((v,i)=>Math.abs(v-b[i])));assert.ok(delta<=tolerance,`${label}: difference ${delta} > ${tolerance}`);return delta;
}
const options={engine,xml:refs.xml,controllerStep:()=>({forceWorld:[0,0,0],torqueBody:[0,0,0],fanCommand:0}),createMemory:()=>({})};
for(const f of refs.fixtures){
  const sim=await createSimulation(cfg,refs.policies.untuned,f.case,options);
  try{
    assert.deepEqual(sim.snapshot().allocation.requestedForceWorld,[0,0,0],'Initial world-force readout must be defined');
    for(let i=0;i<f.steps;i++)sim.advance(f.command,f.fanCommand,f.stabilityContext??{});
    const s=sim.snapshot(),deltas={};
    for(const key of ['pos','quat','vel','omega','wheelRates'])deltas[key]=close(s[key],f.state[key],2e-7,`${f.label}.${key}`);
    close(s.fuel,f.fuel,2e-7,`${f.label}.fuel`);close(s.nozzleForces,f.forces,2e-6,`${f.label}.forces`);
    assert.equal(s.contact.wheelCount,f.contact.wheelCount,`${f.label}.wheelCount`);
    close(s.assemblyCom,f.assemblyCom,1e-10,`${f.label}.assemblyCom`);close(s.assemblyCom,s.pos,1e-10,`${f.label}: frame.pos must be whole COM`);
    for(const key of ['bodyMass','bodyIpos','bodyInertia'])close(s.massProperties[key],f.massProperties[key],1e-9,`${f.label}.${key}`);
    const mp=s.massProperties,p2=mp.bodyIpos.reduce((sum,x)=>sum+x*x,0),wheelInertia=wheelInertiaOffset(cfg),combinedInertia=mp.bodyInertia.map((v,i)=>v+mp.bodyMass*(p2-mp.bodyIpos[i]**2)+wheelInertia[i]);
    close(combinedInertia,unitInertia(cfg).map(v=>v*(mp.bodyMass+80)),1e-9,`${f.label}.wholeAssemblyInertia`);
    close(s.contact.stableSeconds,f.contact.stableSeconds,1e-10,`${f.label}.stableSeconds`);
    if(f.label==='rolling'){assert.ok(s.vel[0]>4.5,'Passive wheel model should keep rolling');assert.ok(s.contact.stableSeconds>=1,'Forward rolling should count as stable contact');}
    if(f.label==='single_jet'){assert.ok(f.pressureHistory[0]<1,'Pressure must decline from the first use');assert.ok(f.pressureHistory.every((p,i,a)=>i===0||p<a[i-1]),'Blowdown must remain continuous');}
    if(f.label==='free_fall'){close(s.pos[0],1.4,1e-6,'free fall x');close(s.vel[2],-1.962,1e-6,'free fall velocity');}
    checks.push({name:f.label,deltas});
  }finally{sim.dispose();}
}
const limits=nozzleLayout(cfg).map(n=>n.maxForce),pure=allocateFixedNozzles([0,0,0,1000,0,0],limits,cfg);
assert.ok(pure.every((f,i)=>f>=0&&f<=limits[i]));
for(const height of [cfg.vehicle.ground_cg_height+.005,5]){const fan=fanModel([0,0,height],qmat([1,0,0,0]),1,cfg);assert.ok(fan.power<=fan.availableFluidPower+1e-6);if(height===5)assert.ok(fan.suctionForce[2]===0);}
{
  const c={...refs.fixtures[3].case,task:'stationary_kickflip',vel:[0,0,0],omega:[0,0,0]},memory=createControllerMemory(c.task,c,cfg);
  Object.assign(memory,{phase:'settle',turnAngle:2*Math.PI,turnComplete:true,wasAirborne:true});
  const s={...c,R:qmat(c.quat),mass:c.mass,fuel:c.fuel,contact:{wheelCount:4}},u=controllerStep(s,memory,refs.policies.handcrafted,cfg),f=fanModel(s.pos,s.R,u.fanCommand,cfg);
  const combined=u.allocation.achievedWrenchBody.slice(0,3).map((v,i)=>v+f.suctionForce[i]+f.axialForce[i]);
  const delta=close(combined,f.suctionForce.map((v,i)=>v+f.axialForce[i]),1e-6,'Settling jets must preserve fan force');assert.ok(combined[2]<0);
  checks.push({name:'settle_preserves_fan_force',deltas:{combinedExtraForce:delta},downforceN:-combined[2]});
}
{
  const c=refs.fixtures[1].case,base={...c,R:qmat(c.quat),contact:{wheelCount:0},mass:c.mass,fuel:c.fuel};let max=0;
  for(const phase of ['boost','flip','recover','land','settle']){const memory=createControllerMemory(c.task,c,cfg);memory.phase=phase;const a=controllerStep(base,memory,refs.policies.handcrafted,cfg),b=controllerStep({...base,vel:[-11,7,base.vel[2]],pos:[50,-40,base.pos[2]]},memory,refs.policies.handcrafted,cfg);max=Math.max(max,close(a.nozzleThrottle,b.nozzleThrottle,1e-12,'Horizontal state must not change control output'));assert.ok(a.forceWorld[0]===0&&a.forceWorld[1]===0);}
  checks.push({name:'no_horizontal_feedback',deltas:{throttle:max}});
}
const sim=await createSimulation(cfg,refs.policies.untuned,refs.fixtures[0].case,options);
try{
  const initial=sim.snapshot();sim.applyImpulse([2000,0,0]);sim.advance(Array(12).fill(0),0);close(sim.snapshot().vel[0]-initial.vel[0],1,1e-6,'Impulse integrated by MuJoCo');
  assert.throws(()=>sim.reset({...refs.fixtures[0].case,pos:[0,0,.1]}),/intersects/);
  assert.throws(()=>sim.reset({...refs.fixtures[0].case,fuel:-1}),/Invalid/);
  assert.throws(()=>sim.reset({...refs.fixtures[0].case,fuel:cfg.fuel.initial_kg+1}),/Invalid/);
  assert.throws(()=>sim.reset({...refs.fixtures[0].case,delivery_mode:'regulated_then_blowdown'}),/unregulated/);
  assert.throws(()=>sim.reset({...refs.fixtures[0].case,mass:100,fuel:0}),/inertia/);
  assert.throws(()=>sim.reset(refs.fixtures[0].case,[NaN]),/parameters/);
  assert.throws(()=>sim.applyImpulse([NaN,0,0]),/Invalid/);
  sim.reset(refs.fixtures[0].case,refs.policies.untuned,{policyId:'no_control'});
  assert.deepEqual(sim.step(1).allocation.requestedForceWorld,[0,0,0],'No-control world-force readout must stay zero');
}finally{sim.dispose();}
assert.throws(()=>sim.step(),/disposed/);
// Isolate the height gate with synthetic completion flags; this is not a flight result.
{
  const c=refs.rollouts.find(r=>r.case.task==='stationary_kickflip').case;
  const zeroController=()=>({memory:{turnComplete:true,wasAirborne:true,turnAngle:2*Math.PI,initialRoll:0},goal:{phase:'settle'},error:{},fanCommand:0,nozzleThrottle:Array(12).fill(0),desiredForceWorld:[0,0,0],allocation:{requestedWrenchBody:Array(6).fill(0),achievedWrenchBody:Array(6).fill(0),residual:Array(6).fill(0),normalizedResidual:0}});
  const run=await createSimulation(cfg,refs.policies.untuned,c,{engine,xml:refs.xml,controllerStep:zeroController,createMemory:()=>({})});
  try{let s=run.snapshot();while(!s.done)s=run.step(1);assert.equal(s.status,'height_shortfall');assert.equal(s.success,false);assert.equal(s.metrics.minimumHeightMet,false);assert.ok(s.contact.stableSeconds>=1);checks.push({name:'height_shortfall_gate',kind:'criterion_test_with_synthetic_completion_flags_not_a_flight',status:s.status,deltas:{},peakHeight:s.metrics.peakHeight,minimumApexCG:s.metrics.minimumApexCG});}finally{run.dispose();}
}
if(process.env.WORLD_ONLY!=='1'){
  for(const r of refs.rollouts){
    const run=await createSimulation(cfg,r.params,r.case,{engine,xml:refs.xml,policyId:r.policyId});
    try{
      let s=run.snapshot(),launch=null,energyWasReady=false,coastSteps=0,observedPeak=s.pos[2],failedPeak=null;
      while(!s.done){
        const before=s;s=run.step(1);assert.ok(s.allocation.requestedForceWorld.every(Number.isFinite));assert.ok(Math.hypot(...s.allocation.requestedForceWorld.slice(0,2))<1e-12,'Reported horizontal force target must remain zero');
        if(failedPeak===null)observedPeak=Math.max(observedPeak,s.pos[2]);
        else{assert.equal(s.metrics.peakHeight,failedPeak,'Post-failure rebound must not increase credited height');assert.equal(s.reward.components.heightProgress,0,'Post-failure height gains must not earn reward');}
        if(failedPeak===null&&['body_contact','hard_contact','out_of_bounds','height_shortfall'].includes(s.status))failedPeak=s.metrics.peakHeight;
        if(energyWasReady)assert.equal(s.goal.energyReady,true,'Energy-ready mode must remain latched');energyWasReady=s.goal.energyReady;
        if(s.goal.energyReady&&s.goal.phase==='flip'){assert.deepEqual(s.allocation.requestedForceWorld,[0,0,0],'Coasting flip requests a pure force couple');coastSteps++;}
        if(s.success)assert.ok(s.metrics.minimumHeightMet&&observedPeak+1e-9>=(cfg.tasks[r.case.task].minimum_peak_cg_height??0),'Success requires the actual measured height');
        assert.ok(Number.isFinite(s.geometryMinimumCG)&&Number.isFinite(s.geometryClearance));
        if(!launch&&r.case.task!=='moving_recovery'&&s.goal.phase==='flip'){
          launch={t:before.t,heightCG:before.pos[2],clearance:before.pos[2]-cfg.vehicle.ground_cg_height,verticalVelocity:before.vel[2],wheelCount:before.contact.wheelCount,bodyContact:before.contact.bodyContact,requestedRollTorque:s.allocation.requestedWrenchBody[3],allocatedRollTorque:s.allocation.achievedWrenchBody[3],actualRollTorque:s.nozzles.reduce((sum,n)=>sum+(n.position[1]*n.direction[2]-n.position[2]*n.direction[1])*n.force,0),requestedForceWorld:s.allocation.requestedForceWorld,actualJetForceWorld:s.jetForceWorld};
        }
      }
      assert.equal(s.status,r.status,`${r.task}/${r.policyId}.status`);
      const deltas={};for(const key of ['pos','quat','vel','omega','wheelRates','jetForceWorld','jetHorizontalImpulse','fanHorizontalImpulse'])deltas[key]=close(s[key],r.final[key],2e-5,`${r.task}/${r.policyId}.${key}`);
      close(s.assemblyCom,s.pos,1e-10,'Final root must be whole COM');
      close(s.fuel,r.final.fuel,2e-5,`${r.task}/${r.policyId}.fuel`);close(s.metrics.stableSeconds,r.metrics.stableSeconds,1e-8,'stable interval');
      close(s.metrics.score,r.metrics.score,2e-4,`${r.task}/${r.policyId}.score`);
      close(s.metrics.peakHeight,r.metrics.peakHeight,2e-5,`${r.task}/${r.policyId}.peakHeight`);close(s.metrics.peakHeight,observedPeak,1e-10,'Peak must come from simulated positions');
      assert.equal(s.metrics.minimumHeightMet,r.metrics.minimumHeightMet);close(s.metrics.minimumApexCG,r.metrics.minimumApexCG,1e-12,'Task height criterion');
      close(s.geometryMinimumCG,r.final.geometryMinimumCG,2e-5,'Actual geometric support bound');close(s.geometryClearance,r.final.geometryClearance,2e-5,'Actual geometric clearance');
      for(const key of ['finalLongitudinalSpeed','finalLateralSpeed','finalVerticalSpeed','peakJetHorizontalForce'])close(s.metrics[key],r.metrics[key],2e-5,`${r.task}/${r.policyId}.${key}`);
      let grip=null;
      if(s.success&&s.goal.phase==='settle'&&-s.fan.suctionForce[2]>100){
        const fanDownforce=-s.fan.suctionForce[2],normalGain=s.contact.totalNormalForce-s.mass*cfg.physics.gravity,jetVertical=s.nozzles.reduce((sum,n)=>sum+n.force*n.directionWorld[2],0);
        assert.ok(normalGain>.8*fanDownforce,`${r.task}: suction must increase actual MuJoCo normal force`);
        assert.ok(Math.abs(jetVertical)<.05*fanDownforce,`${r.task}: jets must not cancel suction`);
        grip={fanDownforce,normalGain,jetVertical};
      }
      if(r.case.task!=='moving_recovery'){
        assert.ok(launch&&r.launch,'Kickflip must issue an airborne roll command');
        assert.equal(launch.wheelCount,0);assert.equal(launch.bodyContact,false);assert.ok(launch.verticalVelocity>0,'Roll must start in the ascending segment');
        assert.ok(launch.clearance>=r.params[PARAMETER_NAMES.indexOf('flip_start_clearance')]);assert.ok(r.params[PARAMETER_NAMES.indexOf('flip_start_clearance')]<=.15,'Current early gate is at most 15 cm');
        assert.ok(launch.requestedRollTorque>0&&launch.allocatedRollTorque>0&&launch.actualRollTorque>0,'Rising vehicle must receive an actual roll moment');
        for(const key of ['t','heightCG','clearance','verticalVelocity','requestedRollTorque','allocatedRollTorque','actualRollTorque','requestedForceWorld','actualJetForceWorld'])close(launch[key],r.launch[key],2e-6,`${r.task}/${r.policyId}.launch.${key}`);
      }
      checks.push({name:`${r.task}/${r.policyId}`,status:s.status,deltas,grip,launch,coastSteps,peakHeight:s.metrics.peakHeight,minimumApexCG:s.metrics.minimumApexCG,minimumHeightMet:s.metrics.minimumHeightMet});
    }finally{run.dispose();}
  }
}
const sourceHashes={};for(const file of ['core.py','config.json','controller.mjs','browser-runtime.mjs',...('final' in refs.policies?['policies.json']:[])])sourceHashes[file]=createHash('sha256').update(await readFile(new URL(file,import.meta.url))).digest('hex');
const finalPolicyGeneration='final' in refs.policies?JSON.parse(await readFile(new URL('./policies.json',import.meta.url),'utf8')).policies.final.generation:null;
const report={revision:refs.revision,status:'final' in refs.policies?'validated_final':'validated_baselines_awaiting_final',validationMeaning:'Native/WASM numerical parity; individual task success or failure is recorded separately.',engine:'MuJoCo 3.15.0 native Python vs official WebAssembly',timestamp:new Date().toISOString(),runtimeWallSeconds:(performance.now()-started)/1000,finalPolicyTested:'final' in refs.policies,finalPolicyGeneration,sourceHashes,checks};
await writeFile(new URL('./browser-verification.json',import.meta.url),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
