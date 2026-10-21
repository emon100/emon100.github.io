// Same goal-conditioned controller as simulation_v4/unified_controller.py.
export * from '../simulation_v3/baseline_r6/controller.mjs';
import {qmat,unitInertia,nozzleLayout,delivery,fanModel,geometryMinimumCG} from '../simulation_v3/baseline_r6/controller.mjs';
import {localCapacity,rotationTime,landingGasBudget} from '../simulation_v3/controller.mjs';
import {clamp,dot,norm,add,sub,mul,cross,mv,tmv,axisQ,wrap,rotationError,wrenchMatrix} from '../simulation_v3/math.mjs';
import {predictVerticalStop} from '../simulation_v3/landing-predictor.mjs';
import {createAllocator} from '../simulation_v3/allocator/conservative_allocator.mjs';
export const PARAMETER_NAMES=['attitude_gain','rate_gain','rate_cap','capacity_fraction','brake_cap','early_rate','time_reserve','vertical_gain','descent_speed','descent_accel','touchdown_speed','ground_margin','prediction_fraction','lookahead_scale'];
export function qmul(a,b){return [a[0]*b[0]-dot(a.slice(1),b.slice(1)),...add(add(mul(b.slice(1),a[0]),mul(a.slice(1),b[0])),cross(a.slice(1),b.slice(1)))];}
export function goalPath(c){
 const q=c.quat,R=qmat(q),speed=norm(c.vel.slice(0,2)),turns=c.goal?.turns??0;let yaw=Math.atan2(R[1][0],R[0][0]);
 if(!Number.isFinite(turns)||turns<0||turns>2)throw Error('翻转目标须在 0 到 2 圈之间');
 if(speed>.5){const heading=Math.atan2(c.vel[1],c.vel[0]),choices=[heading,heading+Math.PI];yaw=Math.abs(wrap(choices[0]-yaw))<=Math.abs(wrap(choices[1]-yaw))?choices[0]:choices[1];}
 const target=axisQ([0,0,1],yaw),points=[[...q]],error=rotationError(q,target),angle=norm(error),count=Math.max(1,Math.ceil(angle/(Math.PI/4)));
 for(let i=1;i<=count;i++)points.push(qmul(q,axisQ(mul(error,1/Math.max(angle,1e-12)),angle*i/count)));
 const steps=Math.max(1,Math.ceil(turns*8));for(let i=1;i<=steps;i++)points.push(qmul(target,axisQ([1,0,0],turns*2*Math.PI*i/steps)));
 const segments=[];for(let i=1;i<points.length;i++){const e=rotationError(points[i-1],points[i]),length=norm(e);if(length>1e-9)segments.push({q:points[i-1],axis:mul(e,1/length),length});}
 return {segments,target,requiredSpin:turns*2*Math.PI,spinAxis:qmat(target).map(r=>r[0])};
}
export function createMemory(c,cfg){
 const plan=goalPath(c);return {...plan,index:0,progress:0,completedArc:0,totalArc:plan.segments.reduce((s,x)=>s+x.length,0),released:false,turnAngle:0,initialRoll:0,spinMeasured:0,spinSatisfied:plan.requiredSpin===0,turnComplete:false,wasAirborne:true,goal:{},error:{},elapsed:0,peakHeight:c.pos[2],initialVelocity:[...c.vel],maxHorizontal:0,airborneDelta:0,firstContactDelta:null,pathFloors:plan.segments.map(x=>Math.max(...[0,.5,1].map(f=>geometryMinimumCG(qmat(qmul(x.q,axisQ(x.axis,x.length*f))),cfg))))};
}
export const createControllerMemory=(_task,c,cfg)=>createMemory(c,cfg);
export function controllerStep(state,m,params,cfg){
 const p=Array.isArray(params)?Object.fromEntries(PARAMETER_NAMES.map((n,i)=>[n,params[i]])):params,R=state.R??qmat(state.quat),s={...state,R},contact=s.contact??{},dt=cfg.physics.control_dt,J=mul(unitInertia(cfg),s.mass),tilt=Math.acos(clamp(R[2][2],-1,1)),rate=norm(s.omega),gravity=cfg.physics.gravity,gap=s.pos[2]-geometryMinimumCG(R,cfg);
 if(m.elapsed>0){const increment=dot(s.omegaWorld,m.spinAxis)*dt;m.turnAngle+=increment;if(!m.spinSatisfied&&m.completedArc+m.progress>=m.totalArc-m.requiredSpin-.015)m.spinMeasured+=increment;}
 const dv=norm(sub(s.vel.slice(0,2),m.initialVelocity.slice(0,2)));m.maxHorizontal=Math.max(m.maxHorizontal,dv);
 if(m.firstContactDelta===null&&!(contact.count??0))m.airborneDelta=Math.max(m.airborneDelta,dv);else if(m.firstContactDelta===null)m.firstContactDelta=dv;
 for(let k=0;k<m.segments.length;k++){if(m.index>=m.segments.length)break;const seg=m.segments[m.index],e=rotationError(seg.q,s.quat),along=dot(e,seg.axis),crossError=norm(sub(e,mul(seg.axis,along)));m.progress=clamp(along,0,seg.length);if(along>=seg.length-.015&&crossError<.25){m.completedArc+=seg.length;m.index++;m.progress=0;}else break;}
 if(m.completedArc+m.progress>=m.totalArc-.05&&m.spinMeasured>=m.requiredSpin-.25)m.spinSatisfied=true;
 const remaining=Math.max(0,m.totalArc-m.completedArc-m.progress),targetError=rotationError(s.quat,m.target);
 let tangent=mv(R,mul(targetError,1/Math.max(norm(targetError),1e-12)));if(m.index<m.segments.length)tangent=mv(qmat(m.segments[m.index].q),m.segments[m.index].axis);if(norm(tangent)<.5)tangent=R.map(r=>r[0]);
 let brakeDistance=remaining,distance=-m.progress;for(let j=m.index;j<m.segments.length-1;j++){const current=m.segments[j],next=m.segments[j+1];distance+=current.length;if(dot(mv(qmat(current.q),current.axis),mv(qmat(next.q),next.axis))<.98){brakeDistance=Math.max(.005,distance);break;}}
 const caps=localCapacity(s,cfg,tangent),accel=Math.min(18,p.capacity_fraction*caps.accelerate),brake=Math.min(p.brake_cap,p.capacity_fraction*caps.brake),toward=dot(s.omegaWorld,tangent),delay=dt+cfg.actuators.lag_seconds,stop=Math.max(0,toward)**2/(2*Math.max(brake,.01))+Math.max(0,toward)*delay,turnTime=rotationTime(remaining,toward,p.rate_cap,accel,brake),floor=Math.max(...m.pathFloors.slice(m.index),geometryMinimumCG(R,cfg))+.1,vz=s.vel[2],disc=vz*vz+2*gravity*(s.pos[2]-floor),coast=disc>=0?Math.max(0,(vz+Math.sqrt(disc))/gravity):0,needed=turnTime+p.time_reserve+(m.requiredSpin>0&&remaining>m.requiredSpin+.015?.35:0);
 if(coast>=needed)m.released=true;
 const boost=!m.released&&remaining>.6&&caps.vertical>1.4*s.mass*gravity;
 let commandRate=Math.min(p.rate_cap,Math.sqrt(2*Math.max(brake,0)*brakeDistance));if(boost)commandRate=Math.min(commandRate,p.early_rate);
 const lookahead=Math.min(remaining,brakeDistance,p.lookahead_scale*commandRate*p.rate_gain/p.attitude_gain);let j=m.index,offset=m.progress+lookahead,reference=m.target;
 while(j<m.segments.length){const seg=m.segments[j];if(offset<=seg.length){reference=qmul(seg.q,axisQ(seg.axis,offset));break;}offset-=seg.length;j++;}
 const holdSupport=!m.released&&m.requiredSpin>0&&remaining<=m.requiredSpin+.25;if(holdSupport)reference=m.target;
 const error=rotationError(s.quat,reference),axisBody=tmv(R,tangent);let alpha=sub(mul(error,p.attitude_gain),mul(s.omega,p.rate_gain)),along=dot(alpha,axisBody);if(toward>.2&&stop>brakeDistance)along=Math.min(along,-brake);alpha=add(alpha,mul(axisBody,clamp(along,-brake,accel)-dot(alpha,axisBody)));
 m.turnComplete=remaining<.08&&norm(targetError)<12*Math.PI/180&&tilt<12*Math.PI/180&&rate<.65&&m.spinSatisfied;
 const settle=m.turnComplete&&(contact.wheelCount??0)>=3,budget=landingGasBudget(s,cfg);let forceZ=s.mass*gravity,targetVz=null,mode='support',forecast=null;
 if(boost){targetVz=Math.min(12,Math.max(2,(floor-s.pos[2]+.5*gravity*needed*needed)/Math.max(needed,.1)+.5));forceZ=s.mass*(gravity+p.vertical_gain*(targetVz-vz));mode='build_airtime';}
 else if(remaining<.6&&tilt<30*Math.PI/180){const net=caps.vertical/s.mass-gravity,cap=Math.min(9,Math.max(p.descent_speed,gap/Math.max(.3,.65*budget.equivalentHoverTime))),available=Math.max(0,gap-p.ground_margin-Math.max(0,-vz)*2*delay),decel=Math.min(p.descent_accel,Math.max(.1,.5*net));targetVz=-Math.min(cap,Math.sqrt(2*decel*available)+p.touchdown_speed);const ff=available>.02&&targetVz>-cap&&targetVz<-.2&&vz<-.2?decel:0;forceZ=s.mass*(gravity+p.vertical_gain*(targetVz-vz)+ff);mode='landing';if(tilt<8*Math.PI/180&&rate<.4&&gap>1.5){forecast=predictVerticalStop(s,cfg,{fraction:p.prediction_fraction,touchdownSpeed:p.touchdown_speed});const margin=.05+Math.max(0,-vz)*dt;forceZ=forecast.feasible&&gap>forecast.distance+margin?0:p.prediction_fraction*caps.vertical;mode=forceZ===0?'predicted_coast':'predicted_brake';}}
 else if(m.released&&coast>=needed){forceZ=0;mode='coast';}
 if(settle){forceZ=0;alpha=mul(alpha,.2);mode='settle';}
 const torque=add(alpha.map((v,i)=>J[i]*v),cross(s.omega,s.omega.map((v,i)=>J[i]*v))),force=[0,0,Math.max(0,forceZ)],fanCommand=settle?.095:0;
 m.elapsed+=dt;m.peakHeight=Math.max(m.peakHeight,s.pos[2]);
 const goal={phase:mode,holdingSupportPose:holdSupport,targetQuat:m.target,referenceQuat:reference,pathRemaining:remaining,pathProgress:m.completedArc+m.progress,pathLength:m.totalArc,requiredSpin:m.requiredSpin,spinMeasured:m.spinMeasured,spinSatisfied:m.spinSatisfied,turnAngle:m.turnAngle,turnComplete:m.turnComplete,targetVerticalSpeed:targetVz,commandAngularRate:commandRate,angularStoppingAngle:stop,distanceToNextBrake:brakeDistance,availableCoastTime:coast,requiredRecoveryTime:needed,pressureBudget:budget,verticalStoppingForecast:forecast,horizontalForceTarget:[0,0],horizontalVelocityError:dv,airborneHorizontalChange:m.airborneDelta,measuredStateFeedback:true,controllerFamily:'unified_goal_path_PD'};m.goal=goal;m.error={tilt,angularSpeed:rate,remaining,horizontalVelocityError:dv};
 const fan=fanModel(s.pos,R,fanCommand,cfg),requestedWrenchBody=[...tmv(R,force),...sub(torque,tmv(R,fan.torqueWorld))],nozzles=nozzleLayout(cfg),A=wrenchMatrix(nozzles),pressure=delivery(s.fuel,cfg).pressureScale,disabled=new Set(s.disabledNozzles??[]),limits=nozzles.map(n=>disabled.has(n.id)?0:n.maxForce*pressure*(s.thrustScale??1)),parts=[[0,0,0,...requestedWrenchBody.slice(3)],[...requestedWrenchBody.slice(0,3),0,0,0],Array(6).fill(0)],allocated=createAllocator(A)(parts,limits,cfg.actuators.total_force_max*pressure*(s.thrustScale??1)),achieved=mv(A,allocated.forces),residual=sub(requestedWrenchBody,achieved),allocation={requestedWrenchBody,achievedWrenchBody:achieved,residual,normalizedResidual:norm(residual)/Math.max(norm(requestedWrenchBody),1),momentScale:allocated.tiltScale,forceScale:allocated.forceScale,reservedZeroScale:allocated.yawScale,allocator:'conservative_analytic_priority',globallyOptimal:false,feasibilityCalls:allocated.feasibilityCalls,solverIterations:allocated.iterations,priority:'attitude_moment_then_vertical_force'};
 return {nozzleThrottle:allocated.forces.map((f,i)=>limits[i]>0?f/limits[i]:0),fanCommand,desiredForceWorld:force,desiredTorqueBody:torque,requestedWrenchBody,allocation,goal,error:m.error,memory:m};
}
