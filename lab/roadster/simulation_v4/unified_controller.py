"""One measured-state path follower for airborne landing with optional full rotations."""
from pathlib import Path
import sys,importlib.util,math,copy,json
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'simulation_v3'))
spec=importlib.util.spec_from_file_location('recovery_v3',HERE.parent/'simulation_v3/controller.py');v3=importlib.util.module_from_spec(spec);spec.loader.exec_module(v3);core=v3.core
PARAMETERS=['attitude_gain','rate_gain','rate_cap','capacity_fraction','brake_cap','early_rate','time_reserve','vertical_gain','descent_speed','descent_accel','touchdown_speed','ground_margin','prediction_fraction','lookahead_scale']
INITIAL=np.array([6.,2.,5.,.5,9.,.2,.3,4.,4.5,3.,.15,.2,.85,1.])
REFERENCE=np.array([30.,8.,8.,.8,20.,0.,.25,10.,8.,7.,.05,.1,.95,1.])
LOWER=np.array([4.,1.5,2.5,.35,5.,0.,.15,3.,2.5,2.,.05,.1,.7,.65])
UPPER=np.array([40.,12.,10.,.85,25.,.5,.8,15.,12.,12.,.3,.5,1.,1.4])

def qmul(a,b):
 return np.r_[a[0]*b[0]-np.dot(a[1:],b[1:]),a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:],b[1:])]
def qpath(case):
 q=np.asarray(case['quat']);R=core.qmat(q);v=np.asarray(case['vel']);yaw=math.atan2(R[1,0],R[0,0]);speed=np.linalg.norm(v[:2]);goal=case.get('goal',{});turns=float(goal.get('turns',0))
 if not np.isfinite(turns) or turns<0 or turns>2:raise ValueError('Goal turns must be between 0 and 2')
 if speed>.5:
  heading=math.atan2(v[1],v[0]);yaw=min([heading,heading+math.pi],key=lambda x:abs(v3.core_wrap(x-yaw)))
 target=core.axis_q([0,0,1],yaw);points=[q.copy()]
 error=core.rot_error(q,target);angle=float(np.linalg.norm(error))
 for a in np.linspace(0,angle,max(1,math.ceil(angle/(math.pi/4)))+1)[1:]:points.append(qmul(q,core.axis_q(error/max(angle,1e-12),a)))
 for a in np.linspace(0,turns*2*math.pi,max(1,math.ceil(turns*8))+1)[1:]:points.append(qmul(target,core.axis_q([1,0,0],a)))
 segments=[]
 for a,b in zip(points,points[1:]):
  e=core.rot_error(a,b);length=float(np.linalg.norm(e))
  if length>1e-9:segments.append({'q':a,'axis':e/length,'length':length})
 return segments,target,turns*2*math.pi,core.qmat(target)[:,0]

class UnifiedPolicy:
 def __init__(self,parameters,task,case,cfg):
  self.cfg=cfg;self.p=dict(zip(PARAMETERS,parameters));self.segments,self.target,self.required_spin,self.spin_axis=qpath(case);self.index=0;self.progress=0.;self.completed_arc=0.;self.total_arc=sum(x['length'] for x in self.segments);self.released=False;self.turn_angle=0.;self.initial_roll=0.;self.spin_measured=0.;self.spin_satisfied=self.required_spin==0;self.turn_complete=False;self.was_airborne=True;self.goal={};self.error={};self.elapsed=0.;self.peak_height=case['pos'][2];self.initial_velocity=np.asarray(case['vel']);self.initial_quat=np.asarray(case['quat']);self.minimum_horizontal=np.inf;self.max_horizontal=0.;self.airborne_delta=0.;self.first_contact_delta=None;self.path_floors=[max(core.geometry_minimum_cg(core.qmat(qmul(x['q'],core.axis_q(x['axis'],x['length']*f))),cfg) for f in [0,.5,1]) for x in self.segments]
 def command(self,s,contact):
  p=self.p;cfg=self.cfg;dt=cfg['physics']['control_dt'];R=s['R'];J=core.unit_inertia(cfg)*s['mass'];tilt=math.acos(np.clip(R[2,2],-1,1));rate=float(np.linalg.norm(s['omega']));gravity=cfg['physics']['gravity'];gap=s['pos'][2]-core.geometry_minimum_cg(R,cfg)
  if self.elapsed>0:
   increment=float(np.dot(s['omegaWorld'],self.spin_axis))*dt;self.turn_angle+=increment
   if not self.spin_satisfied and self.completed_arc+self.progress>=self.total_arc-self.required_spin-.015:self.spin_measured+=increment
  dv=float(np.linalg.norm(s['vel'][:2]-self.initial_velocity[:2]));self.max_horizontal=max(self.max_horizontal,dv)
  if self.first_contact_delta is None and not contact.get('count',0):self.airborne_delta=max(self.airborne_delta,dv)
  elif self.first_contact_delta is None:self.first_contact_delta=dv
  for _ in range(len(self.segments)):
   if self.index>=len(self.segments):break
   seg=self.segments[self.index];e=core.rot_error(seg['q'],s['quat']);along=float(np.dot(e,seg['axis']));cross_error=np.linalg.norm(e-along*seg['axis']);self.progress=float(np.clip(along,0,seg['length']))
   if along>=seg['length']-.015 and cross_error<.25:self.completed_arc+=seg['length'];self.index+=1;self.progress=0.
   else:break
  if self.completed_arc+self.progress>=self.total_arc-.05 and self.spin_measured>=self.required_spin-.25:self.spin_satisfied=True
  remaining=max(0.,self.total_arc-self.completed_arc-self.progress);target_error=core.rot_error(s['quat'],self.target)
  tangent=R@target_error/max(np.linalg.norm(target_error),1e-12)
  if self.index<len(self.segments):tangent=core.qmat(self.segments[self.index]['q'])@self.segments[self.index]['axis']
  if np.linalg.norm(tangent)<.5:tangent=R[:,0]
  brake_distance=remaining
  distance=-self.progress
  for j in range(self.index,len(self.segments)-1):
   current=self.segments[j];nxt=self.segments[j+1];distance+=current['length']
   if np.dot(core.qmat(current['q'])@current['axis'],core.qmat(nxt['q'])@nxt['axis'])<.98:
    brake_distance=max(.005,distance);break
  caps=v3.local_capacity(s,cfg,tangent);accel=min(18.,p['capacity_fraction']*caps['accelerate']);brake=min(p['brake_cap'],p['capacity_fraction']*caps['brake']);toward=float(np.dot(s['omegaWorld'],tangent));delay=dt+cfg['actuators']['lag_seconds'];stop=max(0.,toward)**2/(2*max(brake,.01))+max(0.,toward)*delay
  turn_time=v3.rotation_time(remaining,toward,p['rate_cap'],accel,brake);floor=max(self.path_floors[self.index:]+[core.geometry_minimum_cg(R,cfg)])+.1;vz=float(s['vel'][2]);disc=vz*vz+2*gravity*(s['pos'][2]-floor);coast=max(0.,(vz+math.sqrt(disc))/gravity) if disc>=0 else 0.;needed=turn_time+p['time_reserve']
  if self.required_spin>0 and remaining>self.required_spin+.015:needed+=.35
  if coast>=needed:self.released=True
  boost=not self.released and remaining>.6 and caps['vertical']>1.4*s['mass']*gravity
  command_rate=min(p['rate_cap'],math.sqrt(2*max(brake,0.)*brake_distance));command_rate=min(command_rate,p['early_rate']) if boost else command_rate
  lookahead=min(remaining,brake_distance,p['lookahead_scale']*command_rate*p['rate_gain']/p['attitude_gain']);j=self.index;distance=self.progress+lookahead;reference=self.target
  while j<len(self.segments):
   seg=self.segments[j]
   if distance<=seg['length']:reference=qmul(seg['q'],core.axis_q(seg['axis'],distance));break
   distance-=seg['length'];j+=1
  hold_support=not self.released and self.required_spin>0 and remaining<=self.required_spin+.25
  if hold_support:reference=self.target
  error=core.rot_error(s['quat'],reference);alpha=p['attitude_gain']*error-p['rate_gain']*s['omega'];axis_body=R.T@tangent;along=float(np.dot(alpha,axis_body))
  if toward>.2 and stop>brake_distance:along=min(along,-brake)
  alpha+=axis_body*(float(np.clip(along,-brake,accel))-np.dot(alpha,axis_body))
  self.turn_complete=bool(remaining<.08 and np.linalg.norm(target_error)<math.radians(12) and tilt<math.radians(12) and rate<.65 and self.spin_satisfied)
  settle=self.turn_complete and contact.get('wheelCount',0)>=3
  force_z=s['mass']*gravity;target_vz=None;mode='support';forecast=None;budget=v3.landing_gas_budget(s,cfg)
  if boost:
   target_vz=min(12.,max(2.,(floor-s['pos'][2]+.5*gravity*needed*needed)/max(needed,.1)+.5));force_z=s['mass']*(gravity+p['vertical_gain']*(target_vz-vz));mode='build_airtime'
  elif remaining<.6 and tilt<math.radians(30):
   net=caps['vertical']/s['mass']-gravity;cap=min(9.,max(p['descent_speed'],gap/max(.3,.65*budget['equivalentHoverTime'])));available=max(0.,gap-p['ground_margin']-max(0.,-vz)*2*delay);decel=min(p['descent_accel'],max(.1,.5*net));target_vz=-min(cap,math.sqrt(2*decel*available)+p['touchdown_speed']);ff=decel if available>.02 and -cap<target_vz<-.2 and vz<-.2 else 0.;force_z=s['mass']*(gravity+p['vertical_gain']*(target_vz-vz)+ff);mode='landing'
   if tilt<math.radians(8) and rate<.4 and gap>1.5:
    forecast=v3.predict_vertical_stop(s,cfg,fraction=p['prediction_fraction'],touchdown_speed=p['touchdown_speed']);margin=.05+max(0.,-vz)*dt
    force_z=0. if forecast['feasible'] and gap>forecast['distance']+margin else p['prediction_fraction']*caps['vertical'];mode='predicted_coast' if force_z==0 else 'predicted_brake'
  elif self.released and coast>=needed:force_z=0.;mode='coast'
  if settle:force_z=0.;alpha*=.2;mode='settle'
  torque=J*alpha+np.cross(s['omega'],J*s['omega']);self.elapsed+=dt;self.peak_height=max(self.peak_height,float(s['pos'][2]));self.goal={'phase':mode,'holdingSupportPose':hold_support,'targetQuat':self.target.tolist(),'referenceQuat':reference.tolist(),'pathRemaining':remaining,'pathProgress':self.completed_arc+self.progress,'pathLength':self.total_arc,'requiredSpin':self.required_spin,'spinMeasured':self.spin_measured,'spinSatisfied':self.spin_satisfied,'turnAngle':self.turn_angle,'turnComplete':self.turn_complete,'targetVerticalSpeed':target_vz,'commandAngularRate':command_rate,'angularStoppingAngle':stop,'distanceToNextBrake':brake_distance,'availableCoastTime':coast,'requiredRecoveryTime':needed,'pressureBudget':budget,'verticalStoppingForecast':forecast,'horizontalForceTarget':[0.,0.],'horizontalVelocityError':dv,'airborneHorizontalChange':self.airborne_delta,'measuredStateFeedback':True,'controllerFamily':'unified_goal_path_PD'};self.error={'tilt':tilt,'angularSpeed':rate,'remaining':remaining,'horizontalVelocityError':dv}
  return np.array([0.,0.,max(0.,force_z)]),torque,.095 if settle else 0.

class World(v3.RecoveryWorld):pass

def rollout(parameters,case,cfg,record=False,world=None):
 original=core.FeedbackPolicy;core.FeedbackPolicy=UnifiedPolicy;w=world or World(cfg);holder={}
 class Tracked(UnifiedPolicy):
  def __init__(self,*a):super().__init__(*a);holder['policy']=self
 core.FeedbackPolicy=Tracked
 try:r=core.rollout('moving_recovery',parameters,{**case,'task':'moving_recovery'},cfg,record=record,policy_id='unified',world=w)
 finally:core.FeedbackPolicy=original
 policy=holder['policy'];m=r['metrics'];m.update(airborneHorizontalChange=policy.airborne_delta,firstContactHorizontalChange=policy.first_contact_delta,finalHorizontalChange=float(np.linalg.norm(w.state()['vel'][:2]-np.array(case['vel'][:2]))),pathRemaining=policy.goal.get('pathRemaining',policy.total_arc),requiredSpin=policy.required_spin,measuredSpin=policy.spin_measured)
 # Score only physical terminal state, actual progress and horizontal disturbance; no case label is read.
 m['unifiedScore']=500.*r['success']+120.*(1-policy.goal.get('pathRemaining',policy.total_arc)/max(policy.total_arc,.1))-100.*m['bodyContact']-80.*m['hardContact']-12.*min(m['maxContactClosingSpeed'],15)-.1*m['fuelUsed']-2*m['duration']-70.*min(policy.airborne_delta,10)-25.*min(m['finalHorizontalChange'],10)
 return r
