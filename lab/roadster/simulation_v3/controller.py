"""Engineered recovery repair; unchanged MuJoCo world and hardware. Not learned weights."""
from pathlib import Path
import sys, math, copy
import numpy as np
HERE=Path(__file__).resolve().parent
sys.dont_write_bytecode=True
sys.path.insert(0,str(HERE/'baseline_r6'))
import core
sys.path.insert(0,str(HERE/'allocator'))
from conservative_allocator import allocate as matched_allocate
from landing_predictor import predict_vertical_stop

def local_capacity(s,cfg,axis):
    A=core.wrench_matrix(core.nozzle_layout(cfg));pressure,_=core.delivery(s['fuel'],cfg);limits=np.array([n['maxForce'] for n in core.nozzle_layout(cfg)])*pressure*s['thrustScale'];R=s['R'];J=core.unit_inertia(cfg)*s['mass']
    for i,n in enumerate(core.nozzle_layout(cfg)):
        if n['id'] in s['disabledNozzles']:limits[i]=0.
    body_axis=R.T@axis;result={}
    for name,sign in [('accelerate',1.),('brake',-1.)]:
        target=sign*J*body_axis*1e6;force=core.allocate_fixed_nozzles(np.r_[np.zeros(3),target],limits,cfg);alpha=R@((A@force)[3:]/J);result[name]=max(0.,float(np.dot(alpha,sign*axis)))
    vertical=A@core.allocate_fixed_nozzles(np.r_[R.T@np.array([0.,0.,1e6]),np.zeros(3)],limits,cfg);result['vertical']=max(0.,float((R@vertical[:3])[2]));vertical_forces=core.allocate_fixed_nozzles(np.r_[R.T@np.array([0.,0.,1e6]),np.zeros(3)],limits,cfg);result['verticalFlowPerLift']=float(vertical_forces.sum())/max(result['vertical'],1e-9);return result

def rotation_time(angle,speed,cap,accel,brake):
    if accel<1e-6 or brake<1e-6:return 99.
    delay=0.
    if speed<0:delay=-speed/accel;angle+=speed*speed/(2*accel);speed=0.
    stopping=speed*speed/(2*brake)
    if stopping>angle+1e-12:
        return delay+speed/brake+rotation_time(stopping-angle,0.,cap,accel,brake)
    if speed>cap:
        delay+=(speed-cap)/brake;angle-=(speed*speed-cap*cap)/(2*brake);speed=cap
    aa=max(0.,(cap*cap-speed*speed)/(2*accel));bb=cap*cap/(2*brake)
    if aa+bb<=angle:return delay+(cap-speed)/accel+(angle-aa-bb)/max(cap,.01)+cap/brake
    peak=math.sqrt(max(0.,(2*angle*accel*brake+brake*speed*speed)/(accel+brake)))
    return delay+max(0.,peak-speed)/accel+peak/brake

def landing_gas_budget(s,cfg):
    # Gas mass at which full upright lift has only 25% force margin over weight.
    dry=s['mass']-s['fuel'];lo=0.;hi=cfg['fuel']['initial_kg'];ns=core.nozzle_layout(cfg);limits=np.array([0. if n['id'] in s['disabledNozzles'] else n['maxForce']*s['thrustScale'] for n in ns]);force0=float((core.wrench_matrix(ns)@core.allocate_fixed_nozzles([0,0,1e6,0,0,0],limits,cfg))[2]);gravity=cfg['physics']['gravity']
    for _ in range(32):
        mid=(lo+hi)/2;pressure,_=core.delivery(mid,cfg)
        if force0*pressure>=1.25*(dry+mid)*gravity:hi=mid
        else:lo=mid
    _,isp=core.delivery(s['fuel'],cfg);reachable=force0>=1.25*(dry+cfg['fuel']['initial_kg'])*gravity;usable=max(0.,s['fuel']-hi) if reachable else 0.;hover_time=usable*isp/max(s['mass'],1.)
    return {'forceMarginReachable':bool(reachable),'uprightForceAtCurrentPressure':float(force0*core.delivery(s['fuel'],cfg)[0]),'uprightLiftWeightRatio':float(force0*core.delivery(s['fuel'],cfg)[0]/(s['mass']*gravity)),'minimumFuelForUprightForceMargin':hi if reachable else None,'availableGasAboveMargin':usable,'equivalentHoverTime':hover_time,'marginWeightRatio':1.25,'notice':'Approximate mass/pressure budget; not a guarantee of attitude-dependent landing authority.'}

SETTINGS={'rate_cap':4.,'capacity_fraction':.55,'acceleration_cap':15.,'brake_cap':12.,'early_rate':.15,'landing_reserve':.25,'boost_speed_cap':10.,'support_ratio':1.4,'descent_speed':4.5,'maximum_descent_speed':8.,'descent_acceleration_cap':4.,'vertical_gain':5.5,'touchdown_speed':.12,'ground_margin':.2,'heading_slew':.6,'gyro_feedforward':True,'prediction_overlay':True}

class RecoveryPolicy(core.FeedbackPolicy):
    settings=SETTINGS
    def __init__(self,parameters,task,case,cfg):
        super().__init__(parameters,task,case,cfg);self.c=copy.deepcopy(type(self).settings);self.released=False;self.heading_goal=self.yaw
    def command(self,s,contact):
        if self.task!='moving_recovery':return super().command(s,contact)
        c=self.c;R=s['R'];J=core.unit_inertia(self.cfg)*s['mass'];tilt=math.acos(np.clip(R[2,2],-1,1));rate=float(np.linalg.norm(s['omega']));dt=self.cfg['physics']['control_dt'];gravity=self.cfg['physics']['gravity']
        # Keep the original heading through recovery. Only then slew toward a rolling axis.
        if self.turn_complete and np.linalg.norm(s['vel'][:2])>.5:
            heading=math.atan2(s['vel'][1],s['vel'][0]);choices=[heading,core_wrap(heading+math.pi)];target=min(choices,key=lambda a:abs(core_wrap(a-self.heading_goal)));self.heading_goal+=float(np.clip(core_wrap(target-self.heading_goal),-c['heading_slew']*dt,c['heading_slew']*dt))
        self.goal_q=core.axis_q([0,0,1],self.heading_goal);super().command(s,contact);error=core.rot_error(s['quat'],self.goal_q);angle=float(np.linalg.norm(error));axis_body=error/max(angle,1e-12);axis_world=R@axis_body;toward=float(np.dot(s['omega'],axis_body));caps=local_capacity(s,self.cfg,axis_world if angle>1e-8 else R[:,0]);accel=min(c['acceleration_cap'],c['capacity_fraction']*caps['accelerate']);brake=min(c['brake_cap'],c['capacity_fraction']*caps['brake']);delay=dt+self.cfg['actuators']['lag_seconds'];stop_angle=max(0.,toward)**2/(2*max(brake,.01))+max(0.,toward)*delay;turn_time=rotation_time(angle,toward,c['rate_cap'],accel,brake)
        floor=max(core.geometry_minimum_cg(core.qmat(core.axis_q(axis_world,angle*f))@R,self.cfg) for f in np.linspace(0,1,9))+.1 if angle>1e-8 else core.geometry_minimum_cg(R,self.cfg)+.1;z=float(s['pos'][2]);vz=float(s['vel'][2]);disc=vz*vz+2*gravity*(z-floor);coast=max(0.,(vz+math.sqrt(disc))/gravity) if disc>=0 else 0.;needed=turn_time+c['landing_reserve'];gap=z-core.geometry_minimum_cg(R,self.cfg)
        if coast>=needed:self.released=True
        boost=not self.released and angle>.6 and caps['vertical']>c['support_ratio']*s['mass']*gravity
        # This is the old PD written as a bounded rate reference; the damping gain is unchanged.
        command_rate=min(c['rate_cap'],self.p['recovery_attitude_gain']/self.p['attitude_rate_gain']*angle,math.sqrt(2*max(brake,0.)*angle))
        if boost:command_rate=min(command_rate,c['early_rate'])
        desired_omega=axis_body*command_rate;alpha=self.p['attitude_rate_gain']*(desired_omega-s['omega']);along=float(np.dot(alpha,axis_body));guard=toward>.2 and stop_angle>angle
        if guard:along=min(along,-brake)
        along=float(np.clip(along,-brake,accel));alpha+=axis_body*(along-float(np.dot(alpha,axis_body)))
        budget=landing_gas_budget(s,self.cfg);net=caps['vertical']/s['mass']-gravity;target_vz=None;mode='support_during_recovery';force_z=s['mass']*gravity;forecast=None
        if boost:
            target_vz=min(c['boost_speed_cap'],max(2.,(floor-z+.5*gravity*needed*needed)/max(needed,.1)+.5));force_z=s['mass']*(gravity+c['vertical_gain']*(target_vz-vz));mode='build_recovery_time'
        elif tilt<math.radians(30):
            cap=min(c['maximum_descent_speed'],max(c['descent_speed'],gap/max(.3,.65*budget['equivalentHoverTime'])));reserve=c['ground_margin']+max(0.,-vz)*2*delay;decel=min(c['descent_acceleration_cap'],max(.1,.5*net));available=max(0.,gap-reserve);target_vz=-min(cap,math.sqrt(2*decel*available)+c['touchdown_speed']);ff=decel if available>.02 and -cap<target_vz<-.2 and vz<-.2 else 0.;force_z=s['mass']*(gravity+c['vertical_gain']*(target_vz-vz)+ff);mode='002_descent_law'
            if c['prediction_overlay'] and tilt<math.radians(8) and rate<.4 and gap>1.5:
                forecast=predict_vertical_stop(s,self.cfg,fraction=.85,touchdown_speed=c['touchdown_speed']);margin=.05+max(0.,-vz)*dt
                if forecast['feasible'] and gap>forecast['distance']+margin:force_z=0.;mode='calibrated_prediction_coast'
                else:force_z=.85*caps['vertical'];mode='calibrated_prediction_brake'
        elif self.released and coast>=needed:force_z=0.;mode='coast_during_recovery'
        if self.phase=='settle':force_z=0.;alpha*=.2;mode='settle'
        P=J*self.p['recovery_attitude_gain']*error;D=-J*self.p['attitude_rate_gain']*s['omega'];gyro=np.cross(s['omega'],J*s['omega']) if c['gyro_feedforward'] else np.zeros(3);torque=J*alpha+gyro;correction=torque-P-D-gyro;force=np.array([0.,0.,max(0.,force_z)])
        self.goal.update(feedbackStructure='full_quaternion_PD_with_rate_envelope',PDtermsMeaning='P and D are original torque references; governorCorrection is the bounded-rate/braking modification.',PDGainUnits={'P':'Nm/rad','D':'Nm*s/rad'},proportionalTorqueGains=(J*self.p['recovery_attitude_gain']).tolist(),dampingTorqueGains=(J*self.p['attitude_rate_gain']).tolist(),PDProportionalTorque=P.tolist(),PDDampingTorque=D.tolist(),PDIntegralTorque=[0.,0.,0.],gyroscopicTorque=gyro.tolist(),gyroFeedforward=bool(c['gyro_feedforward']),governorCorrectionTorque=correction.tolist(),attitudeTorqueRequest=torque.tolist(),outerBodyRateReference=desired_omega.tolist(),innerBodyRateError=(desired_omega-s['omega']).tolist(),targetYaw=self.heading_goal,uprightRecovered=bool(self.turn_complete),headingSettled=bool(angle<.08 and rate<.2),horizontalForceTarget=[0.,0.],targetVerticalSpeed=target_vz,verticalMode=mode,recoveryBudget={'attitudeAngle':angle,'angularStoppingAngle':stop_angle,'angularBrakeMargin':angle-stop_angle,'angularBrakeGuard':bool(guard),'availableAngularDeceleration':brake,**budget},recoveryPreview={'estimatedOrientationTime':turn_time,'availableCoastTime':coast,'requiredTime':needed,'pathGeometryFloor':floor,'notice':'Local constant-acceleration orientation budget; not full 6DoF MPC.'},verticalStoppingForecast=forecast)
        return force,torque,self.p['landing_fan'] if self.phase=='settle' else 0.

def core_wrap(x):return (x+math.pi)%(2*math.pi)-math.pi

class RecoveryWorld(core.World):
    def state(self):
        s=super().state();s['actualNozzleThrottle']=np.asarray(getattr(self,'throttle',np.zeros(12))).copy();s['actualNozzleForces']=np.asarray(getattr(self,'forces',np.zeros(12))).copy();return s
    def allocate(self,wrench):
        pressure,_=core.delivery(self.fuel,self.cfg);limits=self.fmax*pressure*self.thrust_scale
        for i,n in enumerate(self.nozzles):
            if n['id'] in self.disabled:limits[i]=0.
        wrench=np.asarray(wrench);parts=[np.r_[np.zeros(3),wrench[3:]],np.r_[wrench[:3],np.zeros(3)],np.zeros(6)];cap=self.cfg['actuators']['total_force_max']*pressure*self.thrust_scale;result=matched_allocate(parts,limits,cap,self.A);forces=np.asarray(result['forces']);ach=self.A@forces
        return np.divide(forces,limits,out=np.zeros(12),where=limits>0),{'requestedWrenchBody':wrench.tolist(),'achievedWrenchBody':ach.tolist(),'residual':(wrench-ach).tolist(),'normalizedResidual':float(np.linalg.norm(wrench-ach)/max(np.linalg.norm(wrench),1.)),'momentScale':result['tiltScale'],'forceScale':result['forceScale'],'reservedZeroScale':result['yawScale'],'allocator':'conservative_analytic_priority','globallyOptimal':False,'feasibilityCalls':result['feasibilityCalls'],'solverIterations':result['iterations'],'priority':'full_recovery_moment_then_vertical_force; heading target slews only after upright recovery'}
