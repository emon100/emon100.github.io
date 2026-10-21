"""Contact-physics control experiment. All poses come from MuJoCo; no pose trajectories."""
from __future__ import annotations
import copy
import json
import math
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import linprog

HERE = Path(__file__).resolve().parent
TASKS = ('stationary_kickflip','moving_kickflip','moving_recovery')
PARAMETERS = ['peak_clearance','flip_start_clearance','climb_speed','height_gain','vertical_speed_gain','roll_speed','roll_brake_accel','roll_rate_gain','axis_align_gain','recovery_attitude_gain','attitude_rate_gain','descent_speed','descent_brake_accel','landing_fan','early_roll_speed']
LOWER = np.array([5.5,.02,8.,.5,1.5,5.,4.,3.,2.,3.,2.,1.0,1.0,0.,.1])
UPPER = np.array([7.5,.15,12.,3.5,9.,9.,25.,16.,18.,18.,12.,4.5,7.,.8,.6])
# Frozen before the first rollout: generic, unoptimized engineering guesses.
UNTUNED = np.array([6.5,.05,10.,1.,2.5,6.5,6.,4.,5.,6.,4.,1.5,2.,0.,.3])
# Explicit manual feedback baseline. It is never presented as learned.
HANDCRAFTED = np.array([6.5,.05,10.,1.8,5.,7.,15.,10.,9.,10.,6.,4.5,4.,.25,.3])


def finite(value):
    if isinstance(value,dict):
        for v in value.values():finite(v)
    elif isinstance(value,(list,tuple)):
        for v in value:finite(v)
    elif isinstance(value,(int,float,np.number)) and not np.isfinite(value):
        raise ValueError('Non-finite data')


def write_json(path,value):
    finite(value)
    Path(path).write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')


def load_config(path=None):
    cfg=json.loads(Path(path or HERE/'config.json').read_text());finite(cfg)
    if cfg['physics']['dt']<=0 or cfg['physics']['control_dt']<cfg['physics']['dt']:raise ValueError('Invalid timestep')
    if cfg['vehicle']['mass']<=cfg['fuel']['initial_kg']:raise ValueError('Invalid dry mass')
    for group,keys in {'physics':['dt','control_dt','max_time','gravity'],'vehicle':['mass','length','width','height','axle_half','track_half','ground_cg_height'],'fuel':['initial_kg','specific_impulse_s','pressure_exponent'],'actuators':['vertical_max_force','horizontal_max_force','total_force_max','lag_seconds','vertical_arm_x','vertical_arm_y','vertical_nozzle_z'],'fan':['electrical_power_w','efficiency','disk_area','exit_area','air_density','effective_floor_area','pressure_limit_pa','gap_limit','gap_decay','seal_leak_area','seal_perimeter','discharge_coefficient']}.items():
        for key in keys:
            if cfg[group][key]<=0:raise ValueError(f'{group}.{key} must be positive')
    ratio=cfg['physics']['control_dt']/cfg['physics']['dt']
    if abs(ratio-round(ratio))>1e-9:raise ValueError('Control timestep must be an integer multiple of physics timestep')
    if any(r<=0 for r in cfg['vehicle']['wheel_radii']) or any(a<=0 for b in cfg['vehicle']['collision_boxes'] for a in b['half_size']):raise ValueError('Collision dimensions must be positive')
    if cfg['fuel']['delivery_mode']!='unregulated_blowdown':raise ValueError('Only full-course unregulated blowdown is supported')
    mass_properties(cfg,cfg['vehicle']['mass']-cfg['fuel']['initial_kg'])
    return cfg


def qmul(a,b):
    return np.r_[a[0]*b[0]-np.dot(a[1:],b[1:]),a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:],b[1:])]


def qconj(q):return q*np.array([1.,-1.,-1.,-1.])
def axis_q(axis,angle):return np.r_[math.cos(angle/2),np.asarray(axis)*math.sin(angle/2)]
def euler_q(roll,pitch,yaw):return qmul(qmul(axis_q([0,0,1],yaw),axis_q([0,1,0],pitch)),axis_q([1,0,0],roll))
def qmat(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def rot_error(q,target):
    e=qmul(qconj(q),target)
    if e[0]<0:e=-e
    n=np.linalg.norm(e[1:]);return e[1:]*(2*math.atan2(n,max(e[0],0.))/max(n,1e-12))


def nozzle_layout(cfg):
    v=cfg['vehicle'];a=cfg['actuators'];out=[]
    for x in [-a['vertical_arm_x'],a['vertical_arm_x']]:
        for y in [-a['vertical_arm_y'],a['vertical_arm_y']]:
            for sign in [1,-1]:
                out.append({'id':f'J{len(out)+1:02d}','position':[x,y,a['nozzle_center_z']-a['vertical_nozzle_z']*sign],'direction':[0.,0.,float(sign)],'maxForce':a['vertical_max_force'],'role':'vertical_up' if sign>0 else 'vertical_down','evidence':'hypothetical_control_configuration'})
    for p,d in [([-v['length']/2-.03,-a['vertical_arm_y'],a['horizontal_nozzle_z']],[1,0,0]),([v['length']/2+.03,a['vertical_arm_y'],a['horizontal_nozzle_z']],[-1,0,0]),([-a['vertical_arm_x'],-v['width']/2-.025,a['horizontal_nozzle_z']],[0,1,0]),([a['vertical_arm_x'],v['width']/2+.025,a['horizontal_nozzle_z']],[0,-1,0])]:
        out.append({'id':f'J{len(out)+1:02d}','position':p,'direction':d,'maxForce':a['horizontal_max_force'],'role':'horizontal_coupled','evidence':'hypothetical_control_configuration'})
    return out


def wrench_matrix(nozzles):return np.array([np.r_[n['direction'],np.cross(n['position'],n['direction'])] for n in nozzles]).T


def controllability(cfg):
    nozzles=nozzle_layout(cfg);A=wrench_matrix(nozzles)
    positive=linprog(np.ones(len(nozzles)),A_eq=A,b_eq=np.zeros(6),bounds=[(1,10)]*len(nozzles),method='highs')
    tests=[]
    for axis in range(6):
        for sign in [-1,1]:
            target=np.eye(6)[axis]*sign*10000
            r=linprog(np.ones(len(nozzles)),A_eq=A,b_eq=target,A_ub=np.ones((1,len(nozzles))),b_ub=[cfg['actuators']['total_force_max']],bounds=[(0,n['maxForce']) for n in nozzles],method='highs')
            tests.append({'axis':axis,'sign':sign,'target':target.tolist(),'feasible':bool(r.success),'forces':r.x.tolist() if r.success else None})
    return {'rank':int(np.linalg.matrix_rank(A)),'positiveNullExists':bool(positive.success),'positiveNullAllocation':positive.x.tolist() if positive.success else None,'axisFeasibility':tests,'wrenchMatrix':A.tolist(),'notice':'局部满秩和正锥性质只证明该假设布局的控制方向；不证明任意初态、有限气量或 Tesla 实车可达。'}


def unit_inertia(cfg):
    L,W,H=[cfg['vehicle'][k] for k in ['length','width','height']]
    return np.array([W*W+H*H,L*L+H*H,L*L+W*W])/12


def wheel_inertia_offset(cfg):
    v=cfg['vehicle'];m=20.;r=np.array(v['wheel_radii']);own=m/5*(np.sum(r*r)-r*r)
    return sum((own+m*(np.dot(p,p)-p*p) for p in (np.array([x,y,v['wheel_z']]) for x in [-v['axle_half'],v['axle_half']] for y in [-v['track_half'],v['track_half']])),start=np.zeros(3))


def mass_properties(cfg,mass):
    """Whole-assembly COM is the root; fuel is assumed centred at that COM."""
    body_mass=float(mass)-80.
    if body_mass<=0:raise ValueError('Dry body mass must be positive')
    body_ipos=np.array([0.,0.,-80.*cfg['vehicle']['wheel_z']/body_mass])
    total_inertia=unit_inertia(cfg)*mass
    body_inertia=total_inertia-wheel_inertia_offset(cfg)-body_mass*(np.dot(body_ipos,body_ipos)-body_ipos*body_ipos)
    if np.any(body_inertia<=0) or np.max(body_inertia)>=np.sum(body_inertia)-np.max(body_inertia):raise ValueError('Body inertia must be positive and physically valid')
    return {'bodyMass':body_mass,'bodyIpos':body_ipos,'bodyInertia':body_inertia,'totalInertia':total_inertia}


def model_xml(cfg):
    v=cfg['vehicle'];p=cfg['physics'];mass=v['mass'];mp=mass_properties(cfg,mass);J=mp['bodyInertia']
    sites=''.join(f'<site name="{n["id"]}" pos="{" ".join(map(str,n["position"]))}" size=".025"/>' for n in nozzle_layout(cfg))
    motors=''.join(f'<general name="{n["id"]}" site="{n["id"]}" gear="{" ".join(map(str,n["direction"]))} 0 0 0" ctrllimited="true" ctrlrange="0 {n["maxForce"]}"/>' for n in nozzle_layout(cfg))
    wheels=''.join(f'<body name="wheelbody{i}" pos="{x} {y} {v["wheel_z"]}"><joint name="wheelspin{i}" type="hinge" axis="0 1 0" damping=".25"/><geom name="wheel{i}" type="ellipsoid" size="{" ".join(map(str,v["wheel_radii"]))}" mass="20" rgba=".1 .1 .1 1"/></body>' for i,(x,y) in enumerate(( (x,y) for x in [-v['axle_half'],v['axle_half']] for y in [-v['track_half'],v['track_half']] )))
    boxes=''.join(f'<geom name="{b["name"]}" type="box" pos="{" ".join(map(str,b["position"]))}" size="{" ".join(map(str,b["half_size"]))}"/>' for b in v['collision_boxes'])
    return f'''<mujoco model="hypothetical_12_jet_contact_experiment"><compiler angle="radian"/><option timestep="{p['dt']}" gravity="0 0 -{p['gravity']}" integrator="implicitfast" cone="elliptic" iterations="60" tolerance="1e-10"/><default><geom condim="{p['contact_dimension']}" friction="{' '.join(map(str,p['friction']))}" solref="{' '.join(map(str,p['contact_solref']))}" solimp=".95 .99 .001" margin=".001"/></default><worldbody><geom name="floor" type="plane" size="100 100 .1" rgba=".2 .25 .3 1"/><body name="car" pos="0 0 {v['ground_cg_height']}"><freejoint name="free"/><inertial pos="{' '.join(map(str,mp['bodyIpos']))}" mass="{mp['bodyMass']}" diaginertia="{' '.join(map(str,J))}"/>{boxes}{wheels}{sites}</body></worldbody><actuator>{motors}</actuator></mujoco>'''


def delivery(fuel,cfg,mode=None):
    f=cfg['fuel'];fraction=np.clip(fuel/f['initial_kg'],0.,1.)
    if (mode or f['delivery_mode'])!='unregulated_blowdown':raise ValueError('Only unregulated blowdown is supported')
    ratio=fraction
    pressure=ratio**f['pressure_exponent'];isp=f['specific_impulse_s']*ratio**((f['pressure_exponent']-1)/2)
    return pressure,max(isp,1e-9)


def fan_model(pos,R,command,cfg):
    f=cfg['fan'];power=np.clip(command,0,1)*f['electrical_power_w']*f['efficiency']
    gap=pos[2]+f['seal_body_z']*R[2,2];near=(-.01<=gap<f['gap_limit']) and R[2,2]>=f['alignment_cosine']
    leak=f['seal_leak_area']+f['seal_perimeter']*max(gap,0)
    coefficient=f['discharge_coefficient']*leak*math.sqrt(2/f['air_density'])
    cap=(power/max(coefficient*(1+f['discharge_coefficient']**2*leak**2/f['exit_area']**2),1e-12))**(2/3)
    pressure=min(f['pressure_limit_pa']*math.exp(-max(gap,0)/f['gap_decay']),cap) if near else 0.
    flow=coefficient*math.sqrt(pressure)
    suction=np.array([0.,0.,-pressure*f['effective_floor_area']])
    free_thrust=(power*math.sqrt(2*f['air_density']*f['disk_area']))**(2/3)
    axial=min(f['air_density']*flow*flow/f['exit_area'],free_thrust) if near else free_thrust
    angle=math.radians(f['exhaust_up_tilt_deg']);axis=np.array([math.cos(angle),0.,-math.sin(angle)])
    axial_force=R@(axial*axis)
    pressure_power=pressure*flow;axial_power=.5*f['air_density']*flow**3/f['exit_area']**2 if near else power
    torque=np.cross(R@np.asarray(f['position']),axial_force)
    return {'suctionForce':suction,'axialForce':axial_force,'torqueWorld':torque,'pressure':pressure,'gap':gap,'power':pressure_power+axial_power,'pressurePower':pressure_power,'axialPower':axial_power,'availableFluidPower':float(power)}


def allocate_fixed_nozzles(wrench,limits,cfg):
    """One common wrench scale preserves its force direction under saturation."""
    Fx,Fy,Fz,tx,ty,tz=np.asarray(wrench);v=cfg['actuators'];l=v['vertical_arm_x'];w=v['vertical_arm_y']
    tx+=v['horizontal_nozzle_z']*Fy;ty-=v['horizontal_nozzle_z']*Fx
    signs=np.array([1.,-1.,-1.,1.])
    base=np.array([Fz/4+tx*y/(4*w*w)-ty*x/(4*l*l) for x in [-l,l] for y in [-w,w]])
    low=-np.asarray(limits)[1:8:2];high=np.asarray(limits)[:8:2]
    def interval(scale):
        aa=(low-scale*base)/signs;bb=(high-scale*base)/signs
        return max(np.minimum(aa,bb)),min(np.maximum(aa,bb))
    lo,hi=interval(1.);scale=1.
    if lo>hi+1e-9:
        left,right=0.,1.
        for _ in range(24):
            mid=(left+right)/2;a,b=interval(mid)
            if a<=b+1e-9:left=mid
            else:right=mid
        scale=left;lo,hi=interval(scale)
    horizontal=np.array([max(Fx,0),max(-Fx,0),max(Fy,0),max(-Fy,0)],dtype=float)
    delta=tz-(w*abs(Fx)-l*abs(Fy))
    if delta>=0:horizontal[:2]+=delta/(2*w)
    else:horizontal[2:]+=-delta/(2*l)
    active=horizontal>1e-9
    hscale=min(1.,float(np.min(np.asarray(limits)[8:][active]/horizontal[active]))) if active.any() else 1.
    scale=min(scale,max(0,hscale));lo,hi=interval(scale)
    alpha=float(np.clip(np.median(-scale*base/signs),lo,hi));net=scale*base+alpha*signs
    result=np.zeros(12);result[:8:2]=np.maximum(net,0);result[1:8:2]=np.maximum(-net,0);result[8:]=horizontal*scale
    cap=cfg['actuators']['total_force_max']*max(np.divide(limits,np.array([n['maxForce'] for n in nozzle_layout(cfg)])))
    if result.sum()>cap:result*=cap/max(result.sum(),1e-12)
    return np.clip(result,0,limits)


class World:
    def __init__(self,cfg):
        self.cfg=cfg;self.model=mujoco.MjModel.from_xml_string(model_xml(cfg));self.data=mujoco.MjData(self.model)
        self.nozzles=nozzle_layout(cfg);self.A=wrench_matrix(self.nozzles);self.fmax=np.array([n['maxForce'] for n in self.nozzles]);self.body=1
        self.wheel_ids={mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,f'wheel{i}') for i in range(4)}
        self.wheel_index={mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,f'wheel{i}'):i for i in range(4)}
        v=cfg['vehicle'];self.wheel_positions=[np.array([x,y,v['wheel_z']]) for x in [-v['axle_half'],v['axle_half']] for y in [-v['track_half'],v['track_half']]]
        self.body_ids={mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,n) for n in ['body','roof']}
        self.tmp=np.zeros(6);self.contact_force=np.zeros(6)
        self.state_spec=mujoco.mjtState.mjSTATE_INTEGRATION;self.saved_state=np.zeros(mujoco.mj_stateSize(self.model,self.state_spec))
    def reset(self,case):
        validate_case(case,self.cfg)
        mujoco.mj_resetData(self.model,self.data)
        self.case=copy.deepcopy(case);self.fuel=float(case.get('fuel',self.cfg['fuel']['initial_kg']));self.initial_fuel=self.fuel
        self.initial_mass=float(case.get('mass',self.cfg['vehicle']['mass']));self.dry_mass=self.initial_mass-self.fuel
        self.update_mass();self.data.qpos[:3]=case['pos'];self.data.qpos[3:7]=case['quat'];self.data.qvel[:3]=case['vel'];self.data.qvel[3:6]=case.get('omega',[0.,0.,0.]);self.data.qvel[6:10]=(qmat(case['quat']).T@np.array(case['vel']))[0]/self.cfg['vehicle']['wheel_radii'][0]
        self.throttle=np.zeros(12);self.forces=np.zeros(12);self.fuel_used=0.;self.pressure_scale,self.effective_isp=delivery(self.fuel,self.cfg);self.fan=fan_model(np.array(case['pos']),qmat(case['quat']),0,self.cfg)
        self.disabled=set(case.get('disabled_nozzles',[]));self.thrust_scale=float(case.get('thrust_scale',1.));self.mode=case.get('delivery_mode')
        self.contact={'count':0,'wheelCount':0,'bodyContact':False,'maxNormalForce':0.,'totalNormalForce':0.,'maxClosingSpeed':0.,'stableSeconds':0.}
        self.peak_closing=0.;self.peak_contact_force=0.;self.any_body_contact=False;self.stable_seconds=0.;self.last_stable=False;self.hard_contact=False;self.jet_horizontal_impulse=np.zeros(2);self.fan_horizontal_impulse=np.zeros(2);self.jet_force_world=np.zeros(3);self.peak_jet_horizontal_force=0.
        mujoco.mj_forward(self.model,self.data)
    def update_mass(self):
        mass=self.dry_mass+self.fuel;mp=mass_properties(self.cfg,mass);self.model.body_mass[self.body]=mp['bodyMass'];self.model.body_ipos[self.body]=mp['bodyIpos'];self.model.body_inertia[self.body]=mp['bodyInertia']
        # mj_setConst uses Data as scratch and resets qpos; preserve every integration state.
        mujoco.mj_getState(self.model,self.data,self.saved_state,self.state_spec)
        mujoco.mj_setConst(self.model,self.data)
        mujoco.mj_setState(self.model,self.data,self.saved_state,self.state_spec)
    def state(self):
        d=self.data;R=d.xmat[self.body].reshape(3,3).copy()
        mujoco.mj_objectVelocity(self.model,d,mujoco.mjtObj.mjOBJ_XBODY,self.body,self.tmp,0)
        return {'pos':d.qpos[:3].copy(),'quat':d.qpos[3:7].copy(),'vel':self.tmp[3:].copy(),'omega':R.T@self.tmp[:3],'omegaWorld':self.tmp[:3].copy(),'R':R,'mass':self.dry_mass+self.fuel,'wheelRates':d.qvel[6:10].copy(),'fuel':float(self.fuel),'thrustScale':self.thrust_scale,'disabledNozzles':list(self.disabled)}
    def allocate(self,wrench):
        pressure,_=delivery(self.fuel,self.cfg,self.mode)
        limits=self.fmax*pressure*self.thrust_scale
        for i,n in enumerate(self.nozzles):
            if n['id'] in self.disabled:limits[i]=0.
        forces=allocate_fixed_nozzles(wrench,limits,self.cfg)
        mass=self.dry_mass+self.fuel;scale=np.r_[np.ones(3)/(mass*self.cfg['physics']['gravity']),np.ones(3)/(unit_inertia(self.cfg)*mass*8.)]
        achieved=self.A@forces
        return np.divide(forces,limits,out=np.zeros(12),where=limits>0),{'requestedWrenchBody':wrench.tolist(),'achievedWrenchBody':achieved.tolist(),'residual':(wrench-achieved).tolist(),'normalizedResidual':float(np.linalg.norm((wrench-achieved)*scale))}
    def advance(self,command,fan_command,stability_context=None):
        dt=self.cfg['physics']['dt'];substeps=round(self.cfg['physics']['control_dt']/dt);response=1-math.exp(-dt/self.cfg['actuators']['lag_seconds'])
        self.update_mass();mujoco.mj_forward(self.model,self.data)
        maximum_closing=0.;maximum_force=0.;maximum_total=0.;body_contact=False;wheel_count=0;minimum_wheels=4;maximum_slip=0.
        for _ in range(substeps):
            s=self.state();self.throttle+=(np.clip(command,0,1)-self.throttle)*response
            pressure,isp=delivery(self.fuel,self.cfg,self.mode);limits=self.fmax*pressure*self.thrust_scale
            for i,n in enumerate(self.nozzles):
                if n['id'] in self.disabled:limits[i]=0.
            forces=self.throttle*limits;cap=min(self.cfg['actuators']['total_force_max']*pressure*self.thrust_scale,self.fuel*isp*self.cfg['physics']['gravity']/dt)
            total=float(forces.sum())
            if total>cap:forces*=cap/max(total,1e-12)
            self.data.ctrl[:]=forces;self.forces=forces.copy();self.pressure_scale=pressure;self.effective_isp=isp
            consumed=float(forces.sum())*dt/(isp*self.cfg['physics']['gravity']);self.fuel=max(0.,self.fuel-consumed);self.fuel_used+=consumed
            self.fan=fan_model(s['pos'],s['R'],fan_command,self.cfg)
            self.data.xfrc_applied[:]=0.;self.data.xfrc_applied[self.body,:3]=self.fan['suctionForce']+self.fan['axialForce'];self.data.xfrc_applied[self.body,3:]=self.fan['torqueWorld']-np.cross(s['R']@self.model.body_ipos[self.body],self.data.xfrc_applied[self.body,:3])
            self.jet_force_world=s['R']@(self.A[:3]@forces);self.peak_jet_horizontal_force=max(self.peak_jet_horizontal_force,float(np.linalg.norm(self.jet_force_world[:2])));self.jet_horizontal_impulse+=self.jet_force_world[:2]*dt;self.fan_horizontal_impulse+=self.fan['axialForce'][:2]*dt
            mujoco.mj_step(self.model,self.data)
            touched=set();total_force=0.;step_slip=0.
            for i in range(self.data.ncon):
                con=self.data.contact[i];geoms={int(con.geom1),int(con.geom2)}
                if 0 not in geoms:continue
                other=next(g for g in geoms if g!=0)
                mujoco.mj_contactForce(self.model,self.data,i,self.contact_force)
                normal=max(0.,float(self.contact_force[0]));total_force+=normal;maximum_force=max(maximum_force,normal)
                point_vel=s['vel']+np.cross(s['omegaWorld'],np.asarray(con.pos)-s['pos'])
                closing=max(0.,-float(point_vel[2]));maximum_closing=max(maximum_closing,closing)
                if other in self.wheel_ids and normal>1:
                    touched.add(other);wi=self.wheel_index[other];center=s['pos']+s['R']@self.wheel_positions[wi]
                    wheel_velocity=point_vel+np.cross(s['R'][:,1]*s['wheelRates'][wi],np.asarray(con.pos)-center)
                    step_slip=max(step_slip,float(np.linalg.norm(wheel_velocity[:2])))
                if other in self.body_ids and normal>1:body_contact=True
            maximum_slip=max(maximum_slip,step_slip)
            wheel_count=len(touched);minimum_wheels=min(minimum_wheels,wheel_count);maximum_total=max(maximum_total,total_force)
            # Contact forces above are those used by this physical integration step.
            # Refresh post-step kinematics; all stability conditions are checked every 2ms.
            mujoco.mj_forward(self.model,self.data);post=self.state();landing=self.cfg['landing']
            self.hard_contact |= maximum_closing>landing['max_contact_closing_speed'] or total_force>post['mass']*self.cfg['physics']['gravity']*landing['max_normal_force_weight_ratio']
            context=stability_context or {};initial_xy=np.array(context.get('initialXY',[0.,0.]))
            states_ok=all(math.acos(np.clip(st['R'][2,2],-1,1))<math.radians(landing['max_tilt_deg']) and abs(st['vel'][2])<landing['max_vertical_speed'] and abs((st['R'].T@st['vel'])[1])<landing['max_lateral_speed'] and np.linalg.norm(st['omega'])<landing['max_angular_speed'] for st in [s,post])
            self.last_stable=bool(wheel_count>=landing['minimum_wheel_contacts'] and step_slip<landing['max_wheel_slip_speed'] and states_ok and context.get('turnComplete',False) and context.get('wasAirborne',False) and not self.hard_contact and not body_contact and not self.any_body_contact)
            self.stable_seconds=self.stable_seconds+dt if self.last_stable else 0.

        mujoco.mj_forward(self.model,self.data)
        self.peak_closing=max(self.peak_closing,maximum_closing);self.peak_contact_force=max(self.peak_contact_force,maximum_total);self.any_body_contact|=body_contact
        self.contact={'count':int(self.data.ncon),'wheelCount':wheel_count,'bodyContact':body_contact,'maxNormalForce':maximum_force,'totalNormalForce':maximum_total,'maxClosingSpeed':maximum_closing,'minimumWheelCount':minimum_wheels,'maxWheelSlipSpeed':maximum_slip,'stableSeconds':self.stable_seconds}
        return self.state()


def geometry_minimum_cg(R,cfg):
    v=cfg['vehicle'];row=R[2]
    heights=[-float(np.dot(row,b['position']))+float(np.dot(np.abs(row),b['half_size'])) for b in v['collision_boxes']]
    radius=math.sqrt(float(np.sum((row*np.asarray(v['wheel_radii']))**2)))
    heights.extend(-float(np.dot(row,[x,y,v['wheel_z']]))+radius for x in [-v['axle_half'],v['axle_half']] for y in [-v['track_half'],v['track_half']])
    return max(heights)


def swept_cg_bound(R,cfg):
    """Conservative enclosure for a full rotation about the present body X axis."""
    v=cfg['vehicle'];rx=float(R[2,0]);yz=math.hypot(float(R[2,1]),float(R[2,2]));heights=[]
    for box in v['collision_boxes']:
        for sx in [-1,1]:
            for sy in [-1,1]:
                for sz in [-1,1]:
                    point=np.array(box['position'])+np.array(box['half_size'])*[sx,sy,sz]
                    heights.append(-rx*point[0]+yz*math.hypot(point[1],point[2]))
    for x in [-v['axle_half'],v['axle_half']]:
        heights.append(-rx*x+yz*(math.hypot(v['track_half'],v['wheel_z'])+max(v['wheel_radii'][1:]))+abs(rx)*v['wheel_radii'][0])
    return float(max(heights))


def launch_planning(s,p,cfg,remaining):
    g=cfg['physics']['gravity'];settings=cfg['launch_energy'];z=float(s['pos'][2]);vz=float(s['vel'][2]);energy=z+max(vz,0.)**2/(2*g)
    pressure,_=delivery(s.get('fuel',cfg['fuel']['initial_kg']),cfg);ns=nozzle_layout(cfg);disabled=set(s.get('disabledNozzles',[]));limits=np.array([0. if n['id'] in disabled else n['maxForce']*pressure*s.get('thrustScale',1.) for n in ns])
    forces=allocate_fixed_nozzles([0,0,0,1e6,0,0],limits,cfg);torque=float(wrench_matrix(ns)[3]@forces);capacity=settings['future_torque_fraction']*torque/max(unit_inertia(cfg)[0]*s['mass'],1e-9)
    accel=min(p['roll_rate_gain']*p['roll_speed'],capacity);brake=min(p['roll_brake_accel'],capacity)
    roll_time=min(99.,remaining/max(p['roll_speed'],.01)+p['roll_speed']/(2*max(accel,.01))+p['roll_speed']/(2*max(brake,.01)))
    floor=swept_cg_bound(s['R'],cfg)+settings['geometry_margin'];disc=vz*vz+2*g*(z-floor);coast=max(0.,(vz+math.sqrt(disc))/g) if disc>=0 else 0.
    return {'energyHeight':energy,'estimatedRollTime':roll_time,'availableCoastTime':coast,'sweptMinimumCG':floor,'rollAccelerationBudget':capacity}


def early_roll_limit(s,requested,p,cfg,accel_budget):
    """Short geometry preview. It limits a request, never sets the actual pose."""
    setting=cfg['launch_energy'];best_gap=-1e6
    for fraction in [1.,.75,.5,.25,0.]:
        command=requested*fraction;accel=float(np.clip(p['roll_rate_gain']*(command-s['omega'][0]),-accel_budget,accel_budget));gap=1e6
        for fraction_t in [1/3,2/3,1.]:
            t=setting['prediction_horizon']*fraction_t;rotation=np.array([s['omega'][0]*t+.5*accel*t*t,s['omega'][1]*t,s['omega'][2]*t]);angle=float(np.linalg.norm(rotation));dR=qmat(axis_q(rotation/angle,angle)) if angle>1e-12 else np.eye(3);R=s['R']@dR
            z=s['pos'][2]+s['vel'][2]*t-.5*cfg['physics']['gravity']*t*t;gap=min(gap,float(z)-geometry_minimum_cg(R,cfg))
        best_gap=gap
        if gap>=setting['geometry_margin']:return command,gap,fraction<1.
    return 0.,best_gap,True


class FeedbackPolicy:
    """State events and feedback. No clock-based pose or position reference."""
    def __init__(self,parameters,task,case,cfg):
        values=np.asarray(parameters,dtype=float)
        if values.shape!=(len(PARAMETERS),) or not np.isfinite(values).all():raise ValueError('Invalid policy parameters')
        self.p=dict(zip(PARAMETERS,values));self.task=task;self.cfg=cfg;self.phase='recover' if task=='moving_recovery' else 'boost';self.initial_xy=np.array(case['pos'][:2])
        R=qmat(case['quat']);self.yaw=math.atan2(R[1,0],R[0,0]);self.base=qmat(axis_q([0,0,1],self.yaw));self.goal_q=axis_q([0,0,1],self.yaw)
        rel=self.base.T@R;self.previous_roll=math.atan2(-rel[1,2],rel[2,2]);self.turn_angle=self.previous_roll;self.initial_roll=self.previous_roll
        self.recovery_height=case['pos'][2];self.initial_height=float(case['pos'][2]);self.peak_height=float(case['pos'][2]);self.energy_ready=False;self.turn_complete=False;self.was_airborne=task=='moving_recovery';self.aligned=True;self.goal={};self.error={}
    def command(self,s,contact):
        p=self.p;R=s['R'];rel=self.base.T@R;phase=math.atan2(-rel[1,2],rel[2,2]);increment=(phase-self.previous_roll+math.pi)%(2*math.pi)-math.pi
        self.turn_angle+=increment;self.previous_roll=phase
        clearance=s['pos'][2]-self.cfg['vehicle']['ground_cg_height'];self.was_airborne|=clearance>.35
        tilt=math.acos(np.clip(R[2,2],-1,1));rate=np.linalg.norm(s['omega'])
        self.aligned &= rel[0,0] > math.cos(math.radians(25.)) if self.task!='moving_recovery' and self.phase=='flip' else True
        if self.phase=='boost' and clearance>=p['flip_start_clearance'] and contact['wheelCount']==0 and not contact.get('bodyContact',False) and s['vel'][2]>0:self.phase='flip'
        remaining=max(0.,2*math.pi-self.turn_angle)
        if self.phase=='flip' and remaining<.14 and tilt<math.radians(12) and rate<.65:
            self.phase='land';self.turn_complete=(self.turn_angle-self.initial_roll>=2*math.pi-.25 and self.aligned)
        if self.phase=='recover' and tilt<math.radians(12) and rate<.65:
            self.phase='land';self.turn_complete=True
        if self.phase=='land' and contact['wheelCount']>=3 and tilt<math.radians(10):self.phase='settle'
        self.peak_height=max(self.peak_height,float(s['pos'][2]));target_apex=self.cfg['vehicle']['ground_cg_height']+p['peak_clearance'];minimum_apex=self.cfg['tasks'][self.task].get('minimum_peak_cg_height',0.)
        planning=launch_planning(s,p,self.cfg,remaining) if self.task!='moving_recovery' else {'energyHeight':float(s['pos'][2]+max(s['vel'][2],0.)**2/(2*self.cfg['physics']['gravity'])),'estimatedRollTime':0.,'availableCoastTime':0.,'sweptMinimumCG':geometry_minimum_cg(R,self.cfg),'rollAccelerationBudget':0.}
        if self.task!='moving_recovery' and not self.energy_ready and planning['energyHeight']>=target_apex and planning['availableCoastTime']>=planning['estimatedRollTime']+self.cfg['launch_energy']['landing_time_reserve']:self.energy_ready=True
        desired_acc=np.zeros(3)
        if self.phase in ['boost','flip']:
            target_vz=min(p['climb_speed'],math.sqrt(2*self.cfg['physics']['gravity']*max(target_apex-self.initial_height,0.)))
        elif self.phase=='recover':
            target_vz=np.clip(p['height_gain']*(self.recovery_height-s['pos'][2]),-2.,p['climb_speed'])
        else:
            target_vz=-min(p['descent_speed'],math.sqrt(2*p['descent_brake_accel']*max(clearance-.10,0))+.06)
        desired_acc[2]=p['vertical_speed_gain']*(target_vz-s['vel'][2])
        if self.phase=='land' and -.15>target_vz>-p['descent_speed'] and clearance>.10:
            desired_acc[2]+=p['descent_brake_accel'] # braking feedforward from remaining height, not time
        desired_force=s['mass']*(desired_acc+np.array([0,0,self.cfg['physics']['gravity']]))
        if self.phase=='settle' or (self.phase in ['boost','flip'] and self.energy_ready):desired_force[:]=0.
        geometry_limited=False;predicted_gap=float(s['pos'][2])-geometry_minimum_cg(R,self.cfg);command_rate=0.;roll_feedforward=0.
        if self.phase=='flip':
            rate_limit=p['roll_speed'] if self.energy_ready else p['early_roll_speed'];command_rate=min(rate_limit,math.sqrt(2*p['roll_brake_accel']*remaining))
            if not self.energy_ready:command_rate,predicted_gap,geometry_limited=early_roll_limit(s,command_rate,p,self.cfg,planning['rollAccelerationBudget'])
            # Align the longitudinal axis while rolling; this does not prescribe roll versus time.
            axis_error=R.T@np.cross(R[:,0],self.base[:,0])
            alpha=np.array([p['roll_rate_gain']*(command_rate-s['omega'][0]),p['axis_align_gain']*axis_error[1]-p['attitude_rate_gain']*s['omega'][1],p['axis_align_gain']*axis_error[2]-p['attitude_rate_gain']*s['omega'][2]])
            if self.energy_ready and 0<command_rate<p['roll_speed'] and s['omega'][0]>0 and self.turn_angle<=2*math.pi:roll_feedforward=-p['roll_brake_accel'];alpha[0]+=roll_feedforward
            if self.turn_angle>2*math.pi:alpha[0]=p['recovery_attitude_gain']*rot_error(s['quat'],self.goal_q)[0]-p['attitude_rate_gain']*s['omega'][0]
        else:alpha=p['recovery_attitude_gain']*rot_error(s['quat'],self.goal_q)-p['attitude_rate_gain']*s['omega']
        if self.phase=='settle':alpha*=.2
        J=unit_inertia(self.cfg)*s['mass'];torque=J*alpha+np.cross(s['omega'],J*s['omega'])
        self.goal={'phase':self.phase,'targetTilt':0.,'turnRemaining':remaining if self.task!='moving_recovery' else tilt,'targetVerticalSpeed':None if self.energy_ready and self.phase=='flip' else float(target_vz),'horizontalControl':'none','turnAngle':float(self.turn_angle),'turnComplete':bool(self.turn_complete),'targetApexCG':float(target_apex),'minimumApexCG':float(minimum_apex),'energyReady':bool(self.energy_ready),'energyHeight':float(planning['energyHeight']),'estimatedRollTime':float(planning['estimatedRollTime']),'availableCoastTime':float(planning['availableCoastTime']),'geometryMinimumCG':geometry_minimum_cg(R,self.cfg),'geometryClearance':float(s['pos'][2])-geometry_minimum_cg(R,self.cfg),'predictedGeometryClearance':float(predicted_gap),'geometryLimited':bool(geometry_limited),'commandRollRate':float(command_rate),'rollBrakeFeedforward':float(roll_feedforward),'launchMode':'coast' if self.energy_ready and self.phase=='flip' else ('build_energy' if self.phase in ['boost','flip'] else self.phase),'minimumHeightMet':bool(self.peak_height>=minimum_apex)}
        self.error={'tilt':float(tilt),'angularSpeed':float(rate),'verticalSpeed':float(s['vel'][2]),'horizontalSpeed':float(np.linalg.norm(s['vel'][:2])),'stationaryDistance':float(np.linalg.norm(s['pos'][:2]-self.initial_xy)),'apexShortfall':float(max(0.,minimum_apex-self.peak_height))}
        return desired_force,torque,p['landing_fan'] if self.phase=='settle' else 0.


def initial_clearance(case,cfg):
    R=qmat(np.asarray(case['quat']));z=case['pos'][2];v=cfg['vehicle']
    lows=[z+(R@np.array(b['position']))[2]-np.dot(np.abs(R[2]),b['half_size']) for b in v['collision_boxes']]
    radius=math.sqrt(np.sum((R[2]*np.array(v['wheel_radii']))**2))
    lows.extend(z+(R@np.array([x,y,v['wheel_z']]))[2]-radius for x in [-v['axle_half'],v['axle_half']] for y in [-v['track_half'],v['track_half']])
    return float(min(lows))


def validate_case(case,cfg):
    finite(case)
    if not 0<=case.get('fuel',cfg['fuel']['initial_kg'])<=cfg['fuel']['initial_kg']:raise ValueError('Fuel must be within tank capacity')
    if case.get('delivery_mode',cfg['fuel']['delivery_mode'])!='unregulated_blowdown':raise ValueError('Only unregulated blowdown is supported')
    if case.get('thrust_scale',1.)<0:raise ValueError('Thrust scale cannot be negative')
    if any(n not in {j['id'] for j in nozzle_layout(cfg)} for n in case.get('disabled_nozzles',[])):raise ValueError('Unknown nozzle id')
    for key,n in [('pos',3),('quat',4),('vel',3),('omega',3)]:
        if len(case.get(key,[]))!=n:raise ValueError(f'{key} has wrong length')
    if abs(np.linalg.norm(case['quat'])-1)>1e-6:raise ValueError('Quaternion must have unit norm')
    if initial_clearance(case,cfg)<-.001:raise ValueError('Initial geometry intersects the floor')
    dry=case.get('mass',cfg['vehicle']['mass'])-case.get('fuel',cfg['fuel']['initial_kg'])
    if dry<=80:raise ValueError('Dry body mass must be positive')
    mass_properties(cfg,dry)


def default_case(task,cfg,seed=0):
    v=cfg['vehicle'];s=cfg['tasks'][task];recovery=task=='moving_recovery';height=9. if recovery else v['ground_cg_height']+.005
    return {'id':f'{task}-demo-{seed}','seed':seed,'task':task,'mass':v['mass'],'fuel':cfg['fuel']['initial_kg'],'thrust_scale':1.,'pos':[0.,0.,height],'quat':euler_q(math.pi if recovery else 0,0,0).tolist(),'vel':s['initial_velocity'].copy(),'omega':[0.,0.,0.]}


def sample_cases(task,count,cfg,seed,free=False):
    rng=np.random.default_rng(seed);out=[];spec=cfg['tasks'][task]
    for i in range(count):
        c=default_case(task,cfg,seed+i);c['id']=f'{task}-{seed}-{i}';c['mass']=float(rng.uniform(*cfg['vehicle']['mass_range']));c['thrust_scale']=float(rng.uniform(.85,1.05))
        if free:
            q=rng.normal(size=4);q/=np.linalg.norm(q);c['quat']=q.tolist();c['pos'][2]=float(rng.uniform(*cfg['evaluation']['free_height_range']));c['vel']=rng.uniform(*cfg['evaluation']['free_speed_range'],size=3).tolist();c['omega']=rng.uniform(*cfg['evaluation']['free_angular_speed_range'],size=3).tolist()
        else:
            yaw=float(rng.uniform(-math.pi,math.pi));roll=math.radians(float(rng.uniform(*spec['training_roll_deg'])));pitch=math.radians(float(rng.uniform(*spec['training_pitch_deg'])));c['quat']=euler_q(roll,pitch,yaw).tolist()
            speed=float(rng.uniform(*spec['training_velocity_x']));vlocal=np.array([speed,rng.uniform(-.6,.6) if task!='stationary_kickflip' else 0.,rng.uniform(-2.,0.) if task=='moving_recovery' else 0.]);c['vel']=(qmat(axis_q([0,0,1],yaw))@vlocal).tolist()
            c['pos'][2]=float(rng.uniform(*spec['training_height']))
            if task!='moving_recovery':
                # Ground starts are exactly upright: roll/pitch noise would interpenetrate the floor.
                c['quat']=axis_q([0,0,1],yaw).tolist();c['pos'][2]=cfg['vehicle']['ground_cg_height']+.005
        if free:c['pos'][2]+=max(0.,.02-initial_clearance(c,cfg))
        c['initialClearance']=initial_clearance(c,cfg)
        out.append(c)
    return out


def rollout(task,parameters,case,cfg,record=False,policy_id='learned',world=None):
    validate_case(case,cfg)
    world=world or World(cfg);world.reset(case);policy=FeedbackPolicy(parameters,task,case,cfg)
    dt=cfg['physics']['control_dt'];stable=0.;success=False;status='timeout';frames=[];total_reward=0.;peak_height=case['pos'][2];hard=False;body=False;turn_best=0.;initial_tilt=math.acos(np.clip(qmat(case['quat'])[2,2],-1,1));last_progress=0.;last_height_progress=0.;failed_at=None
    allocation={'requestedWrenchBody':[0.]*6,'achievedWrenchBody':[0.]*6,'residual':[0.]*6,'normalizedResidual':0.}
    initial_xy=np.array(case['pos'][:2]);final_s=world.state();latest_reward={}
    def frame(s,instant,components):
        f={'t':round(float(world.data.time),6),'controlObservationTime':round(max(0.,float(world.data.time)-dt),6),'pos':s['pos'].tolist(),'quat':s['quat'].tolist(),'vel':s['vel'].tolist(),'omega':s['omega'].tolist(),'mass':float(s['mass']),'geometryMinimumCG':geometry_minimum_cg(s['R'],cfg),'geometryClearance':float(s['pos'][2])-geometry_minimum_cg(s['R'],cfg),'wheelRates':s['wheelRates'].tolist(),'jetForceWorld':world.jet_force_world.tolist(),'jetHorizontalImpulse':world.jet_horizontal_impulse.tolist(),'fanHorizontalImpulse':world.fan_horizontal_impulse.tolist(),'contact':copy.deepcopy(world.contact),'nozzleThrottle':world.throttle.tolist(),'nozzleForces':world.forces.tolist(),'fuel':float(world.fuel),'gasLeft':float(world.fuel),'fuelUsed':float(world.fuel_used),'pressureScale':float(world.pressure_scale),'effectiveIsp':float(world.effective_isp),'fan':{k:v.tolist() if isinstance(v,np.ndarray) else float(v) for k,v in world.fan.items()},'goal':copy.deepcopy(policy.goal),'error':copy.deepcopy(policy.error),'allocation':copy.deepcopy(allocation),'reward':{'instant':float(instant),'total':float(total_reward),'components':components},'status':status}
        finite(f);return f
    for step in range(round(cfg['physics']['max_time']/dt)):
        s=world.state()
        if not all(np.isfinite(s[k]).all() for k in ['pos','quat','vel','omega']):status='diverged';break
        force,torque,fan_command=policy.command(s,world.contact)
        if policy_id=='no_control':command=np.zeros(12);fan_command=0.;allocation={'requestedWrenchBody':[0.]*6,'achievedWrenchBody':[0.]*6,'residual':[0.]*6,'normalizedResidual':0.}
        else:
            fan=fan_model(s['pos'],s['R'],fan_command,cfg);wrench=np.r_[s['R'].T@force,torque-s['R'].T@fan['torqueWorld']];command,allocation=world.allocate(wrench)
        if record and step==0:frames.append(frame(s,0.,{}))
        final_s=world.advance(command,fan_command,{'task':task,'initialXY':initial_xy.tolist(),'turnComplete':bool(policy.turn_complete),'wasAirborne':bool(policy.was_airborne)})
        if failed_at is None:peak_height=max(peak_height,float(final_s['pos'][2]))
        landing=cfg['landing']
        tilt=math.acos(np.clip(final_s['R'][2,2],-1,1));speed=np.linalg.norm(final_s['vel']);rate=np.linalg.norm(final_s['omega']);drift=np.linalg.norm(final_s['pos'][:2]-initial_xy)
        hard |= world.contact['maxClosingSpeed']>landing['max_contact_closing_speed'] or world.contact['totalNormalForce']>(final_s['mass']*cfg['physics']['gravity']*landing['max_normal_force_weight_ratio'])
        body |= world.contact['bodyContact']
        stable=world.stable_seconds;stable_now=world.last_stable
        progress=min(1.,max(0.,(policy.turn_angle-policy.initial_roll)/(2*math.pi))) if task!='moving_recovery' else max(0.,(initial_tilt-tilt)/max(initial_tilt,.1))
        turn_best=max(turn_best,progress)
        minimum_apex=cfg['tasks'][task].get('minimum_peak_cg_height',0.);height_progress=float(np.clip((peak_height-case['pos'][2])/max(minimum_apex-case['pos'][2],.01),0,1)) if minimum_apex>0 else 0.
        components={'heightProgress':120.*(height_progress-last_height_progress),'taskProgress':100.*(progress-last_progress),'fuel':-.03*float(world.forces.sum())*dt/(world.effective_isp*cfg['physics']['gravity']),'angularMotion':-.008*dt*float(rate),'time':-.2*dt,'stableContact':4.*dt if stable_now and policy.turn_complete else 0.}
        instant=sum(components.values());total_reward+=instant;last_progress=progress;last_height_progress=height_progress
        if failed_at is None:
            if body:status='body_contact';components['terminal']=-220.;failed_at=float(world.data.time)
            elif hard:status='hard_contact';components['terminal']=-150.-20.*world.contact['maxClosingSpeed']-min(200.,world.contact['totalNormalForce']/(final_s['mass']*cfg['physics']['gravity']));failed_at=float(world.data.time)
            elif np.linalg.norm(final_s['pos'][:2])>180 or final_s['pos'][2]>40:status='out_of_bounds';components['terminal']=-100.;failed_at=float(world.data.time)
            elif stable+1e-10>=landing['stable_seconds'] and policy.turn_complete and policy.was_airborne:
                if peak_height+1e-9>=minimum_apex:status='success';success=True;components['terminal']=500.
                else:status='height_shortfall';components['terminal']=-100.;failed_at=float(world.data.time)
        if 'terminal' in components:instant+=components['terminal'];total_reward+=components['terminal']
        if record:frames.append(frame(final_s,instant,components))
        if success:break
        if failed_at is not None and (not record or world.data.time-failed_at>=.40):break
        if policy_id=='no_control' and stable>=1.2 and not policy.was_airborne:status='no_takeoff';break
    if status=='timeout':
        if world.fuel<.01:status='fuel_exhausted'
        elif not policy.was_airborne:status='no_takeoff'
        elif not policy.turn_complete:status='turn_incomplete'
        else:status='not_stable'
    if frames:frames[-1]['status']=status
    metrics={'score':float(total_reward),'fuelUsed':float(world.fuel_used),'fuelLeft':float(world.fuel),'duration':float(world.data.time),'turnProgress':float(turn_best),'turnAngle':float(policy.turn_angle-policy.initial_roll),'turnComplete':bool(policy.turn_complete),'peakHeight':float(peak_height),'minimumApexCG':float(cfg['tasks'][task].get('minimum_peak_cg_height',0.)),'minimumHeightMet':bool(peak_height+1e-9>=cfg['tasks'][task].get('minimum_peak_cg_height',0.)),'maxContactClosingSpeed':float(world.peak_closing),'maxTotalContactForce':float(world.peak_contact_force),'stableSeconds':float(stable),'finalTilt':float(math.acos(np.clip(final_s['R'][2,2],-1,1))),'finalSpeed':float(np.linalg.norm(final_s['vel'])),'finalLongitudinalSpeed':float((final_s['R'].T@final_s['vel'])[0]),'finalLateralSpeed':float((final_s['R'].T@final_s['vel'])[1]),'finalVerticalSpeed':float(final_s['vel'][2]),'jetHorizontalImpulse':world.jet_horizontal_impulse.tolist(),'fanHorizontalImpulse':world.fan_horizontal_impulse.tolist(),'peakJetHorizontalForce':float(world.peak_jet_horizontal_force),'finalAngularSpeed':float(np.linalg.norm(final_s['omega'])),'horizontalDisplacement':float(np.linalg.norm(final_s['pos'][:2]-initial_xy)),'bodyContact':bool(body),'hardContact':bool(hard)}
    result={'case':copy.deepcopy(case),'policyId':policy_id,'environment':{'obstacles':[],'floorZ':0.,'vehicleSize':[cfg['vehicle'][k] for k in ['length','width','height']],'nozzles':nozzle_layout(cfg)},'status':status,'success':success,'metrics':metrics,'frames':frames}
    finite(result);return result


def policy_json(parameters,label):
    return {'type':'state_feedback_parameter_policy','label':label,'parameterNames':PARAMETERS,'parameters':np.asarray(parameters).tolist(),'values':dict(zip(PARAMETERS,map(float,parameters))),'phaseClock':'state_events_only','observation':['position','quaternion','linear_velocity','body_angular_velocity','fuel','wheel_contacts','accumulated_longitudinal_turn'],'actions':'12 bounded non-negative nozzle commands plus shared-power fan command','allocation':'exact geometric non-negative allocation for the published 12 fixed jets, common scaling of all six wrench components, shared total force cap, pressure/fuel limit and valve lag'}
