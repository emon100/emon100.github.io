"""Batched rigid-body teaching model. No measured Tesla parameters or pose interpolation."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent

def load_config(path=None):
    config = json.loads(Path(path or HERE / 'config.json').read_text())
    def check_finite(value):
        if isinstance(value,dict):
            for child in value.values():check_finite(child)
        elif isinstance(value,list):
            for child in value:check_finite(child)
        elif isinstance(value,(int,float)) and not np.isfinite(value):raise ValueError('Configuration must contain finite numbers')
    check_finite(config)
    if config['dt'] <= 0 or config['max_time'] <= 0 or config['vehicle']['mass'] <= config['vehicle']['gas_mass']:
        raise ValueError('dt, horizon and dry mass must be positive')
    for key in ['length','width','height','gas_mass','specific_impulse']:
        if config['vehicle'][key]<=0:raise ValueError(f'{key} must be positive')
    return config

def normalize(q):
    return q / np.maximum(np.linalg.norm(q, axis=-1, keepdims=True), 1e-12)

def mul(a, b):
    a, b = np.broadcast_arrays(a, b)
    w = a[..., :1] * b[..., :1] - np.sum(a[..., 1:] * b[..., 1:], axis=-1, keepdims=True)
    xyz = a[..., :1] * b[..., 1:] + b[..., :1] * a[..., 1:] + np.cross(a[..., 1:], b[..., 1:])
    return np.concatenate((w, xyz), axis=-1)

def conj(q):
    return q * np.array([1., -1., -1., -1.])

def axis_quat(axis, angle):
    angle = np.asarray(angle)
    return np.concatenate((np.cos(angle / 2)[..., None], axis * np.sin(angle / 2)[..., None]), axis=-1)

def matrix(q):
    w, x, y, z = np.moveaxis(normalize(q), -1, 0)
    return np.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), axis=-1).reshape(q.shape[:-1]+(3, 3))

def rotate(R, v):
    return np.einsum('...ij,...j->...i', R, v)

def inverse_rotate(R, v):
    return np.einsum('...ji,...j->...i', R, v)

def inertia(mass, cfg):
    v = cfg['vehicle']; L, W, H = v['length'], v['width'], v['height']
    return np.asarray(mass)[..., None] * np.array([W*W+H*H, L*L+H*H, L*L+W*W]) / 12

def dimensions(cfg):
    v = cfg['vehicle']; return np.array([v['length'], v['width'], v['height']]) / 2

def delivery_limits(gas,cfg):
    delivery=cfg.get('gas_delivery',{'mode':'fixed_optimistic'})
    fraction=np.clip(np.asarray(gas)/cfg['vehicle']['gas_mass'],0,1)
    if delivery['mode']=='regulated_then_blowdown':ratio=np.minimum(1,fraction/delivery['regulation_cutoff_fraction'])
    elif delivery['mode']=='unregulated_blowdown':ratio=fraction
    else:ratio=np.ones_like(fraction)
    exponent=delivery.get('pressure_exponent',1.)
    return ratio**exponent,cfg['vehicle']['specific_impulse']*ratio**((exponent-1)/2)

def allocate(force, torque, gas, cfg, dt, authority_scale=None):
    """Six-axis relaxed allocator, with shared positive jet-force and gas limits."""
    a, v = cfg['actuator'], cfg['vehicle']
    authority_scale = np.ones_like(gas) if authority_scale is None else np.asarray(authority_scale)
    pressure_scale,effective_isp=delivery_limits(gas,cfg)
    authority_scale=authority_scale*pressure_scale
    force_limit=np.array(a['force_body_max'])*authority_scale[...,None]
    torque_limit=np.array(a['torque_body_max'])*authority_scale[...,None]
    force = np.clip(force, -force_limit, force_limit)
    torque = np.clip(torque, -torque_limit, torque_limit)
    cost = np.abs(force).sum(axis=-1) + (np.abs(torque) / np.array(a['couple_lever_arm'])).sum(axis=-1)
    available = np.minimum(a['total_jet_force_max']*authority_scale, gas * effective_isp * cfg['gravity'] / dt)
    scale = np.minimum(1., available / np.maximum(cost, 1e-9))
    return force*scale[..., None], torque*scale[..., None], cost*scale

def decompose_wrench(force, torque, cfg):
    values=[]
    for nozzle in cfg['actuator']['virtual_nozzles']:
        axis,sign=nozzle['axis'],nozzle['sign']
        value=np.maximum(0,(force if nozzle['kind']=='translation' else torque)[...,axis]*sign)
        if nozzle['kind']=='couple':value=value/(2*cfg['actuator']['couple_lever_arm'][axis])
        values.append(value)
    return np.stack(values,axis=-1)

def integrate(pos, vel, quat, omega, mass, force_world, torque_body, cfg, dt):
    """Semi-implicit angular velocity and midpoint rotation; world translation is ballistic per step."""
    I = inertia(mass, cfg)
    acceleration = force_world / mass[..., None] + np.array([0., 0., -cfg['gravity']])
    new_pos = pos + vel*dt + .5*acceleration*dt*dt
    new_vel = vel + acceleration*dt
    alpha = (torque_body - np.cross(omega, I*omega)) / I
    new_omega = omega + alpha*dt
    mid = .5*(omega+new_omega)
    angle = np.linalg.norm(mid, axis=-1)*dt
    axis = mid / np.maximum(np.linalg.norm(mid, axis=-1, keepdims=True), 1e-12)
    return new_pos, new_vel, normalize(mul(quat, axis_quat(axis, angle))), new_omega

def fan_forces(pos, quat, command, cfg, ceiling_height=None):
    """Pressure work and exhaust kinetic power share one electrical-power budget."""
    f = cfg['fan']; n = len(pos); R = matrix(quat)
    power = np.clip(command, 0., 1.) * f['electrical_power_w'] * f['efficiency']
    belly = pos - R[:, :, 2] * cfg['vehicle']['height']/2
    normals = np.zeros((n, 3)); normals[:, 2] = 1.
    gap = belly[:, 2].copy()
    if ceiling_height is not None:
        ceil_gap = ceiling_height - belly[:, 2]
        use = (ceil_gap >= -.03) & (ceil_gap < gap) & (np.abs(pos[:, 0]) < cfg['scenarios']['inverted_release']['ramp_half_size'][0]) & (np.abs(pos[:, 1]) < cfg['scenarios']['inverted_release']['ramp_half_size'][1])
        gap = np.where(use, ceil_gap, gap); normals[use, 2] = -1.
    alignment = np.sum(R[:, :, 2] * normals, axis=-1)
    near = (gap >= -.03) & (gap < f['gap_limit']) & (alignment >= f['alignment_cosine'])
    leak = f['seal_leak_area'] + f['seal_perimeter'] * np.maximum(gap, 0.)
    q_coefficient = f['discharge_coefficient'] * leak * np.sqrt(2/f['air_density'])
    # With Q = C sqrt(dp), pressure work + exhaust kinetic power = K dp^(3/2).
    coefficient = q_coefficient * (1 + f['discharge_coefficient']**2 * leak**2 / f['exit_area']**2)
    pressure_power_cap = (power / np.maximum(coefficient, 1e-12)) ** (2/3)
    pressure = np.where(near, np.minimum(f['pressure_limit_pa']*np.exp(-np.maximum(gap, 0.)/f['gap_decay']), pressure_power_cap), 0.)
    flow = q_coefficient * np.sqrt(pressure)
    pressure_force = -normals * (pressure*f['effective_floor_area'])[:, None]
    ideal_free_thrust = (power * np.sqrt(2*f['air_density']*f['disk_area'])) ** (2/3)
    exit_thrust = np.minimum(f['air_density']*flow**2/f['exit_area'], ideal_free_thrust)
    axial = np.where(near, exit_thrust, ideal_free_thrust)
    tilt = np.deg2rad(f['exhaust_up_tilt_deg'])
    axis = np.array([np.cos(tilt), 0., -np.sin(tilt)])
    axial_force=rotate(R, axial[:, None]*axis)
    force = pressure_force + axial_force
    used_power = pressure*flow + .5*f['air_density']*flow**3/f['exit_area']**2
    return force, pressure, np.where(near, used_power, power), {'suctionForce':pressure_force,'axialForce':axial_force,'pressurePower':pressure*flow,'axialPower':np.where(near,.5*f['air_density']*flow**3/f['exit_area']**2,power),'availableFluidPower':power}

def contact_point_speeds(vel,quat,omega,cfg):
    half=dimensions(cfg)
    points=np.array([[x*half[0],y*half[1],-half[2]] for x in [-1,1] for y in [-1,1]])
    local=np.cross(omega[:,None,:],points[None,:,:])
    velocities=vel[:,None,:]+np.einsum('nij,nkj->nki',matrix(quat),local)
    return np.max(np.abs(velocities[:,:,2]),axis=1),np.max(np.linalg.norm(velocities[:,:,:2],axis=-1),axis=1)

def contact_metrics(pos, vel, quat, omega, target, cfg):
    R = matrix(quat); extent = np.einsum('nij,j->ni', np.abs(R), dimensions(cfg))
    clearance = pos[:, 2] - extent[:, 2]
    tilt = np.arccos(np.clip(R[:, 2, 2], -1, 1))
    l = cfg['landing']
    point_vertical,point_horizontal=contact_point_speeds(vel,quat,omega,cfg)
    safe = (point_vertical<=l['max_vertical_speed']) & (point_horizontal<=l['max_horizontal_speed']) & ((np.abs(vel[:, 2]) <= l['max_vertical_speed']) &
            (np.linalg.norm(vel[:, :2], axis=-1) <= l['max_horizontal_speed']) &
            (np.linalg.norm(omega, axis=-1) <= l['max_angular_speed']) &
            (tilt <= np.deg2rad(l['max_tilt_deg'])) &
            (np.linalg.norm(pos[:, :2]-target[:, :2], axis=-1) <= l['position_tolerance']))
    return clearance, tilt, safe, extent

def quintic(u):
    clipped = np.clip(u, 0., 1.)
    s = clipped**3*(10-15*clipped+6*clipped**2)
    ds = 30*clipped**2*(1-clipped)**2
    dds = 60*clipped*(1-clipped)*(1-2*clipped)
    active = (u > 0) & (u < 1)
    return s, np.where(active, ds, 0.), np.where(active, dds, 0.)

PARAMETERS = ['duration','apex','kp_xy','kd_xy','kp_z','kd_z','kp_att','kd_att','rotation_start','rotation_duration','fan_air']
LOWER = np.array([2.4, 5., 1., 1., 2., 1., 3., 1., .02, .35, 0.])
UPPER = np.array([5.5, 16., 10., 9., 14., 10., 24., 12., .25, .85, 1.])
INITIAL = np.array([4., 9., 4., 4., 6., 5., 12., 6., .12, .6, 0.])

def policy_dict(policy):
    return dict(zip(PARAMETERS, map(float, policy)))

def cases(task, count, cfg, seed, heldout=False, grid=None):
    rng = np.random.default_rng(seed); scenario = cfg['scenarios'][task]
    masses = rng.uniform(*cfg['vehicle']['mass_range'], count)
    quat = np.zeros((count, 4)); quat[:, 0] = 1.
    height = np.full(count, scenario['initial_height'])
    if task == 'inverted_release':
        quat[:, 0] = 0.; quat[:, 1] = 1.
        height = rng.uniform(*scenario['evaluation_height_range' if heldout else 'training_height_range'],count)
    elif task == 'attitude_recovery':
        quat = normalize(rng.normal(size=(count, 4)))
        height = rng.uniform(*scenario['evaluation_height_range' if heldout else 'training_height_range'],count)
    if grid is not None:
        height = np.asarray(grid['height'], dtype=float); quat = np.asarray(grid['quat'], dtype=float); masses = np.asarray(grid['mass'], dtype=float)
    return {'mass':masses,'height':height,'quat':quat,'seed':seed}

def rollout(task, policies, inputs, cfg, record=False):
    policies = np.atleast_2d(policies).astype(float)
    if policies.shape[1] != len(PARAMETERS) or not np.isfinite(policies).all(): raise ValueError('Invalid policy')
    P, K = len(policies), len(inputs['mass']); n = P*K
    par = np.repeat(policies, K, axis=0)
    p0 = np.zeros((n, 3)); p0[:, 2] = np.tile(inputs['height'], P)
    pos=p0.copy(); vel=np.zeros_like(pos); quat=np.tile(inputs['quat'],(P,1)); initial_q=quat.copy(); omega=np.zeros_like(pos)
    gas=np.full(n,cfg['vehicle']['gas_mass']); dry=np.tile(inputs['mass'],P)-gas
    target=np.zeros_like(pos); target[:,0]=np.tile(inputs.get('distance',np.full(K,cfg['scenarios'][task]['distance'])),P); target[:,2]=cfg['vehicle']['height']/2
    authority_scale=np.tile(inputs.get('thrust_scale',np.ones(K)),P)
    row_scale=np.tile(inputs.get('row_scale',np.ones(K)),P)
    initial_R=matrix(initial_q)
    heading=np.arctan2(initial_R[:,1,0],initial_R[:,0,0]) if task=='attitude_recovery' else np.zeros(n)
    goal_q=axis_quat(np.tile([0.,0.,1.],(n,1)),heading)
    delta=mul(conj(initial_q),goal_q);delta=np.where((delta[:,:1]<0),-delta,delta)
    turn_angle=2*np.arccos(np.clip(delta[:,0],-1,1));turn_axis=delta[:,1:]/np.maximum(np.linalg.norm(delta[:,1:],axis=-1,keepdims=True),1e-12)
    done=np.zeros(n,dtype=bool); success=np.zeros(n,dtype=bool); status=np.full(n,'timeout',dtype=object)
    initial_extent=np.einsum('nij,j->ni',np.abs(initial_R),dimensions(cfg))
    invalid_initial=(pos[:,2]-initial_extent[:,2])<-.001
    done|=invalid_initial;status=np.where(invalid_initial,'invalid_initial_clearance',status)
    roll_angle=np.zeros(n);world_roll=np.zeros(n);world_phase=np.arctan2(-initial_R[:,1,2],initial_R[:,2,2]);roll_path_aligned=np.ones(n,dtype=bool); peak_height=pos[:,2].copy(); airborne=np.zeros(n,dtype=bool)
    force_body=np.zeros_like(pos); torque=np.zeros_like(pos); frames=[]; elapsed=np.zeros(n)
    dt=cfg['dt']; release_time=.24 if task=='inverted_release' else 0.
    ceiling=np.tile(inputs['height'],P)+cfg['vehicle']['height']/2 if task=='inverted_release' else None
    total_steps=int(cfg['max_time']/dt)
    fan=np.zeros_like(pos);pressure=np.zeros(n);fan_power=np.zeros(n);thrust=np.zeros_like(pos);jet_cost=np.zeros(n)
    fan_detail={'suctionForce':np.zeros_like(pos),'axialForce':np.zeros_like(pos),'pressurePower':np.zeros(n),'axialPower':np.zeros(n),'availableFluidPower':np.zeros(n)}
    pressure_scale,effective_isp=delivery_limits(gas,cfg);actuator_quat=quat[0].copy();t=0.
    def snapshot(sample_time,label,actuation_quat):
        return {'t':round(sample_time,5),'pos':pos[0].tolist(),'quat':quat[0].tolist(),'vel':vel[0].tolist(),'omega':omega[0].tolist(),'gasLeft':float(gas[0]),'mass':float((dry+gas)[0]),'fanForce':fan[0].tolist(),'fanPressure':float(pressure[0]),'fanPower':float(fan_power[0]),'thrust':thrust[0].tolist(),'torque':torque[0].tolist(),'allocation':{'forceBody':force_body[0].tolist(),'torqueBody':torque[0].tolist(),'totalPositiveJetForce':float(jet_cost[0]),'gasFlow':float(jet_cost[0]/(max(effective_isp[0],1e-12)*cfg['gravity'])),'pressureScale':float(pressure_scale[0]),'effectiveIsp':float(effective_isp[0])},'nozzleForces':decompose_wrench(force_body[0],torque[0],cfg).tolist(),'fanSuctionForce':fan_detail['suctionForce'][0].tolist(),'fanAxialForce':fan_detail['axialForce'][0].tolist(),'fanPressurePower':float(fan_detail['pressurePower'][0]),'fanAxialPower':float(fan_detail['axialPower'][0]),'fanAvailablePower':float(fan_detail['availableFluidPower'][0]),'actuatorQuat':actuation_quat.tolist(),'forceSampleTime':round(t,5),'rollAngle':float(world_roll[0]),'bodyRollIntegral':float(roll_angle[0]),'rollPathAligned':bool(roll_path_aligned[0]),'contactMaxVerticalSpeed':float(contact_point_speeds(vel[:1],quat[:1],omega[:1],cfg)[0][0]),'contactMaxHorizontalSpeed':float(contact_point_speeds(vel[:1],quat[:1],omega[:1],cfg)[1][0]),'status':label}
    if record and n!=1:raise ValueError('Recorded rollout requires one policy and one initial state')
    if record and done[0]:frames.append(snapshot(0.,str(status[0]),quat[0]))
    for step in range(total_steps):
        t=step*dt; active=~done
        if not active.any(): break
        mass=dry+gas; R=matrix(quat)
        pressure_scale,effective_isp=delivery_limits(gas,cfg)
        duration=par[:,0]; local=np.maximum(0,t-release_time)
        u=local/duration; s,ds,dds=quintic(u)
        reference=p0+(target-p0)*s[:,None]
        reference_vel=(target-p0)*(ds/duration)[:,None]
        reference_acc=(target-p0)*(dds/duration**2)[:,None]
        if task=='flat_kickflip':
            bounded=np.clip(u,0,1); h=par[:,1]
            reference[:,2]=target[:,2]+h*.5*(1-np.cos(2*np.pi*bounded))
            reference_vel[:,2]=np.where(u<1,h*np.pi/duration*np.sin(2*np.pi*bounded),0)
            reference_acc[:,2]=np.where(u<1,2*h*np.pi**2/duration**2*np.cos(2*np.pi*bounded),0)
        rotation_u=(local/duration-par[:,8])/par[:,9]
        rs,rds,rdds=quintic(rotation_u); rotation_time=duration*par[:,9]
        if task=='flat_kickflip':
            axis=np.tile([1.,0.,0.],(n,1)); angle=np.full(n,2*np.pi)
            reference_q=axis_quat(axis,angle*rs)
        else:
            axis=turn_axis;angle=turn_angle;reference_q=mul(initial_q,axis_quat(axis,angle*rs))
        reference_rate=axis*(angle*rds/rotation_time)[:,None]
        reference_alpha=axis*(angle*rdds/rotation_time**2)[:,None]
        error=mul(conj(quat),reference_q); error=np.where(error[:,:1]<0,-error,error)
        kp=np.stack([par[:,2],par[:,2],par[:,4]],axis=-1);kd=np.stack([par[:,3],par[:,3],par[:,5]],axis=-1)
        desired_force=mass[:,None]*(reference_acc+kp*(reference-pos)+kd*(reference_vel-vel)+[0,0,cfg['gravity']])
        rate_body=inverse_rotate(R,rotate(matrix(reference_q),reference_rate))
        alpha_body=inverse_rotate(R,rotate(matrix(reference_q),reference_alpha))
        desired_alpha=alpha_body+par[:,6,None]*2*error[:,1:]+par[:,7,None]*(rate_body-omega)
        I=inertia(mass,cfg);desired_torque=I*desired_alpha+np.cross(omega,I*omega)
        fan_command=par[:,10].copy()
        attached=(t<release_time) & (task=='inverted_release')
        if attached:
            fan_command[:]=1.;desired_force[:]=0.;desired_torque[:]=0.
        fan,pressure,fan_power,fan_detail=fan_forces(pos,quat,fan_command,cfg,ceiling)
        command_body=inverse_rotate(R,desired_force-fan)
        if attached: command_body[:]=0.
        response=1-np.exp(-dt/cfg['actuator']['response_seconds'])
        force_body+=(command_body-force_body)*response;torque+=(desired_torque-torque)*response
        force_body,torque,jet_cost=allocate(force_body,torque,gas,cfg,dt,authority_scale)
        thrust=rotate(R,force_body)
        if record:
            frames.append(snapshot(t,'airborne' if airborne[0] else 'supported',quat[0]))
        actuator_quat=quat[0].copy()
        new=integrate(pos,vel,quat,omega,mass,thrust+fan,torque,cfg,dt)
        finite=np.logical_and.reduce([np.isfinite(a).all(axis=-1) for a in new])
        diverged=active&~finite;status=np.where(diverged,'diverged',status);done|=diverged;active&=finite
        pos,vel,quat,omega=[np.where(active[:,None],a,b) for a,b in zip(new,(pos,vel,quat,omega))]
        gas=np.maximum(0,gas-np.where(active,jet_cost*dt/(np.maximum(effective_isp,1e-12)*cfg['gravity']),0))
        elapsed=np.where(active,t+dt,elapsed);roll_angle+=np.where(active,omega[:,0]*dt,0);peak_height=np.maximum(peak_height,pos[:,2])
        actual_R=matrix(quat)
        phase=np.arctan2(-actual_R[:,1,2],actual_R[:,2,2])
        phase_delta=np.arctan2(np.sin(phase-world_phase),np.cos(phase-world_phase))
        world_roll+=np.where(active,phase_delta,0);world_phase=phase
        roll_path_aligned&=(actual_R[:,0,0]>=np.cos(np.deg2rad(15)))
        clearance,tilt,safe,extent=contact_metrics(pos,vel,quat,omega,target,cfg)
        airborne|=clearance>.12
        if attached:
            top=pos[:,2]+extent[:,2]
            normal_load=np.maximum(0,(thrust+fan)[:,2]-mass*cfg['gravity'])
            contact=(top>=ceiling-.002)&(pressure>0)&(normal_load>0)
            pos[:,2]-=np.where(contact,np.maximum(top-ceiling,0),0)
            vel[:,2]=np.where(contact,np.minimum(vel[:,2],0),vel[:,2])
            speed=np.linalg.norm(vel[:,:2],axis=-1)
            friction_dv=np.minimum(speed,cfg['fan']['surface_friction_coefficient']*normal_load*dt/mass)*contact
            correction=vel[:,:2]/np.maximum(speed[:,None],1e-12)*friction_dv[:,None]
            vel[:,:2]-=correction;pos[:,:2]-=.5*correction*dt
        obstacle_hit=np.zeros(n,dtype=bool)
        for obstacle in cfg['scenarios'][task]['obstacles']:
            center=np.tile(np.array(obstacle['center']),(n,1));half=np.array(obstacle['size'])/2; yaw=obstacle['heading']
            if cfg['scenarios'][task].get('row_centered_on_path'):
                center[:,0]=target[:,0]/2+(obstacle['center'][0]-cfg['scenarios'][task]['distance']/2)*row_scale
            obstacle_extent=np.array([abs(np.cos(yaw))*half[0]+abs(np.sin(yaw))*half[1],abs(np.sin(yaw))*half[0]+abs(np.cos(yaw))*half[1],half[2]])
            obstacle_hit|=(np.abs(pos-center)<(extent+obstacle_extent)).all(axis=-1)
        ceiling_hit=np.zeros(n,dtype=bool)
        if ceiling is not None and not attached:
            half=np.array(cfg['scenarios'][task]['ramp_half_size'])
            ceiling_hit=(pos[:,2]+extent[:,2]>ceiling+.025)&(np.abs(pos[:,:2])<(half+extent[:,:2])).all(axis=-1)
        ground=(clearance<=cfg['landing']['ground_tolerance']) & ((t>.25)|airborne)
        completed=(world_roll>=2*np.pi-.15)&roll_path_aligned&(peak_height>=3) if task=='flat_kickflip' else np.ones(n,dtype=bool)
        landing=ground&safe&completed&~obstacle_hit&~ceiling_hit
        failed=ground&~landing
        unstable=(np.linalg.norm(omega,axis=-1)>35)|(np.linalg.norm(vel,axis=-1)>100)|(np.abs(pos)>300).any(axis=-1)
        fresh=active&(ground|obstacle_hit|ceiling_hit|unstable)
        success|=active&landing
        status=np.where(active&landing,'landed',status)
        status=np.where(active&failed,'unsafe_ground_contact',status)
        status=np.where(active&obstacle_hit,'obstacle_collision',status)
        status=np.where(active&ceiling_hit,'ramp_collision',status)
        status=np.where(active&unstable,'state_limit',status)
        done|=fresh
        # Initial support is unilateral; it never teleports an airborne body back onto the ground.
        support=active&~airborne&(t<=.25)&(clearance<0)&(task=='flat_kickflip')
        pos[support,2]-=clearance[support];vel[support,2]=np.maximum(vel[support,2],0)
        if record and done[0]:
            frames.append(snapshot(t+dt,str(status[0]),actuator_quat))
            if n==1:break
    if record and not done[0]:frames.append(snapshot(elapsed[0],str(status[0]),actuator_quat))
    clearance,tilt,safe,extent=contact_metrics(pos,vel,quat,omega,target,cfg)
    distance=np.linalg.norm(pos[:,:2]-target[:,:2],axis=-1)
    speed=np.linalg.norm(vel,axis=-1);angular=np.linalg.norm(omega,axis=-1)
    score=200-10*distance-12*speed-55*tilt-12*angular-5*np.abs(clearance)-.08*(cfg['vehicle']['gas_mass']-gas)
    score+=600*success-220*np.isin(status,['obstacle_collision','ramp_collision','state_limit','diverged'])-100*(gas<=.01)
    if task=='flat_kickflip':score+=80*np.clip(world_roll/(2*np.pi),0,1)-100*(peak_height<3)
    if not np.isfinite(score).all():raise FloatingPointError('Non-finite simulator result')
    environment={'obstacles':[],'targetDistance':float(target[0,0]),'ceilingHeight':None if ceiling is None else float(ceiling[0]),'landingTarget':target[0].tolist(),'thrustScale':float(authority_scale[0]),'rowScale':float(row_scale[0]),'vehicleSize':(dimensions(cfg)*2).tolist(),'rampHalfSize':cfg['scenarios'].get('inverted_release',{}).get('ramp_half_size') if ceiling is not None else None,'mirrored':cfg['scenarios'][task].get('mirrored',False)}
    for obstacle in cfg['scenarios'][task]['obstacles']:
        item=dict(obstacle);item['center']=list(item['center'])
        if cfg['scenarios'][task].get('row_centered_on_path'):item['center'][0]=target[0,0]/2+(item['center'][0]-cfg['scenarios'][task]['distance']/2)*row_scale[0]
        environment['obstacles'].append(item)
    return {'score':score.reshape(P,K),'success':success.reshape(P,K),'status':status.reshape(P,K),'gasLeft':gas.reshape(P,K),'elapsed':elapsed.reshape(P,K),'rollAngle':world_roll.reshape(P,K),'bodyRollIntegral':roll_angle.reshape(P,K),'rollPathAligned':roll_path_aligned.reshape(P,K),'frames':frames,'environment':environment,'final':{'pos':pos.tolist(),'vel':vel.tolist(),'quat':quat.tolist(),'omega':omega.tolist()}}
