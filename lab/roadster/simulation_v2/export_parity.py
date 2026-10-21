"""Publish pure-controller vectors and independent replay actions for the WASM port."""
import copy
import numpy as np
from core import *

def memory(p):
    return {'task':p.task,'phase':p.phase,'initialXY':p.initial_xy.tolist(),'yaw':float(p.yaw),'base':p.base.tolist(),'goalQuat':p.goal_q.tolist(),'previousRoll':float(p.previous_roll),'turnAngle':float(p.turn_angle),'initialRoll':float(p.initial_roll),'recoveryHeight':float(p.recovery_height),'initialHeight':float(p.initial_height),'peakHeight':float(p.peak_height),'energyReady':bool(p.energy_ready),'turnComplete':bool(p.turn_complete),'wasAirborne':bool(p.was_airborne),'aligned':bool(p.aligned)}

def main():
    cfg=load_config();vectors=[];replays=[]
    for task in TASKS:
        case=default_case(task,cfg,20261017);w=World(cfg);w.reset(case);policy=FeedbackPolicy(HANDCRAFTED,task,case,cfg);actions=[]
        previous_phase=''
        for i in range(340):
            s=w.state();before=memory(policy);force,torque,fc=policy.command(s,w.contact);fan=fan_model(s['pos'],s['R'],fc,cfg);wrench=np.r_[s['R'].T@force,torque-s['R'].T@fan['torqueWorld']];u,allocation=w.allocate(wrench)
            if i%5==0 or policy.phase!=previous_phase:
                state={k:s[k].tolist() if isinstance(s[k],np.ndarray) else float(s[k]) for k in ['pos','quat','vel','omega','R','mass']};state.update(fuel=float(w.fuel),contact=copy.deepcopy(w.contact),thrustScale=w.thrust_scale,disabledNozzles=list(w.disabled),deliveryMode=w.mode)
                vectors.append({'state':state,'memory':before,'params':HANDCRAFTED.tolist(),'expected':{'nozzleThrottle':u.tolist(),'fanCommand':float(fc),'requestedWrenchBody':wrench.tolist(),'allocation':allocation,'goal':copy.deepcopy(policy.goal),'error':copy.deepcopy(policy.error),'memory':memory(policy)}})
            previous_phase=policy.phase;actions.append({'nozzleThrottle':u.tolist(),'fanCommand':float(fc)});w.advance(u,fc,{'task':task,'initialXY':policy.initial_xy.tolist(),'turnComplete':bool(policy.turn_complete),'wasAirborne':bool(policy.was_airborne)})
            if i==49:
                replays.append({'id':f'{task}-one-second','task':task,'case':case,'parameters':HANDCRAFTED.tolist(),'controlSteps':50,'actions':copy.deepcopy(actions),'expected':{'time':float(w.data.time),'qpos':w.data.qpos.tolist(),'qvel':w.data.qvel.tolist(),'fuel':float(w.fuel),'nozzleForces':w.forces.tolist(),'contact':copy.deepcopy(w.contact)}})
            if task=='stationary_kickflip' and i==309:
                replays.append({'id':'stationary-after-landing','task':task,'case':case,'parameters':HANDCRAFTED.tolist(),'controlSteps':310,'actions':copy.deepcopy(actions),'expected':{'time':float(w.data.time),'qpos':w.data.qpos.tolist(),'qvel':w.data.qvel.tolist(),'fuel':float(w.fuel),'nozzleForces':w.forces.tolist(),'contact':copy.deepcopy(w.contact)}})
    # Synthetic observations cover the launch boundary, independent of successful flights.
    case=default_case('stationary_kickflip',cfg);w=World(cfg);w.reset(case)
    for wheel_count,body_contact,vz,height in [(4,False,1.,.06),(0,True,1.,.06),(0,False,-1.,.06),(0,False,0.,.06),(0,False,1.,.06),(0,False,1.,.019)]:
        s=w.state();s['pos'][2]=cfg['vehicle']['ground_cg_height']+height;s['vel'][2]=vz;contact=copy.deepcopy(w.contact);contact.update(wheelCount=wheel_count,bodyContact=body_contact);policy=FeedbackPolicy(UNTUNED,case['task'],case,cfg);before=memory(policy);force,torque,fc=policy.command(s,contact);fan=fan_model(s['pos'],s['R'],fc,cfg);wrench=np.r_[s['R'].T@force,torque-s['R'].T@fan['torqueWorld']];u,allocation=w.allocate(wrench)
        state={k:s[k].tolist() if isinstance(s[k],np.ndarray) else float(s[k]) for k in ['pos','quat','vel','omega','R','mass']};state.update(fuel=float(w.fuel),contact=contact,thrustScale=w.thrust_scale,disabledNozzles=list(w.disabled),deliveryMode=w.mode)
        vectors.append({'state':state,'memory':before,'params':UNTUNED.tolist(),'expected':{'nozzleThrottle':u.tolist(),'fanCommand':float(fc),'requestedWrenchBody':wrench.tolist(),'allocation':allocation,'goal':copy.deepcopy(policy.goal),'error':copy.deepcopy(policy.error),'memory':memory(policy)}})
    write_json(HERE/'controller-parity.json',{'config':cfg,'vectors':vectors})
    write_json(HERE/'baseline-replay.json',{'engineVersion':mujoco.__version__,'config':cfg,'model':'world.xml','replays':replays})
    (HERE/'world.xml').write_text(model_xml(cfg))
    print({'controllerVectors':len(vectors),'engineReplays':len(replays)})

if __name__=='__main__':main()
