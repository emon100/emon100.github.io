"""Generate native MuJoCo reference values for the WASM regression test."""
import json
import hashlib
import numpy as np
from core import *

cfg=load_config()
ground=cfg['vehicle']['ground_cg_height']+.005
fixtures=[]
for label,changes,command,fan,steps in [
    ('free_fall',dict(pos=[1.,2.,15.],quat=[1.,0.,0.,0.],vel=[2.,0.,0.]),[0.]*12,0.,10),
    ('single_jet',dict(pos=[0.,0.,8.],quat=euler_q(.6,.2,-.4).tolist(),vel=[1.,-2.,.3]),[.6]+[0.]*11,0.,15),
    ('fan_airborne',dict(pos=[0.,0.,8.],quat=euler_q(.2,.1,.3).tolist(),vel=[0.,0.,0.]),[0.]*12,.7,10),
    ('fan_near_floor',dict(pos=[0.,0.,ground],quat=[1.,0.,0.,0.],vel=[0.,0.,0.]),[0.]*12,.3,10),
    ('low_gas_disabled_jets',dict(pos=[0.,0.,8.],quat=[1.,0.,0.,0.],vel=[0.,0.,0.],fuel=12.,disabled_nozzles=['J02','J06']),[.3]*12,0.,10),
    ('rolling',dict(pos=[0.,0.,ground],quat=[1.,0.,0.,0.],vel=[5.,0.,0.]),[0.]*12,0.,65),
]:
    c=default_case('moving_recovery',cfg);c.update(changes);w=World(cfg);w.reset(c)
    context={'task':'moving_kickflip','turnComplete':True,'wasAirborne':True} if label=='rolling' else None
    pressure_history=[]
    for _ in range(steps):
        s=w.advance(np.array(command),fan,context);pressure_history.append(w.pressure_scale)
    fixtures.append(dict(label=label,case=c,command=command,fanCommand=fan,steps=steps,stabilityContext=context,pressureHistory=pressure_history,assemblyCom=w.data.subtree_com[1].tolist(),massProperties={'bodyMass':float(w.model.body_mass[1]),'bodyIpos':w.model.body_ipos[1].tolist(),'bodyInertia':w.model.body_inertia[1].tolist()},state={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in s.items()},fuel=w.fuel,forces=w.forces.tolist(),contact=w.contact))

policies={'untuned':UNTUNED.tolist(),'handcrafted':HANDCRAFTED.tolist()}
revision=None
policy_file=HERE/'policies.json'
if policy_file.exists():
    saved=json.loads(policy_file.read_text())
    current_source=hashlib.sha256((HERE/'core.py').read_bytes()).hexdigest()
    current_controller=hashlib.sha256((HERE/'controller.mjs').read_bytes()).hexdigest()
    if saved.get('sourceSha256')==current_source:revision=saved.get('revision')
    if 'final' in saved['policies'] and saved.get('sourceSha256')==current_source and saved.get('controllerSha256')==current_controller and all(saved['config'].get(k)==v for k,v in cfg.items()):
        policies['final']=saved['policies']['final']['parameters']
rollouts=[]
def first_roll_observation(r):
    if r['case']['task']=='moving_recovery':return None
    frames=r['frames'];A=wrench_matrix(nozzle_layout(cfg))
    for i,frame in enumerate(frames[1:],1):
        if frame['goal'].get('phase')!='flip':continue
        before=frames[i-1]
        return dict(t=before['t'],heightCG=before['pos'][2],clearance=before['pos'][2]-cfg['vehicle']['ground_cg_height'],verticalVelocity=before['vel'][2],wheelCount=before['contact']['wheelCount'],bodyContact=before['contact']['bodyContact'],requestedRollTorque=frame['allocation']['requestedWrenchBody'][3],allocatedRollTorque=frame['allocation']['achievedWrenchBody'][3],actualRollTorque=float((A@np.array(frame['nozzleForces']))[3]),requestedForceWorld=(qmat(before['quat'])@np.array(frame['allocation']['requestedWrenchBody'][:3])).tolist(),actualJetForceWorld=frame['jetForceWorld'])
    return None
for task in TASKS:
    for label,values in policies.items():
        p=np.array(values)
        case=default_case(task,cfg)
        r=rollout(task,p,case,cfg,record=True,policy_id=label)
        rollouts.append(dict(task=task,policyId=label,params=p.tolist(),case=case,status=r['status'],success=r['success'],metrics=r['metrics'],launch=first_roll_observation(r),final=r['frames'][-1]))
for label,changes in [
    ('free_attitude',dict(pos=[1.,-2.,8.],quat=euler_q(2.2,.6,-.4).tolist(),vel=[4.,-2.,-2.],omega=[.2,-.4,.3],mass=2100.)),
    ('too_low',dict(pos=[0.,0.,.8],quat=euler_q(np.pi,0.,0.).tolist(),vel=[1.,0.,-4.])),
]:
    case=default_case('moving_recovery',cfg);case.update(changes)
    if label=='too_low':case['pos'][2]+=.02-initial_clearance(case,cfg)
    p=np.array(policies.get('final',policies['handcrafted']));r=rollout('moving_recovery',p,case,cfg,record=True)
    rollouts.append(dict(task=label,policyId='final' if 'final' in policies else 'handcrafted',params=p.tolist(),case=case,status=r['status'],success=r['success'],metrics=r['metrics'],final=r['frames'][-1]))
print(json.dumps(dict(revision=revision,config=cfg,xml=model_xml(cfg),fixtures=fixtures,policies=policies,rollouts=rollouts),allow_nan=False))
