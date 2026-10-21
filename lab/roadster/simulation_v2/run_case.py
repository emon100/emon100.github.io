"""Run a fresh user-specified initial state in the same MuJoCo world."""
import argparse, json, hashlib
from core import *

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--task',choices=TASKS,default='moving_recovery');ap.add_argument('--case');ap.add_argument('--policy',choices=['no_control','untuned','handcrafted','initial','mid','final'],default='final');ap.add_argument('--policies',default=str(HERE/'policies.json'));ap.add_argument('--output',default=str(HERE/'fresh-rollout.json'))
    for name in ['height','vx','vy','vz','roll','pitch','yaw','wx','wy','wz','mass','fuel','thrust-scale']:ap.add_argument('--'+name,type=float)
    ap.add_argument('--disable',default='');args=ap.parse_args();cfg=load_config()
    case=json.loads(Path(args.case).read_text()) if args.case else default_case(args.task,cfg,20261017)
    task=case.get('task',args.task)
    if task not in TASKS:ap.error('Unknown case task')
    case['task']=task
    if args.height is not None:case['pos'][2]=args.height
    for j,k in enumerate(['vx','vy','vz']):
        if getattr(args,k) is not None:case['vel'][j]=getattr(args,k)
    if any(getattr(args,k) is not None for k in ['roll','pitch','yaw']):case['quat']=euler_q(*[math.radians(getattr(args,k) or 0.) for k in ['roll','pitch','yaw']]).tolist()
    for j,k in enumerate(['wx','wy','wz']):
        if getattr(args,k) is not None:case['omega'][j]=getattr(args,k)
    for k in ['mass','fuel','thrust_scale']:
        if getattr(args,k) is not None:case[k]=getattr(args,k)
    if args.disable:case['disabled_nozzles']=args.disable.split(',')
    if args.policy in ['untuned','initial','no_control']:params=UNTUNED
    elif args.policy=='handcrafted':params=HANDCRAFTED
    else:
        policies=json.loads(Path(args.policies).read_text())
        if policies.get('sourceSha256')!=hashlib.sha256((HERE/'core.py').read_bytes()).hexdigest() or policies.get('controllerSha256')!=hashlib.sha256((HERE/'controller.mjs').read_bytes()).hexdigest():ap.error('Policy belongs to a different physical/controller revision')
        if args.policy not in policies['policies']:ap.error('Requested learned checkpoint is not ready')
        params=policies['policies'][args.policy]['parameters']
    r=rollout(task,params,case,cfg,record=True,policy_id=args.policy);r['engine']={'name':'MuJoCo','version':mujoco.__version__,'dt':cfg['physics']['dt']};r['freshSimulation']=True;write_json(args.output,r)
    print(json.dumps({'output':args.output,'status':r['status'],'success':r['success'],'metrics':r['metrics']},ensure_ascii=False))

if __name__=='__main__':main()
