from pathlib import Path
import argparse,json,hashlib
import unified_controller as u
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--policy',choices=['initial','reference','final'],default='final');p.add_argument('--parameters',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
if args.output.exists():p.error('Output exists. Choose a new filename to preserve the original record.')
cfg=json.loads((HERE/'config.json').read_text());case=json.loads(args.case.read_text());case['task']='moving_recovery';case.setdefault('omega',[0,0,0]);case.setdefault('goal',{'turns':0});case.setdefault('id','custom-airborne')
if case['pos'][2]-u.core.geometry_minimum_cg(u.core.qmat(case['quat']),cfg)<.02:p.error('Airborne test requires at least 0.02 m initial clearance.')
manifest=json.loads((HERE/'manifest.json').read_text()) if (HERE/'manifest.json').exists() else json.loads((HERE/'campaign/results.json').read_text())
if args.policy=='final' and not args.parameters and manifest.get('status')!='training_complete':p.error('Final parameters have not been frozen.')
params=json.loads(args.parameters.read_text()) if args.parameters else manifest.get(args.policy,manifest.get('selectedParameters') if args.policy=='final' else None)
if not isinstance(params,list) or len(params)!=len(u.PARAMETERS):p.error('Expected 14 controller parameters.')
result=u.rollout(params,case,cfg,True);result.update(config=cfg,parameters=params,controllerSha256=hashlib.sha256((HERE/'unified_controller.py').read_bytes()).hexdigest());args.output.parent.mkdir(parents=True,exist_ok=True);u.core.write_json(args.output,result);print(result['status'],result['metrics'])
