"""Actual cross-entropy search over feedback gains/reference timing; all scores are simulated."""
from __future__ import annotations
import argparse
from collections import Counter
import json
from pathlib import Path
import time
import numpy as np
from core import (HERE, load_config, cases, rollout, policy_dict, INITIAL, LOWER, UPPER,
                  axis_quat, dimensions, inertia)

NOTICE = ('All vehicle, actuator and fan parameters are explicit teaching assumptions. '
          'This is parameter search for a designed feedback controller, not a learned neural pilot. '
          'Reference paths are targets; every exported flight frame comes from force/torque integration. '
          'Success applies only to this simplified model, not to Tesla or a real vehicle.')

def write_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n')
    temporary.replace(path)

def one_rollout(task, policy, cfg, seed):
    initial=cases(task,1,cfg,seed)
    if task!='attitude_recovery':
        initial['height'][:]=cfg['scenarios'][task]['initial_height']
        initial['mass'][:]=cfg['vehicle']['mass']
    r=rollout(task,policy,initial,cfg,record=True)
    return {'frames':r['frames'],'environment':r['environment'],'success':bool(r['success'][0,0]),'status':str(r['status'][0,0]),'score':float(r['score'][0,0]),'gasLeft':float(r['gasLeft'][0,0]),'initialHeight':float(initial['height'][0]),'initialQuat':initial['quat'][0].tolist(),'initialMass':float(initial['mass'][0]),'seed':seed,'policy':policy_dict(policy)}

def evaluate(task, policy, cfg, seed):
    inputs=cases(task,cfg['training']['heldout_cases'],cfg,seed,heldout=True)
    rng=np.random.default_rng(seed+1);count=len(inputs['mass'])
    inputs['thrust_scale']=np.resize(np.array(cfg['evaluation']['thrust_scales']),count);rng.shuffle(inputs['thrust_scale'])
    inputs['distance']=rng.uniform(*cfg['evaluation']['flat_distance_range'],count) if task=='flat_kickflip' else np.full(count,cfg['scenarios'][task]['distance'])
    inputs['row_scale']=rng.uniform(*cfg['evaluation']['row_scale_range'],count) if task=='flat_kickflip' else np.ones(count)
    r=rollout(task,policy,inputs,cfg)
    rows=[{'seed':seed,'case':i,'initialHeight':float(inputs['height'][i]),'initialQuat':inputs['quat'][i].tolist(),'mass':float(inputs['mass'][i]),'thrustScale':float(inputs['thrust_scale'][i]),'targetDistance':float(inputs['distance'][i]),'rowScale':float(inputs['row_scale'][i]),'success':bool(r['success'][0,i]),'status':str(r['status'][0,i]),'score':float(r['score'][0,i]),'gasLeft':float(r['gasLeft'][0,i])} for i in range(len(inputs['mass']))]
    by_scale={str(scale):{'successRate':float(r['success'][0,inputs['thrust_scale']==scale].mean()),'trials':int((inputs['thrust_scale']==scale).sum())} for scale in cfg['evaluation']['thrust_scales']}
    failures=np.flatnonzero(~r['success'][0]);example=None
    if len(failures):
        i=int(failures[0]);single={key:(value[i:i+1] if isinstance(value,np.ndarray) else value) for key,value in inputs.items()}
        failed=rollout(task,policy,single,cfg,record=True)
        example={'frames':failed['frames'],'environment':failed['environment'],'success':False,'status':str(failed['status'][0,0]),'score':float(failed['score'][0,0]),'case':rows[i],'policy':policy_dict(policy)}
    return {'episodes':rows,'successRate':float(r['success'].mean()),'meanScore':float(r['score'].mean()),'statusCounts':dict(Counter(str(x) for x in r['status'].flat)),'byThrustScale':by_scale,'failureExample':example}

def train_task(task, cfg, seed, on_update):
    rng=np.random.default_rng(seed)
    inputs=cases(task,cfg['training'].get('flat_training_cases',12) if task=='flat_kickflip' else cfg['training']['training_cases'],cfg,seed+100)
    if task=='flat_kickflip':
        domain=np.random.default_rng(seed+101);count=len(inputs['mass'])
        inputs['distance']=domain.uniform(*cfg['evaluation']['flat_distance_range'],count)
        inputs['row_scale']=domain.uniform(*cfg['evaluation']['row_scale_range'],count)
        inputs['thrust_scale']=domain.uniform(.8,1.,count)
    n=cfg['training']['population'];elite=cfg['training']['elite'];iterations=cfg['training']['iterations']
    mean=INITIAL.copy();std=(UPPER-LOWER)*.18
    if task!='flat_kickflip':std[1]=0
    best=mean.copy();baseline=rollout(task,best,inputs,cfg)
    best_score=float(baseline['score'].mean());evaluations=len(inputs['mass'])
    report={'status':'training','method':'Task-conditioned CEM over bounded feedback/reference parameters','trainingSeed':seed+100,'trainingCases':len(inputs['mass']),'bestPolicy':policy_dict(best),'metrics':[{'generation':0,'evaluations':evaluations,'best':best_score,'mean':best_score,'successRate':float(baseline['success'].mean()),'bestSuccessRate':float(baseline['success'].mean()),'candidateScores':[best_score],'candidateEpisodeScores':baseline['score'].tolist()}],'rollouts':{'early':one_rollout(task,best,cfg,seed+200)},'trainingSeconds':0}
    report['rollouts']['early']['generation']=0
    start=time.perf_counter()
    for generation in range(1,iterations+1):
        population=np.clip(rng.normal(mean,std,(n,len(mean))),LOWER,UPPER)
        population[0]=best
        result=rollout(task,population,inputs,cfg)
        fitness=result['score'].mean(axis=1)
        order=np.argsort(fitness)[::-1];chosen=population[order[:elite]]
        if fitness[order[0]]>best_score:
            best=population[order[0]].copy();best_score=float(fitness[order[0]])
        best_success=float(result['success'][order[0]].mean())
        mean=.2*mean+.8*chosen.mean(axis=0)
        std=.25*std+.75*np.maximum(chosen.std(axis=0),(UPPER-LOWER)*.025)
        if task!='flat_kickflip':std[1]=0
        evaluations+=n*len(inputs['mass'])
        report['metrics'].append({'generation':generation,'evaluations':evaluations,'best':best_score,'mean':float(fitness.mean()),'successRate':float(result['success'].mean()),'bestSuccessRate':best_success,'candidateScores':fitness.tolist(),'candidateSuccessRates':result['success'].mean(axis=1).tolist(),'candidateEpisodeScores':result['score'].tolist(),'candidatePolicies':population.tolist()})
        report['bestPolicy']=policy_dict(best);report['trainingSeconds']=time.perf_counter()-start
        if generation==iterations//2:
            report['rollouts']['mid']=one_rollout(task,best,cfg,seed+200);report['rollouts']['mid']['generation']=generation
        on_update(report)
        print(f'{task} generation={generation:02d} best={best_score:.2f} population_mean={fitness.mean():.2f} success={best_success:.2f} evaluations={evaluations}',flush=True)
    report['rollouts']['final']=one_rollout(task,best,cfg,seed+200);report['rollouts']['final']['generation']=iterations
    report['holdout']=evaluate(task,best,cfg,seed+40000)
    report['status']='complete';report['trainingSeconds']=time.perf_counter()-start
    on_update(report)
    return best,report

def reachability(policy,cfg,seed):
    heights=cfg['reachability']['heights'];angles=cfg['reachability']['angles_deg']
    axes=np.array([[1.,0.,0.],[0.,1.,0.],[1.,1.,0.],[1.,1.,1.]])
    axes/=np.linalg.norm(axes,axis=1,keepdims=True)
    initial_height=[];quats=[];masses=[];locations=[];scales=[]
    for h in heights:
        for deg in angles:
            for axis in axes:
                for mass in cfg['vehicle']['mass_range']:
                    for scale in cfg['evaluation']['thrust_scales']:
                        initial_height.append(float(h));quats.append(axis_quat(axis,np.deg2rad(deg)));masses.append(mass);locations.append((h,deg));scales.append(scale)
    inputs=cases('attitude_recovery',len(quats),cfg,seed,grid={'height':initial_height,'quat':quats,'mass':masses})
    inputs['thrust_scale']=np.array(scales)
    r=rollout('attitude_recovery',policy,inputs,cfg)
    cells=[]
    for h in heights:
        for deg in angles:
            selected=np.array([p==(h,deg) for p in locations]);ok=r['success'][0,selected];states=r['status'][0,selected]
            cells.append({'height':h,'angleDeg':deg,'successRate':float(ok.mean()),'trials':len(ok),'statusCounts':dict(Counter(str(x) for x in states))})
    v=cfg['vehicle'];max_t=cfg['actuator']['total_jet_force_max'];mmax=max(v['mass_range'])
    rotation_lower=2*np.sqrt(np.pi*inertia(np.array(mmax),cfg)/np.array(cfg['actuator']['torque_body_max']))
    return {'heights':heights,'anglesDeg':angles,'cells':cells,'trialsPerCell':len(axes)*2*len(cfg['evaluation']['thrust_scales']),'seed':seed,
            'thrustScales':cfg['evaluation']['thrust_scales'],'axes':axes.tolist(),'massExtremes':v['mass_range'],'interpretation':'Success grid for this trained policy and these finite sampled cases. A failed cell is not a mathematical impossibility proof; a successful cell is not a proof for every orientation or disturbance.',
            'limits':{'maximumBodyRadius':float(np.linalg.norm(dimensions(cfg))),'optimisticMaximumUpwardAcceleration':max_t/mmax-cfg['gravity'],'optimisticPiRotationTimePerBodyAxis':rotation_lower.tolist(),'necessaryConditions':['Initial body geometry must clear the ground.','Finite downward stopping distance and actuator lag require positive clearance.','The reported rotation-time bounds assume isolated full torque and zero initial angular velocity; shared lift/gas limits can only increase the required time.','Arbitrary attitude at arbitrary low height is not guaranteed, even with perfect control.']}}

def compare_steps(best,cfg,reports,seed):
    import copy
    comparison={'baseDt':cfg['dt'],'notice':'Same final policies and same held-out initial states; no retraining at smaller integration steps.','tasks':{}}
    for i,(task,policy) in enumerate(best.items()):
        base=[e['success'] for e in reports[task]['holdout']['episodes']];rows=[]
        for dt in [.02,.01]:
            fine=copy.deepcopy(cfg);fine['dt']=dt
            evaluated=evaluate(task,policy,fine,seed+i*1000+40000)
            changed=sum(a!=b['success'] for a,b in zip(base,evaluated['episodes']))
            rows.append({'dt':dt,'successRate':evaluated['successRate'],'changedClassifications':changed,'cases':len(base),'statusCounts':evaluated['statusCounts']})
        comparison['tasks'][task]=rows
    return comparison

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config');parser.add_argument('--iterations',type=int);parser.add_argument('--population',type=int);parser.add_argument('--output',default=str(HERE/'results.json'));args=parser.parse_args()
    cfg=load_config(args.config)
    if args.iterations:cfg['training']['iterations']=args.iterations
    if args.population:cfg['training']['population']=args.population
    output=Path(args.output);seed=cfg['seed']
    data={'schemaVersion':1,'status':'training','assumptionNotice':NOTICE,'seed':seed,'config':cfg,'tasks':{},'units':{'pos':'m, center of mass, z up','quat':'w,x,y,z, body-to-world','vel':'m/s, world','omega':'rad/s, body','gasLeft':'kg','thrust':'N, world','fanForce':'N, world','torque':'N m, body','fanPressure':'Pa','fanPower':'W useful fluid power'}}
    start=time.perf_counter();best={}
    for i,task in enumerate(cfg['scenarios']):
        def update(report):data['tasks'][task]=report;write_json(output,data)
        best[task],data['tasks'][task]=train_task(task,cfg,seed+i*1000,update)
    data['reachability']=reachability(best['attitude_recovery'],cfg,seed+50000)
    import copy
    unregulated=copy.deepcopy(cfg);unregulated['gas_delivery']['mode']='unregulated_blowdown'
    data['pressureSensitivity']={'mode':'unregulated_blowdown','notice':'Same trained policies and identical held-out initial cases, without retraining; a failure does not prove a different policy cannot succeed.','tasks':{task:evaluate(task,policy,unregulated,seed+40000+i*1000) for i,(task,policy) in enumerate(best.items())}}
    data['numerics']=compare_steps(best,cfg,data['tasks'],seed)
    data['status']='complete';data['wallSeconds']=time.perf_counter()-start
    write_json(output,data)
    print(json.dumps({'output':str(output),'wallSeconds':data['wallSeconds'],'holdout':{t:r['holdout']['successRate'] for t,r in data['tasks'].items()}},ensure_ascii=False),flush=True)

if __name__=='__main__':main()
