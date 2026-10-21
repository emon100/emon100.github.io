"""Reproducible CEM parameter learning, paired baselines, held-out contact rollouts."""
import argparse, copy, hashlib, json, os, time, signal, datetime, fcntl
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from core import *
from split_audit import attach_metadata

def write_json(path,value):
    # Readers may fetch the training history while the next generation is published.
    finite(value);path=Path(path);temporary=path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n');os.replace(temporary,path)

_WORKER_WORLD=None
_WORKER_CFG=None
_WORKER_LEDGER=None
_ACTIVE_LEDGER=None
_STOP_REQUESTED=False

def request_stop(signum,frame):
    global _STOP_REQUESTED
    _STOP_REQUESTED=True

def json_rng_state(value):
    if isinstance(value,dict):return {k:json_rng_state(v) for k,v in value.items()}
    if isinstance(value,list):return [json_rng_state(v) for v in value]
    if isinstance(value,int) and abs(value)>2**53-1:return str(value)
    return value

def append_jsonl(path,record):
    if path is None:return
    finite(record)
    with open(path,'a',encoding='utf-8') as stream:
        fcntl.flock(stream.fileno(),fcntl.LOCK_EX)
        stream.write(json.dumps(record,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n');stream.flush()
        fcntl.flock(stream.fileno(),fcntl.LOCK_UN)

def log_episode(path,parameters,case,result,label,kind,generation=None):
    if path is None:return
    append_jsonl(path,{'kind':kind,'label':label,'generation':generation,'caseId':case['id'],'case':case,'parameterHash':hashlib.sha256(np.asarray(parameters,dtype='<f8').tobytes()).hexdigest(),'success':result['success'],'status':result['status'],'metrics':result['metrics']})


def init_worker(cfg,ledger_path=None):
    global _WORKER_WORLD,_WORKER_CFG,_WORKER_LEDGER
    _WORKER_CFG=cfg;_WORKER_WORLD=World(cfg);_WORKER_LEDGER=ledger_path

def evaluate_job(job):
    parameters,cases,label=job;rows=[]
    for case in cases:
        r=rollout(case['task'],parameters,case,_WORKER_CFG,policy_id=label,world=_WORKER_WORLD)
        log_episode(_WORKER_LEDGER,parameters,case,r,label,'worker_evaluation')
        rows.append({'case':case,'success':r['success'],'status':r['status'],'metrics':r['metrics']})
    return rows

def summary(rows):
    return {'successRate':float(np.mean([r['success'] for r in rows])) if rows else 0.,'successCount':sum(r['success'] for r in rows),'trials':len(rows),'meanScore':float(np.mean([r['metrics']['score'] for r in rows])) if rows else 0.,'statusCounts':dict(Counter(r['status'] for r in rows)),'episodes':rows}

def evaluate(pool,params,cases,label):
    # Split cases so all workers contribute; preserve the exact input order.
    return [row for batch in pool.map(evaluate_job,[(params,cases[i:i+4],label) for i in range(0,len(cases),4)]) for row in batch]

def policy_record(params,label,generation):
    result=policy_json(params,label);result['generation']=generation;return result

def recorded(task,params,case,cfg,label,generation):
    r=rollout(task,params,case,cfg,record=True,policy_id=label);r['generation']=generation;log_episode(_ACTIVE_LEDGER,params,case,r,label,'recorded_replay',generation);return r

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config');parser.add_argument('--output',default=str(HERE/'results.json'));args=parser.parse_args()
    global _ACTIVE_LEDGER
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    cfg=load_config(args.config);rng=np.random.default_rng(cfg['seed']);start=time.perf_counter();tc=cfg['training'];generations=tc['generations'];popsize=tc['population'];elite_count=max(3,round(popsize*tc['elite_fraction']))
    ledger_path=HERE/'training-episodes.jsonl';candidate_path=HERE/'candidate-records.jsonl';ledger_path.write_text('');candidate_path.write_text('');_ACTIVE_LEDGER=str(ledger_path)
    checkpoint_index={'revision':'energy-coupled-r6','campaignId':tc['campaign_id'],'checkpoints':[]};write_json(HERE/'checkpoint-index.json',checkpoint_index);(HERE/'checkpoints').mkdir(exist_ok=True)
    interruption=json.loads((HERE/'archive'/'energy-coupled-r6-short-run'/'INTERRUPTION.json').read_text())
    (HERE/'world.xml').write_text(model_xml(cfg));cfg_public=copy.deepcopy(cfg);cfg_public['nozzles']=nozzle_layout(cfg)
    initial_cases={task:default_case(task,cfg,cfg['seed']) for task in TASKS}
    validation=[c for j,task in enumerate(TASKS) for c in sample_cases(task,tc['validation_cases_per_task'],cfg,cfg['seed']+1000+j*100)]
    holdout={task:sample_cases(task,cfg['evaluation']['cases_per_task'],cfg,cfg['seed']+90000+j*1000) for j,task in enumerate(TASKS)}
    data={'schemaVersion':3,'revision':'energy-coupled-r6','sourceSha256':hashlib.sha256((HERE/'core.py').read_bytes()).hexdigest(),'controllerSha256':hashlib.sha256((HERE/'controller.mjs').read_bytes()).hexdigest(),'trainerSha256':hashlib.sha256((HERE/'train.py').read_bytes()).hexdigest(),'status':'training','campaign':{'id':tc['campaign_id'],'plannedGenerations':generations,'rngStateEncoding':'integers above 2^53-1 are decimal strings','completedGenerations':0,'startedUTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),'episodeLedger':'training-episodes.jsonl','candidateRecords':'candidate-records.jsonl','checkpointIndex':'checkpoint-index.json','priorInterruptedCampaign':{**interruption,'archive':'archive/energy-coupled-r6-short-run'}},'engine':{'name':'MuJoCo','version':mujoco.__version__,'dt':cfg['physics']['dt'],'controlDt':cfg['physics']['control_dt'],'model':'world.xml','poseSource':'mj_step rigid-body and wheel/contact integration'},'config':cfg_public,'controllability':controllability(cfg),'algorithm':{'name':'CEM over a task-conditioned state-feedback controller','sharedAcrossTasks':True,'controllerRevisionNotice':'r6 adds 5m actual-apex validation, learned early roll speed and energy-qualified coast/roll. A nominal-development angular braking feedforward and manually selected descent baseline are explicit structural/manual changes, not CEM discoveries. Reward scores differ from r5 because an actual-height progress term was added.','parameters':PARAMETERS,'lower':LOWER.tolist(),'upper':UPPER.tolist(),'seed':cfg['seed'],'selection':'training population elites; checkpoint selected on fixed development set (2 states also appear in training); held-out cases never used for selection','initialization':'generic untuned parameter vector fixed before rollouts; handcrafted baseline excluded from search population','observations':'state and contact, no prescribed clock-dependent pose trajectory','reward':'actual apex-height progress (120, flip tasks only) + turn/upright progress + sustained stable-contact bonus + success bonus - fuel/time/angular rate - contact severity','success':'actual peak CG >= configured minimum (5m for flip tasks), required turn, no body/hard contact, then >=1s with >=3 wheel contacts, bounded tilt, body angular speed, vertical speed, lateral slip and contact-point rolling slip; forward rolling allowed, no horizontal position/speed goal'},'tasks':{},'evaluationCount':0,'trainingHistory':[],'checkpoints':[],'sources':[{'title':'MuJoCo contact dynamics','url':'https://mujoco.readthedocs.io/en/stable/computation/index.html'},{'title':'MuJoCo Python API','url':'https://mujoco.readthedocs.io/en/stable/python.html'}]}
    for task in TASKS:
        data['tasks'][task]={'label':cfg['tasks'][task]['label'],'metrics':[],'policies':{'untuned':policy_record(UNTUNED,'未调参控制器',0),'handcrafted':policy_record(HANDCRAFTED,'人工开发基线',None),'initial':policy_record(UNTUNED,'训练初始',0)},'comparisons':{}}
        for label,p in [('no_control',UNTUNED),('untuned',UNTUNED),('handcrafted',HANDCRAFTED)]:
            data['tasks'][task]['comparisons'][label]=recorded(task,p,initial_cases[task],cfg,label,None if label in ['no_control','handcrafted'] else 0)
        data['tasks'][task]['comparisons']['initial']=copy.deepcopy(data['tasks'][task]['comparisons']['untuned']);data['tasks'][task]['comparisons']['initial']['policyId']='initial'
    data['evaluationCount']+=9;write_json(args.output,data)
    write_json(HERE/'policies.json',{'schemaVersion':3,'revision':data['revision'],'status':'training','sourceSha256':data['sourceSha256'],'controllerSha256':data['controllerSha256'],'engineVersion':mujoco.__version__,'config':cfg_public,'policies':{'initial':policy_record(UNTUNED,'训练初始',0),'untuned':policy_record(UNTUNED,'未调参',0),'handcrafted':policy_record(HANDCRAFTED,'人工开发基线',None)}})
    mean=(UNTUNED-LOWER)/(UPPER-LOWER);sigma=np.full(len(PARAMETERS),.24);best=UNTUNED.copy();best_generation=0;mid=UNTUNED.copy();mid_generation=0
    workers=min(tc['workers'],os.cpu_count() or 1)
    with ProcessPoolExecutor(max_workers=workers,initializer=init_worker,initargs=(cfg,str(ledger_path))) as pool:
        validation_rows=evaluate(pool,best,validation,'initial');best_validation=float(np.mean([r['metrics']['score'] for r in validation_rows]));data['evaluationCount']+=len(validation_rows)
        for task in TASKS:
            rows=[r for r in validation_rows if r['case']['task']==task];data['tasks'][task]['metrics'].append({'generation':0,'evaluations':0,'best':float(np.mean([r['metrics']['score'] for r in rows])),'mean':float(np.mean([r['metrics']['score'] for r in rows])),'successRate':float(np.mean([r['success'] for r in rows])),'validationSuccessRate':float(np.mean([r['success'] for r in rows])),'selectedPolicyGeneration':0})
        for generation in range(1,generations+1):
            candidates=np.clip(rng.normal(mean,sigma,(popsize,len(mean))),0,1);candidates[0]=mean;candidates[1]=(best-LOWER)/(UPPER-LOWER);parameters=LOWER+candidates*(UPPER-LOWER)
            cases=[c for j,task in enumerate(TASKS) for c in sample_cases(task,tc['cases_per_task'],cfg,cfg['seed']+generation*100+j*10000)]
            batches=list(pool.map(evaluate_job,[(p,cases,f'candidate/generation-{generation:03d}/{i:03d}') for i,p in enumerate(parameters)]));fitness=np.array([np.mean([r['metrics']['score'] for r in rows]) for rows in batches]);order=np.argsort(fitness);elite=candidates[order[-elite_count:]]
            mean=.25*mean+.75*np.mean(elite,axis=0);sigma=np.maximum(.035,.25*sigma+.75*np.std(elite,axis=0))
            candidate=parameters[order[-1]];candidate_rows=evaluate(pool,candidate,validation,f'development/generation-{generation:03d}');candidate_validation=float(np.mean([r['metrics']['score'] for r in candidate_rows]))
            if candidate_validation>best_validation:
                best=candidate.copy();best_generation=generation;best_validation=candidate_validation;validation_rows=candidate_rows
            data['evaluationCount']+=popsize*len(cases)+len(validation)
            for task in TASKS:
                per_candidate=[[r for r in rows if r['case']['task']==task] for rows in batches];chosen=per_candidate[order[-1]];val=[r for r in validation_rows if r['case']['task']==task]
                data['tasks'][task]['metrics'].append({'generation':generation,'evaluations':generation*popsize*tc['cases_per_task'],'best':float(np.mean([r['metrics']['score'] for r in chosen])),'mean':float(np.mean([r['metrics']['score'] for rows in per_candidate for r in rows])),'successRate':float(np.mean([r['success'] for r in chosen])),'validationSuccessRate':float(np.mean([r['success'] for r in val])),'selectedPolicyGeneration':best_generation})
            append_jsonl(candidate_path,{'kind':'generation','campaignId':tc['campaign_id'],'generation':generation,'cases':cases,'candidates':[{'candidateIndex':i,'parameters':p.tolist(),'meanScore':float(fitness[i]),'episodes':batch} for i,(p,batch) in enumerate(zip(parameters,batches))],'generationWinnerIndex':int(order[-1]),'developmentEvaluation':candidate_rows,'selectedAtGeneration':best_generation,'selectedParameters':best.tolist(),'selectedDevelopmentScore':best_validation,'searchMean':(LOWER+mean*(UPPER-LOWER)).tolist(),'searchSigma':(sigma*(UPPER-LOWER)).tolist(),'rngState':json_rng_state(rng.bit_generator.state)})
            data['campaign']['completedGenerations']=generation
            data['trainingHistory'].append({'generation':generation,'candidateScores':fitness.tolist(),'selectedParameters':best.tolist(),'selectedAtGeneration':best_generation,'validationScore':best_validation,'searchMean':(LOWER+mean*(UPPER-LOWER)).tolist(),'searchSigma':(sigma*(UPPER-LOWER)).tolist()})
            data['checkpoints'].append(policy_record(best,f'第 {generation} 代结束后的开发集最优',best_generation))
            if generation==generations//2:
                mid=best.copy();mid_generation=best_generation
                for task in TASKS:
                    data['tasks'][task]['policies']['mid']=policy_record(mid,'训练中期',mid_generation);data['tasks'][task]['comparisons']['mid']=recorded(task,mid,initial_cases[task],cfg,'mid',mid_generation)
                data['evaluationCount']+=3
            if generation%tc['checkpoint_replay_interval']==0 or generation==generations:
                shard={'revision':data['revision'],'campaignId':tc['campaign_id'],'generation':generation,'selectedAtGeneration':best_generation,'parameters':best.tolist(),'parameterNames':PARAMETERS,'tasks':{task:recorded(task,best,initial_cases[task],cfg,f'checkpoint_g{generation:03d}',best_generation) for task in TASKS}}
                relative=f'checkpoints/generation-{generation:03d}.json';write_json(HERE/relative,shard);checkpoint_index['checkpoints'].append({'generation':generation,'selectedAtGeneration':best_generation,'path':relative});write_json(HERE/'checkpoint-index.json',checkpoint_index);data['evaluationCount']+=len(TASKS)
            data['resumeState']={'completedGeneration':generation,'mean':mean.tolist(),'sigma':sigma.tolist(),'best':best.tolist(),'bestGeneration':best_generation,'bestDevelopmentScore':best_validation,'developmentRows':validation_rows,'mid':mid.tolist(),'midGeneration':mid_generation,'rngState':json_rng_state(rng.bit_generator.state)}
            data['wallSeconds']=time.perf_counter()-start;write_json(args.output,data)
            print(json.dumps({'generation':generation,'bestTraining':round(float(fitness.max()),2),'meanTraining':round(float(fitness.mean()),2),'bestValidation':round(best_validation,2),'validationSuccess':round(float(np.mean([r['success'] for r in validation_rows])),3),'selectedAt':best_generation,'elapsed':round(data['wallSeconds'])}),flush=True)
            if _STOP_REQUESTED or (HERE/'STOP_TRAINING').exists():
                data['status']='stopped_at_generation_boundary';data['campaign']['stopNotice']='Stopped only after every candidate/development/replay evaluation in this generation was logged.';write_json(args.output,data);return
        data['status']='evaluating';write_json(args.output,data)
        write_json(HERE/'policies.json',{'schemaVersion':3,'revision':data['revision'],'status':'evaluating','sourceSha256':data['sourceSha256'],'controllerSha256':data['controllerSha256'],'engineVersion':mujoco.__version__,'config':cfg_public,'nozzleMatrix':wrench_matrix(nozzle_layout(cfg)).tolist(),'policies':{key:policy_record(par,label,generation) for key,par,label,generation in [('untuned',UNTUNED,'未调参',0),('initial',UNTUNED,'训练初始',0),('handcrafted',HANDCRAFTED,'人工开发基线',None),('mid',mid,'训练中期',mid_generation),('final',best,'训练最终',best_generation)]}})
        for task in TASKS:
            report=data['tasks'][task];report['policies']['final']=policy_record(best,'训练最终',best_generation);report['comparisons']['final']=recorded(task,best,initial_cases[task],cfg,'final',best_generation)
            report['baselineEvaluation']={}
            for label,p in [('no_control',UNTUNED),('untuned',UNTUNED),('handcrafted',HANDCRAFTED),('mid',mid),('final',best)]:
                rows=evaluate(pool,p,holdout[task],label);report['baselineEvaluation'][label]=summary(rows);data['evaluationCount']+=len(rows)
            report['holdout']=report['baselineEvaluation']['final'];failed=next((r for r in report['holdout']['episodes'] if not r['success']),None)
            report['failureExamples']=[recorded(task,best,failed['case'],cfg,'final',best_generation)] if failed else []
            data['evaluationCount']+=len(report['failureExamples'])
            report['sensitivity']={}
            for label in ['disable_four_downward_jets','thrust_80_percent','initial_charge_80_percent']:
                changed=copy.deepcopy(holdout[task][:12])
                for c in changed:
                    if label=='disable_four_downward_jets':c['disabled_nozzles']=['J02','J04','J06','J08']
                    elif label=='thrust_80_percent':c['thrust_scale']*=.8
                    else:c['fuel']*=.8
                rows=evaluate(pool,best,changed,'final');report['sensitivity'][label]=summary(rows);data['evaluationCount']+=len(rows)
            data['wallSeconds']=time.perf_counter()-start;write_json(args.output,data)
            print(task,'holdout',report['holdout']['successCount'],'/',report['holdout']['trials'],flush=True)
        free=sample_cases('moving_recovery',cfg['evaluation']['free_so3_cases'],cfg,cfg['seed']+500000,free=True);data['freeEvaluation']={}
        for label,p in [('untuned',UNTUNED),('handcrafted',HANDCRAFTED),('final',best)]:
            rows=evaluate(pool,p,free,label);data['freeEvaluation'][label]=summary(rows);data['evaluationCount']+=len(rows)
        failed=next((r for r in data['freeEvaluation']['final']['episodes'] if not r['success']),None)
        data['freeFailureExample']=recorded('moving_recovery',best,failed['case'],cfg,'final',best_generation) if failed else None
        data['evaluationCount']+=int(failed is not None)
    data['evaluationCount']+=3
    with open(ledger_path,encoding='utf-8') as stream:ledger_count=sum(1 for _ in stream)
    assert ledger_count==data['evaluationCount'],f'Evaluation ledger mismatch: {ledger_count} vs {data["evaluationCount"]}'
    data['campaign']['ledgerVerifiedEvaluations']=ledger_count;data['status']='complete';attach_metadata(data);data['wallSeconds']=time.perf_counter()-start;assert data['sourceSha256']==hashlib.sha256((HERE/'core.py').read_bytes()).hexdigest(),'Core changed during training';assert data['controllerSha256']==hashlib.sha256((HERE/'controller.mjs').read_bytes()).hexdigest(),'Controller changed during training';write_json(args.output,data)
    write_json(HERE/'policies.json',{'schemaVersion':3,'revision':'energy-coupled-r6','status':'complete','sourceSha256':data['sourceSha256'],'controllerSha256':data['controllerSha256'],'campaign':data['campaign'],'dataSplitAudit':data['dataSplitAudit'],'selectionNotice':data['dataSplitAudit']['selectionNotice'],'engineVersion':mujoco.__version__,'config':cfg_public,'nozzleMatrix':wrench_matrix(nozzle_layout(cfg)).tolist(),'policies':{'untuned':policy_record(UNTUNED,'未调参',0),'handcrafted':policy_record(HANDCRAFTED,'人工开发基线',None),'initial':policy_record(UNTUNED,'训练初始',0),'mid':policy_record(mid,'训练中期',mid_generation),'final':policy_record(best,'训练最终',best_generation)}})
    print(json.dumps({'output':args.output,'evaluations':data['evaluationCount'],'wallSeconds':round(data['wallSeconds']),'heldout':{task:data['tasks'][task]['holdout']['successRate'] for task in TASKS},'free':data['freeEvaluation']['final']['successRate']}),flush=True)

if __name__=='__main__':main()
