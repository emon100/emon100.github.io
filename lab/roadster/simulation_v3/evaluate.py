"""Frozen paired evaluation. No parameter selection or tuning in this script."""
from pathlib import Path
import sys,json,copy,time,hashlib,collections,traceback,argparse
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
import controller as rc
core=rc.core

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def verify(manifest):
    for key,name in [('sourceSha256','baseline_r6/core.py'),('controllerSha256','controller.py'),('allocatorSha256','allocator/conservative_allocator.py'),('canonicalizerSha256','allocator/canonical_allocation.py'),('predictorSha256','landing_predictor.py'),('configSha256','config.json')]:
        assert manifest[key]==digest(HERE/name),(key,'source changed after freeze')
def cases():
    cfg=json.loads((HERE/'config.json').read_text());manifest=json.loads((HERE/'manifest.json').read_text());verify(manifest)
    source=HERE.parent/'simulation_v2/archive/energy-coupled-r6-100-generations/results.json';old=json.loads(source.read_text());spec=manifest['evaluation']
    groups={'old_fixed_recovery':[r['case'] for r in old['tasks']['moving_recovery']['holdout']['episodes']], 'old_fixed_SO3':[r['case'] for r in old['freeEvaluation']['final']['episodes']], 'new_seed_recovery':core.sample_cases('moving_recovery',spec['newRecoveryCount'],cfg,spec['newRecoverySeed']), 'new_seed_SO3':core.sample_cases('moving_recovery',spec['newSO3Count'],cfg,spec['newSO3Seed'],free=True)}
    artifact={'revision':manifest['revision'],'selection':'Fixed before execution. No results used for tuning.','baselineResultsSha256':digest(source),'groups':groups}
    path=HERE/'evaluation-cases.json'
    if path.exists():assert json.loads(path.read_text())==artifact,'Evaluation cases already frozen with different content'
    else:core.write_json(path,artifact)
    return cfg,manifest,artifact,old

def summarize(rows):
    good=sum(r['success'] for r in rows);return {'trials':len(rows),'successCount':good,'successRate':good/max(len(rows),1),'statusCounts':dict(collections.Counter(r['status'] for r in rows))}
def execute(case,cfg,params,kind,record=True):
    original=core.FeedbackPolicy;core.FeedbackPolicy=rc.RecoveryPolicy if kind=='candidate' else original
    try:return core.rollout('moving_recovery',params,case,cfg,record=record,policy_id='v3_recovery' if kind=='candidate' else 'r6_final',world=rc.RecoveryWorld(cfg) if kind=='candidate' else core.World(cfg))
    finally:core.FeedbackPolicy=original

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--prepare-only',action='store_true');args=ap.parse_args();cfg,manifest,plan,old=cases();params=manifest['parameters']
    if args.prepare_only:print('Frozen 240 cases; no simulations executed.');return
    out=HERE/'evaluation';out.mkdir(exist_ok=True);result_path=HERE/'results.json'
    if result_path.exists():raise SystemExit('Evaluation already exists; not overwritten')
    report={'revision':manifest['revision'],'kind':'engineered_repair','status':'evaluating','sourceSha256':manifest['sourceSha256'],'controllerSha256':manifest['controllerSha256'],'allocatorSha256':manifest['allocatorSha256'],'predictorSha256':manifest['predictorSha256'],'casesSha256':digest(HERE/'evaluation-cases.json'),'config':cfg,'parameters':params,'settings':manifest['settings'],'evaluationCount':0,'groups':{},'errors':[],'notice':'Old fixed sets were inspected across versions. New seeds were registered before generation and evaluation. No tuning occurs after this freeze. Success uses unchanged contacts for one second. Computational errors are distinct from physical failures.'}
    start=time.perf_counter()
    for group,group_cases in plan['groups'].items():
        target={'baseline':[],'candidate':[]};report['groups'][group]=target
        if group.startswith('old_'):
            previous=old['tasks']['moving_recovery']['holdout'] if group=='old_fixed_recovery' else old['freeEvaluation']['final'];target['baseline']=copy.deepcopy(previous['episodes']);target['baselineReusedFromFrozenR6']=True
        for kind in (['candidate'] if group.startswith('old_') else ['baseline','candidate']):
            for i,case in enumerate(group_cases):
                t=time.perf_counter();rel=f'evaluation/{group}-{kind}-{i:03}.json';path=HERE/rel
                try:
                    r=execute(case,cfg,params,kind,record=True);r['wallSeconds']=time.perf_counter()-t;core.write_json(path,r);summary={k:v for k,v in r.items() if k not in ['frames','environment']};summary['rolloutPath']=rel;summary['rolloutSha256']=digest(path);target[kind].append(summary)
                except Exception as ex:
                    e={'case':case,'group':group,'policy':kind,'status':'computation_error','success':False,'error':repr(ex),'traceback':traceback.format_exc(),'wallSeconds':time.perf_counter()-t};target[kind].append(e);report['errors'].append(e)
                report['evaluationCount']+=1;target[kind+'Summary']=summarize(target[kind]);report['wallSeconds']=time.perf_counter()-start;core.write_json(result_path,report)
                print(json.dumps({'group':group,'policy':kind,'case':i,'status':target[kind][-1]['status'],'done':report['evaluationCount'],'success':target[kind+'Summary']['successCount'],'wall':round(report['wallSeconds'],2)}),flush=True)
        target['baselineSummary']=summarize(target['baseline'])
    verify(manifest);report['status']='complete';report['wallSeconds']=time.perf_counter()-start;core.write_json(result_path,report)
    manifest['status']='evaluation_complete';manifest['evaluation'].update(status='complete',resultsPath='results.json',casesPath='evaluation-cases.json',evaluationCount=report['evaluationCount'],groups={name:{k:value[k] for k in ['baselineSummary','candidateSummary']} for name,value in report['groups'].items()});core.write_json(HERE/'manifest.json',manifest)
if __name__=='__main__':main()
