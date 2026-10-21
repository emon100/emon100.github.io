"""Describe the actual deterministic data split without changing any scores or policies."""
import hashlib, json
from pathlib import Path
from core import TASKS, sample_cases, finite


def signature(case):
    # IDs/seeds are labels; compare the actual physical initial conditions, even across tasks.
    values={k:v for k,v in case.items() if k not in ['id','seed','task']}
    return hashlib.sha256(json.dumps(values,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def attach_metadata(data):
    cfg=data['config'];seed=cfg['seed'];t=cfg['training']
    training=[c for generation in range(1,t['generations']+1) for j,task in enumerate(TASKS) for c in sample_cases(task,t['cases_per_task'],cfg,seed+generation*100+j*10000)]
    development=[c for j,task in enumerate(TASKS) for c in sample_cases(task,t['validation_cases_per_task'],cfg,seed+1000+j*100)]
    holdout=[r['case'] for task in TASKS for r in data['tasks'][task]['holdout']['episodes']]
    free=[r['case'] for r in data['freeEvaluation']['final']['episodes']]
    banks={'training':training,'development':development,'holdout':holdout,'freeSO3':free};indexed={name:{signature(c):c for c in cases} for name,cases in banks.items()};overlaps={}
    for a,b in [('training','development'),('training','holdout'),('training','freeSO3'),('development','holdout'),('development','freeSO3'),('holdout','freeSO3')]:
        common=sorted(set(indexed[a])&set(indexed[b]));overlaps[f'{a}__{b}']={'count':len(common),'caseIds':[{'first':indexed[a][k]['id'],'second':indexed[b][k]['id'],'stateHash':k} for k in common]}
    isolated=all(overlaps[k]['count']==0 for k in ['training__holdout','training__freeSO3','development__holdout','development__freeSO3'])
    assert isolated,'Final unseen evaluation overlaps training or policy-selection cases'
    data['dataSplitAudit']={'method':'Reconstructs deterministic training/development batches from recorded config/seed with sample_cases, then compares physical initial-state signatures against actual final evaluation cases. IDs, seed labels and task names are excluded from the signature.','counts':{k:len(v) for k,v in banks.items()},'uniqueCounts':{k:len(v) for k,v in indexed.items()},'overlaps':overlaps,'finalEvaluationDisjointFromTrainingAndSelection':isolated,'developmentLabel':'固定开发评估集（12例，其中2例也用于训练）','selectionNotice':'Checkpoint selection uses this fixed development set. It is not an independent validation claim. Final conclusions use the separate 120 held-out task cases and 80 SO(3) cases.'}
    if data.get('revision') in ['launch-coupled-r5','energy-coupled-r6']:
        reference='launch-coupled-r5' if data.get('revision')=='energy-coupled-r6' else 'attitude-only-blowdown-r4'
        data['dataSplitAudit']['crossRevisionReuse']={'referenceRevision':reference,'sameEvaluationInitialStates':True,'notice':'The 120 task cases and 80 SO(3) cases are reused for paired comparison with the prior revision. They were not fitted or selected on in this run, but their prior-revision results were previously inspected during development; they are not a newly unseen engineering test set.'}
        previous=json.loads((Path(__file__).resolve().parent/'archive'/reference/'results.json').read_text())
        old_holdout=[r['case'] for task in TASKS for r in previous['tasks'][task]['holdout']['episodes']];old_free=[r['case'] for r in previous['freeEvaluation']['final']['episodes']]
        matches={'taskCases':sum(signature(a)==signature(b) for a,b in zip(holdout,old_holdout)),'freeSO3Cases':sum(signature(a)==signature(b) for a,b in zip(free,old_free))}
        assert matches['taskCases']==len(holdout)==len(old_holdout) and matches['freeSO3Cases']==len(free)==len(old_free),'Expected paired r4/r5 evaluation cases'
        data['dataSplitAudit']['crossRevisionReuse']['verifiedMatchingInitialStates']=matches
        data['algorithm']['evaluationReuseNotice']=data['dataSplitAudit']['crossRevisionReuse']['notice']
    data['algorithm']['selection']='population elites on training cases; checkpoint selected on fixed development set (12 cases, including 2 training repeats); final held-out and SO(3) cases never used for selection'
    data['algorithm']['developmentEvaluationLabel']=data['dataSplitAudit']['developmentLabel']
    for report in data['tasks'].values():
        for metric in report['metrics']:
            metric['developmentSuccessRate']=metric['validationSuccessRate']
        for policy in report['policies'].values():policy['selectedOn']='fixed development set, not independent validation' if policy.get('generation') else 'baseline or initial policy'
    for history in data['trainingHistory']:history['developmentScore']=history['validationScore']
    data['legacyFieldNotice']='validationSuccessRate and validationScore are retained as compatibility aliases for developmentSuccessRate and developmentScore; these refer to the disclosed development set.'
    finite(data);return data


if __name__=='__main__':
    root=Path(__file__).resolve().parent;path=root/'results.json';data=json.loads(path.read_text());assert data['status']=='complete';attach_metadata(data)
    temporary=path.with_name(path.name+'.tmp');temporary.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n');temporary.replace(path)
    pp=root/'policies.json';p=json.loads(pp.read_text());p['status']='complete';p['selectionNotice']=data['dataSplitAudit']['selectionNotice'];p['dataSplitAudit']=data['dataSplitAudit'];pp.write_text(json.dumps(p,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
    print(json.dumps(data['dataSplitAudit'],ensure_ascii=False,indent=2))
