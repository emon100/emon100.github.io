"""Verify full campaign logs, deterministic candidate generation and replay provenance."""
import hashlib,json,argparse
from pathlib import Path
import numpy as np
from core import HERE,PARAMETERS,LOWER,UPPER,UNTUNED,TASKS,sample_cases,finite,write_json
from train import json_rng_state


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--partial',action='store_true');args=parser.parse_args()
    data=json.loads((HERE/'results.json').read_text())
    if not args.partial:assert data['status']=='complete'
    cfg=data['config'];tc=cfg['training'];records=[json.loads(line) for line in (HERE/'candidate-records.jsonl').read_text().splitlines() if line.endswith('}')]
    completed=data['campaign']['completedGenerations'];records=[r for r in records if r['generation']<=completed]
    ledger=[json.loads(line) for line in (HERE/'training-episodes.jsonl').read_text().splitlines() if line.endswith('}')];finite(records);finite(ledger)
    assert len(records)==(completed if args.partial else tc['generations'])
    if args.partial:assert len(ledger)>=data['evaluationCount']
    else:assert len(ledger)==data['evaluationCount']==data['campaign']['ledgerVerifiedEvaluations']
    assert data['sourceSha256']==hashlib.sha256((HERE/'core.py').read_bytes()).hexdigest()
    assert data['controllerSha256']==hashlib.sha256((HERE/'controller.mjs').read_bytes()).hexdigest()
    assert data['trainerSha256']==hashlib.sha256((HERE/'train.py').read_bytes()).hexdigest()
    indexed={}
    for row in ledger:
        if row['label'].startswith(('candidate/','development/')):
            key=(row['label'],row['caseId']);assert key not in indexed;indexed[key]=row
    validation=[c for j,task in enumerate(TASKS) for c in sample_cases(task,tc['validation_cases_per_task'],cfg,cfg['seed']+1000+j*100)]
    initial={row['caseId']:row for row in ledger if row['kind']=='worker_evaluation' and row['label']=='initial'}
    assert set(initial)=={c['id'] for c in validation}
    best_score=float(np.mean([initial[c['id']]['metrics']['score'] for c in validation]));best=UNTUNED.copy();best_generation=0
    rng=np.random.default_rng(cfg['seed']);mean=(UNTUNED-LOWER)/(UPPER-LOWER);sigma=np.full(len(PARAMETERS),.24)
    elite_count=max(3,round(tc['population']*tc['elite_fraction']));candidate_count=0;episode_count=0
    for generation,record in enumerate(records,1):
        assert record['generation']==generation
        cases=[c for j,task in enumerate(TASKS) for c in sample_cases(task,tc['cases_per_task'],cfg,cfg['seed']+generation*100+j*10000)]
        assert record['cases']==cases;assert len(record['candidates'])==tc['population']
        candidates=np.clip(rng.normal(mean,sigma,(tc['population'],len(mean))),0,1);candidates[0]=mean;candidates[1]=(best-LOWER)/(UPPER-LOWER);parameters=LOWER+candidates*(UPPER-LOWER)
        scores=[]
        for i,candidate in enumerate(record['candidates']):
            assert candidate['candidateIndex']==i;np.testing.assert_array_equal(candidate['parameters'],parameters[i]);assert [row['case'] for row in candidate['episodes']]==cases
            score=float(np.mean([row['metrics']['score'] for row in candidate['episodes']]));assert score==candidate['meanScore'];scores.append(score)
            digest=hashlib.sha256(np.asarray(parameters[i],dtype='<f8').tobytes()).hexdigest()
            for row in candidate['episodes']:
                logged=indexed[(f'candidate/generation-{generation:03d}/{i:03d}',row['case']['id'])]
                assert logged['parameterHash']==digest;assert logged['case']==row['case'];assert logged['metrics']==row['metrics'];assert logged['success']==row['success'];assert logged['status']==row['status'];episode_count+=1
            candidate_count+=1
        order=np.argsort(scores);elite=candidates[order[-elite_count:]];mean=.25*mean+.75*np.mean(elite,axis=0);sigma=np.maximum(.035,.25*sigma+.75*np.std(elite,axis=0))
        assert record['generationWinnerIndex']==int(order[-1]);np.testing.assert_array_equal(record['searchMean'],LOWER+mean*(UPPER-LOWER));np.testing.assert_array_equal(record['searchSigma'],sigma*(UPPER-LOWER));assert record['rngState']==json_rng_state(rng.bit_generator.state)
        development=record['developmentEvaluation'];assert [row['case'] for row in development]==validation
        for row in development:
            logged=indexed[(f'development/generation-{generation:03d}',row['case']['id'])];assert logged['metrics']==row['metrics'] and logged['status']==row['status'] and logged['success']==row['success']
        score=float(np.mean([row['metrics']['score'] for row in development]))
        if score>best_score:best=parameters[order[-1]].copy();best_score=score;best_generation=generation
        assert record['selectedAtGeneration']==best_generation;assert record['selectedDevelopmentScore']==best_score;np.testing.assert_array_equal(record['selectedParameters'],best)
        np.testing.assert_array_equal(data['trainingHistory'][generation-1]['candidateScores'],scores);np.testing.assert_array_equal(data['checkpoints'][generation-1]['parameters'],best)
    index=json.loads((HERE/'checkpoint-index.json').read_text());index['checkpoints']=[x for x in index['checkpoints'] if x['generation']<=completed];expected=list(range(tc['checkpoint_replay_interval'],completed+1,tc['checkpoint_replay_interval']))
    if completed==tc['generations'] and tc['generations'] not in expected:expected.append(tc['generations'])
    assert [x['generation'] for x in index['checkpoints']]==expected;replays=0
    for entry in index['checkpoints']:
        shard=json.loads((HERE/entry['path']).read_text());finite(shard);record=records[entry['generation']-1]
        assert shard['generation']==entry['generation'];assert shard['selectedAtGeneration']==entry['selectedAtGeneration']==record['selectedAtGeneration'];assert shard['parameters']==record['selectedParameters'];assert set(shard['tasks'])==set(TASKS)
        for task,rollout in shard['tasks'].items():
            matching=[row for row in ledger if row['kind']=='recorded_replay' and row['label']==f'checkpoint_g{entry["generation"]:03d}' and row['caseId']==rollout['case']['id']]
            assert len(matching)==1 and matching[0]['metrics']==rollout['metrics'];assert rollout['frames'];replays+=1
    previous=json.loads((HERE/data['campaign']['priorInterruptedCampaign']['archive']/'results.json').read_text());prefix=min(len(previous['trainingHistory']),len(records))
    for i in range(prefix):np.testing.assert_array_equal(previous['trainingHistory'][i]['candidateScores'],data['trainingHistory'][i]['candidateScores'])
    report={'completeCampaign':not args.partial,'campaignId':data['campaign']['id'],'generations':len(records),'candidatePolicies':candidate_count,'candidateEpisodes':episode_count,'developmentEpisodes':len(validation)*(len(records)+1),'totalEpisodeLedgerEntries':len(ledger),'checkpointShards':len(index['checkpoints']),'actualCheckpointReplays':replays,'deterministicCandidatesMatchSeedAndSearchState':True,'candidateResultsMatchEpisodeLedger':True,'selectedParametersUseOnlyFixedDevelopmentScores':True,'sourceFingerprintsMatch':True,'repeatedShortCampaignPrefix':{'generationsCompared':prefix,'candidateScoreVectorsMatchExactly':True},'priorInterruptedCampaign':data['campaign']['priorInterruptedCampaign']}
    write_json(HERE/('training-record-audit.partial.json' if args.partial else 'training-record-audit.json'),report);print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
