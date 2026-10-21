"""Assert exported frames, actual evaluation counts, wrench decomposition and power budgets."""
import json
import math
from pathlib import Path
import numpy as np
from core import HERE, matrix


def audit(path=HERE/'results.json'):
    data=json.loads(Path(path).read_text());assert data['status']=='complete'
    def finite(value):
        if isinstance(value,dict):
            for child in value.values():finite(child)
        elif isinstance(value,list):
            for child in value:finite(child)
        elif isinstance(value,(int,float)):assert math.isfinite(value)
    finite(data)
    cfg=data['config'];nozzles=cfg['actuator']['virtual_nozzles']
    directions=np.array([n['direction'] for n in nozzles]);positions=np.array([n['position'] for n in nozzles])
    max_forces=np.array([n['maxForce'] for n in nozzles]);frames_checked=0;episodes_checked=0
    def check_rollout(r):
        nonlocal frames_checked
        if not r:return
        previous=cfg['vehicle']['gas_mass'];authority=r['environment']['thrustScale']
        assert r['frames'], 'Every recorded failure must retain its initial pose'
        for f in r['frames']:
            frames_checked+=1
            assert f['gasLeft']>=-1e-9 and f['gasLeft']<=previous+1e-9;previous=f['gasLeft']
            assert abs(np.linalg.norm(f['quat'])-1)<1e-7
            jets=np.array(f['nozzleForces']);a=f['allocation'];scale=a['pressureScale']*authority
            assert (jets>=-1e-9).all() and (jets<=max_forces*scale+1e-6).all()
            np.testing.assert_allclose(jets@directions,a['forceBody'],atol=1e-6)
            np.testing.assert_allclose(jets@np.cross(positions,directions),a['torqueBody'],atol=1e-6)
            np.testing.assert_allclose(matrix(np.array(f['actuatorQuat']))@np.array(a['forceBody']),f['thrust'],atol=1e-6)
            assert abs(jets.sum()-a['totalPositiveJetForce'])<1e-6
            assert jets.sum()<=cfg['actuator']['total_jet_force_max']*scale+1e-6
            np.testing.assert_allclose(np.array(f['fanSuctionForce'])+f['fanAxialForce'],f['fanForce'],atol=1e-6)
            assert abs(f['fanPressurePower']+f['fanAxialPower']-f['fanPower'])<1e-6
            assert f['fanPower']<=f['fanAvailablePower']+1e-6
    for task,report in data['tasks'].items():
        count=0;best=-math.inf
        for metric in report['metrics']:
            episode_scores=np.array(metric['candidateEpisodeScores']);count+=episode_scores.size
            assert metric['evaluations']==count
            candidate_scores=episode_scores.mean(axis=1)
            np.testing.assert_allclose(candidate_scores,metric['candidateScores'])
            best=max(best,float(candidate_scores.max()));assert abs(best-metric['best'])<1e-7
        episodes_checked+=count
        for r in report['rollouts'].values():check_rollout(r)
        check_rollout(report['holdout']['failureExample'])
    for report in data['pressureSensitivity']['tasks'].values():check_rollout(report['failureExample'])
    result={'framesChecked':frames_checked,'trainingEpisodesChecked':episodes_checked,'finiteNumbers':True,'wrenchAndGasLimits':True,'fanPowerBudget':True,'evaluationCounts':True}
    print(json.dumps(result))
    return result

if __name__=='__main__':audit()
