"""Audit emitted records and recompute a small paired timestep sensitivity set."""
import argparse, copy, hashlib, json
from core import *

def all_rollouts(data):
    for task,report in data['tasks'].items():
        for name,r in report['comparisons'].items():yield f'{task}/{name}',r
        for i,r in enumerate(report.get('failureExamples',[])):yield f'{task}/failure{i}',r
    if data.get('freeFailureExample'):yield 'free/failure',data['freeFailureExample']
    if data.get('campaign',{}).get('checkpointIndex'):
        index=json.loads((HERE/data['campaign']['checkpointIndex']).read_text())
        for entry in index['checkpoints']:
            shard=json.loads((HERE/entry['path']).read_text())
            for task,r in shard['tasks'].items():yield f'checkpoint{entry["generation"]}/{task}',r

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--results',default=str(HERE/'results.json'));parser.add_argument('--steps',action='store_true');args=parser.parse_args();data=json.loads(Path(args.results).read_text());finite(data);assert data['status']=='complete';cfg=data['config'];assert data['engine']['version']==mujoco.__version__=='3.15.0'
    nozzles=nozzle_layout(cfg);count=0;worst_fuel_error=0.;maximum_bound_excess=0.;comparisons=0;maximum_requested_horizontal=0.;maximum_allocated_horizontal=0.;launch_events=[]
    for label,r in all_rollouts(data):
        fuel0=r['case'].get('fuel',cfg['fuel']['initial_kg']);previous=None
        for f in r['frames']:
            count+=1;assert abs(np.linalg.norm(f['quat'])-1)<1e-6,label
            if 'geometryMinimumCG' in f:
                reference=geometry_minimum_cg(qmat(f['quat']),cfg);assert abs(f['geometryMinimumCG']-reference)<1e-9;assert abs(f['geometryClearance']-(f['pos'][2]-reference))<1e-9
            assert len(f['nozzleForces'])==12 and all(x>=-1e-9 for x in f['nozzleForces'])
            limits=np.array([n['maxForce'] for n in nozzles])*f['pressureScale']*r['case'].get('thrust_scale',1.)
            for i,n in enumerate(nozzles):
                if n['id'] in r['case'].get('disabled_nozzles',[]):limits[i]=0.
            excess=float(np.max(np.array(f['nozzleForces'])-limits));maximum_bound_excess=max(maximum_bound_excess,excess);assert excess<1e-6,(label,excess)
            assert sum(f['nozzleForces'])<=cfg['actuators']['total_force_max']*f['pressureScale']*r['case'].get('thrust_scale',1.)+1e-6
            fuel_error=abs(f['fuel']+f['fuelUsed']-fuel0);worst_fuel_error=max(worst_fuel_error,fuel_error);assert fuel_error<1e-6
            assert f['fan']['power']<=f['fan']['availableFluidPower']+1e-5
            if previous:
                assert f['t']>previous['t'];assert f['fuel']<=previous['fuel']+1e-9;assert f['pressureScale']<=previous['pressureScale']+1e-9
                R=qmat(previous['quat']);requested=R@np.array(f['allocation']['requestedWrenchBody'][:3]);allocated=R@np.array(f['allocation']['achievedWrenchBody'][:3]);maximum_requested_horizontal=max(maximum_requested_horizontal,float(np.linalg.norm(requested[:2])));maximum_allocated_horizontal=max(maximum_allocated_horizontal,float(np.linalg.norm(allocated[:2])))
                assert np.linalg.norm(requested[:2])<1e-7 and np.linalg.norm(allocated[:2])<1e-7,(label,'horizontal target')
                assert f['contact']['stableSeconds']<=previous['contact']['stableSeconds']+(f['t']-previous['t'])+1e-8
            previous=f
        if data.get('revision') in ['launch-coupled-r5','energy-coupled-r6'] and r['case']['task']!='moving_recovery':
            for prior,current in zip(r['frames'],r['frames'][1:]):
                if current['goal'].get('phase')=='flip' and abs(current['allocation']['requestedWrenchBody'][3])>10. and (data.get('revision')!='energy-coupled-r6' or current['goal'].get('commandRollRate',0)>0):
                    assert prior['contact']['wheelCount']==0 and not prior['contact']['bodyContact'] and prior['vel'][2]>0,(label,'roll was not launched during clear ascent')
                    launch_events.append({'rollout':label,'commandTime':current['controlObservationTime'],'riseAboveGroundCG':prior['pos'][2]-cfg['vehicle']['ground_cg_height'],'verticalVelocity':prior['vel'][2],'pressureRatio':float(delivery(prior['fuel'],cfg)[0]),'requestedRollTorque':current['allocation']['requestedWrenchBody'][3],'allocatedRollTorque':current['allocation']['achievedWrenchBody'][3]})
                    break
        if r['success']:
            assert r['frames'][-1]['contact']['stableSeconds']+1e-9>=cfg['landing']['stable_seconds'];assert not r['metrics']['hardContact'];assert not r['metrics']['bodyContact'];assert r['metrics']['turnComplete'];assert r['metrics']['peakHeight']+1e-9>=cfg['tasks'][r['case']['task']].get('minimum_peak_cg_height',0.)
        comparisons+=1
    for task,report in data['tasks'].items():
        cases=[r['case'] for r in report['comparisons'].values()];assert all(c==cases[0] for c in cases[1:]),task
        ref=[r['case'] for r in report['holdout']['episodes']]
        for name,baseline in report['baselineEvaluation'].items():assert [r['case'] for r in baseline['episodes']]==ref,(task,name)
    audit={'revision':data['revision'],'campaignId':data.get('campaign',{}).get('id'),'sourceSha256':data['sourceSha256'],'controllerSha256':data['controllerSha256'],'trainerSha256':data.get('trainerSha256'),'resultsSha256':hashlib.sha256(Path(args.results).read_bytes()).hexdigest(),'selectedPolicyGeneration':data['tasks'][TASKS[0]]['policies']['final']['generation'],'engine':mujoco.__version__,'rolloutsAudited':comparisons,'framesAudited':count,'maxFuelAccountingError':worst_fuel_error,'maxNozzleLimitExcess':maximum_bound_excess,'maximumRequestedHorizontalForce':maximum_requested_horizontal,'maximumAllocatedHorizontalForce':maximum_allocated_horizontal,'maximumRealizedHorizontalJetForce':max(r['metrics']['peakJetHorizontalForce'] for task in TASKS for r in data['tasks'][task]['holdout']['episodes']),'fullCourseMonotonePressure':'passed','launchGateEvents':launch_events,'pairedCaseComparison':'passed','allValuesFinite':True,'sourceMatches':data['sourceSha256']==hashlib.sha256((HERE/'core.py').read_bytes()).hexdigest()}
    audit['actualHeightGoalEvaluation']={task:{'minimumCG':cfg['tasks'][task]['minimum_peak_cg_height'],'reachedMinimum':sum(row['metrics']['minimumHeightMet'] for row in report['holdout']['episodes']),'stableLandingSuccesses':report['holdout']['successCount'],'trials':report['holdout']['trials'],'minimumObservedPeak':min(row['metrics']['peakHeight'] for row in report['holdout']['episodes']),'maximumObservedPeak':max(row['metrics']['peakHeight'] for row in report['holdout']['episodes'])} for task,report in data['tasks'].items() if cfg['tasks'][task].get('minimum_peak_cg_height',0)>0}
    assert audit['sourceMatches']
    if args.steps:
        from concurrent.futures import ProcessPoolExecutor
        from train import init_worker, evaluate_job
        params=data['tasks'][TASKS[0]]['policies']['final']['parameters'];rows=[]
        for task in TASKS:
            cases=[e['case'] for e in data['tasks'][task]['holdout']['episodes']];baseline=[e['success'] for e in data['tasks'][task]['holdout']['episodes']];fine=copy.deepcopy(cfg);fine['physics']['dt']=.001
            with ProcessPoolExecutor(max_workers=4,initializer=init_worker,initargs=(fine,)) as pool:
                reruns=[r for batch in pool.map(evaluate_job,[(params,cases[i:i+4],'final') for i in range(0,len(cases),4)]) for r in batch]
            rows.append({'task':task,'dt':.001,'cases':len(cases),'successes':sum(r['success'] for r in reruns),'changedClassifications':sum(r['success']!=a for r,a in zip(reruns,baseline)),'changedFailureStatuses':sum(r['status']!=a['status'] for r,a in zip(reruns,data['tasks'][task]['holdout']['episodes'])),'maximumPeakHeightDifference':max(abs(r['metrics']['peakHeight']-a['metrics']['peakHeight']) for r,a in zip(reruns,data['tasks'][task]['holdout']['episodes'])),'statuses':[r['status'] for r in reruns]})
            if any(r['success']!=a for r,a in zip(reruns,baseline)):
                # Refine the original success cases around this contact threshold.
                finer=copy.deepcopy(cfg);finer['physics']['dt']=.0005
                for original,medium in zip(data['tasks'][task]['holdout']['episodes'],reruns):
                    if not original['success']:continue
                    refined=rollout(task,params,original['case'],finer)
                    audit.setdefault('contactBoundaryCases',[]).append({'caseId':original['case']['id'],'steps':[{'dt':.002,'status':original['status'],'metrics':original['metrics']},{'dt':.001,'status':medium['status'],'metrics':medium['metrics']},{'dt':.0005,'status':refined['status'],'metrics':refined['metrics']}]})
        audit['stepSensitivity']=rows
    write_json(HERE/'audit.json',audit);print(json.dumps(audit,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
