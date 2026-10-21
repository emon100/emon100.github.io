"""Extract distinct launch events from actual, revision-specific recorded trajectories."""
from pathlib import Path
import argparse, copy, hashlib, importlib.util, json, math, sys
import numpy as np
import core

HERE=Path(__file__).resolve().parent
ARCHIVE=HERE/'archive'/'launch-coupled-r5'
R4=HERE/'archive'/'attitude-only-blowdown-r4'
TORQUE_THRESHOLD=10.0
MOTION_THRESHOLD=.05

def module_at(path):
    spec=importlib.util.spec_from_file_location('archived_launch_core',path);mod=importlib.util.module_from_spec(spec);previous=sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode=True;spec.loader.exec_module(mod)
    finally:sys.dont_write_bytecode=previous
    return mod

def read(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def measure_recording(rollout,cfg,params,parameter_names,task,label):
    """Commands use the prior observation; actuator and motion events use frame time."""
    frames=rollout['frames'];case=rollout['case'];A=core.wrench_matrix(rollout['environment']['nozzles'])
    parameters=dict(zip(parameter_names,params));events={k:None for k in ['takeoff','rollPhaseEntry','firstNonzeroRollCommand','firstRealizedRollTorque','firstRollMotion','firstAscentApex','firstReturnContact','energyReadyEvent']}
    def event(f,t=None):
        return {'t':float(f['t'] if t is None else t),'heightCG':float(f['pos'][2]),'riseAboveGroundCG':float(f['pos'][2]-cfg['vehicle']['ground_cg_height']),'riseAboveInitialCG':float(f['pos'][2]-case['pos'][2]),'verticalVelocity':float(f['vel'][2]),'pressureRatio':float(core.delivery(f['fuel'],cfg)[0]),'fuel':float(f['fuel']),'wheelCount':int(f['contact']['wheelCount']),'bodyRollRate':float(f['omega'][0])}
    previous=frames[0]
    for i,f in enumerate(frames):
        goal=f['goal'];obs=previous if i else f
        command_time=float(f.get('controlObservationTime',obs['t']))
        if abs(command_time-float(obs['t']))>1e-6:raise ValueError('Recorded control observation does not match previous physical frame')
        if events['takeoff'] is None and f['contact']['wheelCount']==0 and f['vel'][2]>0 and f['pos'][2]>case['pos'][2]+.001:events['takeoff']=event(f)
        if events['takeoff'] and events['firstAscentApex'] is None and previous['vel'][2]>0>=f['vel'][2]:events['firstAscentApex']=event(f)
        requested=float(f['allocation']['requestedWrenchBody'][3]);allocated=float(f['allocation']['achievedWrenchBody'][3]);realized=float((A@np.array(f['nozzleForces']))[3])
        # r4/r5 recorded no commandRollRate field. Recover the reference from their
        # archived state-feedback formula, never from the r6 controller.
        if 'commandRollRate' in goal:
            rate=float(goal['commandRollRate']);rate_source='recorded_goal'
        else:
            rate=min(parameters['roll_speed'],math.sqrt(2*parameters['roll_brake_accel']*max(0.,goal['turnRemaining']))) if goal['phase']=='flip' else 0.
            rate_source='archived_r4_r5_feedback_formula'
        def command_event():
            e=event(obs,command_time);e.update(requestedRollTorque=requested,allocatedRollTorque=allocated,realizedRollTorqueBeforeCommand=float((A@np.array(obs['nozzleForces']))[3]),realizedRollTorqueAfterFirstControlStep=realized,commandRollRate=rate,commandRollRateSource=rate_source,gateParameter=parameters['flip_start_clearance'],geometryLimited=bool(goal.get('geometryLimited',False)))
            return e
        if goal['phase']=='flip' and events['rollPhaseEntry'] is None:events['rollPhaseEntry']=command_event()
        if goal['phase']=='flip' and abs(requested)>TORQUE_THRESHOLD and rate>0 and events['firstNonzeroRollCommand'] is None:events['firstNonzeroRollCommand']=command_event()
        if abs(realized)>TORQUE_THRESHOLD and events['firstRealizedRollTorque'] is None:
            events['firstRealizedRollTorque']=event(f);events['firstRealizedRollTorque']['realizedRollTorque']=realized
        if abs(f['omega'][0])>MOTION_THRESHOLD and events['firstRollMotion'] is None:events['firstRollMotion']=event(f)
        if goal.get('energyReady',False) and events['energyReadyEvent'] is None:
            events['energyReadyEvent']=event(obs,command_time);events['energyReadyEvent'].update({k:float(goal[k]) for k in ['energyHeight','estimatedRollTime','availableCoastTime']})
        if events['takeoff'] and events['firstReturnContact'] is None and (f['contact']['wheelCount']>0 or f['contact']['bodyContact']):events['firstReturnContact']=event(f)
        previous=f
        # Exclude the extra post-failure display tail from launch/flight evidence.
        if f['status'] not in ['timeout','success']:break
    takeoff=events['takeoff'];phase=events['rollPhaseEntry'];command=events['firstNonzeroRollCommand'];apex=events['firstAscentApex']
    return {'task':task,'policy':label,'case':copy.deepcopy(case),'parameters':list(map(float,params)),**events,
        'firstRollCommand':copy.deepcopy(phase),'firstRollCommandDeprecatedAlias':'rollPhaseEntry',
        'delayAfterTakeoff':float(phase['t']-takeoff['t']) if phase and takeoff else None,
        'nonzeroCommandDelayAfterTakeoff':float(command['t']-takeoff['t']) if command and takeoff else None,
        'rollCommandBeforeFirstApex':bool(command and (not apex or command['t']<apex['t'])),
        'observedPeakCGHeight':float(rollout['metrics']['peakHeight']),'status':rollout['status'],'sourceFrameCount':len(frames)}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--old-only',action='store_true');args=ap.parse_args()
    history=read(ARCHIVE/'launch-timing-audit.json');r4=read(R4/'results.json');r5=read(ARCHIVE/'results.json');old_comparisons=read(ARCHIVE/'launch-comparisons.json');r5_policies=read(ARCHIVE/'policies.json')
    cache_path=HERE/'history-timing-replays.json'
    cache=read(cache_path) if cache_path.exists() else {'notice':'Four additional diagnostic replays of archived r4 parameter vectors under the frozen r5 early gate. These were not CEM evaluations and are not included in either campaign ledger.','sourceSha256':sha(ARCHIVE/'core.py'),'replays':{}}
    if cache['sourceSha256']!=sha(ARCHIVE/'core.py'):raise ValueError('Historical replay source mismatch')
    archived=None;rows=[]
    for row in history['rows']:
        comparison,task,label=row['comparison'],row['task'],row['policy']
        cfg=r4['config'] if comparison=='r4_original' else r5['config']
        if comparison=='r4_original':rollout=r4['tasks'][task]['comparisons'][label];source=str((R4/'results.json').relative_to(HERE));source_sha=r4['sourceSha256']
        elif comparison=='r4_parameters_with_r5_launch_gate_only':
            source_sha=r5['sourceSha256']
            if label=='final':rollout=old_comparisons['tasks'][task]['early_gate_only'];source=str((ARCHIVE/'launch-comparisons.json').relative_to(HERE))
            else:
                key=f'{task}/{label}'
                if key not in cache['replays']:
                    archived=archived or module_at(ARCHIVE/'core.py')
                    cache['replays'][key]=archived.rollout(task,row['parameters'],row['case'],cfg,record=True,policy_id=label)
                    core.write_json(cache_path,cache)
                rollout=cache['replays'][key];source='history-timing-replays.json'
        else:rollout=r5['tasks'][task]['comparisons'][label];source=str((ARCHIVE/'results.json').relative_to(HERE));source_sha=r5['sourceSha256']
        measured=measure_recording(rollout,cfg,row['parameters'],r5_policies['policies']['final']['parameterNames'],task,label)
        rows.append({'comparison':comparison,**measured,'trajectorySource':source,'sourceSha256':source_sha})
    current=read(HERE/'results.json')
    if not args.old_only and current.get('revision')=='energy-coupled-r6':
        for task in ['stationary_kickflip','moving_kickflip']:
            for label,rollout in current['tasks'][task]['comparisons'].items():
                if label not in ['untuned','handcrafted','mid','final']:continue
                if label=='untuned':params=core.UNTUNED
                elif label=='handcrafted':params=core.HANDCRAFTED
                else:
                    params=current['tasks'][task]['policies'][label]['parameters']
                rows.append({'comparison':'r6_learned' if label in ['mid','final'] else 'r6_baseline',**measure_recording(rollout,current['config'],params,core.PARAMETERS,task,label),'trajectorySource':'results.json','sourceSha256':current['sourceSha256']})
    report={'revision':'energy-coupled-r6','observationIntervalSeconds':.02,'takeoffDefinition':history['takeoffDefinition'],'eventDefinitions':{
        'rollPhaseEntry':'First control observation whose command is in flip phase. This is a state-machine event, not evidence of nonzero commanded or realized roll.',
        'firstNonzeroRollCommand':'First flip command with abs(requested nozzle body-X torque)>10 Nm and positive roll-rate reference; timestamp and state are the pre-command observation.',
        'firstRealizedRollTorque':'First actual frame with abs(body-X moment of the 12 recorded nozzle forces)>10 Nm; excludes fan/contact moments.',
        'firstRollMotion':'First actual frame with abs(body-X angular velocity)>0.05 rad/s.',
        'firstRollCommand':'Deprecated alias for rollPhaseEntry. Do not label this as physical roll onset.'},
        'thresholds':{'rollTorqueNm':TORQUE_THRESHOLD,'rollRateRadiansPerSecond':MOTION_THRESHOLD},
        'historyNotice':'Events are extracted from revision-specific raw recordings. Four older gate-only baselines without saved frames are replayed once using archived r5 source and cached separately, never with the 15-parameter r6 controller. Command and motion onset are sampled at 20 ms, not exact substep times.',
        'historicalDiagnosticReplayCount':len(cache['replays']),'currentSourceSha256':sha(HERE/'core.py'),'historicalSourceSha256':history['historicalSourceSha256'],'rows':rows}
    comparisons=copy.deepcopy(old_comparisons)
    for task in ['stationary_kickflip','moving_kickflip']:
        old=copy.deepcopy(r5['tasks'][task]['comparisons']['final']);policy=r5_policies['policies']['final'];old.update(policyId='r5_low_height',parameters=policy['parameters'],parameterNames=policy['parameterNames'],generation=policy['generation'],trainedUnderRevision='launch-coupled-r5',sourceSha256=r5['sourceSha256']);comparisons['tasks'][task]['r5_low_height']=old
    comparisons['notice']='Historical fixed-case replay evidence. r4 and gate-only final A/B are imported from the r5 archive; r5_low_height is its original 14-parameter final rollout. None is rerun or counted as r6 training.'
    core.write_json(HERE/'launch-comparisons.json',comparisons);core.write_json(HERE/'launch-timing-audit.json',report)
    for row in rows:
        if not row['comparison'].startswith('r6_'):continue
        print(row['comparison'],row['task'],row['policy'],{k:row[k]['t'] if row[k] else None for k in ['takeoff','rollPhaseEntry','firstNonzeroRollCommand','firstRealizedRollTorque','firstRollMotion']})

if __name__=='__main__':main()
