"""Post-evaluation observational audit; never selects or modifies policies."""
from pathlib import Path
import json,math,collections,hashlib
import numpy as np
from evaluate import HERE,core,verify,digest

def observation(row,cfg):
 frames=row['frames'];limit=cfg['landing'];failure=None;previous=None
 for f in frames:
  flags={'body_contact':bool(f['contact']['bodyContact']),'closing_speed_limit':f['contact']['maxClosingSpeed']>limit['max_contact_closing_speed'],'normal_force_limit':f['contact']['totalNormalForce']>f['mass']*cfg['physics']['gravity']*limit['max_normal_force_weight_ratio']}
  if any(flags.values()):failure=(f,previous,flags);break
  previous=f
 if failure is None:return {'category':'success' if row['success'] else row['status']}
 f,prev,flags=failure;R=core.qmat(np.array(f['quat']));R0=core.qmat(np.array(prev['quat'])) if prev else R;heading=math.atan2(R[1,0],R[0,0]);target=f['goal'].get('targetYaw',heading);category='body_contact' if flags['body_contact'] else ('closing_speed_limit' if flags['closing_speed_limit'] else 'normal_force_limit_only')
 return {'category':category,'flags':flags,'firstFailingWindowEnd':f['t'],'CGHeight':f['pos'][2],'geometryClearance':f['geometryClearance'],'verticalSpeed':f['vel'][2],'tiltRadians':math.acos(np.clip(R[2,2],-1,1)),'angularSpeed':float(np.linalg.norm(f['omega'])),'yawErrorRadians':(target-heading+math.pi)%(2*math.pi)-math.pi,'fuelLeft':f['fuel'],'pressureScale':f['pressureScale'],'requestedVerticalForce':float((R0@np.array(f['allocation']['requestedWrenchBody'][:3]))[2]),'allocatedVerticalForceAtWindowStart':float((R0@np.array(f['allocation']['achievedWrenchBody'][:3]))[2]),'actualVerticalForceAtWindowEnd':f['jetForceWorld'][2],'contactClosingSpeed':f['contact']['maxClosingSpeed'],'totalNormalForce':f['contact']['totalNormalForce'],'wheelSlipSpeed':f['contact']['maxWheelSlipSpeed'],'uprightRecovered':f['goal'].get('uprightRecovered',False),'headingSettled':f['goal'].get('headingSettled',False),'notice':'Observed limit crossing, not an isolated causal diagnosis; time is end of a20ms control window.'}

def main():
 manifest=json.loads((HERE/'manifest.json').read_text());verify(manifest);result=json.loads((HERE/'results.json').read_text());assert result['status']=='complete';cfg=result['config'];rows=[];groups={}
 for name,group in result['groups'].items():
  selected=[]
  for record in group['candidate']:
   if 'rolloutPath' not in record:continue
   r=json.loads((HERE/record['rolloutPath']).read_text());obs=observation(r,cfg);entry={'group':name,'caseId':r['case']['id'],'status':r['status'],'success':r['success'],'rolloutPath':record['rolloutPath'],'rolloutSha256':record['rolloutSha256'],'observation':obs};rows.append(entry);selected.append(entry)
  groups[name]={'counts':dict(collections.Counter(r['observation']['category'] for r in selected)),'failedButUprightRecovered':sum(not r['success'] and r['observation'].get('uprightRecovered',False) for r in selected),'total':len(selected)}
 audit={'revision':manifest['revision'],'controllerSha256':manifest['controllerSha256'],'resultsSha256':digest(HERE/'results.json'),'groups':groups,'rows':rows,'notice':'Categories report measured contact-limit crossings. They do not assert that a single cause explains every failure.'};core.write_json(HERE/'failure-audit.json',audit)
 result['baselineReuse']={'path':'../simulation_v2/archive/energy-coupled-r6-100-generations/results.json','sha256':digest(HERE.parent/'simulation_v2/archive/energy-coupled-r6-100-generations/results.json'),'groups':['old_fixed_recovery','old_fixed_SO3'],'newEvaluationsCharged':0,'failureMetricWindowNotice':'Reused r6 failures stop at first failing window (record=False); new formal recordings retain0.4s after failure. Success/status are comparable. Failed fuelUsed/duration/final states/peak contact metrics must not be interpreted as matched-window savings. New-seed baseline/candidate and eight development comparisons both use record=True.'};result['evaluationScriptSha256']=digest(HERE/'evaluate.py');result['caseLedgerPath']='evaluation-cases.json';result['failureAuditPath']='failure-audit.json';core.write_json(HERE/'results.json',result)
 # Record the final results hash after adding provenance metadata.
 audit['resultsSha256']=digest(HERE/'results.json');core.write_json(HERE/'failure-audit.json',audit)
 manifest['resultsSha256']=digest(HERE/'results.json');manifest['comparisonPath']='comparisons.json';manifest['comparisonsSha256']=digest(HERE/'comparisons.json');manifest['failureAuditPath']='failure-audit.json';core.write_json(HERE/'manifest.json',manifest)
 print(json.dumps(groups,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
