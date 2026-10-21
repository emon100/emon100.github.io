from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json,copy
import unified_controller as u
def execute(job):
 params,case,cfg=job;r=u.rollout(params,case,cfg,False);return {k:v for k,v in r.items() if k not in ['frames','environment']}
HERE=Path(__file__).resolve().parent
def main():
 frozen=json.loads((HERE/'evaluation-freeze.json').read_text());audit=json.loads((HERE/'step-audit.json').read_text());manifest=json.loads((HERE/'manifest.json').read_text());cases=[c for cs in frozen['cases'].values() for c in cs];one=copy.deepcopy(frozen['config']);one['physics']['dt']=.001;half=copy.deepcopy(one);half['physics']['dt']=.0005
 jobs=[('initial1ms',(frozen['parameters']['initial'],c,one)) for c in cases]+[('demo1ms',(frozen['parameters']['final'],d['case'],one)) for d in manifest['demos']]+[('changedHalfMs',(frozen['parameters']['final'],x['case'],half)) for x in audit['changedClassifications']]
 rows=[]
 with ProcessPoolExecutor(max_workers=6) as pool:
  for (group,_),r in zip(jobs,pool.map(execute,[x[1] for x in jobs])):rows.append({'auditGroup':group,**r})
 report={'rows':rows,'notice':'After two classifications changed at 1 ms, the initial policy was also rerun on all 80 cases at 1 ms for a matched comparison. Six predefined demonstrations were checked at 1 ms. Only the two changed cases were checked at 0.5 ms; no full-cohort 0.5 ms rate is claimed.'};u.core.write_json(HERE/'step-audit-followup.json',report)
 print({g:{'trials':sum(x['auditGroup']==g for x in rows),'successes':sum(x['auditGroup']==g and x['success'] for x in rows)} for g in ['initial1ms','demo1ms','changedHalfMs']},flush=True)
if __name__=='__main__':main()
