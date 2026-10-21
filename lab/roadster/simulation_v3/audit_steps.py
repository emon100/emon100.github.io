"""Whole fixed evaluation suite at 1ms; no policy changes or selection."""
from pathlib import Path
import json,time,copy,collections
from evaluate import HERE,core,rc,cases,verify,execute,digest,summarize

def main():
 cfg,manifest,plan,_=cases();out=HERE/'step-audit.json'
 if out.exists():raise SystemExit('Step audit exists; not overwritten')
 changed=copy.deepcopy(cfg);changed['physics']['dt']=.001;report={'revision':manifest['revision'],'controllerSha256':manifest['controllerSha256'],'allocatorSha256':manifest['allocatorSha256'],'sourceSha256':manifest['sourceSha256'],'casesSha256':digest(HERE/'evaluation-cases.json'),'status':'running','dt':.001,'controlDt':.02,'policySelection':'none','evaluationCount':0,'groups':{},'errors':[]};start=time.perf_counter()
 for group,group_cases in plan['groups'].items():
  rows=[];report['groups'][group]={'episodes':rows}
  for i,case in enumerate(group_cases):
   try:r=execute(case,changed,manifest['parameters'],'candidate',record=False);r.pop('frames');r.pop('environment');rows.append(r)
   except Exception as e:rows.append({'case':case,'status':'computation_error','success':False,'error':repr(e)});report['errors'].append(rows[-1])
   report['evaluationCount']+=1;report['groups'][group]['summary']=summarize(rows);report['wallSeconds']=time.perf_counter()-start;core.write_json(out,report)
   print(json.dumps({'group':group,'case':i,'status':rows[-1]['status'],'done':report['evaluationCount'],'success':report['groups'][group]['summary']['successCount']}),flush=True)
 verify(manifest);report['status']='complete';core.write_json(out,report)
if __name__=='__main__':main()
