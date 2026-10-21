"""Compare completed2ms and1ms suites; resolve classification changes at0.5ms."""
from pathlib import Path
import json,copy,time,collections
from evaluate import HERE,core,execute,verify,digest,summarize

def main():
 m=json.loads((HERE/'manifest.json').read_text());verify(m);r=json.loads((HERE/'results.json').read_text());a=json.loads((HERE/'step-audit.json').read_text());assert r['status']==a['status']=='complete';cfg=copy.deepcopy(r['config']);cfg['physics']['dt']=.0005;out=HERE/'step-comparison.json'
 if out.exists():raise SystemExit('Step comparison exists')
 report={'revision':m['revision'],'sourceSha256':m['sourceSha256'],'controllerSha256':m['controllerSha256'],'allocatorSha256':m['allocatorSha256'],'predictorSha256':m['predictorSha256'],'casesSha256':digest(HERE/'evaluation-cases.json'),'status':'running','notice':'The frozen policy uses the plant timestep for its fixed-attitude preview; this audit refines both to1ms/0.5ms while keeping20ms feedback. Only success-classification changes are replayed at0.5ms. No tuning.','groups':{},'changed':[],'extraEvaluations':0}
 for group,v in r['groups'].items():
  other={x['case']['id']:x for x in a['groups'][group]['episodes']};records=[]
  for row in v['candidate']:
   at1=other[row['case']['id']]
   if row['success']!=at1['success']:
    start=time.perf_counter();fine=execute(row['case'],cfg,m['parameters'],'candidate',record=True);path=f"evaluation/{group}-halfms-{len(report['changed']):03}.json";core.write_json(HERE/path,fine);record={'group':group,'case':row['case'],'dt2ms':{'status':row['status'],'success':row['success'],'metrics':row.get('metrics')},'dt1ms':{'status':at1['status'],'success':at1['success'],'metrics':at1.get('metrics')},'dtHalfms':{'status':fine['status'],'success':fine['success'],'metrics':fine['metrics']},'rolloutPath':path,'rolloutSha256':digest(HERE/path),'wallSeconds':time.perf_counter()-start};records.append(record);report['changed'].append(record);report['extraEvaluations']+=1;print(group,row['case']['id'],row['status'],at1['status'],fine['status'],flush=True);core.write_json(out,report)
  report['groups'][group]={'dt2msSummary':v['candidateSummary'],'dt1msSummary':a['groups'][group]['summary'],'successClassificationChanges':len(records),'successfulAtBoth2msAnd1ms':sum(x['success'] and other[x['case']['id']]['success'] for x in v['candidate'])}
 report['status']='complete';verify(m);core.write_json(out,report)
if __name__=='__main__':main()
