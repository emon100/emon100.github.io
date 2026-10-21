"""Fresh seeded tests after parameter freeze. All policies use the same plant and rules."""
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json,copy,gzip,hashlib,time
import numpy as np
import unified_controller as u
import train
HERE=Path(__file__).resolve().parent
def execute(job):
 group,label,parameters,case,cfg=job;result=u.rollout(parameters,case,cfg,True)
 name=f'{group}-{case["id"]}-{label}.json.gz';raw=json.dumps(result,separators=(',',':')).encode()
 with gzip.open(HERE/'evaluation'/name,'wb',compresslevel=6) as f:f.write(raw)
 return {'group':group,'policy':label,'path':name,'sha256':hashlib.sha256(raw).hexdigest(),**{k:v for k,v in result.items() if k not in ['frames','environment']}}
def stats(rows):
 return {'successCount':sum(r['success'] for r in rows),'trials':len(rows),'horizontalMedian':float(np.median([r['metrics']['airborneHorizontalChange'] for r in rows])),'horizontalP95':float(np.percentile([r['metrics']['airborneHorizontalChange'] for r in rows],95)),'maxHorizontalChange':max(r['metrics']['airborneHorizontalChange'] for r in rows)}
def main():
 report=json.loads((HERE/'campaign/results.json').read_text())
 assert report['status']=='training_complete'
 assert report['controllerSha256']==hashlib.sha256((HERE/'unified_controller.py').read_bytes()).hexdigest()
 folder=HERE/'evaluation';folder.mkdir(exist_ok=False)
 params={'initial':report['initial'],'reference':report['reference'],'final':report['selectedParameters']};groups={'inDomain':train.cases(report['holdoutSeeds']['inDomain'],40),'stress':train.cases(report['holdoutSeeds']['stress'],40,True)}
 frozen={'controllerSha256':report['controllerSha256'],'parameters':params,'cases':groups,'config':train.CFG};u.core.write_json(HERE/'evaluation-freeze.json',frozen)
 jobs=[(group,label,p,c,train.CFG) for group,cs in groups.items() for label,p in params.items() for c in cs];rows=[];start=time.perf_counter()
 with ProcessPoolExecutor(max_workers=6) as pool:
  for r in pool.map(execute,jobs):
   rows.append(r)
   with (HERE/'evaluation-ledger.jsonl').open('a') as f:f.write(json.dumps(r,separators=(',',':'))+'\n')
   if len(rows)%20==0:print('evaluation',len(rows),'/',len(jobs),flush=True)
 summaries=[]
 for group,label in [('inDomain','7–14 m 留出'),('stress','1–18 m 宽范围')]:
  for turns in [0,1]:summaries.append({'group':group,'turns':turns,'label':label+' · '+('一圈' if turns else '回正'),**{policy:stats([r for r in rows if r['group']==group and r['policy']==policy and r['case']['goal']['turns']==turns]) for policy in params}})
 result={'revision':report['revision'],'status':'complete','controllerSha256':report['controllerSha256'],'summaries':summaries,'rows':rows,'seconds':time.perf_counter()-start,'notice':'每组每种目标 20 个新 seed 初态，冻结参数后执行。宽范围含低高度和较大角速度，不保证全部可恢复。水平 Δv 为首次接触前的最大值；所有失败也计入分布。人工参考参数单列在下载数据中。'}
 u.core.write_json(HERE/'evaluation.json',result);u.core.write_json(HERE.parent/'site/dist/simulation_v4/evaluation.json',result)
 print(json.dumps(summaries,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
