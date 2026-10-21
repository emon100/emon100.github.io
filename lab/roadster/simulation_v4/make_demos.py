from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json,gzip
import unified_controller as u
import train
HERE=Path(__file__).resolve().parent
def execute(job):
 label,params,c=job;r=u.rollout(params,c,train.CFG,True);return label,r
def main():
 m=json.loads((HERE.parent/'site/dist/simulation_v4/manifest.json').read_text());assert m['status']=='training_complete';folder=HERE/'demos';folder.mkdir(exist_ok=False);jobs=[(label,m[label],d['case']) for d in m['demos'] for label in ['initial','reference','final']];rows=[]
 with ProcessPoolExecutor(max_workers=6) as pool:
  for label,r in pool.map(execute,jobs):
   name=r['case']['id']+'-'+label+'.json.gz'
   with gzip.open(folder/name,'wt',compresslevel=9) as f:json.dump(r,f,separators=(',',':'))
   rows.append({'policy':label,'path':name,**{k:v for k,v in r.items() if k not in ['frames','environment']}});print(r['case']['id'],label,r['status'],flush=True)
 u.core.write_json(HERE/'demos.json',{'notice':'Six fixed demonstration cases were specified before final parameter selection. They are not a generalization rate.','rows':rows})
if __name__=='__main__':main()
