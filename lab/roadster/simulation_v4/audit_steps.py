from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import json,copy
import unified_controller as u
HERE=Path(__file__).resolve().parent
def execute(job):
 p,c,cfg=job;r=u.rollout(p,c,cfg,False);return {k:v for k,v in r.items() if k not in ['frames','environment']}
def main():
 source=json.loads((HERE/'evaluation-freeze.json').read_text());base=json.loads((HERE/'evaluation.json').read_text());cfg=copy.deepcopy(source['config']);cfg['physics']['dt']=.001;cases=[c for cs in source['cases'].values() for c in cs]
 baseline={r['case']['id']:r for r in base['rows'] if r['policy']=='final'}
 with ProcessPoolExecutor(max_workers=6) as pool:rows=list(pool.map(execute,[(source['parameters']['final'],c,cfg) for c in cases]))
 changed=[{'case':r['case'],'twoMs':baseline[r['case']['id']]['status'],'oneMs':r['status']} for r in rows if r['status']!=baseline[r['case']['id']]['status']]
 result={'dt':.001,'trials':len(rows),'twoMsSuccesses':sum(r['success'] for r in baseline.values()),'oneMsSuccesses':sum(r['success'] for r in rows),'changedClassifications':changed,'rows':rows,'notice':'Only final parameters rechecked at 1 ms. These cases were not used to select parameters. Failure tails differ because this audit stops at first failure; compare success and first failure classification, not terminal fuel or tail maxima.'}
 u.core.write_json(HERE/'step-audit.json',result);print({k:v for k,v in result.items() if k not in ['rows','changedClassifications']},flush=True)
if __name__=='__main__':main()
