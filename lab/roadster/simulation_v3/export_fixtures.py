"""Exact observations, memory and commands for native/JS controller parity."""
from pathlib import Path
import json,copy,hashlib,time
import numpy as np
from evaluate import HERE,core,rc,cases,verify,digest

def jsonable(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if isinstance(v,dict):return {k:jsonable(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [jsonable(x) for x in v]
 return v

def memory(policy):return jsonable({k:v for k,v in policy.__dict__.items() if k not in ['p','task','cfg','goal','error','c']})
def main():
 cfg,manifest,_,_=cases();source=json.loads((HERE/'development/002/results.json').read_text());traces=[];current=[]
 class TracePolicy(rc.RecoveryPolicy):
  def command(self,s,contact):
   before=memory(self);t=super().command(s,contact);current.append({'state':jsonable(s),'contact':copy.deepcopy(contact),'memoryBefore':before,'forceWorld':t[0].tolist(),'torqueBody':t[1].tolist(),'fanCommand':float(t[2]),'memoryAfter':memory(self),'goal':copy.deepcopy(self.goal),'error':copy.deepcopy(self.error)});return t
 class TraceWorld(rc.RecoveryWorld):
  def allocate(self,wrench):
   u,a=super().allocate(wrench);current[-1]['allocationInputWrench']=np.asarray(wrench).tolist();current[-1]['nozzleThrottle']=u.tolist();current[-1]['allocation']=a;return u,a
 original=core.FeedbackPolicy;core.FeedbackPolicy=TracePolicy
 try:
  for row in source['rows']:
   current=[];world=TraceWorld(cfg);r=core.rollout('moving_recovery',manifest['parameters'],row['case'],cfg,record=True,world=world);traces.append({'case':row['case'],'status':r['status'],'success':r['success'],'metrics':r['metrics'],'finalState':jsonable(world.state()),'finalTime':float(world.data.time),'steps':current});print(row['case']['id'],len(current),r['status'],flush=True)
 finally:core.FeedbackPolicy=original
 result={'revision':manifest['revision'],'config':cfg,'parameters':manifest['parameters'],'settings':manifest['settings'],'sourceSha256':manifest['sourceSha256'],'controllerSha256':manifest['controllerSha256'],'predictorSha256':manifest['predictorSha256'],'allocatorSha256':manifest['allocatorSha256'],'sequences':traces,'memoryExcludedConstants':['p','task','cfg','c'],'notice':'Each state is the exact native observation before a 20ms control window. P/D arrays and forecast are outputs; physical replay still uses real MuJoCo.'};core.write_json(HERE/'browser-fixtures.json',result);verify(manifest)
if __name__=='__main__':main()
