"""CEM trains one parameter vector on both commanded-turn and no-turn airborne cases."""
from pathlib import Path
import json,copy,time,hashlib,os,datetime
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import unified_controller as ctl
HERE=Path(__file__).resolve().parent
CFG=json.loads((HERE/'config.json').read_text());W=None

def cases(seed,count,stress=False):
 rng=np.random.default_rng(seed);out=[]
 for i in range(count):
  turns=i%2;heading=rng.uniform(-np.pi,np.pi);q=ctl.core.euler_q(rng.uniform(-.35,.35),rng.uniform(-.25,.25),heading)
  if turns==0 or stress or rng.random()<.25:
   q=rng.normal(size=4);q/=np.linalg.norm(q)
  speed=rng.uniform(0,8);v=[speed*np.cos(heading),speed*np.sin(heading),rng.uniform(-2,2)];w=rng.uniform(-.4,.4,3)
  if stress:v=rng.uniform(-8,8,3).tolist();w=rng.uniform(-1.5,1.5,3)
  c={'id':f'{seed}-{i}','task':'moving_recovery','mass':float(rng.uniform(1800,2200)),'fuel':float(rng.uniform(230,260)),'thrust_scale':float(rng.uniform(.9,1.05)),'pos':[0.,0.,float(rng.uniform(1 if stress else 7,18 if stress else 14))],'quat':q.tolist(),'vel':list(map(float,v)),'omega':w.tolist(),'goal':{'turns':turns}}
  c['pos'][2]=max(c['pos'][2],ctl.core.geometry_minimum_cg(ctl.core.qmat(q),CFG)+.1);out.append(c)
 return out

def init():
 global W;W=ctl.World(CFG)
def run(job):
 p,cs=job;return [{k:v for k,v in ctl.rollout(p,c,CFG,world=W).items() if k not in ['frames','environment']} for c in cs]
def fitness(rows):
 grouped=[[r['metrics']['unifiedScore'] for r in rows if r['case']['goal']['turns']==turns] for turns in [0,1]];means=[np.mean(g) for g in grouped if g];return float(np.mean(means)+.3*min(means))
def summary(rows):return {'successCount':sum(r['success'] for r in rows),'trials':len(rows),'byGoal':{str(t):{'successCount':sum(r['success'] for r in rows if r['case']['goal']['turns']==t),'trials':sum(r['case']['goal']['turns']==t for r in rows)} for t in [0,1]},'fitness':fitness(rows)}
def write(path,value):
 tmp=path.with_suffix('.tmp');ctl.core.write_json(tmp,value);tmp.replace(path)

def main():
 folder=HERE/'campaign';folder.mkdir(exist_ok=False);codehash=hashlib.sha256((HERE/'unified_controller.py').read_bytes()).hexdigest();seed=2026101801;rng=np.random.default_rng(seed);gens=60;pop=18;dev=cases(seed+10000,12);best=ctl.INITIAL.copy();mean=(best-ctl.LOWER)/(ctl.UPPER-ctl.LOWER);sigma=np.full(len(best),.23)
 report={'revision':'unified-airborne-v4','status':'training','algorithm':'CEM over one goal-conditioned measured-state quaternion path controller','controllerSha256':codehash,'plannedGenerations':gens,'population':pop,'seed':seed,'holdoutSeeds':{'inDomain':2026101881,'stress':2026101882},'newTaskIdsUsedByController':False,'parameters':ctl.PARAMETERS,'initial':ctl.INITIAL.tolist(),'reference':ctl.REFERENCE.tolist(),'referenceUse':'One declared manual reference candidate is included in generation 1; training starts from INITIAL. Its held-out performance is also reported.','config':CFG,'history':[],'evaluations':0,'notice':'Goal.turns is a user command; no scenario-specific gains or controller branch. All training runs use unchanged 2ms MuJoCo physics. Horizontal force target is zero; actual horizontal velocity changes enter fitness.'};write(folder/'results.json',report);write(folder/'development-cases.json',dev);write(folder/'source.json',{'unified_controller.py':codehash,'train.py':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()});start=time.perf_counter()
 with ProcessPoolExecutor(max_workers=min(6,os.cpu_count() or 1),initializer=init) as pool:
  bestrows=list(pool.map(run,[(best,dev)]))[0];bestscore=fitness(bestrows);report['evaluations']+=len(dev);report['initialDevelopment']=bestrows;selected=0
  for g in range(1,gens+1):
   x=np.clip(rng.normal(mean,sigma,(pop,len(best))),0,1);x[0]=mean;
   if g==1:x[2]=(ctl.REFERENCE-ctl.LOWER)/(ctl.UPPER-ctl.LOWER)
   x[1]=(best-ctl.LOWER)/(ctl.UPPER-ctl.LOWER);ps=ctl.LOWER+x*(ctl.UPPER-ctl.LOWER);cs=cases(seed+g*100,6);batches=list(pool.map(run,[(p,cs) for p in ps]));scores=np.array([fitness(b) for b in batches]);order=np.argsort(scores);elite=x[order[-5:]];mean=.3*mean+.7*elite.mean(0);sigma=np.maximum(.04,.3*sigma+.7*elite.std(0));report['evaluations']+=pop*len(cs)
   if g%5==0 or g==1:
    trial=ps[order[-1]];rows=list(pool.map(run,[(trial,dev)]))[0];report['evaluations']+=len(dev)
    if fitness(rows)>bestscore:best=trial.copy();bestscore=fitness(rows);bestrows=rows;selected=g
   event={'generation':g,'cases':cs,'parameters':ps.tolist(),'scores':scores.tolist(),'rows':batches,'selectedParameters':best.tolist(),'selectedGeneration':selected}
   with (folder/'training-ledger.jsonl').open('a') as f:f.write(json.dumps(event,separators=(',',':'))+'\n')
   report['history'].append({'generation':g,'bestTraining':float(scores.max()),'meanTraining':float(scores.mean()),'selectedGeneration':selected,'development':summary(bestrows),'parameters':best.tolist()});report['selectedParameters']=best.tolist();report['selectedGeneration']=selected;report['elapsedSeconds']=time.perf_counter()-start;write(folder/'results.json',report)
   print(json.dumps({'generation':g,'evaluations':report['evaluations'],'selectedGeneration':selected,'development':summary(bestrows),'seconds':round(report['elapsedSeconds'])}),flush=True)
   if g%5==0:
    write(folder/f'checkpoint-{g:03}.json',{'generation':g,'selectedGeneration':selected,'parameters':best.tolist(),'development':bestrows})
  report['status']='training_complete';report['finalDevelopment']=bestrows;write(folder/'results.json',report)
 assert codehash==hashlib.sha256((HERE/'unified_controller.py').read_bytes()).hexdigest(),'Controller changed during training'
if __name__=='__main__':main()
