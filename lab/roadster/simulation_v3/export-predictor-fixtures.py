"""Independent frozen-predictor fixtures; no changes to control or plant."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from landing_predictor import predict_vertical_stop,core
cfg=json.loads((HERE/'config.json').read_text())
rows=json.loads((HERE/'landing-prediction-calibration.json').read_text())['rows']
def plain(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {k:plain(v) for k,v in x.items()}
    if isinstance(x,list):return [plain(v) for v in x]
    return x
fixtures=[]
for row in rows:
    c=row['case'];s={'pos':c['pos'],'quat':c['quat'],'R':core.qmat(c['quat']),'vel':c['vel'],'omega':c['omega'],'mass':c['mass'],'fuel':c['fuel'],'thrustScale':c['thrust_scale'],'disabledNozzles':c.get('disabled_nozzles',[]),'actualNozzleThrottle':row['initialValveThrottle']}
    fixtures.append({'id':c['id'],'state':plain(s),'reference':plain(predict_vertical_stop(s,cfg)),'calibrationReference':row['prediction2ms']})
s=dict(fixtures[0]['state']);s.update(fuel=0.,vel=[0.,0.,-2.],R=np.eye(3),actualNozzleThrottle=np.ones(12)*.7)
fixtures.append({'id':'empty_tank_full_horizon','state':plain(s),'reference':plain(predict_vertical_stop(s,cfg))})
s=dict(fixtures[0]['state']);s.update(fuel=50.,vel=[0.,0.,-8.],R=np.eye(3),disabledNozzles=['J01','J03'],actualNozzleThrottle=np.linspace(0,1,12))
fixtures.append({'id':'failed_jets_current_valves','state':plain(s),'reference':plain(predict_vertical_stop(s,cfg))})
report={'revision':json.loads((HERE/'manifest.json').read_text())['revision'],'predictorSha256':hashlib.sha256((HERE/'landing_predictor.py').read_bytes()).hexdigest(),'config':cfg,'fixtures':fixtures}
(HERE/'predictor-fixtures.json').write_text(json.dumps(report,separators=(',',':'))+'\n');print(len(fixtures))
