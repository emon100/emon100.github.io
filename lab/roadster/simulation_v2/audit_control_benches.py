"""Separate native diagnostic benches, never counted as CEM/holdout episodes."""
import copy, hashlib
from core import *

def main():
    cfg=load_config();source=hashlib.sha256((HERE/'core.py').read_bytes()).hexdigest();revision='energy-coupled-r6'
    c=default_case('moving_recovery',cfg);c['quat']=euler_q(.4,.2,.7).tolist();w=World(cfg);w.reset(c);a=w.state();b=copy.deepcopy(a);b['vel'][:2]=[91.,-63.];b['pos'][:2]=[20.,-30.]
    pa=FeedbackPolicy(HANDCRAFTED,c['task'],c,cfg);pb=FeedbackPolicy(HANDCRAFTED,c['task'],c,cfg);oa=pa.command(a,w.contact);ob=pb.command(b,w.contact)
    scope={'revision':revision,'sourceSha256':source,'parameterCount':len(PARAMETERS),'changedOnlyHorizontalPositionAndVelocity':{'forceDifference':float(np.linalg.norm(oa[0]-ob[0])),'torqueDifference':float(np.linalg.norm(oa[1]-ob[1])),'desiredHorizontalForce':oa[0][:2].tolist()},'randomAllocationStates':500,'maxAllocatedWorldHorizontalForce':0.,'maxPureTorqueResultantForce':0.,'pressureSamples':[],'wholeCOMGroundHeight':cfg['vehicle']['ground_cg_height'],'physicalResidualNotice':'Static allocated target has zero horizontal force. Body rotation and valve lag can create realized horizontal jet force, separately recorded. Fan reaction and ground friction also change horizontal velocity.'}
    assert scope['changedOnlyHorizontalPositionAndVelocity']['forceDifference']==0 and scope['changedOnlyHorizontalPositionAndVelocity']['torqueDifference']==0
    rng=np.random.default_rng(12345);A=w.A;limits=w.fmax
    for _ in range(500):
        q=rng.normal(size=4);q/=np.linalg.norm(q);R=qmat(q);torque=rng.uniform(-50000,50000,size=3);scaled=limits*rng.uniform(.05,1);F=np.array([0.,0.,rng.uniform(-100000,100000)])
        f=allocate_fixed_nozzles(np.r_[R.T@F,torque],scaled,cfg);scope['maxAllocatedWorldHorizontalForce']=max(scope['maxAllocatedWorldHorizontalForce'],float(np.linalg.norm((R@(A@f)[:3])[:2])))
        f=allocate_fixed_nozzles(np.r_[np.zeros(3),torque],scaled,cfg);scope['maxPureTorqueResultantForce']=max(scope['maxPureTorqueResultantForce'],float(np.linalg.norm((A@f)[:3])))
    assert scope['maxAllocatedWorldHorizontalForce']<1e-8 and scope['maxPureTorqueResultantForce']<1e-8
    for fuel in [260.,250.,200.,100.,0.]:
        pressure,isp=delivery(fuel,cfg);scope['pressureSamples'].append({'fuel':fuel,'pressureRatio':pressure,'Isp':isp})
    assert all(a['pressureRatio']>b['pressureRatio'] for a,b in zip(scope['pressureSamples'],scope['pressureSamples'][1:]))
    write_json(HERE/'control-scope-audit.json',scope)
    rows=[]
    for command in [0.,.25,.8]:
        w=World(cfg);c=default_case('stationary_kickflip',cfg);w.reset(c)
        for _ in range(25):w.advance(np.zeros(12),0.)
        params=HANDCRAFTED.copy();params[PARAMETERS.index('landing_fan')]=command;p=FeedbackPolicy(params,c['task'],c,cfg);p.phase='settle';p.turn_complete=True;p.was_airborne=True
        for _ in range(25):
            s=w.state();F,T,fc=p.command(s,w.contact);fan=fan_model(s['pos'],s['R'],fc,cfg);u,_=w.allocate(np.r_[s['R'].T@F,T-s['R'].T@fan['torqueWorld']]);w.advance(u,fc,{'task':c['task'],'initialXY':[0.,0.],'turnComplete':True,'wasAirborne':True})
        row={'fanCommand':command,'wheelNormalForce':float(w.contact['totalNormalForce']),'suctionForce':float(-w.fan['suctionForce'][2]),'axialForceWorld':w.fan['axialForce'].tolist(),'jetForceWorld':w.jet_force_world.tolist(),'horizontalVelocity':w.state()['vel'][:2].tolist(),'fluidPower':float(w.fan['power']),'fluidPowerBudget':float(w.fan['availableFluidPower'])};rows.append(row)
        assert row['fluidPower']<=row['fluidPowerBudget']+1e-6 and np.linalg.norm(row['jetForceWorld'])<1e-6
    assert rows[1]['wheelNormalForce']>rows[0]['wheelNormalForce']+rows[1]['suctionForce']*.8
    write_json(HERE/'fan-grip-probe.json',{'revision':revision,'sourceSha256':source,'notice':'Separate physical bench, manually initialized in settle mode. No horizontal feedback or fan-force cancellation. This is not a learned flight success.','rows':rows})
    print('Native control-scope and fan-grip benches passed; 500 allocations and 3 fan trials.')

if __name__=='__main__':main()
