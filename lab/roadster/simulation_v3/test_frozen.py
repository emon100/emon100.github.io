"""Regression tests for recovery-only changes; underlying plant remains frozen r6."""
import unittest,copy,json,math
from pathlib import Path
import numpy as np
import controller as rc
from landing_predictor import predict_vertical_stop
core=rc.core;HERE=Path(__file__).resolve().parent
class FrozenRecoveryTests(unittest.TestCase):
 def setUp(self):
  self.cfg=json.loads((HERE/'config.json').read_text());self.params=json.loads((HERE/'manifest.json').read_text())['parameters'];self.case=core.default_case('moving_recovery',self.cfg)
 def test_zero_horizontal_target_and_valid_nozzles(self):
  w=rc.RecoveryWorld(self.cfg);w.reset(self.case);p=rc.RecoveryPolicy(self.params,'moving_recovery',self.case,self.cfg);s=w.state();f,t,_=p.command(s,w.contact);self.assertEqual(f[:2].tolist(),[0.,0.]);u,a=w.allocate(np.r_[s['R'].T@f,t]);self.assertTrue(np.all((u>=0)&(u<=1)));self.assertLess(np.linalg.norm((s['R']@np.array(a['achievedWrenchBody'])[:3])[:2]),1e-4);self.assertFalse(a['globallyOptimal'])
 def test_pd_bookkeeping(self):
  c=copy.deepcopy(self.case);c['quat']=core.euler_q(2.1,.4,.8).tolist();c['omega']=[.5,-.3,.6];w=rc.RecoveryWorld(self.cfg);w.reset(c);p=rc.RecoveryPolicy(self.params,'moving_recovery',c,self.cfg);f,t,_=p.command(w.state(),w.contact);g=p.goal;total=sum((np.array(g[k]) for k in ['PDProportionalTorque','PDDampingTorque','gyroscopicTorque','governorCorrectionTorque']));np.testing.assert_allclose(t,total,atol=1e-10);self.assertEqual(g['PDIntegralTorque'],[0.,0.,0.])
 def test_predictor_does_not_mutate(self):
  w=rc.RecoveryWorld(self.cfg);c=copy.deepcopy(self.case);c['quat']=[1,0,0,0];c['vel'][2]=-5.;w.reset(c);s=w.state();before=copy.deepcopy(s);cfg=copy.deepcopy(self.cfg);a=predict_vertical_stop(s,self.cfg);b=predict_vertical_stop(s,self.cfg);self.assertEqual(a,b)
  for k,v in s.items():
   if isinstance(v,np.ndarray):np.testing.assert_array_equal(v,before[k])
   else:self.assertEqual(v,before[k])
  self.assertEqual(cfg,self.cfg);self.assertAlmostEqual(a['targetLiftWeightRatio'],.85*a['maximumZeroMomentLiftWeightRatio']);self.assertTrue(a['feasible'])
 def test_overspeed_not_clamped(self):
  self.assertGreater(rc.rotation_time(.1,6.,4.,5.,5.),6./5.)
 def test_disabled_nozzles_remove_authority(self):
  c=copy.deepcopy(self.case);c['disabled_nozzles']=[n['id'] for n in core.nozzle_layout(self.cfg)];w=rc.RecoveryWorld(self.cfg);w.reset(c);u,a=w.allocate([0,0,20000,1000,0,0]);np.testing.assert_array_equal(u,np.zeros(12));self.assertFalse(rc.landing_gas_budget(w.state(),self.cfg)['forceMarginReachable'])
 def test_empty_gas_naturally_falls(self):
  c=copy.deepcopy(self.case);c['fuel']=0.;w=rc.RecoveryWorld(self.cfg);w.reset(c);before=w.state();after=w.advance(np.ones(12),0.);self.assertLess(after['vel'][2],before['vel'][2]);np.testing.assert_array_equal(w.forces,np.zeros(12))
 def test_nominal_requires_stable_contact(self):
  original=core.FeedbackPolicy;core.FeedbackPolicy=rc.RecoveryPolicy
  try:r=core.rollout('moving_recovery',self.params,self.case,self.cfg,record=True,world=rc.RecoveryWorld(self.cfg))
  finally:core.FeedbackPolicy=original
  self.assertTrue(r['success']);self.assertGreaterEqual(r['metrics']['stableSeconds'],1.);self.assertFalse(r['metrics']['hardContact']);self.assertFalse(r['metrics']['bodyContact'])
if __name__=='__main__':unittest.main()
