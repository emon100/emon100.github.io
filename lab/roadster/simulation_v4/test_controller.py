import unittest,copy,json
from pathlib import Path
import numpy as np
import unified_controller as u
class Tests(unittest.TestCase):
 def setUp(self):self.cfg=json.loads((Path(__file__).parent/'config.json').read_text());self.c=u.core.default_case('moving_recovery',self.cfg);self.c['goal']={'turns':0};self.w=u.World(self.cfg);self.w.reset(self.c)
 def test_label_invariance(self):
  s=self.w.state();a=u.UnifiedPolicy(u.INITIAL,'kickflip',self.c,self.cfg);c=copy.deepcopy(self.c);c['id']='inverted-other';c['task']='other';b=u.UnifiedPolicy(u.INITIAL,'inverted',c,self.cfg)
  x=a.command(s,self.w.contact);y=b.command(s,self.w.contact)
  for i in [0,1]:np.testing.assert_array_equal(x[i],y[i])
 def test_commanded_full_turn_path(self):
  self.c['quat']=[1,0,0,0];self.c['goal']['turns']=1;segments,target,spin,axis=u.qpath(self.c);self.assertAlmostEqual(sum(s['length'] for s in segments),2*np.pi);self.assertAlmostEqual(spin,2*np.pi)
 def test_no_horizontal_target(self):
  for turns in [0,1]:
   c=copy.deepcopy(self.c);c['goal']['turns']=turns;p=u.UnifiedPolicy(u.INITIAL,'ignored',c,self.cfg);s=self.w.state();original=copy.deepcopy(s);f,t,_=p.command(s,self.w.contact);np.testing.assert_array_equal(f[:2],[0,0]);np.testing.assert_array_equal(s['quat'],original['quat']);self.assertTrue(np.isfinite(t).all())
 def test_completed_turn_survives_disturbance(self):
  c=copy.deepcopy(self.c);c['goal']['turns']=1;c['quat']=[0,1,0,0];p=u.UnifiedPolicy(u.REFERENCE,'ignored',c,self.cfg);state=self.w.state();state['pos'][2]=18
  for seg in p.segments:
   for angle in np.linspace(0,seg['length'],101):
    q=u.qmul(seg['q'],u.core.axis_q(seg['axis'],angle));R=u.core.qmat(q);rate=seg['length']/2.;world_rate=u.core.qmat(seg['q'])@seg['axis']*rate;state.update(quat=q,R=R,omega=R.T@world_rate,omegaWorld=world_rate);p.command(state,self.w.contact)
  self.assertTrue(p.spin_satisfied);measured=p.spin_measured
  state.update(omega=np.array([-.5,0,0]),omegaWorld=np.array([-.5,0,0]));p.command(state,self.w.contact)
  self.assertTrue(p.spin_satisfied);self.assertEqual(p.spin_measured,measured)
 def test_empty_gas(self):
  self.c['fuel']=0;self.w.reset(self.c);p=u.UnifiedPolicy(u.INITIAL,'ignored',self.c,self.cfg);s=self.w.state();f,t,_=p.command(s,self.w.contact);command,_=self.w.allocate(np.r_[s['R'].T@f,t]);np.testing.assert_array_equal(command,np.zeros(12))
if __name__=='__main__':unittest.main()
