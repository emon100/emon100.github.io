import unittest
import numpy as np
from core import load_config, integrate, fan_forces, inertia, allocate, contact_metrics, axis_quat, decompose_wrench, rollout, cases, INITIAL, delivery_limits

class PhysicsTests(unittest.TestCase):
    def setUp(self):
        self.c=load_config();self.p=np.array([[0.,0.,10.]]);self.v=np.zeros((1,3));self.q=np.array([[1.,0.,0.,0.]]);self.w=self.v.copy();self.m=np.array([2000.])
    def test_hover_and_freefall(self):
        zero=np.zeros((1,3));lift=np.array([[0.,0.,self.m[0]*self.c['gravity']]])
        p,v,q,w=integrate(self.p,self.v,self.q,self.w,self.m,lift,zero,self.c,.2)
        np.testing.assert_allclose(p,self.p);np.testing.assert_allclose(v,0,atol=1e-12)
        p,v,_,_=integrate(self.p,self.v,self.q,self.w,self.m,zero,zero,self.c,.2)
        self.assertAlmostEqual(p[0,2],10-.5*9.81*.2**2);self.assertAlmostEqual(v[0,2],-9.81*.2)
    def test_pure_couple(self):
        lift=np.array([[0.,0.,19620.]]);tau=np.array([[1000.,0.,0.]])
        _,v,q,w=integrate(self.p,self.v,self.q,self.w,self.m,lift,tau,self.c,.1)
        np.testing.assert_allclose(v,0,atol=1e-12);self.assertAlmostEqual(w[0,0],1000/inertia(self.m,self.c)[0,0]*.1)
        self.assertAlmostEqual(np.linalg.norm(q),1);self.assertGreater(q[0,1],0)
    def test_suction_disappears_away_from_surface(self):
        near=np.array([[0.,0.,self.c['vehicle']['height']/2]]);_,dp,power,detail=fan_forces(near,self.q,np.ones(1),self.c)
        self.assertGreater(dp[0],0);self.assertLessEqual(power[0],48000+1e-6)
        np.testing.assert_allclose(detail['pressurePower']+detail['axialPower'],power)
        free,dp,power,detail=fan_forces(self.p,self.q,np.ones(1),self.c)
        self.assertEqual(dp[0],0);self.assertGreater(free[0,0],0);self.assertLessEqual(free[0,2],0)
        self.assertLess(np.linalg.norm(free),19620)
    def test_gas_budget_and_shared_wrench(self):
        force,tau,cost=allocate(np.ones((1,3))*1e6,np.ones((1,3))*1e6,np.array([.1]),self.c,.04)
        used=cost[0]*.04/(60*9.81);self.assertLessEqual(used,.10000001)
        self.assertLessEqual(cost[0],self.c['actuator']['total_jet_force_max'])
        force,tau,cost=allocate(force,tau,np.zeros(1),self.c,.04)
        np.testing.assert_allclose(force,0);np.testing.assert_allclose(tau,0)
    def test_pressure_loss_limits_near_empty_thrusters(self):
        scale,isp=delivery_limits(np.array([250.,75.,25.]),self.c)
        self.assertAlmostEqual(scale[0],1);self.assertAlmostEqual(scale[1],1)
        self.assertLess(scale[2],.25);self.assertLess(isp[2],60)
    def test_virtual_nozzles_reconstruct_wrench(self):
        force=np.array([[12000.,-9000.,20000.]]);tau=np.array([[2500.,-8000.,4000.]])
        jets=decompose_wrench(force,tau,self.c)
        nozzles=self.c['actuator']['virtual_nozzles'];directions=np.array([n['direction'] for n in nozzles]);positions=np.array([n['position'] for n in nozzles])
        np.testing.assert_allclose(jets@directions,force,atol=1e-9)
        np.testing.assert_allclose(jets@np.cross(positions,directions),tau,atol=1e-9)
        expected=np.abs(force).sum()+(np.abs(tau)/np.array(self.c['actuator']['couple_lever_arm'])).sum()
        self.assertAlmostEqual(jets.sum(),expected)
    def test_landing_criteria(self):
        ground=np.array([[0.,0.,self.c['vehicle']['height']/2]]);clearance,_,safe,_=contact_metrics(ground,self.v,self.q,self.w,ground,self.c)
        self.assertAlmostEqual(clearance[0],0);self.assertTrue(safe[0])
        fast=np.array([[0.,0.,-4.]]);self.assertFalse(contact_metrics(ground,fast,self.q,self.w,ground,self.c)[2][0])
        pitching=np.array([[0.,.6,0.]]);descending=np.array([[0.,0.,-1.]])
        self.assertFalse(contact_metrics(ground,descending,self.q,pitching,ground,self.c)[2][0])
        inverted=axis_quat(np.array([[1.,0.,0.]]),np.array([np.pi]));self.assertFalse(contact_metrics(ground,self.v,inverted,self.w,ground,self.c)[2][0])

    def test_weak_fan_does_not_artificially_hold_ceiling(self):
        import copy
        weak=copy.deepcopy(self.c);weak['fan']['pressure_limit_pa']=500.;weak['max_time']=.2
        inputs=cases('inverted_release',1,weak,99);inputs['height'][:]=20.
        result=rollout('inverted_release',INITIAL,inputs,weak,record=True)
        self.assertLess(result['final']['pos'][0][2],19.9)
        self.assertNotEqual(result['final']['pos'][0][0],0.)

if __name__=='__main__':unittest.main()
