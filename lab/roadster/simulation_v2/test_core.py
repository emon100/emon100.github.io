import unittest, copy
import numpy as np
from core import *

class PhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.cfg=load_config()
    def test_settle_fan_increases_real_wheel_load_instead_of_being_canceled(self):
        w=World(self.cfg);c=default_case('stationary_kickflip',self.cfg);w.reset(c)
        for _ in range(25):w.advance(np.zeros(12),0)
        base_load=w.contact['totalNormalForce'];params=HANDCRAFTED.copy();params[PARAMETERS.index('landing_fan')]=.8;p=FeedbackPolicy(params,c['task'],c,self.cfg);p.phase='settle';p.turn_complete=True;p.was_airborne=True
        for _ in range(25):
            st=w.state();force,torque,fc=p.command(st,w.contact);fan=fan_model(st['pos'],st['R'],fc,self.cfg);requested=np.r_[st['R'].T@force,torque-st['R'].T@fan['torqueWorld']];u,a=w.allocate(requested);w.advance(u,fc,{'task':c['task'],'initialXY':[0,0],'turnComplete':True,'wasAirborne':True})
        suction=-w.fan['suctionForce'][2];self.assertGreater(suction,3000.);self.assertGreater(w.contact['totalNormalForce'],base_load+suction*.8)
        actual_jet=w.A@w.forces;self.assertLess(np.linalg.norm(actual_jet[:3]),1e-6);self.assertGreater(w.fan_horizontal_impulse[0],0.)
    def test_free_fall_and_mass_update_do_not_reset_state(self):
        w=World(self.cfg);c=default_case('moving_recovery',self.cfg);c.update(pos=[1.,2.,15.],quat=[1.,0.,0.,0.],vel=[2.,0.,0.]);w.reset(c)
        for _ in range(10):w.advance(np.zeros(12),0)
        s=w.state();self.assertAlmostEqual(s['pos'][0],1.4,delta=1e-5);self.assertAlmostEqual(s['vel'][2],-1.962,places=5);self.assertLess(abs(s['pos'][2]-(15-.5*9.81*.2**2)),.003)
    def test_layout_and_pure_couple(self):
        report=controllability(self.cfg);self.assertEqual(report['rank'],6);self.assertTrue(report['positiveNullExists'])
        ns=nozzle_layout(self.cfg);limits=np.array([n['maxForce'] for n in ns]);target=np.array([0.,0.,0.,1000.,0.,0.]);f=allocate_fixed_nozzles(target,limits,self.cfg)
        np.testing.assert_allclose(wrench_matrix(ns)@f,target,atol=1e-6);self.assertTrue(np.all(f>=0));self.assertTrue(np.all(f<=limits))
        for axis in range(6):
            for sign in [-1,1]:
                target=np.eye(6)[axis]*sign*1000;f=allocate_fixed_nozzles(target,limits,self.cfg);np.testing.assert_allclose(wrench_matrix(ns)@f,target,atol=1e-6)
    def test_disabled_downward_jets_cannot_supply_negative_z(self):
        w=World(self.cfg);c=default_case('moving_recovery',self.cfg);c['disabled_nozzles']=['J02','J04','J06','J08'];w.reset(c);u,a=w.allocate(np.array([0,0,-20000,0,0,0]));self.assertAlmostEqual(a['achievedWrenchBody'][2],0.,places=5);self.assertTrue(all(u[i]==0 for i in [1,3,5,7]))
    def test_fuel_empty_produces_no_jet_impulse(self):
        w=World(self.cfg);c=default_case('moving_recovery',self.cfg);c.update(fuel=0.,quat=[1.,0.,0.,0.],vel=[0.,0.,0.]);w.reset(c);s=w.advance(np.ones(12),0)
        self.assertEqual(float(w.forces.sum()),0.);self.assertLess(s['vel'][2],-.19);self.assertEqual(w.fuel,0.)
    def test_fan_seal_and_power_budget(self):
        near=fan_model(np.array([0.,0.,self.cfg['vehicle']['ground_cg_height']+.005]),np.eye(3),1,self.cfg);far=fan_model(np.array([0.,0.,4.]),np.eye(3),1,self.cfg)
        self.assertLess(near['suctionForce'][2],0);self.assertEqual(float(np.linalg.norm(far['suctionForce'])),0.);self.assertLess(far['axialForce'][2],0.)
        for f in [near,far]:self.assertLessEqual(f['power'],f['availableFluidPower']+1e-6)
    def test_passive_wheels_preserve_initial_rolling_speed(self):
        w=World(self.cfg);c=default_case('moving_kickflip',self.cfg);w.reset(c)
        for _ in range(10):s=w.advance(np.zeros(12),0)
        self.assertGreater(s['vel'][0],4.8);self.assertFalse(w.any_body_contact)
    def test_rolling_contact_requires_one_second_without_braking(self):
        w=World(self.cfg);c=default_case('moving_kickflip',self.cfg);w.reset(c);ctx={'turnComplete':True,'wasAirborne':True}
        for _ in range(45):w.advance(np.zeros(12),0,ctx)
        self.assertLess(w.stable_seconds,1.);self.assertGreater(w.state()['vel'][0],4.5)
        for _ in range(20):w.advance(np.zeros(12),0,ctx)
        self.assertGreaterEqual(w.stable_seconds,1.);self.assertGreater(w.state()['vel'][0],4.5);self.assertLess(w.contact['maxWheelSlipSpeed'],.5)
        w.data.qvel[6:10]=0;mujoco.mj_forward(w.model,w.data);w.advance(np.zeros(12),0,ctx)
        self.assertLess(w.stable_seconds,self.cfg['physics']['control_dt']);self.assertGreater(w.contact['maxWheelSlipSpeed'],.5)
    def test_low_inverted_case_fails_instead_of_snapping_upright(self):
        c=default_case('moving_recovery',self.cfg);c['pos'][2]=.80;c['vel'][2]=-4.
        r=rollout('moving_recovery',HANDCRAFTED,c,self.cfg,record=True,policy_id='handcrafted');self.assertFalse(r['success']);self.assertIn(r['status'],['body_contact','hard_contact']);self.assertLess(r['metrics']['duration'],1.)
    def test_no_control_inverted_body_falls_and_hits_floor(self):
        r=rollout('moving_recovery',UNTUNED,default_case('moving_recovery',self.cfg),self.cfg,record=True,policy_id='no_control')
        self.assertFalse(r['success']);self.assertTrue(r['metrics']['bodyContact']);self.assertEqual(r['metrics']['fuelUsed'],0.);self.assertGreater(r['metrics']['maxContactClosingSpeed'],10.)
    def test_actual_mujoco_sites_match_the_published_nozzle_wrenches(self):
        w=World(self.cfg);A=wrench_matrix(nozzle_layout(self.cfg))
        for q in [euler_q(0,0,0),euler_q(math.pi/2,0,0),euler_q(.3,.2,1.1)]:
            c=default_case('moving_recovery',self.cfg);c.update(pos=[0,0,15],quat=q.tolist(),vel=[0,0,0],omega=[0,0,0]);w.reset(c)
            for j in range(12):
                w.data.ctrl[:]=0;w.data.ctrl[j]=1000;mujoco.mj_forward(w.model,w.data);expected=np.r_[qmat(q)@A[:3,j]*1000,A[3:,j]*1000];np.testing.assert_allclose(w.data.qfrc_actuator[:6],expected,atol=1e-8)
    def test_impossible_dry_inertia_is_rejected(self):
        c=default_case('moving_recovery',self.cfg);c.update(mass=100.,fuel=0.)
        with self.assertRaises(ValueError):World(self.cfg).reset(c)
    def test_geometry_and_nozzle_exits_are_consistent(self):
        v=self.cfg['vehicle'];top=max(b['position'][2]+b['half_size'][2] for b in v['collision_boxes']);self.assertAlmostEqual(top-v['wheel_z']+v['wheel_radii'][2],1.20)
        for n in nozzle_layout(self.cfg):
            p=np.array(n['position'])
            for b in v['collision_boxes']:self.assertFalse(np.all(np.abs(p-b['position'])<np.array(b['half_size'])))
            for x in [-v['axle_half'],v['axle_half']]:
                for y in [-v['track_half'],v['track_half']]:self.assertGreater(np.sum(((p-[x,y,v['wheel_z']])/v['wheel_radii'])**2),1.)
    def test_free_inputs_reject_negative_fuel_or_force_scaling(self):
        for key in ['fuel','thrust_scale']:
            c=default_case('moving_recovery',self.cfg);c[key]=-1
            with self.assertRaises(ValueError):World(self.cfg).reset(c)
    def test_stability_counter_resets_on_actual_loss_of_contact(self):
        w=World(self.cfg);c=default_case('stationary_kickflip',self.cfg);w.reset(c);context={'task':c['task'],'initialXY':[0,0],'turnComplete':True,'wasAirborne':True}
        for _ in range(20):w.advance(np.zeros(12),0,context)
        self.assertGreater(w.stable_seconds,.3)
        up=np.zeros(12);up[::2][:4]=1.
        for _ in range(6):w.advance(up,0,context)
        self.assertEqual(w.stable_seconds,0.);self.assertEqual(w.contact['minimumWheelCount'],0)
    def test_whole_com_is_the_root_at_all_fuel_levels_and_attitudes(self):
        w=World(self.cfg);c=default_case('moving_recovery',self.cfg);c.update(pos=[1,2,15],quat=euler_q(.4,.7,1.2).tolist(),vel=[3,2,-1],omega=[.3,.4,.5]);w.reset(c)
        for fuel in [260.,150.,0.]:
            w.fuel=fuel;w.update_mass();mujoco.mj_forward(w.model,w.data)
            np.testing.assert_allclose(w.data.subtree_com[1],w.data.xpos[1],atol=1e-12)
            np.testing.assert_allclose(w.state()['vel'],w.data.qvel[:3],atol=1e-12)
        self.assertAlmostEqual(self.cfg['vehicle']['ground_cg_height'],.5)
    def test_fan_wrench_reference_conversion_is_physically_equivalent(self):
        w=World(self.cfg);c=default_case('moving_recovery',self.cfg);c.update(pos=[0,0,15],quat=euler_q(.5,.3,.9).tolist());w.reset(c);st=w.state()
        F=np.array([100.,200.,-300.]);T=np.array([40.,-50.,60.]);root=st['pos'];body_com=w.data.xipos[1].copy();at_root=np.zeros(w.model.nv);at_body=np.zeros(w.model.nv)
        mujoco.mj_applyFT(w.model,w.data,F,T,root,1,at_root)
        mujoco.mj_applyFT(w.model,w.data,F,T-np.cross(body_com-root,F),body_com,1,at_body)
        np.testing.assert_allclose(at_root,at_body,atol=1e-10)
    def test_no_horizontal_feedback_and_pure_moments_have_zero_net_force(self):
        cfg=self.cfg;w=World(cfg);c=default_case('moving_recovery',cfg);c['quat']=euler_q(.4,.2,.7).tolist();w.reset(c);a=w.state();b=copy.deepcopy(a);b['vel'][:2]=[91.,-63.];b['pos'][:2]=[20.,-30.]
        pa=FeedbackPolicy(HANDCRAFTED,c['task'],c,cfg);pb=FeedbackPolicy(HANDCRAFTED,c['task'],c,cfg)
        outa=pa.command(a,w.contact);outb=pb.command(b,w.contact)
        for aa,bb in zip(outa[:2],outb[:2]):np.testing.assert_array_equal(aa,bb)
        np.testing.assert_array_equal(outa[0][:2],np.zeros(2))
        A=w.A;limits=w.fmax
        for q in [euler_q(.3,.8,1),euler_q(2.,.7,-.9)]:
            R=qmat(q)
            for torque in [[15000,-40000,20000],[-3000,6000,-10000]]:
                f=allocate_fixed_nozzles(np.r_[R.T@np.array([0,0,80000]),torque],limits*.35,cfg)
                self.assertLess(np.linalg.norm((R@(A@f)[:3])[:2]),1e-8)
                f=allocate_fixed_nozzles(np.r_[np.zeros(3),torque],limits*.35,cfg)
                self.assertLess(np.linalg.norm((A@f)[:3]),1e-8)
    def test_pressure_declines_from_first_gas_use_and_cannot_be_regulated(self):
        pairs=[delivery(f,self.cfg) for f in [260.,250.,200.,100.,0.]]
        self.assertTrue(all(pairs[i][0]>pairs[i+1][0] for i in range(4)))
        with self.assertRaises(ValueError):delivery(260,self.cfg,'regulated_then_blowdown')
        c=default_case('moving_recovery',self.cfg);c['fuel']=261.
        with self.assertRaises(ValueError):validate_case(c,self.cfg)
        w=World(self.cfg);c=default_case('moving_recovery',self.cfg);w.reset(c);p=[]
        for _ in range(8):w.advance(np.ones(12)*.3,0);p.append(w.pressure_scale)
        self.assertTrue(all(p[i]>p[i+1] for i in range(len(p)-1)))
    def test_launch_gate_requires_clear_airborne_ascent(self):
        cfg=self.cfg;c=default_case('stationary_kickflip',cfg);w=World(cfg);w.reset(c);base=w.state();base['pos'][2]=cfg['vehicle']['ground_cg_height']+.06;base['vel'][2]=1.
        for wheel_count,body_contact,vz,expected in [(4,False,1.,'boost'),(0,True,1.,'boost'),(0,False,-1.,'boost'),(0,False,0.,'boost'),(0,False,1.,'flip')]:
            s=copy.deepcopy(base);s['vel'][2]=vz;p=FeedbackPolicy(UNTUNED,c['task'],c,cfg);p.command(s,{'wheelCount':wheel_count,'bodyContact':body_contact});self.assertEqual(p.phase,expected)
        s=copy.deepcopy(base);s['pos'][2]=cfg['vehicle']['ground_cg_height']+.019;p=FeedbackPolicy(UNTUNED,c['task'],c,cfg);p.command(s,{'wheelCount':0,'bodyContact':False});self.assertEqual(p.phase,'boost')
    def test_real_launch_starts_roll_while_rising_near_ground_without_pose_assignment(self):
        cfg=self.cfg
        for task in ['stationary_kickflip','moving_kickflip']:
            c=default_case(task,cfg);w=World(cfg);w.reset(c);p=FeedbackPolicy(UNTUNED,task,c,cfg);found=False
            for _ in range(40):
                s=w.state();old_quat=w.data.qpos[3:7].copy();F,T,fc=p.command(s,w.contact);np.testing.assert_array_equal(old_quat,w.data.qpos[3:7]);fan=fan_model(s['pos'],s['R'],fc,cfg);u,_=w.allocate(np.r_[s['R'].T@F,T-s['R'].T@fan['torqueWorld']])
                if p.phase=='flip':
                    self.assertGreater(s['vel'][2],0);self.assertEqual(w.contact['wheelCount'],0);self.assertLess(s['pos'][2]-cfg['vehicle']['ground_cg_height'],.15);self.assertGreater(T[0],0);found=True;break
                w.advance(u,fc)
            self.assertTrue(found)
    def test_energy_reference_is_not_actual_height_and_coast_uses_zero_net_force(self):
        cfg=self.cfg;c=default_case('stationary_kickflip',cfg);w=World(cfg);w.reset(c);st=w.state();st['pos'][2]=3.;st['vel'][2]=10.;p=FeedbackPolicy(HANDCRAFTED,c['task'],c,cfg);F,T,_=p.command(st,{'wheelCount':0,'bodyContact':False})
        self.assertTrue(p.energy_ready);self.assertGreater(p.goal['energyHeight'],7.);self.assertFalse(p.goal['minimumHeightMet']);np.testing.assert_array_equal(F,[0,0,0]);self.assertGreater(T[0],0)
        st['pos'][2]=2.;st['vel'][2]=-5.;p.command(st,{'wheelCount':0,'bodyContact':False});self.assertTrue(p.energy_ready)
    def test_geometry_reference_and_conservative_roll_sweep(self):
        for angle,height in [(0,.5),(math.pi,.7),(math.pi/2,.86)]:self.assertAlmostEqual(geometry_minimum_cg(qmat(euler_q(angle,0,0)),self.cfg),height)
        R=qmat(euler_q(.4,.3,.2));bound=swept_cg_bound(R,self.cfg)
        for a in np.linspace(0,2*math.pi,49):self.assertGreaterEqual(bound+1e-10,geometry_minimum_cg(R@qmat(euler_q(a,0,0)),self.cfg))
    def test_real_apex_requirement_is_separate_from_stable_contact(self):
        cfg=copy.deepcopy(self.cfg);case=default_case('stationary_kickflip',cfg);r=rollout(case['task'],HANDCRAFTED,case,cfg)
        self.assertTrue(r['success']);self.assertTrue(r['metrics']['minimumHeightMet']);self.assertGreaterEqual(r['metrics']['peakHeight'],5.);self.assertGreaterEqual(r['metrics']['stableSeconds'],1.)
        cfg['tasks']['stationary_kickflip']['minimum_peak_cg_height']=100.
        r=rollout(case['task'],HANDCRAFTED,case,cfg);self.assertFalse(r['success']);self.assertEqual(r['status'],'height_shortfall');self.assertFalse(r['metrics']['minimumHeightMet']);self.assertGreaterEqual(r['metrics']['stableSeconds'],1.)
    def test_reward_is_accounted_and_finite(self):
        r=rollout('stationary_kickflip',HANDCRAFTED,default_case('stationary_kickflip',self.cfg),self.cfg,record=True,policy_id='handcrafted')
        finite(r);self.assertAlmostEqual(sum(f['reward']['instant'] for f in r['frames']),r['metrics']['score'],places=7)

if __name__=='__main__':unittest.main(verbosity=2)
