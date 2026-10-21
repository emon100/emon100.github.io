"""Fixed-attitude 1D braking preview with the plant's 12 valve/pressure states."""
from pathlib import Path
import sys,math
import numpy as np
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent/'baseline_r6'))
import core


def target_upward_throttles(state,cfg,fraction=.85):
    A=core.wrench_matrix(core.nozzle_layout(cfg));fmax=np.array([n['maxForce'] for n in core.nozzle_layout(cfg)])*state.get('thrustScale',1.)
    for i,n in enumerate(core.nozzle_layout(cfg)):
        if n['id'] in state.get('disabledNozzles',[]):fmax[i]=0.
    R=np.asarray(state['R']);at_unit=core.allocate_fixed_nozzles(np.r_[R.T@np.array([0.,0.,1e6]),np.zeros(3)],fmax,cfg);force_z=float((R@(A@at_unit)[:3])[2]);u=np.divide(at_unit*fraction,fmax,out=np.zeros(12),where=fmax>0)
    return u,fmax,A,force_z


def predict_vertical_stop(state,cfg,fraction=.85,touchdown_speed=.2,horizon=6.):
    dt=cfg['physics']['dt'];window=round(cfg['physics']['control_dt']/dt);gravity=cfg['physics']['gravity'];velocity=float(state['vel'][2]);fuel=float(state['fuel']);dry=float(state['mass'])-fuel;initial_fuel=fuel;R=np.asarray(state['R']);u=np.asarray(state.get('actualNozzleThrottle',np.zeros(12)),dtype=float).copy();target,fmax,A,unit_lift=target_upward_throttles(state,cfg,fraction);response=1-math.exp(-dt/cfg['actuators']['lag_seconds']);distance=0.;elapsed=0.;mass=dry+fuel
    if velocity>=-touchdown_speed:return {'feasible':True,'distance':0.,'time':0.,'gasUsed':0.,'endFuel':fuel,'endPressure':float(core.delivery(fuel,cfg)[0]),'maximumZeroMomentLiftWeightRatio':float(unit_lift*core.delivery(fuel,cfg)[0]/(mass*gravity)),'targetLiftWeightRatio':float(fraction*unit_lift*core.delivery(fuel,cfg)[0]/(mass*gravity)), 'endLiftWeightRatioMeaning':'legacy full zero-moment capacity, not actual thrust', 'endLiftWeightRatio':float(unit_lift*core.delivery(fuel,cfg)[0]/(mass*gravity)),'dt':dt,'massUpdateInterval':cfg['physics']['control_dt']}
    for step in range(round(horizon/dt)):
        if step%window==0:mass=dry+fuel
        u+=(target-u)*response;pressure,isp=core.delivery(fuel,cfg);forces=u*fmax*pressure;cap=min(cfg['actuators']['total_force_max']*pressure*state.get('thrustScale',1.),fuel*isp*gravity/dt);total=float(forces.sum())
        if total>cap:forces*=cap/max(total,1e-12)
        fuel=max(0.,fuel-float(forces.sum())*dt/(isp*gravity));force_z=float((R@(A@forces)[:3])[2]);velocity+=(force_z/mass-gravity)*dt;distance-=velocity*dt;elapsed=(step+1)*dt
        if velocity>=-touchdown_speed:break
    pressure,_=core.delivery(fuel,cfg)
    return {'feasible':bool(velocity>=-touchdown_speed),'distance':float(distance),'time':float(elapsed),'gasUsed':float(initial_fuel-fuel),'endFuel':float(fuel),'endPressure':float(pressure),'maximumZeroMomentLiftWeightRatio':float(unit_lift*pressure/max((dry+fuel)*gravity,1.)),'targetLiftWeightRatio':float(fraction*unit_lift*pressure/max((dry+fuel)*gravity,1.)),'lastPredictedActualLiftWeightRatio':float(force_z/(mass*gravity)),'endLiftWeightRatioMeaning':'legacy full zero-moment capacity, not actual thrust','endLiftWeightRatio':float(unit_lift*pressure/max((dry+fuel)*gravity,1.)),'predictedFinalVz':float(velocity),'dt':dt,'massUpdateInterval':cfg['physics']['control_dt'],'assumption':'Fixed attitude, fixed zero-moment upward throttle target. Exact valve/flow/pressure and 20ms mass schedule; does not predict changing attitude or additional future attitude commands.'}
