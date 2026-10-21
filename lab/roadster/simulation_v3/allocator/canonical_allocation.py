"""Choose a deterministic minimum-flow solution for this exact 12-jet geometry.

This does not solve a new control objective. HiGHS has already selected the
achievable wrench. Four signed vertical forces have one nullspace coordinate;
their L1 minimum is a median interval. Horizontal minimum-flow pairs are unique.
"""
import numpy as np

def canonicalize(wrench,limits,matrix):
    a=np.asarray(matrix);w=-a[3,0];l=a[4,0];z=a[4,8]
    if w<=0 or l<=0:raise ValueError('Unsupported nozzle geometry')
    Fx,Fy,Fz,tx,ty,tz=map(float,wrench);tx+=z*Fy;ty-=z*Fx
    base=np.array([Fz/4+tx*a[3,j]/(4*w*w)+ty*a[4,j]/(4*l*l) for j in [0,2,4,6]])
    signs=np.array([1.,-1.,-1.,1.]);low=-np.asarray(limits)[1:8:2];high=np.asarray(limits)[:8:2]
    aa=(low-base)/signs;bb=(high-base)/signs;left=float(max(np.minimum(aa,bb)));right=float(min(np.maximum(aa,bb)))
    if left>right+1e-5:raise ValueError('LP wrench outside vertical feasible interval')
    roots=sorted((-base/signs).tolist());middle=(roots[1]+roots[2])/2
    alpha=(left+right)/2 if left>right else max(left,min(right,middle))
    net=base+alpha*signs;forces=np.zeros(12);forces[:8:2]=np.maximum(net,0);forces[1:8:2]=np.maximum(-net,0)
    horizontal=np.array([max(Fx,0),max(-Fx,0),max(Fy,0),max(-Fy,0)])
    delta=tz-(w*abs(Fx)-l*abs(Fy))
    if delta>=0:horizontal[:2]+=delta/(2*w)
    else:horizontal[2:]+=-delta/(2*l)
    forces[8:]=horizontal
    if np.max(forces-np.asarray(limits))>1e-5:raise ValueError('Canonical force outside nozzle bounds')
    forces=np.clip(forces,0,limits)
    if np.max(np.abs(a@forces-wrench))>1e-5:raise ValueError('Canonical wrench changed')
    return forces
