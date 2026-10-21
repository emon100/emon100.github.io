"""Conservative geometry-specific priority allocation, without an LP solver.

Later force/yaw components are zero while the earlier priority is selected.
This is feasible lexicographic guidance, not the globally optimal coupled LP.
"""
import numpy as np
from canonical_allocation import canonicalize

BISECTION_STEPS=40

def allocate(parts,limits,cap,matrix):
    parts=np.asarray(parts,dtype=float);limits=np.asarray(limits,dtype=float);A=np.asarray(matrix,dtype=float)
    if parts.shape!=(3,6) or limits.shape!=(12,) or A.shape!=(6,12) or not np.isfinite(parts).all() or not np.isfinite(limits).all() or not np.isfinite(A).all() or np.any(limits<0) or not np.isfinite(cap) or cap<0:raise ValueError('Invalid allocation inputs')
    canonicalize(np.zeros(6),limits,A) # Validate this geometry before feasibility probes.
    scales=np.zeros(3);calls=0;iterations=[]
    def feasible(candidate):
        nonlocal calls
        calls+=1
        try:forces=canonicalize(candidate@parts,limits,A)
        except ValueError:return None
        return forces if float(forces.sum())<=cap else None
    best=np.zeros(12)
    for priority in range(3):
        candidate=scales.copy();candidate[priority]=1.;forces=feasible(candidate)
        if forces is not None:scales=candidate;best=forces;iterations.append(0);continue
        low=0.;high=1.
        for _ in range(BISECTION_STEPS):
            candidate=scales.copy();candidate[priority]=(low+high)/2;forces=feasible(candidate)
            if forces is None:high=candidate[priority]
            else:low=candidate[priority];best=forces
        scales[priority]=low;iterations.append(BISECTION_STEPS)
    achieved=A@best;requested=scales@parts
    if np.max(np.abs(achieved-requested))>1e-5 or np.any(best<0) or np.any(best>limits) or float(best.sum())>cap:raise RuntimeError('Analytic allocation certificate failed')
    return {'forces':best.tolist(),'tiltScale':float(scales[0]),'forceScale':float(scales[1]),'yawScale':float(scales[2]),'achievedWrench':achieved.tolist(),'totalForce':float(best.sum()),'iterations':iterations+[0],'feasibilityCalls':calls,'method':'conservative_analytic_priority','globallyOptimal':False,'solverRetries':0}
