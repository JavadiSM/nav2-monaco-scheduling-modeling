"""Ideal user-defined geometric edge coverage; no transport is simulated."""
from math import hypot, isfinite

def reachable_endpoints(position_xy, endpoints, *, radius_m=5.):
    if not isfinite(radius_m) or radius_m < 0:
        raise ValueError('Coverage radius must be finite and nonnegative')
    x,y=position_xy
    if not all(isfinite(v) for v in (x,y)):
        raise ValueError('Position must be finite')
    return [e['id'] for e in endpoints if hypot(e['x']-x,e['y']-y)<=radius_m]

def communication_cost_s():
    return 0.
