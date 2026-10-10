"""Extensible geometric admission and task/data transport delays."""
from math import hypot, isfinite
from dataclasses import dataclass

def reachable_endpoints(position_xy, endpoints, *, radius_m=5.):
    if not isfinite(radius_m) or radius_m < 0:
        raise ValueError('Coverage radius must be finite and nonnegative')
    x,y=position_xy
    if not all(isfinite(v) for v in (x,y)):
        raise ValueError('Position must be finite')
    return [e['id'] for e in endpoints if hypot(e['x']-x,e['y']-y)<=radius_m]

def distance_delay_s(distance_m):
    distance_m=float(distance_m)
    if not isfinite(distance_m) or distance_m<0:raise ValueError('Invalid transmission distance')
    if distance_m<1:return 0.
    if distance_m<2:return .003
    if distance_m<3:return .004
    if distance_m<=5:return .005
    return .010


def task_upload_cost_s(*,distance_m,task_size_bytes=None,**kwargs):
    return distance_delay_s(distance_m)


def edge_data_cost_s(*,distance_m,edge_size_bytes=None,**kwargs):
    return distance_delay_s(distance_m)


def communication_cost_s(*args, **kwargs):
    distance=kwargs.get('distance_m')
    if distance is None and 'position_xy' in kwargs and 'endpoint' in kwargs:
        p,e=kwargs['position_xy'],kwargs['endpoint'];distance=hypot(p[0]-e['x'],p[1]-e['y'])
    return distance_delay_s(0. if distance is None else distance)


def send_eligible(position_xy, endpoint, *, radius_m=5., **kwargs):
    """Check request admission at the instant of sending, inclusively at 5 m."""
    return bool(reachable_endpoints(position_xy,[endpoint],radius_m=radius_m))


@dataclass(frozen=True)
class Transmission:
    endpoint_id:str
    direction:str
    sent_at_s:float
    delay_s:float
    arrives_at_s:float


class CommunicationModel:
    """Request-time coverage with unrestricted delivery of the admitted result."""
    def __init__(self,*,radius_m=5.,cost_function=None):
        if not isfinite(radius_m) or radius_m<0:raise ValueError('Invalid coverage radius')
        self.radius_m=radius_m
        self.cost_function=communication_cost_s if cost_function is None else cost_function

    def cost_s(self,**kwargs):
        delay=float(self.cost_function(**kwargs))
        if not isfinite(delay) or delay<0:raise ValueError('Communication delay must be finite, nonnegative seconds')
        return delay

    def _send(self,endpoint_id,direction,send_time_s,**kwargs):
        if not isfinite(send_time_s) or send_time_s<0:raise ValueError('Invalid send time')
        delay=self.cost_s(endpoint_id=endpoint_id,direction=direction,send_time_s=send_time_s,**kwargs)
        return Transmission(endpoint_id,direction,send_time_s,delay,send_time_s+delay)

    def send_request(self,*,position_xy,endpoint,send_time_s,**kwargs):
        if not send_eligible(position_xy,endpoint,radius_m=self.radius_m):raise ConnectionError('Endpoint outside send-time coverage')
        return self._send(endpoint['id'],'request',send_time_s,position_xy=position_xy,endpoint=endpoint,**kwargs)

    def send_result(self,request,*,send_time_s,**kwargs):
        if request.direction!='request' or send_time_s<request.arrives_at_s:raise ValueError('Result precedes admitted request arrival')
        # The accepted request remains valid after the vehicle leaves coverage.
        return self._send(request.endpoint_id,'result',send_time_s,request=request,**kwargs)
