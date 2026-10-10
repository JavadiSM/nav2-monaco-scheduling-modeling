"""Device placement is independent of the per-device scheduling policy."""
import math,random

class DevicePlacement:
    def __init__(self,name='local',*,radius_m=5.,random_scope='covered'):
        if name not in ('local','offload','random','greedy'):raise ValueError('Unknown placement policy')
        if random_scope not in ('covered','all'):raise ValueError('Unknown random candidate scope')
        if not math.isfinite(radius_m) or radius_m<0:raise ValueError('Invalid coverage radius')
        self.name=name;self.radius_m=radius_m;self.random_scope=random_scope;self.rng=random.SystemRandom()

    def choose(self,*,owner_device,position_xy,endpoints,**kwargs):
        distances={d:math.dist(position_xy,p) for d,p in endpoints.items()}
        covered=sorted(d for d,x in distances.items() if x<=self.radius_m)
        candidates=[owner_device]+(sorted(endpoints) if self.random_scope=='all' else covered)
        if self.name=='local':selected=owner_device
        elif self.name=='offload':selected=min(covered,key=lambda d:(distances[d],d)) if covered else owner_device
        elif self.name=='greedy':
            temperatures=kwargs['device_temperatures_c']
            selected=min([owner_device]+covered,key=lambda d:(temperatures[d],d))
        else:selected=self.rng.choice(candidates)
        return selected,covered,candidates
