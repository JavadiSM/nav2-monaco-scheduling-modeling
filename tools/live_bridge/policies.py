"""Replaceable ready-job selection, core assignment and operating-point policies."""
import math
from dataclasses import dataclass
from typing import Protocol,Sequence,TYPE_CHECKING
if TYPE_CHECKING:
    from .engine import LiveEngine,LiveJob
    from tools.abstract_compute.hardware import Core


@dataclass(frozen=True)
class Assignment:
    job_id:int
    core_id:int


class SchedulingPolicy(Protocol):
    name:str
    def assignments(self,*,engine:'LiveEngine',device_id:int,ready_jobs:Sequence['LiveJob'],free_cores:Sequence['Core'],**kwargs)->Sequence[Assignment]:...
    def dvfs_level(self,*,engine:'LiveEngine',job:'LiveJob',core:'Core',**kwargs)->int|None:...


class ReadyFIFOOldestIdle:
    name='ready_FIFO_oldest_idle_core_nonpreemptive'

    def assignments(self,*,engine,device_id,ready_jobs,free_cores,**kwargs):
        jobs=sorted(ready_jobs,key=lambda j:(j.ready_tick,j.release_tick,j.job_id))
        cores=sorted(free_cores,key=lambda c:(engine.idle_since[device_id,c.core_id],c.core_id))
        return [Assignment(j.job_id,c.core_id) for j,c in zip(jobs,cores)]

    def dvfs_level(self,*,engine,job,core,**kwargs):
        return None


class FIFOShortestFinish:
    """Map eligible jobs in FIFO order to the least predicted finish-time queue.

    Queue predictions use the selected operating point, current remaining work and
    FIFO reservations. Future thermal pauses cannot be predicted by this policy;
    the engine enforces every thermal guard when executing the reservations.
    """
    name='ready_FIFO_shortest_finish_queue_nonpreemptive'

    def __init__(self):self.queues={}

    def service_ticks(self,engine,job,core):
        rate=engine.planned_point(job,core).frequency_mhz*core.core_type.performance_eta
        return max(0,math.ceil(job.remaining/(rate*engine.dt)-1e-9))

    def assignments(self,*,engine,device_id,ready_jobs,free_cores,**kwargs):
        cores=engine.devices[device_id].cores
        for cid in cores:
            q=self.queues.setdefault((device_id,cid),[])
            q[:]=[j for j in q if engine.jobs[j].status=='queued']
        for job in sorted(ready_jobs,key=lambda j:(j.ready_tick,j.release_tick,j.job_id)):
            if job.queued_core_id is not None:continue
            predicted={}
            for cid,core in cores.items():
                current=engine.running[device_id,cid]
                tail=engine.tick
                if current is not None:tail+=self.service_ticks(engine,engine.jobs[current],core)
                tail+=sum(self.service_ticks(engine,engine.jobs[j],core) for j in self.queues[device_id,cid])
                predicted[cid]=tail+self.service_ticks(engine,job,core)
            cid=min(predicted,key=lambda c:(predicted[c],c))
            job.queued_core_id=cid;self.queues[device_id,cid].append(job.job_id)
            engine.event('core_queue_assignment',job_id=job.job_id,device_id=device_id,core_id=cid,predicted_finish_tick=predicted[cid],candidate_finish_ticks=predicted)
        ready_ids={j.job_id for j in ready_jobs}
        result=[]
        for core in sorted(free_cores,key=lambda c:c.core_id):
            q=self.queues[device_id,core.core_id]
            if q and q[0] in ready_ids:result.append(Assignment(q[0],core.core_id))
        return result

    def dvfs_level(self,**kwargs):return None


def configured_scheduler(config):
    name=config.get('policy',ReadyFIFOOldestIdle.name)
    if name==ReadyFIFOOldestIdle.name:return ReadyFIFOOldestIdle()
    if name==FIFOShortestFinish.name:return FIFOShortestFinish()
    raise ValueError('Unknown scheduling policy; supply a SchedulingPolicy instance')
