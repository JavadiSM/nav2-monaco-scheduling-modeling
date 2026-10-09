"""Replaceable ready-job selection, core assignment and operating-point policies."""
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


def configured_scheduler(config):
    name=config.get('policy',ReadyFIFOOldestIdle.name)
    if name==ReadyFIFOOldestIdle.name:return ReadyFIFOOldestIdle()
    raise ValueError('Unknown scheduling policy; supply a SchedulingPolicy instance')
