"""Incremental local FIFO model driven exclusively by observed callback entries."""
from dataclasses import dataclass,field
from dataclasses import replace
import math,json
from pathlib import Path
from tools.abstract_compute.hardware import load_platform
from tools.abstract_compute.thermal import ThermalModel
from tools.abstract_compute.power import PowerModel
from tools.abstract_compute.task_model import load_tasks
from .policies import ReadyFIFOOldestIdle

TASK_NAMES=('control_iteration','local_costmap_update','global_costmap_update',
 'velocity_smoothing_tick','bt_tick','planning_request','amcl_scan_callback',
 'mppi_noise_generation','velocity_command_callback','collision_check','controller_path_install')

@dataclass
class LiveJob:
    job_id:int
    device:int
    task:str
    release_tick:int
    parents:set=field(default_factory=set)
    status:str='shadow'
    staged_tick:int|None=None
    ready_tick:int|None=None
    start_tick:int|None=None
    budget_end_tick:int|None=None
    finish_tick:int|None=None
    core_id:int|None=None
    remaining:float=0.
    real_cpu_ns:int=0
    outputs_committed:bool=False
    publications:int=0
    criticality:str="HI"
    budget_mode:str|None=None
    selected_budget_s:float|None=None
    selected_work_mcycles:float|None=None
    observed_max_exceeded:bool=False
    measured_cpu_ns:int|None=None
    consumed_work_mcycles:float=0.
    overrun_tick:int|None=None
    execution_segments:list=field(default_factory=list)

class LiveEngine:
    def __init__(self,vehicle_count=1,step_ns=1_000_000,platform_path=None,*,dual_budget=True,task_parameters=None,dvfs_selector=None,scheduling_policy=None):
        if step_ns not in (1_000_000,10_000_000):raise ValueError('Supported common lattices are 1 ms and 10 ms')
        self.step_ns=step_ns;self.dt=step_ns/1e9;self.tick=0
        # A policy returns a paired operating-point ID for each active interval.
        self.dvfs_selector=dvfs_selector
        self.scheduling_policy=scheduling_policy if scheduling_policy is not None else ReadyFIFOOldestIdle()
        platform_path=Path(platform_path) if platform_path is not None else Path(__file__).resolve().parents[2]/'config/abstract_compute.json'
        self.platform_config=json.loads(platform_path.read_text())
        self.dual_budget=dual_budget;self.tasks=load_tasks(task_parameters,dual=dual_budget);self.devices=load_platform(platform_path,device_classes=('vehicle',)*vehicle_count)
        for d in self.devices.values():d.thermal_spec=replace(d.thermal_spec,control_epoch_s=self.dt)
        self.thermal={i:ThermalModel(d) for i,d in self.devices.items()}
        for m in self.thermal.values():m.validate_idle_recovery()
        self.power=PowerModel();self.jobs={};self.next_id=1;self.events=[]
        self.running={(d,c):None for d,p in self.devices.items() for c in p.cores}
        self.idle_since={key:0 for key in self.running};self.cooling={d:False for d in self.devices}
        self.samples=[];self.unfinished=set();self._equilibria={};self.event_sink=None
    def event(self,kind,**data):
        row=dict(tick=self.tick,time_s=self.tick*self.dt,kind=kind,**data);self.events.append(row)
        if self.event_sink:self.event_sink(row)
    def arrive(self,device,task,parents=()):
        if device not in self.devices or task not in self.tasks:raise ValueError('Unknown live device/task')
        jid=self.next_id;self.next_id+=1
        j=LiveJob(jid,device,task,self.tick,remaining=0. if self.dual_budget else self.tasks[task].work_mcycles,criticality=self.tasks[task].criticality)
        self.jobs[jid]=j;self.unfinished.add(jid)
        for p in parents:self.add_parent(jid,p)
        self.event('arrival',job_id=jid,device_id=device,task=task)
        return jid
    def add_parent(self,jid,parent):
        if not parent:return
        j=self.jobs[jid]
        if parent not in self.jobs or parent==jid or self.jobs[parent].device!=j.device:raise ValueError('Invalid live parent')
        if j.start_tick is not None and (self.jobs[parent].finish_tick is None or self.jobs[parent].finish_tick>j.start_tick):raise RuntimeError('Dependency discovered after modeled child dispatch')
        frontier=[parent];seen=set()
        while frontier:
            p=frontier.pop()
            if p==jid:raise RuntimeError('Cyclic live dependency')
            if p not in seen:seen.add(p);frontier.extend(self.jobs[p].parents)
        if parent not in j.parents:j.parents.add(parent);self.event('dependency',job_id=jid,parent_id=parent)
    def stage(self,jid,cpu_ns=None,*,dispatch=True):
        j=self.jobs[jid]
        if j.status=='shadow':
            spec=self.tasks[j.task]
            if self.dual_budget:
                if cpu_ns is None:raise RuntimeError('A complete actual CPU measurement is required before staging')
                mode,budget,exceeded=spec.select_budget(cpu_ns)
                j.measured_cpu_ns=cpu_ns;j.budget_mode=mode;j.selected_budget_s=budget;j.observed_max_exceeded=exceeded
            else:j.budget_mode='Q95';j.selected_budget_s=spec.budget_s
            j.selected_work_mcycles=spec.reference_work(j.selected_budget_s);j.remaining=j.selected_work_mcycles
            j.status='queued';j.staged_tick=self.tick
            self.event('budget_selected',job_id=jid,criticality=j.criticality,budget_mode=j.budget_mode,actual_cpu_ns=j.measured_cpu_ns,C_LO_s=spec.budget_lo_s,C_HI_s=spec.budget_hi_s,selected_budget_s=j.selected_budget_s,selected_work_mcycles=j.selected_work_mcycles,observed_max_exceeded=j.observed_max_exceeded)
            self.event('result_staged',job_id=jid)
        if dispatch:self.dispatch()
    def dispatch(self):
        for jid in tuple(self.unfinished):
            j=self.jobs[jid]
            if j.status=='queued' and j.ready_tick is None and all(self.jobs[p].finish_tick is not None for p in j.parents):
                j.ready_tick=max([j.release_tick,j.staged_tick]+[self.jobs[p].finish_tick for p in j.parents])
                self.event('ready',job_id=j.job_id,eligible_tick=j.ready_tick)
        for did,device in self.devices.items():
            if self.cooling[did]:continue
            ready=[self.jobs[k] for k in sorted(self.unfinished) if (j:=self.jobs[k]).device==did and j.status=='queued' and j.ready_tick is not None]
            free=[core for c,core in device.cores.items() if self.running[did,c] is None]
            decisions=list(self.scheduling_policy.assignments(engine=self,device_id=did,ready_jobs=tuple(ready),free_cores=tuple(free)))
            eligible={j.job_id for j in ready};available={c.core_id for c in free}
            seen_jobs=set();seen_cores=set()
            for decision in decisions:
                if decision.job_id not in eligible or decision.core_id not in available or decision.job_id in seen_jobs or decision.core_id in seen_cores:
                    raise RuntimeError('Scheduling policy selected an ineligible job or occupied/duplicate core')
                seen_jobs.add(decision.job_id);seen_cores.add(decision.core_id)
            for decision in decisions:
                j=self.jobs[decision.job_id];c=decision.core_id
                assert all(self.jobs[p].finish_tick is not None and self.jobs[p].finish_tick<=self.tick for p in j.parents)
                j.status='running';j.core_id=c;j.start_tick=self.tick;self.running[did,c]=j.job_id
                device.cores[c].current_dvfs_level_id=device.cores[c].core_type.default_dvfs_level_id
                self.event('start',job_id=j.job_id,device_id=did,core_id=c)
                if j.remaining<=1e-12:j.budget_end_tick=self.tick;j.status='budget_done'
    def deliverable(self,jid):
        j=self.jobs[jid]
        return j.status=='budget_done' and not self.cooling[j.device]
    def finish(self,jid,cpu_ns=0,*,dispatch=True):
        j=self.jobs[jid]
        if not self.deliverable(jid):raise RuntimeError('Attempted early/thermal-blocked result completion')
        j.finish_tick=self.tick;j.real_cpu_ns=cpu_ns;j.status='completed';self.unfinished.remove(jid)
        key=(j.device,j.core_id);self.running[key]=None;self.idle_since[key]=self.tick
        self.event('finish',job_id=jid,device_id=j.device,core_id=j.core_id,real_cpu_ns=cpu_ns)
        if dispatch:self.dispatch()
    def _mark_overrun(self,j):
        if not self.dual_budget or j.budget_mode!='HI' or j.overrun_tick is not None:return
        threshold=self.tasks[j.task].reference_work(self.tasks[j.task].budget_lo_s)
        if j.consumed_work_mcycles>=threshold-1e-9:
            j.overrun_tick=self.tick
            self.event('job_overrun',job_id=j.job_id,consumed_work_mcycles=j.consumed_work_mcycles,remaining_work_mcycles=j.remaining,policy='remaining_work_at_maximum_point')

    def _operating_point(self,j,core):
        self._mark_overrun(j)
        requested=None
        if j.overrun_tick is None:
            requested=self.dvfs_selector(self,j,core) if self.dvfs_selector is not None else self.scheduling_policy.dvfs_level(engine=self,job=j,core=core)
        level_id=core.core_type.default_dvfs_level_id if requested is None else requested
        if isinstance(level_id,bool) or not isinstance(level_id,int) or level_id not in core.core_type.dvfs_by_id:
            raise ValueError('DVFS policy must return a valid integer operating-point ID or None')
        previous=core.current_dvfs_level_id
        core.current_dvfs_level_id=level_id
        if previous!=level_id:
            point=core.dvfs_level()
            self.event('dvfs_change',job_id=j.job_id,device_id=j.device,core_id=core.core_id,previous_level_id=previous,level_id=level_id,frequency_mhz=point.frequency_mhz,voltage_v=point.voltage_v,reason='overrun' if j.overrun_tick is not None else 'scheduler')
        return core.dvfs_level()

    def _consume_interval(self,j,core,level):
        rate=level.frequency_mhz*core.core_type.performance_eta
        work=min(j.remaining,rate*self.dt)
        j.remaining=max(0.,j.remaining-work)
        j.consumed_work_mcycles=j.selected_work_mcycles-j.remaining
        spans=j.execution_segments
        if spans and spans[-1]['end_tick']==self.tick and spans[-1]['level_id']==level.level_id:
            spans[-1]['end_tick']=self.tick+1;spans[-1]['work_mcycles']+=work
        else:
            spans.append(dict(start_tick=self.tick,end_tick=self.tick+1,level_id=level.level_id,frequency_mhz=level.frequency_mhz,voltage_v=level.voltage_v,work_mcycles=work))

    def advance(self):
        self.dispatch()
        for did,device in self.devices.items():
            powers=[]
            for cid,core in device.cores.items():
                jid=self.running[did,cid];active=jid is not None and self.jobs[jid].status=='running' and not self.cooling[did]
                if active:
                    j=self.jobs[jid];level=self._operating_point(j,core)
                    self._consume_interval(j,core,level)
                powers.append(self.power.runtime_power_w(core,is_active=active))
            temp=self.thermal[did].evolve_temperature_c(self.thermal[did].temperature_vector_c(),powers,self.dt)
            for value,core in zip(temp,device.cores.values()):core.temperature_c=float(value)
            if self.tick%10==0:
                sample=dict(tick=self.tick+1,device_id=did,temperature_c=list(map(float,temp)),power_w=powers,cooling=self.cooling[did])
                self.samples.append(sample)
                if self.event_sink:self.event_sink(dict(kind='thermal',**sample))
        self.tick+=1
        for jid in self.running.values():
            if jid is not None and self.jobs[jid].status=='running':self._mark_overrun(self.jobs[jid])
        # Guards precede output release at a coincident completion/cooling tick.
        for did,device in self.devices.items():
            hot=[c for c,v in device.cores.items() if v.temperature_c>=v.max_temperature_c-1e-9]
            resume=all(c.temperature_c<=c.balance_temperature_c+1e-9 for c in device.cores.values())
            changed=(not self.cooling[did] and bool(hot)) or (self.cooling[did] and resume)
            if changed:
                self.cooling[did]=not self.cooling[did]
                for c in device.cores.values():c.thermal_forced_idle=self.cooling[did]
                self.event('cooling_start' if self.cooling[did] else 'cooling_end',device_id=did,trigger_core_ids=hot,temperature_c={c:v.temperature_c for c,v in device.cores.items()})
            for cid in device.cores:
                jid=self.running[did,cid]
                if jid is not None and self.jobs[jid].status=='running' and self.jobs[jid].remaining<=1e-9:
                    j=self.jobs[jid];j.status='budget_done';j.budget_end_tick=self.tick;self.event('budget_complete',job_id=jid)
        self.dispatch()
