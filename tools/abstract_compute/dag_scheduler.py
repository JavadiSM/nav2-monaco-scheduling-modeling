"""Dependency-safe ready FIFO on dynamically chosen local heterogeneous cores."""
from __future__ import annotations
import math
from .scheduler import Job
from .thermal_scheduler import ThermalScheduler


class DependencyFIFOScheduler(ThermalScheduler):
    """FIFO begins at eligibility: max(release, every selected parent's finish).

    Dispatch uses the core that has been idle longest, then its numeric ID.
    Jobs are mapped only when starting; no future core is reserved by a child.
    Thermal cooling retains running ownership and parent completion semantics.
    """
    def __init__(self, devices, *, device_id=0):
        if device_id not in devices:
            raise ValueError('Unknown local device')
        # The initial baseline intentionally uses only this endpoint.
        super().__init__({device_id: devices[device_id]})
        self.device_id = device_id
        self.graph_jobs = {}
        self.input_order = {}
        self.idle_since = {}
        self._finished_seen = set()

    def _dependencies_satisfied(self, node_id, time_s):
        item = self.state[node_id]
        return (time_s >= item['release_s'] and
                all(self.state[p]['status'] == 'completed' for p in item['parents']))

    def _prepare_jobs(self, jobs):
        super()._prepare_jobs(jobs)
        self.core_queues = {}
        for n, item in self.state.items():
            g = self.graph_jobs[n]
            item.update(task=g.task, parents=list(g.parents), core_id=None, eligible_s=None)
        self.idle_since = {cid: 0. for cid in self.devices[self.device_id].cores}

    def _dispatch_device(self, device_id, time_s):
        cores = self.devices[device_id].cores
        for n, item in self.state.items():
            if item['status'] == 'completed' and n not in self._finished_seen:
                self.idle_since[item['core_id']] = item['finish_s']
                self._finished_seen.add(n)
            if item['status'] == 'planned' and item['eligible_s'] is None and self._dependencies_satisfied(n, time_s):
                item['eligible_s'] = max([item['release_s']] + [self.state[p]['finish_s'] for p in item['parents']])
                self.events.append(dict(kind='ready', time_s=item['eligible_s'], job_id=n))
        if any(c.thermal_forced_idle for c in cores.values()):
            return
        ready = sorted((n for n, x in self.state.items()
                        if x['status'] == 'planned' and x['eligible_s'] is not None),
                       key=lambda n: (self.state[n]['eligible_s'], self.state[n]['release_s'], self.input_order[n]))
        free = sorted((cid for cid in cores if self.core_running.get((device_id, cid)) is None),
                      key=lambda cid: (self.idle_since[cid], cid))
        for n, cid in zip(ready, free):
            self.state[n]['core_id'] = cid
            self.state[n]['dvfs_level_id'] = cores[cid].core_type.level(self.graph_jobs[n].dvfs_level_id).level_id
            self.core_queues[(device_id, cid)] = [n]
            if not self._try_start(n, time_s):
                raise RuntimeError('Eligible job could not be dispatched to a free core')

    def run_graph(self, jobs, *, max_time_s=600., sample_period_s=.01):
        jobs = list(jobs)
        self.graph_jobs = {j.job_id: j for j in jobs}
        if len(self.graph_jobs) != len(jobs):
            raise ValueError('Job IDs must be unique')
        self.input_order = {j.job_id: i for i, j in enumerate(jobs)}
        children = {j.job_id: [] for j in jobs}
        indegree = {}
        for j in jobs:
            if len(set(j.parents)) != len(j.parents):
                raise ValueError('Duplicate parent')
            indegree[j.job_id] = len(j.parents)
            for p in j.parents:
                if p not in self.graph_jobs:
                    raise ValueError('Unknown parent')
                children[p].append(j.job_id)
        frontier = [n for n in indegree if not indegree[n]]
        seen = 0
        while frontier:
            n = frontier.pop()
            seen += 1
            for child in children[n]:
                indegree[child] -= 1
                if not indegree[child]:
                    frontier.append(child)
        if seen != len(jobs):
            raise ValueError('Dependencies must form a DAG')
        placeholder = min(self.devices[self.device_id].cores)
        converted = [Job(j.job_id, j.demand_mcycles, self.device_id, placeholder,
                         release_s=j.release_s, rank=i, dvfs_level_id=j.dvfs_level_id,
                         activity_factor=j.activity_factor) for i, j in enumerate(jobs)]
        return super().run(converted, max_time_s=max_time_s, sample_period_s=sample_period_s)
