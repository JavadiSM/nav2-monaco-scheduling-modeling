"""Virtual-time execution of explicitly assigned, independent test jobs."""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from types import SimpleNamespace

from .queue_policy import QueuePolicy


@dataclass(frozen=True)
class Job:
    job_id: int
    demand_mcycles: float
    device_id: int
    core_id: int
    release_s: float = 0.0
    ready_s: float = 0.0
    rank: int = 0
    criticality: int = 0
    dvfs_level_id: int | None = None
    activity_factor: float = .5

    def __post_init__(self):
        for name in ('job_id', 'device_id', 'core_id', 'rank'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f'{name} must be an integer')
        for name in ('demand_mcycles', 'release_s', 'ready_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if not math.isfinite(self.activity_factor) or not 0 <= self.activity_factor <= 1:
            raise ValueError('activity_factor must be in [0, 1]')
        if self.criticality not in (0, 1):
            raise ValueError('criticality must be 0 or 1')


class VirtualScheduler(QueuePolicy):
    """A serial lane per core, with parallel cores and external readiness times.

    Mapping and DVFS are input decisions. No dependency graph is interpreted.
    All jobs default to the same class; HI is an optional test input only.
    """

    def __init__(self, devices):
        self.devices = devices
        self.scheduler = SimpleNamespace(devices=devices)
        self.state = {}
        self.rank = {}
        self.task_nodes = []
        self.core_queues = {}
        self.core_running = {}
        self._in_control_epoch = set()
        self._events = []
        self.events = []
        self._ran = False
        self._sequence = 0

    def _dependencies_satisfied(self, node_id, time_s):
        item = self.state[node_id]
        return time_s >= max(item['release_s'], item['ready_s'])

    def _is_hi(self, node_id):
        return self.state[node_id]['criticality'] == 1

    def _push(self, time_s, kind, node_id):
        self._sequence += 1
        heapq.heappush(self._events, (time_s, kind, self._sequence, node_id))

    def _try_start(self, node_id, time_s):
        item = self.state[node_id]
        key = (item['device_id'], item['core_id'])
        if item['status'] != 'planned' or not self._dependencies_satisfied(node_id, time_s):
            return False
        if self.core_running.get(key) is not None or self._queue_head(key, time_s) != node_id:
            return False
        core = self.devices[key[0]].cores[key[1]]
        metrics = core.execution_metrics(item['demand_mcycles'], item['dvfs_level_id'])
        finish = time_s + metrics['time_s']
        item.update(status='running', start_s=time_s, finish_s=finish, metrics=metrics)
        self.core_running[key] = node_id
        core.current_dvfs_level_id = metrics['dvfs_level_id']
        core.lane.reserve(node_id, time_s, finish, {'dvfs_level_id': metrics['dvfs_level_id']})
        self.events.append({'time_s': time_s, 'kind': 'start', 'job_id': node_id,
                            'device_id': key[0], 'core_id': key[1]})
        self._push(finish, 0, node_id)
        return True

    def _prepare_jobs(self, jobs):
        self._ran = True
        for device in self.devices.values():
            device.reset()
        for job in jobs:
            item = dict(vars(job), status='planned')
            core = self.devices[job.device_id].cores[job.core_id]
            item['dvfs_level_id'] = core.core_type.level(job.dvfs_level_id).level_id
            self.state[job.job_id] = item
            self.rank[job.job_id] = job.rank
            self.task_nodes.append(job.job_id)
            self._push(max(job.release_s, job.ready_s), 1, job.job_id)
        self._rebuild_core_queues()

    def run(self, jobs):
        """Consume a finite release trace without sleeping or executing ROS code."""
        if self._ran:
            raise RuntimeError('Create a fresh scheduler for each independent run')
        jobs = list(jobs)
        if len({j.job_id for j in jobs}) != len(jobs):
            raise ValueError('job_id must be unique')
        for job in jobs:
            if job.device_id not in self.devices or job.core_id not in self.devices[job.device_id].cores:
                raise ValueError('Job refers to an unknown device or core')
        self._prepare_jobs(jobs)
        while self._events:
            now = self._events[0][0]
            batch = []
            while self._events and self._events[0][0] == now:
                batch.append(heapq.heappop(self._events))
            for _, kind, _, node_id in batch:
                if kind == 0:
                    item = self.state[node_id]
                    item['status'] = 'completed'
                    self.core_running[(item['device_id'], item['core_id'])] = None
                    self.events.append({'time_s': now, 'kind': 'finish', 'job_id': node_id})
                else:
                    self.events.append({'time_s': now, 'kind': 'ready', 'job_id': node_id})
            for device_id in sorted(self.devices):
                self._dispatch_device(device_id, now)
        if any(item['status'] != 'completed' for item in self.state.values()):
            raise RuntimeError('Virtual execution stalled with unfinished jobs')
        return [self.state[job.job_id].copy() for job in jobs]
