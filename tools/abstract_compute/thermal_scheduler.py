"""Virtual CPU execution with device-wide thermal pause and recovery."""
from __future__ import annotations

import math

from .power import PowerModel
from .scheduler import VirtualScheduler
from .thermal import ThermalModel


class ThermalScheduler(VirtualScheduler):
    """Progress stops on every core of a cooling device; queued work is retained.

    Thermal guards run on a global control lattice. Completion is handled before
    guards at a coincident timestamp. This is a compute-only pause policy.
    """

    def __init__(self, devices):
        super().__init__(devices)
        self.power_model = PowerModel()
        self.models = {}
        self.samples = []
        self.cooling_events = []
        self.work_segments = []

    def _try_start(self, node_id, time_s):
        item = self.state[node_id]
        did, cid = item['device_id'], item['core_id']
        key = (did, cid)
        if any(c.thermal_forced_idle for c in self.devices[did].cores.values()):
            return False
        if item['status'] != 'planned' or not self._dependencies_satisfied(node_id, time_s):
            return False
        if self.core_running.get(key) is not None or self._queue_head(key, time_s) != node_id:
            return False
        core = self.devices[did].cores[cid]
        metrics = core.execution_metrics(item['demand_mcycles'], item['dvfs_level_id'])
        core.current_dvfs_level_id = metrics['dvfs_level_id']
        item.update(status='running', start_s=time_s, remaining_mcycles=item['demand_mcycles'],
                    executed_mcycles=0., active_time_s=0., metrics=metrics)
        self.core_running[key] = node_id
        self.events.append({'time_s': time_s, 'kind': 'start', 'job_id': node_id,
                            'device_id': did, 'core_id': cid})
        return True

    def _guard(self, did, now):
        cores = self.devices[did].cores
        hot = [cid for cid, c in cores.items() if c.temperature_c >= c.max_temperature_c - 1e-9]
        was_idle = any(c.thermal_forced_idle for c in cores.values())
        if not was_idle and hot:
            forced, kind = True, 'cooling_start'
        elif was_idle and all(c.temperature_c <= c.balance_temperature_c + 1e-9 for c in cores.values()):
            forced, kind = False, 'cooling_end'
        else:
            return
        for cid, core in cores.items():
            core.thermal_forced_idle = forced
            n = self.core_running.get((did, cid))
            if n is not None:
                self.state[n]['status'] = 'thermal_wait' if forced else 'running'
        event = {'time_s': now, 'kind': kind, 'device_id': did,
                 'affected_core_ids': sorted(cores), 'trigger_core_ids': hot,
                 'temperature_c': {cid: c.temperature_c for cid, c in cores.items()}}
        self.cooling_events.append(event)
        self.events.append(event)

    def _power(self, did):
        values = []
        for cid, core in self.devices[did].cores.items():
            n = self.core_running.get((did, cid))
            active = n is not None and self.state[n]['status'] == 'running'
            item = self.state[n] if n is not None else {}
            values.append(self.power_model.runtime_power_w(
                core, is_active=active, activity_factor=item.get('activity_factor'),
                dvfs_level_id=item.get('dvfs_level_id')))
        return values

    def run(self, jobs, *, max_time_s=600., sample_period_s=.01):
        if self._ran:
            raise RuntimeError('Create a fresh scheduler for each run')
        jobs = list(jobs)
        if len({j.job_id for j in jobs}) != len(jobs):
            raise ValueError('job_id must be unique')
        for j in jobs:
            if j.device_id not in self.devices or j.core_id not in self.devices[j.device_id].cores:
                raise ValueError('Unknown device or core')
        if not math.isfinite(max_time_s) or max_time_s <= 0 or sample_period_s <= 0:
            raise ValueError('Simulation limits must be positive')
        self._prepare_jobs(jobs)
        self._events.clear()
        self.models = {did: ThermalModel(device) for did, device in self.devices.items()}
        for model in self.models.values():
            model.validate_idle_recovery()
        periods = {round(d.thermal_spec.control_epoch_s * 1e9) for d in self.devices.values()}
        if len(periods) != 1 or min(periods) <= 0:
            raise ValueError('Devices must share one positive control period')
        tick_ns = periods.pop()
        tick_index = 0
        next_sample = 0.
        now = 0.
        while any(x['status'] != 'completed' for x in self.state.values()):
            if now > max_time_s:
                raise RuntimeError('Virtual-time limit reached with unfinished work')
            for n, item in self.state.items():
                if item['status'] in ('running', 'thermal_wait') and item['remaining_mcycles'] <= 1e-9:
                    item.update(status='completed', finish_s=now)
                    key = (item['device_id'], item['core_id'])
                    self.core_running[key] = None
                    self.devices[key[0]].cores[key[1]].lane.reserve(n, item['start_s'], now)
                    self.events.append({'time_s': now, 'kind': 'finish', 'job_id': n})
            # Integer lattice bookkeeping prevents floating-point control drift.
            if round(now * 1e9) == tick_index * tick_ns:
                for did in self.devices:
                    self._guard(did, now)
                tick_index += 1
            for did in sorted(self.devices):
                self._dispatch_device(did, now)
            power = {did: self._power(did) for did in self.devices}
            if now >= next_sample or all(x['status'] == 'completed' for x in self.state.values()):
                for did, device in self.devices.items():
                    self.samples.append({'time_s': now, 'device_id': did,
                        'temperature_c': {cid: c.temperature_c for cid, c in device.cores.items()},
                        'power_w': dict(zip(device.cores, power[did])),
                        'cooling': any(c.thermal_forced_idle for c in device.cores.values())})
                next_sample = now + sample_period_s
            if all(x['status'] == 'completed' for x in self.state.values()):
                break
            next_times = [tick_index * tick_ns / 1e9]
            for item in self.state.values():
                if item['status'] == 'planned':
                    ready = max(item['release_s'], item['ready_s'])
                    if ready > now:
                        next_times.append(ready)
                elif item['status'] == 'running':
                    rate = item['metrics']['performance_eta'] * item['metrics']['frequency_mhz']
                    next_times.append(now + item['remaining_mcycles'] / rate)
            end = min(next_times)
            dt = end - now
            if dt < 0:
                raise RuntimeError('Virtual clock moved backwards')
            for did, model in self.models.items():
                device = self.devices[did]
                before = model.temperature_vector_c()
                after = model.evolve_temperature_c(before, power[did], dt)
                for index, (cid, core) in enumerate(device.cores.items()):
                    core.temperature_c = float(after[index])
                    n = self.core_running.get((did, cid))
                    if n is not None and self.state[n]['status'] == 'running':
                        item = self.state[n]
                        rate = item['metrics']['performance_eta'] * item['metrics']['frequency_mhz']
                        work = min(item['remaining_mcycles'], rate * dt)
                        item['remaining_mcycles'] -= work
                        item['executed_mcycles'] += work
                        item['active_time_s'] += dt
                        self.work_segments.append({'job_id': n, 'device_id': did, 'core_id': cid,
                            'start_s': now, 'end_s': end, 'executed_mcycles': work})
                model.time_s = end
            now = end
        return [self.state[j.job_id].copy() for j in jobs]
