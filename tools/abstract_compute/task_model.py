"""Measured task budgets and explicitly normalized virtual work units."""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

@dataclass(frozen=True)
class TaskSpec:
    task: str
    task_id: str
    activation: str
    budget_s: float
    period_s: float | None
    observed_interval_s: float
    reference_frequency_mhz: float
    reference_eta: float
    criticality: str = "HI"
    budget_lo_s: float | None = None
    budget_hi_s: float | None = None

    def reference_work(self, seconds):
        return seconds * self.reference_frequency_mhz * self.reference_eta

    def select_budget(self, actual_cpu_ns):
        if actual_cpu_ns < 0:
            raise ValueError("CPU demand cannot be negative")
        lo = self.budget_s if self.budget_lo_s is None else self.budget_lo_s
        hi = self.budget_s if self.budget_hi_s is None else self.budget_hi_s
        actual_s = actual_cpu_ns / 1e9
        mode = "LO" if actual_s <= lo else "HI"
        return mode, lo if mode == "LO" else hi, actual_s > hi

    @property
    def work_mcycles(self):
        return self.budget_s * self.reference_frequency_mhz * self.reference_eta

    @property
    def activation_rate_hz(self):
        return None if self.period_s is None else 1. / self.period_s

    def execution_s(self, core, level_id=None):
        level = core.core_type.level(level_id)
        return self.work_mcycles / (level.frequency_mhz * core.core_type.performance_eta)

    def period_cycles(self, core, level_id=None):
        if self.period_s is None:
            return None
        return self.period_s * core.core_type.level(level_id).frequency_mhz * 1e6

    def execution_cycles(self, core, level_id=None):
        level = core.core_type.level(level_id)
        return self.execution_s(core, level_id) * level.frequency_mhz * 1e6


def load_tasks(parameters=None, model_config=None, *, dual=False):
    parameters = Path(parameters or ROOT / ('docs/evidence/dual-budget-parameters.json' if dual else 'docs/evidence/extracted-parameters.json'))
    model_config = Path(model_config or ROOT / 'config/task_execution.json')
    config = json.loads(model_config.read_text())
    reference = config['reference']
    frequency, eta = reference['frequency_mhz'], reference['performance_eta']
    if any(not math.isfinite(x) or x <= 0 for x in (frequency, eta)):
        raise ValueError('Reference frequency and performance must be finite and positive')
    tasks = {}
    for name, row in json.loads(parameters.read_text())['tasks'].items():
        c, period = row['C_HI_s'] if dual else row['C_model_s'], row['T_nominal_s']
        lo, hi = (row['C_LO_s'], row['C_HI_s']) if dual else (c, c)
        if not (math.isfinite(lo) and math.isfinite(hi) and 0 <= lo <= hi):
            raise ValueError('Invalid LO/HI budgets')
        if not math.isfinite(c) or c < 0 or (period is not None and (not math.isfinite(period) or period <= 0)):
            raise ValueError('Invalid task budget or period')
        tasks[name] = TaskSpec(name, row['task_id'], row['class'], c, period,
                              row['observed_mean_inter_entry_s'], frequency, eta,
                              row.get('criticality', 'HI'), lo, hi)
    return tasks

@dataclass(frozen=True)
class GraphJob:
    job_id: int
    task: str
    demand_mcycles: float
    release_s: float = 0.
    parents: tuple[int, ...] = ()
    dvfs_level_id: int | None = None
    activity_factor: float = .5


def instantiate_graph(tasks, graph, *, release_s=0., releases=None, id_offset=0):
    """Instantiate selected versions, not every possible family-to-family edge."""
    releases = releases or {}
    names = list(graph['nodes'])
    ids = {name: i + id_offset for i, name in enumerate(names)}
    parents = {name: [] for name in names}
    for edge in graph['edges']:
        if edge['from'] not in ids or edge['to'] not in ids:
            raise ValueError('Graph edge references an unknown vertex')
        parents[edge['to']].append(ids[edge['from']])
    return [GraphJob(ids[name], graph['nodes'][name]['task'],
                     tasks[graph['nodes'][name]['task']].work_mcycles,
                     releases.get(name, release_s), tuple(parents[name])) for name in names]


def periodic_jobs(task, horizon_s, *, phase_s=0., id_offset=0):
    """Aperiodic jobs must be supplied with explicit arrivals instead."""
    if task.period_s is None:
        raise ValueError('Aperiodic tasks have no periodic release generator')
    if not math.isfinite(horizon_s) or horizon_s < 0 or not math.isfinite(phase_s) or phase_s < 0:
        raise ValueError('Invalid finite release horizon')
    count = max(0, math.ceil((horizon_s - phase_s) / task.period_s))
    return [GraphJob(id_offset + k, task.task, task.work_mcycles, phase_s + k * task.period_s)
            for k in range(count) if phase_s + k * task.period_s < horizon_s]
