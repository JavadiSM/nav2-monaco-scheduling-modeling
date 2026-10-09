"""Small operating-point tables and ordered CPU interval reservations."""
from __future__ import annotations
from bisect import insort_right
from dataclasses import dataclass, field
import math


def finite_float(value, default=0.):
    try:
        number = float(value)
        if math.isfinite(number):
            return number
    except (TypeError, ValueError, OverflowError):
        pass
    return float(default)


def probability(value, default=0.):
    number = finite_float(value, default)
    return max(0., min(number, 1.))


def nonnegative_float(value, default=0.):
    return max(finite_float(value, default), 0.)


@dataclass(frozen=True)
class OperatingPoint:
    level_id: int
    frequency_mhz: float
    voltage_v: float


def make_operating_points(frequency_mhz, voltage_v, levels):
    """Interpolate the two configured endpoints; no external table formats."""
    if isinstance(levels, bool) or not isinstance(levels, int) or levels < 1:
        raise ValueError('Operating-point count must be a positive integer')
    f0, f1 = map(float, frequency_mhz)
    v0, v1 = map(float, voltage_v)
    if not all(math.isfinite(x) and x > 0 for x in (f0, f1, v0, v1)):
        raise ValueError('Frequency and voltage endpoints must be positive and finite')
    if f0 > f1 or v0 > v1:
        raise ValueError('Operating-point ranges must be ascending')
    count = 1 if f0 == f1 else max(levels, 2)
    points = []
    for index in range(count):
        fraction = index / (count - 1) if count > 1 else 1.
        points.append(OperatingPoint(index, f0 + fraction * (f1 - f0), v0 + fraction * (v1 - v0)))
    return tuple(points)


@dataclass
class ExecutionLane:
    timeline: list = field(default_factory=list)

    def earliest_slot(self, duration, ready_time):
        candidate = float(ready_time)
        for occupied in self.timeline:
            if occupied['finish'] <= candidate:
                continue
            if occupied['start'] - candidate >= duration:
                return candidate, candidate + duration
            candidate = occupied['finish']
        return candidate, candidate + duration

    def reserve(self, node_id, start, finish, metadata=None):
        entry = dict(metadata or {})
        entry.update(node_id=node_id, start=float(start), finish=float(finish))
        insort_right(self.timeline, entry, key=lambda interval: interval['start'])

    @property
    def available_time(self):
        return self.timeline[-1]['finish'] if self.timeline else 0.

    def reset(self):
        self.timeline.clear()
