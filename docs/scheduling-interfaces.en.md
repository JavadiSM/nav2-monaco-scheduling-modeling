# Scheduling and communication extension interfaces

Device placement and scheduling inside each device are independent interfaces. The `local`, `offload`, `random` and `greedy` experiments use the same per-device FIFO / shortest predicted finish-time queue policy and normal middle DVFS with the same device-wide maximum-frequency reaction after overrun.

## Placement

`DevicePlacement.choose(*, owner_device, position_xy, endpoints, **kwargs)` returns an execution device and its recorded candidate sets. `owner_device` identifies the original ROS vehicle namespace; it never changes when computation is offloaded. `position_xy` is Gazebo ground truth at the current acknowledged physics tick.

- `local`: select the vehicle.
- `offload`: select the nearest RSU within the inclusive 5 m send-time range, breaking equal-distance ties by device ID; use the vehicle if no RSU qualifies.
- `random`: uniformly select the vehicle or an RSU inside the same 5 m range, using system randomness. Selection draws are retained as actual device IDs in the trace.
- `greedy`: select the eligible device with the lowest current maximum core temperature; equal temperatures use device ID. Eligibility includes the vehicle and all RSUs inside the same 5 m range. It does not optimize queue delay, parent placement or transmission cost. The recorded placement event includes the temperature snapshot used for selection.

A job's selected device remains fixed throughout execution. Initial task admission is separate from parent-data transfer and reception. RSU-to-RSU data can cross distances greater than 5 m; accepted arrivals are not rechecked against vehicle coverage.

## Per-device policies

`SchedulingPolicy` in `tools/live_bridge/policies.py` exposes two keyword-based hooks:

```python
class MyPolicy:
    name = 'my_nonpreemptive_policy'

    def assignments(self, *, engine, device_id, ready_jobs, free_cores, **kwargs):
        return []  # Assignment(job_id, core_id) for selected eligible jobs.

    def dvfs_level(self, *, engine, job, core, **kwargs):
        return None  # Maximum paired frequency/voltage by default.
```

Inject an instance into `LiveEngine(scheduling_policy=MyPolicy())` or `Broker(..., scheduling_policy=MyPolicy())`. The engine rejects early children, occupied cores, duplicate assignments and invalid operating points before applying a dispatch batch.

`FIFOShortestFinish` maps newly eligible jobs in `(ready_tick, release_tick, job_id)` order to per-core queues. For each core it predicts the current tick plus running remaining service, all FIFO reservations and the new job's service at the selected operating point. It chooses the smallest predicted finish; ties use core ID. Only a free core's eligible queue head can start. This prediction holds the currently selected rate and excludes future thermal pauses or boost transitions; actual execution always enforces whole-device cooling. Blocked children do not reserve cores. There is no job preemption or migration.

The historical `ReadyFIFOOldestIdle` class remains explicitly selectable; it is a different policy, not an invariant of the engine. A custom `dvfs_selector(engine, job, core)` may override the normal operating-point hook. `overrun_policy='device_maximum_until_overruns_complete'` is a system reaction, independent of the placement algorithm. After any HI-selected job crosses its LO work, every core of its device uses its own maximum paired point. The device returns to normal selection after the last active overrun job finishes. Other devices are unaffected. The active set survives cooling pauses. Per-job and no-boost variants remain separately selectable.

## Two independent communication costs

```python
def my_upload(*, distance_m, task_size_bytes=None, **kwargs):
    return 0.003  # seconds

def my_edge(*, distance_m, edge_size_bytes=None, **kwargs):
    return 0.004  # seconds

engine = LiveEngine(
    edge_config={'placement': 'offload', 'coverage_radius_m': 5.0},
    upload_cost=my_upload,
    data_cost=my_edge,
)
```

`task_upload_cost_s` receives the vehicle-to-selected-device distance and an optional task size. `edge_data_cost_s` receives the parent-device-to-child-device distance and an optional edge size. Additional context includes source/target device IDs, job, parent ID, send time and engine, so a replacement can use bandwidth, payloads or other future parameters without changing the interface. The current model ignores the optional sizes.

| Send-time distance (m) | Delay (s) |
| --- | ---: |
| [0, 1) | 0 |
| [1, 2) | 0.003 |
| [2, 3) | 0.004 |
| [3, 5] | 0.005 |
| (5, infinity) | 0.010 |

Same-device transfers take zero time. The functions must return finite nonnegative seconds. The engine rounds each duration up to the common lattice and records send/arrival ticks. Parent data can be sent only after modeled parent completion and after the child destination is known. The child waits for its task upload and every selected parent transfer. Results remain buffered until the selected device finishes the modeled computation and its thermal gate permits delivery. The live placement experiment charges these two transfer types, without an additional implicit result-return hop.

`CommunicationModel` retains its standalone request/result convenience interface for extensions; live placement uses the two independently replaceable functions above. Link contention, packet loss and payload serialization are not included in the distance-only model.

## Hardware and trace identity

Each of the 25 RSUs is bound to its own processor through `load_edge_resources`: one A15 core, Tmax 46.5 degrees C and Tbalance 46.0 degrees C. The vehicle has one A7, Tmax 46.2 and Tbalance 45.6. Initial and ambient temperatures are 45 degrees C. Ordinary idle and special cooling powers remain distinct.

Physical RC parameters are sampled with system randomness. Their explicit realized values can be saved using `physical_realization(devices)` and loaded through `realized_physical_parameters` in a hardware JSON. Reusing these values gives all four policies the same actual modeled hardware without relying on a random initialization key.

Trace schema 4 keeps both `owner_device` and execution `device`, per-job deadlines, exact placement positions, uploads, parent transfers, core queue reservations, work segments, thermal events and original publication ownership. `analyze_placement_trial.py` independently checks these invariants and exports original per-job and per-transfer CSVs.

The current relative deadline is `uniform(1.1,1.3) * W_HI / (2000 MHz * 1.8)`, computed from HI normalized work and maximum A15 speed. Queueing, transfer costs and thermal pauses never enter deadline construction. The sampled per-family values are saved in `config/task_deadlines.json`; nominal ROS activation periods remain seconds and do not scale with CPU frequency.
