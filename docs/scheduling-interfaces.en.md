# Scheduling and communication extension interfaces

## Independent scheduling policies

`SchedulingPolicy` in `tools/live_bridge/policies.py` defines two hooks, both accepting keyword arguments and extensible context:

```python
class MyPolicy:
    name = 'my_nonpreemptive_policy'

    def assignments(self, *, engine, device_id, ready_jobs, free_cores, **kwargs):
        # Return Assignment(job_id, core_id) objects for any selected subset.
        # Order jobs and choose their cores here.
        return []

    def dvfs_level(self, *, engine, job, core, **kwargs):
        # Return a valid paired operating-point ID, or None for maximum.
        return None
```

Inject an instance into `LiveEngine(scheduling_policy=MyPolicy())` or `Broker(..., scheduling_policy=MyPolicy())`. The default `ReadyFIFOOldestIdle` class implements the existing baseline. FIFO ordering and oldest-idle-core assignment belong exclusively to that class; another policy can replace both without editing the execution engine. Its operating point remains maximum throughout every job by default.

The engine computes actual eligibility and exposes only ready jobs and currently free local cores. It validates every proposed assignment before applying any of the batch, rejecting blocked children, occupied cores and duplicate job/core use. Selected predecessor completion, whole-device cooling and exact output gates are shared execution constraints. Policies may deliberately leave cores idle. The current interface supports nonpreemptive job dispatch with per-interval DVFS; job preemption, migration and remote placement need separate extensions.

A separately supplied `dvfs_selector(engine, job, core)` can override the policy's normal operating-point hook for experiments. The engine's per-job maximum-point remainder rule takes precedence over both hooks. Invalid paired point IDs fail explicitly. Policy identity is recorded in the trace and live display. The analyzer checks FIFO-specific ordering only for the named FIFO baseline; ownership, precedence, work accounting and output timing remain checked for every policy.

## Communication cost and request admission

The current live baseline remains local; these functions prepare future offloading without silently enabling it. `CommunicationModel` in `tools/abstract_compute/communication.py` accepts a custom `cost_function(**kwargs)` returning a finite, nonnegative **duration in seconds**. The default `communication_cost_s(*args, **kwargs)` returns **0.0 s**.

```python
model = CommunicationModel(radius_m=5.0, cost_function=my_cost)
request = model.send_request(
    position_xy=(vehicle_x, vehicle_y), endpoint=rsu,
    send_time_s=now_s, payload_bytes=input_size, job_id=job_id)
result = model.send_result(
    request, send_time_s=server_finish_s,
    position_xy=current_vehicle_position, payload_bytes=result_size,
    job_id=job_id)
```

Both directions call the same cost function. Its context contains `direction`, `endpoint_id`, `send_time_s` and the extra supplied keyword arguments. The returned transmission retains send time, delay and arrival time = send time + delay.

Request coverage is inclusive Euclidean distance **<= 5 m**, evaluated only at request sending. An out-of-range request is rejected. Once admitted, its result may be returned and received after the vehicle has left coverage: there is no result-time position recheck. Server execution cannot return a result before the request's modeled arrival. The custom cost function still determines return delay, which is currently zero. No bandwidth, congestion, loss or radio attenuation is assumed beyond this explicit idealization.

## Endpoint configuration and placement

| Device class | A7 cores | A15 cores | Initial / ambient (°C) | Tmax (°C) | Tbalance (°C) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Vehicle | 1 | 1 | 45 / 45 | 46.2 | 45.6 |
| RSU | 2 | 2 | 45 / 45 | 46.5 | 46.0 |

`config/abstract_compute.json` supplies shared thermal defaults plus per-class `thermal` overrides. `load_platform` resolves these separately for each device. `load_edge_resources()` binds every scene RSU to its own four-core processor, position and independent physical realization. This mapping does not spawn a moving vehicle or activate offloading.

The circuit now has **25 RSUs**: the original 19 checkpoint-associated cabinets, roadside start/finish cabinets and four inserted cabinets. Consecutive order follows route station on the open start-to-finish track; separation is Euclidean XY distance. Every consecutive gap is <= 10 m; the measured maximum is **9.595640 m**. Start/finish cabinets are beside the road at those route stations, preserving the vehicle's start and finish clearance. Added cabinets reuse the same blue appearance and antennas.

[scripts/extend_rsu_layout.py](../scripts/extend_rsu_layout.py) preserves the centreline, map, checkpoints, existing endpoint poses and all non-RSU world elements. Its updated manifest validates the explicitly authorized endpoint additions. [Placement evidence](evidence/rsu-layout.json), [metric map](figures/metric-map/circuit-dimensions.png) and [5 m send-time coverage](figures/metric-map/edge-coverage-5m.png) retain the resulting geometry.
