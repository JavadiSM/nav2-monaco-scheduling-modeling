# Historical standalone task replay

The standalone Q95 figures below are retained historical evidence. The current live system assigns every existing task HI criticality and selects mean/observed-maximum budgets using actual CPU measurements; see the [current bridge contract](live-bridge.en.md) and [dual-budget table](evidence/dual-budget-parameters.csv).

The accepted driving route is frozen; current RSU additions are documented in [extension interfaces](scheduling-interfaces.en.md). Eleven primary Nav2 work units use their exact 95%-ECDF assumed WCET from 57 complete missions. The historical replay below used one A7 and one A15 on the vehicle, and two A7 plus two A15 per server. The [current full-course comparison](full-course-comparison.en.md) uses one A7 on the vehicle and one A15 per RSU. The first baseline is local, nonpreemptive ready FIFO. The replay in this document is independent of ROS. The separate [live bridge](live-bridge.en.md) applies frozen mean/observed-maximum budgets to actual Nav2 outputs and advances Gazebo on the same 0.001 s clock. The current bridge uses one red vehicle and a 2 s live Gantt window.

## Timing and equivalent work

Let C_i^ref be the measured thread-CPU Q95 budget, T_i the configured periodic HOST-wall interval, f_ref a virtual reference clock, and eta_ref its performance multiplier. The default f_ref=1000 MHz and eta_ref=1 are an explicit normalization, not a measurement of laptop clock cycles or target CPU calibration.

$$W_i=C_i^{ref}\eta_{ref}f_{ref}\quad\text{(million equivalent cycles, when }f_{ref}\text{ is MHz)}.$$

At operating point ell on core k:

$$C_{i,k,\ell}=\frac{W_i}{\eta_k f_{k,\ell}},\qquad N^{exec}_{i,k}=\frac{10^6W_i}{\eta_k},\qquad N^{period}_{i,k,\ell}=10^6T_i f_{k,\ell}.$$

N denotes a continuous equivalent cycle count in this abstraction. It is not a hardware counter observation or an instruction count. Changing reference frequency scales modeled CPU demand; changing target DVFS scales duration. Frequency and voltage are resolved independently for the selected core when dispatch occurs.

$$f_i^{activation}=1/T_i,\quad T_i^{ms}=1000T_i,\quad C_i^{ms}=1000C_i^{ref}.$$

Activation Hz and processor MHz are different quantities. Periodic releases retain their time interval when a task moves between cores. Their cycle representation changes with the target clock. Aperiodic tasks have no invented T; measured mean inter-entry is exported separately, alongside explicit arrivals. All time axes and graph timing labels use seconds.

For Control, Cref=0.0100682 s, T=0.05 s, activation rate=20 Hz, W=10.0682 Mcycles. At maximum DVFS, its A7 compute budget is 0.006292625 s and its A15 compute budget is 0.002796722222 s. One period is 80,000,000 A7 clock cycles or 100,000,000 A15 clock cycles. These target durations depend on the stated reference normalization.

The complete [CSV](evidence/modeled-task-parameters.csv) and [JSON](evidence/modeled-task-parameters.json) include all eleven tasks, milliseconds, seconds, nominal activation Hz, observed mean intervals, equivalent demand, target execution budgets and target period cycles. Inclusive child scopes are not additionally charged on top of an inclusive primary budget.

## Readiness, precedence and FIFO

For job j with explicit selected parents pred(j), release r_j and actual modeled finish F_p:

$$a_j=\max\left(r_j,\max_{p\in pred(j)}F_p\right),\qquad S_j\ge a_j.$$

Only completed parents satisfy a dependency; a parent's estimated compute-only finish cannot release its child during thermal cooling. A blocked child occupies no core and reserves no future slot. Ready FIFO orders by (a_j,r_j,input order). An incoming job does not preempt a running one. Available cores are ordered by the time they most recently became idle, then core ID. This implements oldest-idle selection, rather than choosing the core with the shortest predicted finish.

Selected cached versions are explicit job IDs. A control iteration can reuse an earlier completed map; it does not wait for every new map update. Repeated noise/control feedback is unrolled into distinct jobs. Graph validation rejects unknown parents, duplicate IDs and cycles before execution.

The coarse scheduling abstraction delivers results atomically at modeled completion. Original ROS can publish or commit shared state before callback return. This is a stated abstraction, not evidence that whole-function finish-to-start ordering already holds in upstream ROS.

## Replay and figures

The example replays 15 selected measured jobs and eight matched dependencies from one trace window. Recorded callback-entry offsets are release inputs here, not exact OS-ready instants. Seven isolated representatives remain without edges; their isolation applies to this measured window. The replay spans 0.5864912185 s. It uses Q95 budgets for every job, with power and thermal behavior enabled. No cooling occurs in this short example. Thermal-precedence tests also exercise longer workloads that do enter cooling.

![Local FIFO detail](figures/task-fifo/local-fifo-detail.png)

![Selected job DAG](figures/task-fifo/selected-job-dag.png)

The [full Gantt](figures/task-fifo/local-fifo-gantt.png), [job-level CSV](evidence/task-fifo-jobs.csv), [machine-readable events/results](evidence/task-fifo-demo.json), and [release inputs](../config/task_fifo_window.json) are retained. Unmodeled infrastructure remains ordinary background work. Deadlines and criticality classes remain unassigned.

Reproduce without launching ROS or accessing raw logs:

```bash
python3 scripts/demo_task_fifo.py
python3 -m unittest discover -s tests -v
python3 scripts/verify_frozen_scene.py
```

`periodic_jobs` generates nominal periodic releases over a finite horizon. `GraphJob` accepts explicit aperiodic releases and selected parent IDs; `instantiate_graph` accepts both exported graph representations. `DependencyFIFOScheduler.run_graph` dynamically maps ready jobs locally and returns per-job timing and event evidence.

## Communication and scene preservation

The historical standalone replay above used a zero-cost communication hook. The current live bridge checks a 5 m Euclidean task-upload radius only at sending. A selected request remains valid after leaving coverage. Upload and parent-data delays use separate extensible functions: below 1 m costs 0 ms; [1,2) m costs 3 ms; [2,3) m costs 4 ms; [3,5] m costs 5 ms; beyond 5 m costs 10 ms. Same-device transfers cost zero. A child waits for each selected parent's result and the corresponding data transfer. Bandwidth, congestion, loss and obstacle attenuation remain outside this distance-only model.

![Ideal coverage](figures/metric-map/edge-coverage-5m.png)

The scene manifest pins the accepted map, geometry, checkpoints, robot description, navigation parameters and scenario configuration. The normal launcher verifies their hashes and no longer regenerates them. New coverage plots and camera configurations are separate rendering artifacts.

The current [local full-course recording](media/local-full-course-8x.gif) shows actual Gazebo chase/overview views and the connected live Gantt. It completed all 20 targets in 272.202 simulation seconds, with zero recoveries and zero scheduling-audit errors. It plays at **8× simulation time**. The chase camera remains approximately 0.75 m behind the car. The [full comparison](full-course-comparison.en.md) includes four policies and four-nearest-RSU recordings; the independent Q95 replay above remains historical evidence.

Original recordings, obsolete previews and runtime logs remain in ignored local archives. Current publication GIFs and useful scene screenshots live in `docs/media/`. The route remains fixed; current RSU additions, placement and communication hooks are described in [extension interfaces](scheduling-interfaces.en.md).

## Visual-only server update

Blue server cabinets with antennas provide a minimal visual representation of the edge devices. The current scene has 25 RSUs, including start/finish and infill locations. Compute and temperature behavior are modeled separately from those visuals. The circuit map, route and navigation algorithms remain fixed for the full-course comparisons. Original scene screenshots and metric maps are retained.
