# Current resources and configurable experiment parameters

Inspected on 2026-10-09 in the running WSL environment. No resource limits or scheduling policies were changed by this inspection.

| Item | Current state | Configuration mechanism |
| --- | --- | --- |
| Host CPU identity reported to WSL | Intel Core i7-13620H | Physical CPU identity is not a per-worker software setting |
| WSL logical CPUs | 16, IDs 0–15 | Global `processors` setting in Windows `.wslconfig`; per-process/thread CPU affinity |
| Nav2 CPU affinity | CPUs 0–15 | `taskset` / thread affinity; affinity alone does not reserve exclusive CPU time |
| CPU time quota | `cpu.cfs_quota_us=-1`, period 100000 us in current init scope | cgroup quota/period or equivalent container controls |
| WSL memory | About 15 GiB total, 4 GiB swap | Global `.wslconfig` memory settings; per-worker cgroup/container memory limits |
| OS scheduling | `SCHED_OTHER`, nice 0 | Linux priority/policy settings; RT policies need appropriate privileges |
| ROS execution | Upstream `component_container_isolated`, isolated executors plus component-owned threads | Executor/callback scheduling integration; not an EDF/RM YAML switch |
| Edge worker resources | None; parked cars have no worker processes | Add real workers, then assign measured resource capacities and limits |
| Formal WCET / job deadlines / criticality | Unspecified | Measurement/analysis and explicit timing model required |
| Network model | Inactive; rate, bandwidth, latency and loss unspecified | Actual worker transport plus application emulation or network shaping |
| Graphical rendering | llvmpipe software rendering; not GPU accelerated | Host/WSLg graphics configuration; this is independent of edge-worker compute capacity |

The current kernel is `5.15.167.4-microsoft-standard-WSL2`, with a hybrid cgroup hierarchy and CPU/memory controllers under cgroup v1. Instructions that assume `/sys/fs/cgroup/cpu.max` at the hierarchy root would not match this installation.

Example resource allocation for a future worker: affinity to logical CPUs 2 and 3 plus a quota of 150000 us per 100000 us period permits up to 1.5 CPUs of aggregate execution time. A quota of 50000 us per 100000 us period permits half a CPU's time. Neither setting means a particular GHz value or bounds a specific callback's execution time. On shared laptop hardware, these represent controlled capacity differences, not different physical server architectures.

Current navigation settings:

| Component | Current setting |
| --- | --- |
| Controller | 20 Hz, nominal 50 ms period; MPPI `vx_max=0.5 m/s`, `vx_min=-0.35 m/s`, `wz_max=1.9 rad/s` |
| Velocity smoother | 20 Hz; maximum velocity `[0.5, 0.0, 2.0]` |
| Physics drive | Linear velocity at most 0.46 m/s; linear acceleration at most 1 m/s²; angular velocity at most 1.9 rad/s |
| Local costmap | 5 Hz updates, 2 Hz publication |
| Global costmap | 1 Hz updates and publication |
| LiDAR | 5 Hz in simulation time |
| Odometry | 30 Hz in simulation time |
| BT loop | 10 ms configured loop interval |
| Global planning requests | Event-driven, normally throttled to approximately 1 Hz during navigation by the default BT; `expected_planner_frequency=20` is an overrun-warning threshold, not a 20 Hz timer |

The smaller physics limit caps commanded forward speed at 0.46 m/s. Increasing velocity requires coordinating the controller, smoother, drive plugin and acceleration limits, then checking corner behavior, sensor updates and tracking. Speed, task release rates, resource capacities and communication should be held equal across scheduling baselines; otherwise the comparison changes several variables at once.

A nominal 50 ms controller period does not establish a 50 ms formal deadline or known WCET. DDS deadline QoS concerns expected message intervals; it is not an execution deadline scheduler. Choosing FIFO/RR at the OS layer also does not implement DAG-aware precedence, EDF or mixed-criticality mode changes at the ROS application layer.

Sources: [Microsoft WSL settings](https://learn.microsoft.com/en-us/windows/wsl/wsl-config), [Linux cgroup CPU bandwidth](https://docs.kernel.org/scheduler/sched-bwc.html), [ROS executor documentation source](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Executors.rst).

## Future simulated CPU model

Gazebo currently models physical motion and sensors; application computation executes on the laptop. A later experiment can model each edge CPU with a core count, service-time model per real Nav2 job, ready queue and communication delays. Results computed on the laptop must be buffered until the modeled finish and transmission events; inserting sleep after a result has already been consumed would not enforce the experiment. The custom scheduler chooses eligible jobs and virtual resources while tracking per-core availability, deadlines and remaining execution time. Timing attributed to a Cortex-A15 must be measured/calibrated or explicitly declared an assumption. Laptop compute time is distinct from the modeled service time. If the laptop is slower than the modeled resource, virtual time or offline/replayed execution is needed rather than pretending a wall-clock result arrived sooner.

This layer is not implemented by the setup. It requires integration at actual callback/worker boundaries so result delivery, shared-state updates and downstream releases obey the model. Sequential FollowWaypoints currently slows at each separate target; a later common pass-through mission can use upstream NavigateThroughPoses to avoid treating each checkpoint as an independent stop. That mission choice must be identical across scheduling baselines.
