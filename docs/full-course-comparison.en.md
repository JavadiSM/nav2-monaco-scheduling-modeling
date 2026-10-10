# Full-course Nav2 placement comparison

Status: PARTIAL. Successful full courses: local=1, random=1, offload=1, greedy=1.
Complete calibration laps: 3. Attempt outcomes across calibration and comparison: {'complete': 7, 'failed_or_partial': 5}. All valid comparison traversals, including supplementary incomplete blocks: {'local': 1, 'random': 1, 'offload': 1, 'greedy': 1}.

One A7 in the vehicle; one A15 in each of 25 RSUs. The circuit, navigation algorithms and 1 ms physics/hardware lattice are shared. Normal execution uses the middle paired operating point. An overrun boosts its entire device until the last active overrun completes.

$W_i^{q}=1000 C_{i,ref}^{q},\ q\in\{LO,HI\};\quad D_i=U_i W_i^{HI}/3600,\ U_i\sim U(1.1,1.3);\quad d_j=r_j+D_i.$

Reference CPU demand is measured on the host and converted through an explicit 1000 MHz, eta=1 normalization. Periods and deadlines remain seconds; hardware speed changes service time. Deadlines exclude thermal, queueing, dependency and communication costs. The calibration table remains fixed throughout comparisons; a later job may exceed the empirical HI maximum and is recorded as such without changing it.

Placement is local; uniformly random among vehicle and covered RSUs; nearest covered RSU with local fallback; or greedy minimum current device temperature among vehicle and covered RSUs. Greedy ties use device ID. No placement rule optimizes queue lengths or parent locations. All devices share FIFO / shortest predicted finish-time queue mapping.

| Policy | Full courses | Completion (s), mean ± SD | Overruns, mean | DMR (%) | Mean response (s) | Cooling entries/device |
|---|---:|---:|---:|---:|---:|---:|
| local | 1 | 272.202 ± — | 38730.0 | 12.370 | 0.010880 | 3.462 |
| random | 1 | 271.962 ± — | 41590.0 | 4.037 | 0.008558 | 0.769 |
| offload | 1 | 274.646 ± — | 24409.0 | 0.000 | 0.032516 | 17.077 |
| greedy | 1 | 271.283 ± — | 44267.0 | 10.390 | 0.006893 | 0.000 |

Frozen parameters (seconds):

| Task | Activation | Nominal T | Observed mean inter-arrival | C_LO (host CPU) | C_HI (host CPU) | D |
|---|---|---:|---:|---:|---:|---:|
| control_iteration | P | 0.050000 | 0.050001 | 0.003747 | 0.007990 | 0.002630 |
| local_costmap_update | P | 0.200000 | 0.200001 | 0.000398 | 0.000834 | 0.000277 |
| global_costmap_update | P | 1.000000 | 1.000041 | 0.000452 | 0.000825 | 0.000259 |
| velocity_smoothing_tick | P | 0.050000 | 0.050000 | 0.000009 | 0.000135 | 0.000045 |
| bt_tick | P | 0.010000 | 0.010026 | 0.000304 | 0.003204 | 0.001036 |
| planning_request | A | — | 1.027482 | 0.016509 | 0.033701 | 0.011358 |
| amcl_scan_callback | A | — | 0.199998 | 0.002343 | 0.006659 | 0.002061 |
| mppi_noise_generation | A | — | 0.050000 | 0.001681 | 0.004081 | 0.001377 |
| velocity_command_callback | A | — | 0.050001 | 0.000006 | 0.000060 | 0.000020 |
| collision_check | A | — | 0.050000 | 0.000219 | 0.000693 | 0.000221 |
| controller_path_install | A | — | 1.027464 | 0.000079 | 0.000329 | 0.000112 |

P denotes a configured periodic family; A denotes an event/message-driven aperiodic family. Observed inter-arrival is descriptive, including for aperiodic families; it is not a guaranteed sporadic minimum. Nominal periodic T is a native configuration value, not a synthetic release generator. All comparisons create jobs from actual callback arrivals. C_HI is an empirical observed maximum, with later exceedances retained in per-run metrics.

The campaign closed with 1 complete balanced block(s), against the planned three, at the fixed cutoff 2026-10-10T08:00:00+03:30. Failed and cutoff attempts are preserved and excluded from the primary comparison.

greedy has the shortest mean full-course time (271.283 s; 0.34% below local). local has the highest run-mean DMR (12.370%), and greedy has the lowest mean completed-job response (0.006893 s). These are descriptive rankings.

Each policy has one audited full course. The mean equals that single run; run-to-run sample standard deviation is undefined and no SD whiskers are drawn. Three repetitions per policy were planned, so these results remain descriptive.

local: 50,131 entered jobs across 1 full courses; 0.00% placed on RSUs. A7 cooling entries average 90.00 per course; all A15 entries together average 0.00. Observed upload delay alone exceeds D_i for 0/50,131 evaluated jobs (0.00%). For 42,246/50,131 evaluated jobs (84.27%), even maximum-frequency service on the selected device plus its observed upload delay exceeds D_i, before parent waiting, queueing or cooling. Including the shared middle-then-maximum reaction makes this lower-bound count 42,350/50,131 (84.48%).

random: 56,752 entered jobs across 1 full courses; 74.38% placed on RSUs. A7 cooling entries average 0.00 per course; all A15 entries together average 20.00. Observed upload delay alone exceeds D_i for 42,024/56,752 evaluated jobs (74.05%). For 54,175/56,752 evaluated jobs (95.46%), even maximum-frequency service on the selected device plus its observed upload delay exceeds D_i, before parent waiting, queueing or cooling. Including the shared middle-then-maximum reaction makes this lower-bound count 54,253/56,752 (95.60%).

offload: 32,416 entered jobs across 1 full courses; 100.00% placed on RSUs. A7 cooling entries average 0.00 per course; all A15 entries together average 444.00. Observed upload delay alone exceeds D_i for 32,180/32,416 evaluated jobs (99.27%). For 32,318/32,416 evaluated jobs (99.70%), even maximum-frequency service on the selected device plus its observed upload delay exceeds D_i, before parent waiting, queueing or cooling. Including the shared middle-then-maximum reaction makes this lower-bound count 32,325/32,416 (99.72%).

greedy: 57,718 entered jobs across 1 full courses; 30.51% placed on RSUs. A7 cooling entries average 0.00 per course; all A15 entries together average 0.00. Observed upload delay alone exceeds D_i for 17,550/57,718 evaluated jobs (30.41%). For 50,190/57,718 evaluated jobs (86.96%), even maximum-frequency service on the selected device plus its observed upload delay exceeds D_i, before parent waiting, queueing or cooling. Including the shared middle-then-maximum reaction makes this lower-bound count 50,285/57,718 (87.12%).

The first comparison replicate of every policy records real camera views and the live Gantt. To limit software-rendering overhead, only the two presentation GUI processes are duty-cycled after 60 active SIM seconds; physics, navigation and the scheduling bridge continue independently. Later replicates run without camera GUIs. Presentation metadata and monotonic clock pairs retain these conditions and support the 8x SIM playback. Host CPU variability can affect the later LO/HI budget choice, while the reference budgets and deadlines remain fixed.

A 1 ms lattice rounds positive modeled service upward to whole steps. Very short deadlines and upload delays can therefore dominate DMR even when the vehicle finishes successfully. Completion time, task mix, response and DMR answer different questions; similar DMR values alone do not establish equivalent navigation performance. The deadline lower-bound counts overlap actual misses and each other; they are diagnostics, not additive causal attributions.

1 successful full-course recording(s) required recovery of presentation postprocessing. Original run errors, presentation metadata and raw recordings remain preserved; native mission completion and scheduling audits passed. Recovered results use the original successful physical run and unchanged raw traces.

A campaign-manager interruption affected 1 retained trial(s). Their physical execution continued independently; process exit codes remain unavailable, while original evidence and subsequent audit results are retained. Only successful full native missions with passing scheduling audits enter the comparison.

An initial random traversal reached the finish but failed exact output-gate auditing: 3 publications from 2 jobs were one 1 ms step late. A postcompletion mutex wait could temporarily clear the bridge busy flag. The bridge now keeps SIM frozen during output commit; precompletion waits still allow modeled work to progress. That traversal is preserved and excluded. Calibration and the earlier accepted local traversal passed their original exact-gate audits; frozen CPU budgets, deadlines and hardware were not changed. Per-trial source hashes retain this correction boundary.

![Full-course metrics](figures/full-course-comparison/comparison.png)

![Entered task-family shares](figures/full-course-comparison/task-family-pies.png)

![Response distributions](figures/full-course-comparison/response-and-deadlines.png)

The pie charts pool entered jobs across validated full courses and show their total count above each policy. Counts can change in a closed-loop system: scheduling changes output availability, which changes motion, duration, native action requests and message arrivals. Counts alone do not establish a cause; per-job, mission, clock and transfer evidence are retained for inspection.

Primary comparisons use complete balanced blocks only. Additional valid full traversals from an incomplete block remain in all-valid-full-runs.csv. All failed or partial attempts remain in the campaign manifest and are excluded from full-course means. Completed-job response is censored by the final mission cutoff; due unfinished jobs count as deadline misses. Cooling entries count transitions into whole-device cooling, not hot samples. A7/A15 counts and cooling durations appear separately in policy-means.csv and cooling-by-device.csv.

Raw per-job data, activation times, measured CPU demand, selected budgets, work/frequency intervals, parent transfers, native TF/sensor timestamps, physical/hardware clocks and mission trajectories remain in the local campaign archive. Published tables retain per-run results, shares and evidence hashes.
