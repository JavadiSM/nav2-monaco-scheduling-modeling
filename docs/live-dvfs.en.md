# Live per-job DVFS

**Current placement experiment:** one A7 vehicle at level 2 (1200 MHz / 1.0 V), one A15 per RSU at level 2 (1500 MHz / 1.2 V). `device_maximum_until_overruns_complete` overrides normal selection after an overrun: all cores of the affected device use their maximum paired point until the last active overrun job finishes. Other devices are unaffected; this is a common system reaction, not part of placement.

The current live vehicle has one Cortex-A7; each RSU has one Cortex-A15. Vehicle Tmax is **46.2 °C**, ambient and initial core temperatures are **45 °C**, and vehicle Tbalance is **45.6 °C**. RSUs use Tmax = 46.5 °C and Tbalance = 46.0 °C. Ready FIFO / shortest predicted finish queues are the active, replaceable baseline; see [policy and communication interfaces](scheduling-interfaces.en.md). Ordinary arrivals do not preempt or migrate jobs; changing the operating point changes the speed of the job already occupying its core.

## Paired operating points

| Level ID | A7 frequency (MHz) | A7 voltage (V) | A15 frequency (MHz) | A15 voltage (V) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 800 | 0.90 | 1000 | 1.10 |
| 1 | 1000 | 0.95 | 1250 | 1.15 |
| 2 | 1200 | 1.00 | 1500 | 1.20 |
| 3 | 1400 | 1.05 | 1750 | 1.25 |
| 4 | 1600 | 1.10 | 2000 | 1.30 |

The A7 performance coefficient is 1; A15 is 1.8. A selector returns a valid paired level ID, rather than choosing frequency and voltage independently. No selection (`None`) means level 4. Switching latency is an explicit **zero model assumption**; nonzero settings are rejected until a switching model is added.

## Work accounting and result release

Let the selected job budget be B_j, as defined by the existing measured-CPU mean/maximum contract. Equivalent work in million cycles is

$$W_j=B_j f_{ref}\eta_{ref},\qquad f_{ref}=1000\;\mathrm{MHz},\quad\eta_{ref}=1.$$

For interval n of length delta = 0.001 s, on core k at paired point l(n):

$$q_{j,n}=\begin{cases}\min\{w_{j,n},\eta_k f_{k,l(n)}\delta\},&\text{job active and device not cooling},\\0,&\text{otherwise},\end{cases}$$

$$w_{j,n+1}=w_{j,n}-q_{j,n},\qquad w_{j,0}=W_j.$$

A frequency change preserves w and all previously completed work. Completion is the first lattice boundary where w reaches zero. The last interval occupies a full tick; its charged work is clamped to the remaining work. Power uses the selected paired point for that interval. The coupled thermal model receives those powers on the same lattice. During cooling, all cores of the affected device consume no job work, retain their assigned jobs and use cooling power. Ordinary idle retains its separate idle-power definition.

The adapter seals the actual host calculation before selecting its total budget. Buffered outputs still wait for selected modeled completion, parent completion and thermal permission, then flush before the next physics step. DVFS changes this modeled completion time; it does not bypass the output gate.

## Minimal per-job overrun rule

All existing tasks still have HI criticality. There are no LO tasks or global mode/drop/recovery policies. A job whose actual CPU demand exceeded the reference mean selects its total HI work at seal. Define

$$W_i^{LO}=C_i^{LO} f_{ref}\eta_{ref}.$$

For such a job, the overrun boundary is the first common-lattice boundary at which accumulated consumed work reaches W_i^LO. With `remaining_work_at_maximum_point`, from that boundary onward any remaining work uses level 4 regardless of scheduler requests. With the current `device_maximum_until_overruns_complete` configuration, every core of the affected device switches to maximum until the last active overrun job finishes. The active set is retained during cooling; no job is dropped. Normal selection resumes afterwards. A threshold crossed inside an interval is detected at its end, within one 1 ms tick; no fractional physics step is introduced. If the same tick completes the whole budget, there is no remainder to accelerate. The selected HI demand remains the total demand, rather than adding C_HI after C_LO. The next job receives its own normal DVFS decision.

## Scheduler interface

Both `LiveEngine(..., dvfs_selector=selector)` and `Broker(..., dvfs_selector=selector)` accept a callback:

```python
def selector(engine, job, core):
    elapsed_ticks = engine.tick - job.start_tick
    return 0 if elapsed_ticks < 3 else None
```

The callback runs before each active interval. It can inspect remaining work, core temperature and current model time. Returning `None` uses the maximum paired point. The engine applies the per-job overrun override before consulting this callback. Invalid IDs fail explicitly. No selection is requested while the device is cooling.

`config/live_bridge.json` selects `maximum` by default. A `fixed` policy with `level_id` is also available for functional verification; `--dvfs-level 0` creates that policy only in the trial snapshot, leaving the repository default unchanged:

```bash
source scripts/environment.sh
python3 scripts/run_live_bridge.py --seconds 8 --output artifacts/live-bridge/<new-max-trial>
python3 scripts/run_live_bridge.py --seconds 8 --dvfs-level 0 --output artifacts/live-bridge/<new-low-trial>
python3 scripts/analyze_live_bridge.py artifacts/live-bridge/<trial> --output artifacts/live-bridge/<trial>/analysis
python3 -m unittest discover -s tests
```

Trace schema 3 records each job's paired f/V execution segments, accumulated and remaining work, DVFS changes and overrun boundary. The analyzer reconstructs the work integral independently and verifies coverage of all active ticks, earliest completion, no cooling execution, valid paired points, maximum-only remainder, selected dependencies and exact output-release ticks. Historical fixed-frequency traces retain their original validation path.

## Candidate scheduling comparisons

These are proposed experiments, not newly enabled schedulers. Apply every priority rule only to jobs whose selected parents have completed. Keep actual Nav2 releases, the same task budgets and the same thermal guards for all comparisons.

| Candidate | Ready-job priority / core assignment | Additional input |
| --- | --- | --- |
| FIFO baseline | Oldest ready job; oldest-idle core; maximum DVFS | None; historical configuration |
| FIFO + earliest finish | Same ready-job order; choose the core with the earliest predicted finish at maximum DVFS | Explicit conservative prediction of busy/cooling availability |
| Ready EDF + earliest finish | Earliest absolute deadline among ready jobs; earliest predicted finish core | User-defined relative deadlines, including event-driven jobs |
| DAG rank + earliest finish | Descending known successor-path cost among ready jobs; earliest predicted finish core | A defined graph instance or declared structural lookahead; an online HEFT-inspired adaptation |
| Proposed thermal-aware DVFS | Combine deadline/path urgency, remaining work and per-core temperature; change the paired point during execution | A concrete policy and, for slack decisions, deadlines |

[Topcuoglu, Hariri and Wu (2002)](https://experts.arizona.edu/en/publications/performance-effective-and-low-complexity-task-scheduling-for-hete/) define HEFT using upward ranks and insertion-based earliest-finish placement for a known DAG. The live Nav2 graph is revealed as actual inputs arrive; a policy using only the revealed graph must be identified as an online adaptation, and a replay with future knowledge must be identified as offline.

[Liu and Layland (1973)](https://bears.ece.ucsb.edu/class/ece253/papers/laylandliu77.pdf) provide the classic RM/EDF foundations under independent periodic, preemptive uniprocessor assumptions. Here, ready EDF is a practical comparison with explicitly assigned deadlines; those original optimality/utilization results do not transfer to the heterogeneous, dependent, thermally suspended nonpreemptive model. RM can be a separate periodic-subset comparison after an explicit event-task service rule is defined.

Thermal-aware allocation has established precedent; see [Hung et al.](https://arxiv.org/abs/0710.4660). The proposed policy above remains a design direction, not a claim of novelty or a measured improvement. First compare priority/core assignment at maximum DVFS, then compare DVFS policies under the same priority rule. Measure navigation completion, response times, deadline misses once deadlines exist, cooling duration, per-core temperature and selected frequency distribution.

## Bounded live verification

The earlier DVFS development check passed **64 unit tests**, including mid-job changes, default-maximum selection, work conservation, both core types, paired f/V power, noninteger LO crossings, cooling suspension and retained parent gates. The two retained 8 s active Nav2 trials below preceded the latest RSU/configuration update and used Tmax = 46.3 °C, Tbalance = 45.8 °C. Each also includes 6 s of initialization and 14,000 matching 1 ms physics/hardware clock pairs.

| Evidence | Default maximum | Fixed level 0, then maximum after LO crossing |
| --- | ---: | ---: |
| Actual jobs | 1077 | 1354 |
| Modeled completed jobs | 1071 | 1348 |
| Started jobs with independently verified DVFS accounting | 1073 | 1350 |
| Verified selected dependency edges | 813 | 1002 |
| Output-release events at exact selected finish | 670 | 778 |
| Per-job LO-crossing events | 77 | 122 |
| Validation errors | 0 | 0 |
| Unresolved selected inputs | 0 | 0 |
| Actual odometry distance (m) | 3.440521 | 3.371059 |

[Maximum-point validation](evidence/live-dvfs-max-01-validation.json) and [low-point validation](evidence/live-dvfs-low-01-validation.json) retain trace hashes and counts. Full original traces, interval-level f/V records, launch logs and per-job data remain under the ignored `artifacts/live-bridge/` trial directories. In the low-point trial, A7 consumed 1,894 active ticks at level 0 and 649 at level 4; A15 consumed 1,356 and 322 respectively. Jobs spanning their LO threshold actually changed points during execution; the validator confirmed maximum-only work after that boundary.

Some newly queued/running jobs remain incomplete at the intentional cutoff. One completed budget in the low-point trial was still thermally blocked, so its output correctly remained held. Neither trial aborted navigation. These are functional checks of work accounting and release semantics; two short trials do not establish a scheduling-performance ranking or full-course success.


The retained [local-policy preview](live-bridge.en.md#current-policyrsu-preview) uses the final vehicle thresholds 46.2 / 45.6 °C, the replaceable FIFO policy and 25 RSUs. All 71 tests pass after the interface and placement changes. Its default maximum-point execution is validated independently; the low-point development trace above remains the evidence for within-job frequency changes.
