# Nav2 task profiling protocol

This instrumented characterization keeps the Monaco world, robot dynamics, navigation configuration and algorithms unchanged. It measures the existing system. Deadlines, mixed-criticality levels, offloading and A15/A7 costs are not assigned.

The authorized campaign ends on **2026-10-09 at 09:00 Asia/Tehran / 05:30 UTC**. The campaign runner stops its simulator first, with a 20-second safety margin. Analysis is permitted afterward; starting or continuing simulation afterward is not.

## Measurement cohorts

`full-v2-20261009` is the primary comparable cohort. Its frozen C++ preload probe records named algorithm spans, nested phases, selected mutex events, framework callbacks, callback binding generations, timer due points, Rate::sleep boundaries, ROS/RMW publications and successful takes. Older basic, causal and full-v1 probes are supporting cohorts and are never pooled into the primary cost distributions.

Each attempt is preserved, including failed startup and incomplete runs. A successful lap must have a successful real Nav2 mission result. `manifest.json` lists sealed attempts; `metadata.json` is initially written at startup and is only sealed after shutdown. The analyzer follows the manifest, so it never treats a growing live raw file as a complete trial.

## Timing semantics

Raw timestamps are nanoseconds. Displayed CPU demand and elapsed intervals are milliseconds; divide by 1000 for seconds. Nominal frequency converts as `T_seconds = 1 / frequency_hz`, retaining its original clock domain.

- `wall_*_ns`: CLOCK_MONOTONIC. These measure host elapsed time.
- `cpu_*_ns`: CLOCK_THREAD_CPUTIME_ID. These measure running CPU time on the recorded thread.
- `sim_*_ns`: latest process-level delivered /clock cache. They can be stale or regress and are not exact job simulator timestamps.
- `stamp_ns`: event-dependent. It is a source header stamp for a scan/path, a period for Rate::sleep, an expected-call time for timer_due, or metadata. Read it with event/kind.
- RMW source/received stamps are host epoch nanoseconds supplied by middleware. They measure the current host transport, rather than a modeled edge link.
- `signature`: event-dependent payload hash, pointer link, writer scope ID or affinity bitmask. It is not a universal message ID.
- `aux`: event-dependent particle/path count, resource version, sleep status, callback binding generation or timer actual-call time.

`span` rows are written at scope end. Thread-local buffer flushes can interleave in a process file; global file order is **not chronological**. Sort actual timestamps within the appropriate actor/thread when reconstructing dependencies. Parent/child scope events in one thread retain nesting order.

CPU cost is inclusive by default. Exclusive CPU subtracts directly nested child CPU. Do not add parent and child costs. The noise generator's separate thread has separate demand. Whole-loop CPU is derived between one Rate::sleep return and the following Rate::sleep call on the same loop thread. Its release interpretation remains software continuation, not OS readiness.

The probe does not replace ROS computations with dummy tasks or change task priorities. It does perturb runtime with instrumentation and shares the laptop with Gazebo, RViz and analysis. Do not label measured means or finite observed maxima as WCET bounds.

## Data and outputs

Every run retains:

- Original `raw/events-PID.csv` files with all recorded framework and algorithm events.
- `observer.jsonl` with independent topic receipt/source stamps, plus `host-samples.jsonl` and process snapshots.
- Mission targets/outcomes in `mission.json`; shutdown flags, hashes and file sizes in `metadata.json`.
- Original launch, observer and mission logs.
- Derived `job-samples.csv` for algorithm/worker spans. All framework samples remain in the original raw files.
- `summary.json`, compact `callback-binding-details.json`, numeric `analysis-cache.npz`, and a finite `trace-graph-evidence.json` window.

CSV originals are preserved. A proposed compression step that would replace/delete originals was rejected by automatic review and was not executed.

The English report, figures and tables are generated from sealed evidence:

```bash
python3 scripts/analyze_task_campaign.py
python3 scripts/analyze_task_campaign.py --no-report
# Only after the user cutoff:
python3 scripts/analyze_task_campaign.py --final
```

The analyzer never starts a simulator. The final flag refuses to run before the cutoff. On a fresh environment it needs system Python with NumPy/Matplotlib, Graphviz `dot`, DejaVu fonts, and ReportLab; `--reportlab-path` can select the bundled pure-Python packages. Scientific figures are PNG/SVG/DOT. The PDF includes mathematical formulas rendered with Matplotlib mathtext.

The primary outputs are `docs/task-model.en.md`, `docs/task-model.en.pdf`, `docs/evidence/task-characterization-summary.json`, `task-characterization-table.csv`, `task-characterization-per-run.csv`, `callback-characterization-table.csv` and `docs/figures/task-model/`.

## Scope limits that remain explicit

Callback start is not necessarily release. Sensor generation, middleware arrival, successful take, TF eligibility and callback execution are separate events. Expected steady-timer due points support identified due-to-start/finish and finite jitter statistics; no scheduler-ready timestamp is inferred for other tasks.

Map/noise epochs cover the instrumented locked paths. Global NavFn start-cell mutation occurs before its map-copy mutex and therefore the conservative planning epoch does not represent every grid-byte version. No exact global-grid or TF-cache provenance is invented. The observed control phase DAG uses recorded local-map/noise events and labels observed thread ordering separately from application data dependencies.

The current graph inventory is not an exhaustive internal task trace of Gazebo physics/render plugins, kernel scheduling, DDS worker internals or all arbitrary future recovery paths. Infrastructure callback groups are listed separately. Measured thread affinity is an allowed-CPU mask, not a record of each job's actual core or migration.
