# ROS 2 Scheduling in Heterogeneous Edge Environments

Local repository: `ros2-heterogeneous-edge-warehouse-scheduling`

A ROS 2 / Nav2 simulation foundation for scheduling experiments. The current scene is an open circuit inspired by Monaco: one moving car visits 19 turn-entry checkpoints, then a finish gate. Nineteen parked cars mark future edge-server locations. The simulator runs on Ubuntu 24.04 / WSL2 with ROS 2 Jazzy, Gazebo Harmonic and RViz.

## Current implementation

The track is designed in this repository from a rough Monaco outline. The car uses the ready-made TurtleBot3 differential-drive dynamics, sensors and upstream Nav2 algorithms, with a racecar appearance. It is not an Ackermann racing-vehicle physics model. The ordered checkpoint mission uses Nav2's existing `NavigateThroughPoses` action. All checkpoints and the finish are sent as one continuous mission; intermediate gates are pass-through targets. The track uses four iterations of Chaikin corner rounding, preserving straight stretches while smoothing turns.

There is no custom scheduler, mixed-criticality mode switching, remote worker or task offloading yet. Parked edge cars are static Gazebo models. `config/edge_communication.yaml` reserves communication parameters without activating them. Native ROS dependencies are not, by themselves, a formally specified real-time DAG with WCETs, job deadlines and criticality levels.

## Install

On Ubuntu 24.04:

```bash
sudo bash scripts/install_wsl.sh
```

WSLg is required for graphical windows. The installer uses official ROS repositories and adds the Python packages used by the scene generator.

## Run the circuit

In a WSL terminal at the repository root:

```bash
bash scripts/launch_monaco.sh
```

This regenerates the world and map from `config/monaco.yaml`, launches Gazebo and RViz, spawns one moving car and loads upstream Nav2. The car initially waits at the start gate.

In a second WSL terminal:

```bash
bash scripts/run_monaco.sh
```

The mission initializes AMCL at the actual starting pose, requests all checkpoints in order and verifies the action result, zero Nav2 error code and estimated proximity to every target in order. Results are written to `artifacts/monaco-run.json`. Restart the simulation before requesting another mission: this script initializes the car at the starting pose.

To record the entire real Gazebo run instead of using the preceding mission command:

```bash
source scripts/environment.sh
python3 scripts/record_monaco.py
```

After a successful recorded mission, produce the video with a labeled map of the recorded position:

```bash
python3 scripts/render_monaco_video.py
```

This writes `artifacts/monaco-smooth-start-to-finish.mp4`. The main view is the actual Gazebo recording. The inset is explicitly labeled as the AMCL estimated trajectory; target labels use recorded Nav2 feedback.

The recording wrapper requests the mission and stops capture after its result. It writes `artifacts/monaco-smooth-race-raw.mp4`, recording metadata, a frozen copy of the recorded scene and the mission report. The previous waypoint-run video is retained separately. Gazebo's window must retain its configured 1600 by 900 size during capture. A failed mission is reported as failed; a recording alone does not prove course completion.

Stop the simulation with Ctrl+C or:

```bash
bash scripts/stop_demo.sh
```

The graphical camera shows the entire track; yellow gates are checkpoints, green is start, red is finish, and blue cars are parked edge locations. RViz displays the map, planning and checkpoint labels.

The simulation-only focused test `bash scripts/run_monaco.sh --from-checkpoint 9 --limit 3` repositions the car at checkpoint 9 and checks targets 10–12; it writes a separate report and is not a full race.

## Configuration and evidence

See `docs/resources-and-timing.md` for actual CPU, memory, scheduling and speed settings. See `docs/monaco-scenario.md` for geometry, changes to scenario parameters, communication options and validation. `scenarios/monaco/layout.png` is a design overview; actual screenshots and recordings are stored separately under ignored `artifacts/`.

The original official TurtleBot sandbox remains available through `scripts/launch_demo.sh` and `scripts/verify_demo.sh`. Its installation and successful navigation were recorded in `docs/setup-report.md`.

The generated racecar description, baseline Nav2 parameter file, pass-through behavior tree and Gazebo GUI template retain their upstream Apache-2.0 attribution in `scenarios/monaco/NOTICE.md` and `LICENSE.upstream`. Original project scripts and track design use the root MIT license.

## Publish later

Everything is local Git; no GitHub login is required. Create an empty public repository named `ros2-heterogeneous-edge-warehouse-scheduling`, then use your account:

```bash
git remote add origin https://github.com/YOUR_USERNAME/ros2-heterogeneous-edge-warehouse-scheduling.git
git push -u origin main
```

## Upstream projects

- [Navigation2](https://github.com/ros-navigation/navigation2)
- [Minimal TurtleBot simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation)
- [Gazebo Sim](https://github.com/gazebosim/gz-sim)
- [ROS apt source configuration](https://github.com/ros-infrastructure/ros-apt-source)

## Task abstraction investigation

The [Persian technical study](docs/task-abstraction-study.fa.md) separates existing frequencies, triggers and timing fields from unmeasured CPU demand and proposes a causal job DAG and controlled virtual-time result delivery for modeled CPUs. [The inspection inventory](docs/evidence/task-abstraction-inventory.json) preserves configuration hashes and a read-only runtime graph snapshot. The original inventory is a pre-measurement snapshot. Instrumented task profiling is now implemented; CPU target calibration and custom scheduling remain deferred. The [English real-time report](docs/task-model.en.md), [PDF](docs/task-model.en.pdf), [task statistics](docs/evidence/task-characterization-table.csv), [per-run statistics](docs/evidence/task-characterization-per-run.csv), and [profiling protocol](docs/task-profiling.en.md) separate configured activation contracts from measured CPU demand and empirical intervals. The final campaign stopped before the authorized 09:00 Tehran cutoff on 2026-10-09: 59 main-cohort attempts produced 57 successful full laps, one navigation abort and one intentional cutoff truncation. The [cutoff audit](docs/evidence/task-campaign-finalization.json) verifies shutdown and unchanged scenario hashes; the [incident record](docs/evidence/task-campaign-incidents.json) separates navigation failure from administrative truncation.
