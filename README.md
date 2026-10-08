# ROS 2 Scheduling in Heterogeneous Edge Warehouse Environments

Repository: `ros2-heterogeneous-edge-warehouse-scheduling`

An experimental ROS 2 foundation for comparing mixed-criticality DAG scheduling on heterogeneous edge resources in a warehouse robotics application.

## Current scope: verified upstream simulation

This first stage installs ROS 2 Jazzy, Navigation2, Gazebo Harmonic and RViz on Ubuntu 24.04 / WSL2. It runs the official single-TurtleBot3 sandbox and checks sensor data, simulation time, Nav2 lifecycle activation and autonomous navigation to a goal. The warehouse, multiple robots, edge servers and custom scheduling policies are future stages; they are not implemented by this setup.

Nav2 is used as an upstream dependency, not copied into this repository or represented as original work. A fork is unnecessary until upstream code changes are required.

## Install

In Ubuntu 24.04:

```bash
sudo bash scripts/install_wsl.sh
```

The installer adds the official ROS apt-source package and installs the required packages. It does not replace your Ubuntu distribution or perform a blanket system upgrade. For graphical WSL operation, WSLg must be available.

## Run

```bash
bash scripts/launch_demo.sh
```

Gazebo and RViz should appear as Windows desktop windows through WSLg. The wrapper uses the official `nav2_bringup/tb3_simulation_launch.py`, with GUI enabled and simulation time enabled. It sets WSLg display variables only when their sockets exist, and keeps the demo on ROS domain 42 by default to avoid mixing it with unrelated ROS applications.

In a second WSL terminal:

```bash
bash scripts/verify_demo.sh
```

The verification publishes an initial localization pose and requests a navigation goal. It writes `artifacts/verification.json` and returns a nonzero exit status if verification fails. The goal can be changed, for example:

```bash
bash scripts/verify_demo.sh --goal-x -2.0 --goal-y 0.5
```

Stop the launch with Ctrl+C, or from another terminal:

```bash
bash scripts/stop_demo.sh
```

To request another goal, run verification again only after restarting the demo, or set the initial pose arguments to the robot's actual current pose.

## Evidence and reproducibility

The setup report in `docs/setup-report.md` distinguishes installation, GUI startup, sensor checks and completed navigation. Screenshots and raw local run logs are kept in ignored `artifacts/`; only intentionally selected evidence belongs in `docs/evidence/`.

The report records the apt package versions used in the tested run. `upstream.repos` records upstream source references for later source builds; the initial demo runs the apt binaries, not those source checkouts.

After verification, create a local visual report with:

```bash
python3 scripts/create_report.py
```

Open `artifacts/setup-report.html` in your browser. These local artifacts are excluded from Git; selected screenshots and timing evidence are documented separately.

## Publish later

This setup is a local Git repository. No GitHub login or remote publication is required to run it. When ready, create an empty public GitHub repository named `ros2-heterogeneous-edge-warehouse-scheduling`, then run:

```bash
git remote add origin https://github.com/YOUR_USERNAME/ros2-heterogeneous-edge-warehouse-scheduling.git
git push -u origin main
```

The initial local setup commit is attributed to the setup agent. Configure your own Git name and email before making your own commits.

## Pending scenario design

The intended demonstration is a race in which multiple robots visit marked waypoint cells and reach a final destination. All robots should use the same ready-made Nav2 algorithms; scheduling policies are the experimental variable.

Robot count, obstacle layout, edge resources, timing metadata and scheduler integration are pending discussion. No warehouse, multi-robot race, edge worker or mixed-criticality scheduler has been implemented in this setup. New artificial robot behaviours are not part of the current scope.

## Upstream projects

- [Navigation2](https://github.com/ros-navigation/navigation2)
- [Minimal TurtleBot simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation)
- [ROS apt source configuration](https://github.com/ros-infrastructure/ros-apt-source)
- [Nav2 quickstart](https://docs.nav2.org/rolling/getting_started/quickstart/quickstart/)
