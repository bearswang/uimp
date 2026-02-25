# UIMP ROS + CARLA Demo

This project provides a multi-vehicle platooning/path-tracking demo in CARLA. The core ROS package is `uimp_ros`, which includes:

- Trajectory planning (`planner/`)
- MPC control (`controller/`)
- CARLA map visualization and vehicle spawning (`script/`, `example/auto_driving/launch/`)

---

## 1. Environment Requirements

It is recommended to run this project on **Ubuntu + ROS1 (catkin)** (native Ubuntu or WSL2 on Windows).

### Required Components

- CARLA Simulator (preferably version-matched with `ros-bridge`)
- ROS1 (with `catkin_make` support)
- Python3
- `catkin` workspace toolchain

### Key ROS Dependencies

The `uimp_ros` package declares the following main dependencies:

- `roscpp`
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `message_generation` / `message_runtime`
- `costmap_converter`
- `carla_msgs`
- `ackermann_msgs`
- `derived_object_msgs`

> If any package is missing, install it via `rosdep` or your system package manager before building.

---

## 2. Repository Setup and Build

From the workspace root, run:

```bash
bash ./setup.sh
```

`setup.sh` performs two steps:

1. Clone `carla-simulator/ros-bridge` under `src/` (if not already present)
2. Return to the workspace root and run `catkin_make`

After building, make sure this file exists:

- `devel/setup.bash`

---

## 3. Start CARLA

Start the CARLA server first (in another terminal):

```bash
cd $CARLA_ROOT
bash ./CarlaUE4.sh
```

`CARLA_ROOT` is your CARLA installation root directory.

---

## 4. One-Command Run

### 4.1 Full pipeline (spawn + planning + control)

```bash
bash ./demo.sh
```

This script launches the following in order:

1. `roslaunch uimp_ros Town04_spawn_car.launch`
2. `rosrun uimp_ros run_planner.py`
3. `rosrun uimp_ros main_launch.py agent_0`
4. `rosrun uimp_ros main_launch.py agent_1`
5. `rosrun uimp_ros main_launch.py agent_2`

### 4.2 Control only (without re-planning)

```bash
bash ./demo_control.sh
```

This script starts vehicle spawning and controllers, but does not run `run_planner.py`.

---

## 5. Manual Step-by-Step Run (Recommended for Debugging)

From the workspace root:

```bash
source devel/setup.bash
```

Then run the following steps:

```bash
roslaunch uimp_ros Town04_spawn_car.launch
```

```bash
rosrun uimp_ros run_planner.py
```

```bash
rosrun uimp_ros main_launch.py agent_0
rosrun uimp_ros main_launch.py agent_1
rosrun uimp_ros main_launch.py agent_2
```

---

## 6. Outputs

The planner script generates reference/actual trajectory files:

- `src/uimp_ros/planner/data/refpath_agent_0.txt`
- `src/uimp_ros/planner/data/refpath_agent_1.txt`
- `src/uimp_ros/planner/data/refpath_agent_2.txt`

These files can be used for offline analysis or replay.

---

## 7. Directory Overview

```text
impm_ws/
├─ demo.sh                      # one-command full pipeline
├─ demo_control.sh              # one-command control only
├─ setup.sh                     # clone ros-bridge + catkin_make
└─ src/uimp_ros/
	├─ controller/               # MPC control entry and core logic
	├─ planner/                  # platoon/trajectory planning and outputs
	├─ script/                   # CARLA map visualization scripts
	└─ example/auto_driving/
		├─ launch/                # Town04 launch files
		└─ configure/town04/      # vehicle and RViz configs
```

---

## 8. Troubleshooting

### 8.1 `devel/setup.bash` does not exist

The workspace may not have built successfully. Re-run:

```bash
bash ./setup.sh
```

Or run manually:

```bash
catkin_make
```

### 8.2 `Town04_spawn_car.launch` cannot be found

Ensure `source devel/setup.bash` has been executed and `uimp_ros` has been built successfully with `catkin_make`.

### 8.3 Cannot connect to CARLA (default `localhost:2000`)

- Make sure CARLA is started first
- Make sure port `2000` is not occupied
- If using a remote host, update `host/port` arguments in the launch configuration

### 8.4 Python module error (e.g., `carla`)

Your current Python environment is missing required modules. Install them in the same environment where ROS nodes are launched, then retry.

