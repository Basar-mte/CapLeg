# Software setup

There are three ways in, and each works on its own:

| Path | Needs | Gives you |
|---|---|---|
| [A. MuJoCo](#a-mujoco-any-os-5-minutes) | Python ≥ 3.10 on Windows, macOS or Linux | Closed-loop treadmill simulation, metrics, plots, GIF |
| [B. ROS 2 + Gazebo](#b-ros-2-jazzy--gazebo-harmonic) | Ubuntu 24.04 (native or WSL2) | RViz model, Gazebo physics, `ros2_control`, live gain tuning |
| [C. Fusion 360](#c-fusion-360) | Autodesk Fusion (free personal licence is enough) | Edit the CAD, re-export, regenerate every model |

First, clone the repository:

```bash
git clone https://github.com/Basar-mte/CapLeg.git
cd CapLeg
```

> **Windows:** clone into a short path such as `C:\dev\CapLeg`. Some mesh paths are long, and Windows' 260-character
> limit can break `git` in deep folders (for example under OneDrive). If you must use a deep folder, run
> `git config --global core.longpaths true` first.

---

## A. MuJoCo (any OS, 5 minutes)

```bash
python -m venv .venv
source .venv/bin/activate              # Windows PowerShell: .venv\Scripts\Activate.ps1
pip install -r requirements.txt        # add -dev to also get pytest + ruff
```

```bash
python mujoco/run_rig.py                         # 12 strides headless -> mujoco/results/
python mujoco/run_rig.py --view                  # interactive viewer (needs a display)
python mujoco/run_rig.py --stride-time 1.0 --body-mass 85 --phase time
python mujoco/run_rig.py --out mujoco/results_fast --stride-time 1.0    # keep the reference run intact
python mujoco/render_side_view.py                # side-view GIF of the last run (no OpenGL needed)
```

The run writes these files to `--out` (default `mujoco/results/`):

| File | Content |
|---|---|
| `metrics.json` | Tracking errors, phase-estimation error, torques, power, work, GRF, foot clearance |
| `capleg_rig_results.png` | Knee and ankle angles vs reference, GRF, torques and gait phase |
| `capleg_rig_log.csv` | Full 200 Hz log, including `qpos` for replay |
| `capleg_rig_side_view.gif` | From `render_side_view.py` |

On macOS, `--view` needs `mjpython mujoco/run_rig.py --view`. That is a MuJoCo requirement.

### Run the tests

```bash
pip install -r requirements-dev.txt
pytest          # 22 tests in a few seconds: core, model consistency, a 5-stride closed-loop run
ruff check .
```

## B. ROS 2 Jazzy + Gazebo Harmonic

### Install (Ubuntu 24.04)

Follow the official [ROS 2 Jazzy installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
(`ros-jazzy-desktop`), then:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui python3-colcon-common-extensions
```

**On Windows**, use WSL2:

1. In PowerShell, run `wsl --install -d Ubuntu-24.04`.
2. Inside Ubuntu, follow the steps above. WSLg displays RViz and Gazebo windows directly.
3. Clone the repo inside the Linux file system (`~/CapLeg`), not under `/mnt/c`. Builds are much faster there.

### Build

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -y     # optional: resolves anything missing
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
# 1. Model only: RViz with joint sliders
ros2 launch capleg_description display.launch.py
ros2 launch capleg_description display.launch.py config:=transtibial

# 2. Kinematic gait playback on the rig (two terminals)
ros2 launch capleg_description display.launch.py rig:=true gui:=false
ros2 run capleg_control gait_player --ros-args -p rig:=true

# 3. Physics: Gazebo treadmill rig with ros2_control and the impedance controller
ros2 launch capleg_gazebo sim.launch.py
ros2 launch capleg_gazebo sim.launch.py config:=transtibial gui:=false rviz:=true
ros2 launch capleg_gazebo sim.launch.py stride_time:=1.0 body_mass:=85.0

# Live gain tuning while it walks
ros2 param set /capleg_controller knee.k_stance 500.0
ros2 param list /capleg_controller
```

### Interfaces

| Topic | Type | Notes |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | from `joint_state_broadcaster` |
| `/capleg/gait_phase` | `std_msgs/Float64` | estimated phase, 0 = heel strike |
| `/capleg/reference` | `sensor_msgs/JointState` | knee and ankle references and commanded torques |
| `/capleg/load_cell` | `geometry_msgs/WrenchStamped` | Gazebo F/T sensor via `ros_gz_bridge` |
| `/capleg/{thigh,shank,foot}_imu` | `sensor_msgs/Imu` | no thigh IMU in the transtibial config |
| `/capleg_effort_controller/commands` | `std_msgs/Float64MultiArray` | `[rig_z, knee, ankle_pitch, ankle_roll, toe]` |
| `/rig_position_controller/commands` | `std_msgs/Float64MultiArray` | `[rig_x, hip]` |

All controller parameters are listed, with defaults and units, in
[`capleg_control/config/controller.yaml`](../ros2_ws/src/capleg_control/config/controller.yaml).

### Other simulators

`ros2_ws/src/capleg_description/urdf/generated/capleg_{transfemoral,transtibial}.urdf` are standalone URDFs with
relative mesh paths. They load directly in PyBullet, the Isaac Sim URDF importer, Webots (`urdf2webots`) and
MuJoCo's URDF loader.

## C. Fusion 360

See [CAD_WORKFLOW.md](CAD_WORKFLOW.md) to:

- rebuild the design from the script;
- edit parameters;
- export and regenerate every simulation model.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ModuleNotFoundError: capleg_control` when running the MuJoCo scripts | Run them from the repo, as `python mujoco/run_rig.py`. They add `ros2_ws/src/capleg_control` to the path themselves |
| `--view` opens no window | Needs a display. Over SSH or in CI, drop `--view`. On macOS, use `mjpython` |
| Gazebo shows no meshes | `source install/setup.bash` in the terminal you launch from. The launch file sets `GZ_SIM_RESOURCE_PATH` from the installed package |
| Controllers fail to load in Gazebo | Check that `ros-jazzy-gz-ros2-control` and `ros-jazzy-ros2-controllers` are installed |
| `Filename too long` during `git clone` on Windows | `git config --global core.longpaths true`, or clone to a shorter path |
| Swing looks different in Gazebo than in MuJoCo | Expected: Gazebo/DART models neither rotor inertia (`armature`) nor contact stiffness from SDF. See [ARCHITECTURE.md](ARCHITECTURE.md#simulator-differences) |
