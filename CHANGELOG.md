# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses [Semantic Versioning](https://semver.org/).

## [1.0.0] - 2026-10-10

First public release.

### Added
- Parametric Fusion 360 builder (`cad/fusion/CapLeg_Builder`) for the complete jointed knee–ankle assembly, and an
  export script (`cad/fusion/CapLeg_Export`) for STL, STEP, F3D and metadata.
- 26 print-oriented parts in `cad/stl_print/` with a print report, plus a STEP assembly.
- `tools/build_sim_models.py`: one generator from the CAD export to the xacro/URDF, MJCF, merged meshes, inertias and
  print STLs.
- ROS 2 Jazzy packages: `capleg_description` (RViz), `capleg_gazebo` (Gazebo Harmonic treadmill rig with
  ros2_control) and `capleg_control` (impedance controller node and gait player).
- A simulator-independent control core: a thigh-angle phase estimator, phase-based impedance control, an actuator
  model and the hip-simulator logic.
- MuJoCo treadmill hip simulator (`mujoco/run_rig.py`) with metrics and plots, plus an OpenGL-free GIF renderer.
- Documentation: a build guide, software setup, architecture, the CAD workflow and the BOM.
- Continuous integration for the Python core, the MuJoCo smoke test, the reproducibility of the generated models and
  the ROS 2 build.
