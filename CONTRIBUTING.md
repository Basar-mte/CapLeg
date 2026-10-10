# Contributing to CapLeg

Thank you for your interest. Bug reports, simulation results, build reports and pull requests are all welcome.

## Ground rules

- **The CAD model is the single source of truth.** Do not hand-edit generated files. The following are rewritten by
  `tools/build_sim_models.py`:
  - `ros2_ws/src/capleg_description/urdf/*.xacro` and `urdf/generated/*.urdf`
  - `ros2_ws/src/capleg_description/meshes/`
  - `ros2_ws/src/capleg_description/config/mass_properties.yaml`
  - `mujoco/capleg_rig.xml` and `mujoco/assets/`
  - `cad/stl_print/`

  To change geometry, edit `cad/fusion/CapLeg_Builder/CapLeg_Builder.py` (or the design in Fusion), export, then
  regenerate. See [docs/CAD_WORKFLOW.md](docs/CAD_WORKFLOW.md).
- **Physical and control parameters live in one place:**
  `ros2_ws/src/capleg_control/capleg_control/core/params.py`. MuJoCo, ROS 2 and the generator all import it.
- **Keep the core simulator-independent.** `capleg_control/core/` must import nothing from ROS or MuJoCo.

## Development setup

```bash
python -m venv .venv
source .venv/bin/activate            # Windows: .venv\Scripts\activate
pip install -r requirements-dev.txt
```

## Before you open a pull request

```bash
ruff check .                                          # lint
pytest                                                # unit + integration tests (no ROS needed)
python tools/build_sim_models.py && git status        # generated files must not change unless you meant to
```

If your change affects the simulated gait, re-run `python mujoco/run_rig.py` and include the new
`mujoco/results/metrics.json` in the PR. Explain any metric that moved.

CI runs the same checks, and it also builds and tests the ROS 2 workspace on ROS 2 Jazzy.

## Commit style

Use short imperative subjects, such as `Add stair-ascent reference` or `Fix PEA sign in Gazebo emulation`.
Reference an issue where there is one.

## Safety

CapLeg is a research prototype, not a medical device. Never test with a person until you have read
[Safety](docs/BUILD_GUIDE.md#0-safety-first) and met every requirement there.
