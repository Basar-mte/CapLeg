## What and why

<!-- One or two sentences. Link the issue if there is one. -->

## Checklist

- [ ] `ruff check .` and `pytest` pass
- [ ] Generated files were not hand-edited. If the CAD changed, I committed the export and the output of
      `python tools/build_sim_models.py` together
- [ ] If the gait or controller changed: the new `mujoco/results/metrics.json` is included, and the moved metrics
      are explained below
- [ ] The docs are updated if behaviour or usage changed
