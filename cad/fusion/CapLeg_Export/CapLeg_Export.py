# -*- coding: utf-8 -*-
"""CapLeg - export the open CapLeg design for simulation / printing.

Writes (relative to the repository root, i.e. three folders above this script):
    cad/stl_world/*.stl, cad/step/CapLeg_assembly.step, cad/CapLeg.f3d, cad/fusion_export_meta.json
Then run   python3 tools/build_sim_models.py   to regenerate URDF/xacro, MJCF, meshes, inertias and print STLs.

Body names and the CapLeg_* component names must be kept (they drive link assignment and mass properties).
"""
import os
import sys
import traceback

import adsk.core

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'CapLeg_Builder'))


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        import CapLeg_Builder as cl
        meta = cl.export_all(ROOT)
        ui.messageBox('Exported %d bodies to\n%s\n\nNext: python3 tools/build_sim_models.py' % (len(meta['bodies']), ROOT))
    except Exception:
        ui.messageBox('CapLeg export failed:\n{}'.format(traceback.format_exc()))
