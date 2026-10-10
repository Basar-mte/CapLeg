#!/usr/bin/env python3
"""Generate the CapLeg simulation models and print files from the Fusion 360 export.

Inputs  (written by cad/fusion/CapLeg_Builder -> export_all):
    cad/stl_world/*.stl            every body, zero pose, assembly frame, millimetres
    cad/fusion_export_meta.json    body -> component map, appearances, joint data

Outputs:
    ros2_ws/src/capleg_description/meshes/visual/*.stl     merged per link & colour, link frame, metres
    ros2_ws/src/capleg_description/urdf/*.xacro            parametric robot description (ROS 2)
    ros2_ws/src/capleg_description/urdf/generated/*.urdf   expanded URDFs with relative mesh paths
    ros2_ws/src/capleg_description/config/mass_properties.yaml
    mujoco/capleg_rig.xml + mujoco/assets/*.stl            MuJoCo model on a treadmill hip simulator
    cad/stl_print/*.stl + cad/stl_print/print_report.csv   printable parts, print orientation, mm

Usage:  python3 tools/build_sim_models.py            (requires numpy, trimesh)
"""
import csv
import json
import math
import os
import shutil
import sys

import numpy as np
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'capleg_control'))
from capleg_control.core.params import ANKLE_DRIVE, GEOM, JOINTS, KNEE_DRIVE, PEA  # noqa: E402

CAD = os.path.join(ROOT, 'cad')
DESC = os.path.join(ROOT, 'ros2_ws', 'src', 'capleg_description')
MJ = os.path.join(ROOT, 'mujoco')

# ------------------------------------------------------------------ link structure (CAD mm frame)
LINK_ORIGIN_MM = {
    'thigh': (0.0, 0.0, 430.0),        # hip centre
    'shank': (0.0, 0.0, 0.0),          # knee axis
    'ankle_module': (0.0, 0.0, -303.0),  # 6-axis load cell centre (fixed joint)
    'talus': (0.0, 0.0, -420.0),       # ankle pitch axis
    'foot': (0.0, 0.0, -464.0),        # subtalar (roll) axis
    'toe': (120.0, 0.0, -498.5),       # MTP axis
}
KNEE_MODULE = {'P05', 'P06', 'P07', 'P08', 'P09', 'P10', 'H30', 'H32', 'H11', 'H12', 'H13'}
PYLON_GROUP = {'H20', 'P17'}
ANKLE_MODULE = {'P11', 'P12', 'P13', 'P14', 'P15', 'P16', 'P18', 'H31', 'H33', 'H14', 'H21'}

COLORS = {
    'Accent Red': (0.827, 0.173, 0.196, 1.0), 'Graphite': (0.16, 0.16, 0.17, 1.0),
    'Smoke Clear': (0.55, 0.57, 0.60, 0.55), 'Aluminium': (0.77, 0.78, 0.79, 1.0),
    'Steel': (0.65, 0.66, 0.67, 1.0), 'Actuator Black': (0.09, 0.09, 0.10, 1.0),
    'TPU Black': (0.07, 0.07, 0.07, 1.0), 'Cable White': (0.92, 0.92, 0.89, 1.0),
    'PCB Green': (0.10, 0.28, 0.18, 1.0),
}


def slug(s):
    return s.lower().replace(' ', '_')


def group_of(body):
    """Return (link, visual_group) for a Fusion body."""
    code = body['name'].split('_')[0]
    comp = body['link']
    if comp.endswith('Thigh'):
        return 'thigh', 'thigh'
    if comp.endswith('Shank'):
        if code in KNEE_MODULE or ('Knee' in body['name'] and code == 'H10'):
            return 'shank', 'shank_knee'
        if code in PYLON_GROUP:
            return 'shank', 'shank_pylon'
        return 'ankle_module', 'ankle_module'
    for k in ('Talus', 'Foot', 'Toe'):
        if comp.endswith(k):
            return k.lower(), k.lower()
    raise ValueError(body['name'])


def material_of(name):
    """('density', g/cm^3) or ('mass', kg). Printed parts use effective (infill) densities."""
    code = name.split('_')[0]
    if code == 'P01':
        return 'density', 1.27          # PETG check socket, solid walls
    if code in ('P02', 'P04', 'P10', 'P18', 'P20'):
        return 'density', 2.70          # aluminium: pyramid receiver, capstan disc/sector, drums
    if code in ('P08', 'P09', 'P14', 'P15'):
        return 'density', 1.15          # 3 mm covers, near-solid
    if code == 'P16':
        return 'mass', 0.10             # PEA cartridge incl. steel torsion spring
    if code in ('P22', 'P25'):
        return 'density', 0.70          # TPU 95A pads, ~50 % infill
    if code == 'P23':
        return 'density', 1.10          # TPU roll springs, solid
    if code == 'P24':
        return 'density', 1.00
    if code.startswith('P'):
        return 'density', 0.80          # PA-CF / PETG-CF structure, 5 walls + 40 % gyroid
    if code in ('H30', 'H31'):
        return 'mass', 0.485            # CubeMars AK80-9
    if code == 'H12':
        return 'mass', 0.12             # MCU + IMU + CAN stack
    if code == 'H21':
        return 'mass', 0.20             # 6-axis load cell
    if code in ('H10', 'H51'):
        return 'density', 4.7           # ball bearings (~60 % steel fill)
    if code in ('H01', 'H40', 'H42', 'H50'):
        return 'density', 7.85          # steel shafts / pins
    if code in ('H13', 'H14'):
        return 'density', 1.0           # cables
    return 'density', 2.70              # aluminium hardware


def mass_props(mesh_mm, name):
    kind, val = material_of(name)
    vol_m3 = mesh_mm.volume * 1e-9
    mass = val * 1000.0 * vol_m3 if kind == 'density' else val
    com = mesh_mm.center_mass * 1e-3
    m1 = mesh_mm.copy()
    m1.density = 1.0
    I_unit = m1.moment_inertia * 1e-15 / vol_m3 if vol_m3 > 0 else np.zeros((3, 3))  # per kg, m^2
    return mass, com, I_unit * mass


def combine(props, origin_m):
    M = sum(p[0] for p in props)
    C = sum(p[0] * (p[1] - origin_m) for p in props) / M
    I = np.zeros((3, 3))
    for m, c, Ic in props:
        d = (c - origin_m) - C
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, C, I


# ------------------------------------------------------------------ collision primitives (link frames, m)
COLLISION = {
    'foot': [('sphere', (-0.052, 0.018, -0.028), 0.018), ('sphere', (-0.052, -0.018, -0.028), 0.018),
             ('sphere', (0.104, 0.030, -0.034), 0.012), ('sphere', (0.104, -0.026, -0.034), 0.012),
             ('sphere', (0.030, -0.030, -0.036), 0.010)],
    'toe': [('sphere', (0.030, 0.022, -0.0025), 0.009), ('sphere', (0.030, -0.020, -0.0025), 0.009),
            ('sphere', (0.060, 0.012, -0.0025), 0.009)],
    'shank': [('capsule', ((-0.006, 0.0, -0.040), (-0.006, 0.0, -0.165)), 0.058)],
    'ankle_module': [('capsule', ((-0.020, 0.0, -0.040), (-0.060, 0.0, -0.075)), 0.050)],
    'thigh': [('capsule', ((0.0, 0.0, -0.060), (0.0, 0.0, -0.300)), 0.062)],
}

SITES = {   # sensor frames (link frames, m)
    'thigh': [('thigh_imu', (0.0, -0.020, -0.396))],
    'shank': [('shank_imu', (0.030, 0.019, -0.130))],
    'ankle_module': [('load_cell', (0.0, 0.0, 0.0))],
    'foot': [('foot_imu', (0.040, 0.0, -0.031)), ('heel', (-0.052, 0.0, -0.046)), ('ball', (0.104, 0.0, -0.046))],
    'toe': [('toe_tip', (0.070, 0.0, -0.0115))],
}

PRINT_ORIENT = {   # part -> rotation (axis, deg) applied before placing on the bed
    'P01': None, 'P02': None, 'P07': None, 'P13': None, 'P17': None, 'P21': None, 'P22': None,
    'P23': None, 'P24': None, 'P25': None,
    'P03': ('x', 90), 'P04': ('x', 90), 'P05': ('x', -90), 'P06': ('x', 90), 'P08': ('x', -90),
    'P09': ('x', 90), 'P10': ('x', 90), 'P11': ('x', -90), 'P12': ('x', 90), 'P14': ('x', -90),
    'P15': ('x', 90), 'P16': ('x', -90), 'P18': ('x', 90), 'P19': None, 'P20': ('x', 90),
}
PRINT_MATERIAL = {
    'P01': 'PETG (clear) check socket - patient specific in practice',
    'P02': 'Al 7075 / Ti (clinical); PA-CF for bench prototypes',
    'P04': 'Al 6061 (recommended) or PA-CF', 'P10': 'Al 6061 (recommended) or PA-CF',
    'P18': 'Al 6061 (recommended) or PA-CF', 'P20': 'Al 6061 (recommended) or PA-CF',
    'P08': 'PETG / ASA', 'P09': 'PETG / ASA', 'P14': 'PETG / ASA', 'P15': 'PETG / ASA',
    'P16': 'PA-CF housing + steel torsion spring', 'P22': 'TPU 95A', 'P23': 'TPU 85A-95A',
    'P25': 'TPU 95A', 'P24': 'PA-CF or PETG-CF',
}


def load():
    with open(os.path.join(CAD, 'fusion_export_meta.json'), encoding='utf-8') as fh:
        meta = json.load(fh)
    meshes = {}
    for b in meta['bodies']:
        meshes[b['name']] = trimesh.load(os.path.join(CAD, 'stl_world', b['name'] + '.stl'), force='mesh')
    return meta, meshes


def build_visuals_and_inertials(meta, meshes):
    vis_dir = os.path.join(DESC, 'meshes', 'visual')
    clean_dir(vis_dir)
    groups = {}            # (group, appearance) -> [mesh]
    props = {}             # link -> list of (m, com_world_m, I)
    props_tt_shank = []
    bodies_table = []
    for b in meta['bodies']:
        link, grp = group_of(b)
        m = meshes[b['name']]
        groups.setdefault((link, grp, b['appearance']), []).append(m)
        mp = mass_props(m, b['name'])
        props.setdefault(link, []).append(mp)
        if grp == 'shank_pylon':
            props_tt_shank.append(mp)
        bodies_table.append(dict(name=b['name'], link=link, group=grp, mass_kg=round(mp[0], 4),
                                 appearance=b['appearance']))
    visuals = {}           # link -> list of dicts(file, rgba, group)
    for (link, grp, app), ms in sorted(groups.items()):
        merged = trimesh.util.concatenate(ms)
        o = np.array(LINK_ORIGIN_MM[link])
        merged.apply_translation(-o)
        merged.apply_scale(0.001)
        fn = 'capleg_%s__%s.stl' % (grp, slug(app))
        merged.export(os.path.join(vis_dir, fn))
        visuals.setdefault(link, []).append(dict(file=fn, rgba=COLORS.get(app, (0.5, 0.5, 0.5, 1)), group=grp,
                                                 material=slug(app)))
    inertials = {}
    for link, pl in props.items():
        M, C, I = combine(pl, np.array(LINK_ORIGIN_MM[link]) * 1e-3)
        inertials[link] = (M, C, I)
    M, C, I = combine(props_tt_shank, np.array(LINK_ORIGIN_MM['shank']) * 1e-3)
    inertials['shank_tt'] = (M, C, I)
    return visuals, inertials, bodies_table


# ------------------------------------------------------------------ text helpers
def f(x, n=6):
    return ('%.' + str(n) + 'g') % x


def vec(v, n=6):
    return ' '.join(f(x, n) for x in v)


def clean_dir(path):
    """Empty (or create) an output folder. The folder itself is kept, so synced or open folders on Windows are fine."""
    os.makedirs(path, exist_ok=True)
    for name in os.listdir(path):
        full = os.path.join(path, name)
        if os.path.isdir(full):
            shutil.rmtree(full)
        else:
            os.remove(full)


def write_text(path, text):
    """Write a generated text file byte-identically on every OS (UTF-8, LF line endings)."""
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)


def inertia_xml_urdf(M, C, I, indent):
    return (f'{indent}<inertial>\n{indent}  <origin xyz="{vec(C)}" rpy="0 0 0"/>\n{indent}  <mass value="{f(M)}"/>\n'
            f'{indent}  <inertia ixx="{f(I[0,0])}" ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" iyy="{f(I[1,1])}" '
            f'iyz="{f(I[1,2])}" izz="{f(I[2,2])}"/>\n{indent}</inertial>\n')


def urdf_link(name, link, visuals, inertials, mesh_prefix, indent='  ', tt=False, xacro=False):
    vis = visuals.get(link, [])
    M, C, I = inertials['shank_tt' if (tt and link == 'shank') else link]
    s = f'{indent}<link name="capleg_{name}">\n'
    s += inertia_xml_urdf(M, C, I, indent + '  ')
    for v in vis:
        cond_open, cond_close = '', ''
        if link == 'shank' and v['group'] == 'shank_knee':
            if tt:
                continue
            if xacro:
                cond_open = f'{indent}  <xacro:if value="${{config == \'transfemoral\'}}">\n'
                cond_close = f'{indent}  </xacro:if>\n'
        s += cond_open
        s += (f'{indent}  <visual>\n{indent}    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
              f'{indent}    <geometry><mesh filename="{mesh_prefix}{v["file"]}"/></geometry>\n'
              f'{indent}    <material name="capleg_{v["material"]}"/>\n{indent}  </visual>\n')
        s += cond_close
    for c in COLLISION.get(link, []):
        if c[0] == 'sphere':
            s += (f'{indent}  <collision>\n{indent}    <origin xyz="{vec(c[1])}" rpy="0 0 0"/>\n'
                  f'{indent}    <geometry><sphere radius="{f(c[2])}"/></geometry>\n{indent}  </collision>\n')
        else:
            (a, b), r = c[1], c[2]
            a, b = np.array(a), np.array(b)
            mid, L = (a + b) / 2, np.linalg.norm(b - a)
            d = (b - a) / L
            pitch = math.atan2(d[0], d[2])        # rotate z-axis cylinder into the a->b direction (XZ plane)
            s += (f'{indent}  <collision>\n{indent}    <origin xyz="{vec(mid)}" rpy="0 {f(pitch)} 0"/>\n'
                  f'{indent}    <geometry><cylinder radius="{f(r)}" length="{f(L)}"/></geometry>\n'
                  f'{indent}  </collision>\n')
    s += f'{indent}</link>\n'
    return s


def materials_xml(indent='  '):
    return ''.join(f'{indent}<material name="capleg_{slug(k)}"><color rgba="{vec(v, 4)}"/></material>\n'
                   for k, v in COLORS.items())


def joint_rel(child, parent):
    a, b = np.array(LINK_ORIGIN_MM[child]), np.array(LINK_ORIGIN_MM[parent])
    return (a - b) * 1e-3


PEA_MAX = PEA['stiffness'] * JOINTS['ankle_pitch']['upper']      # parallel-spring torque at full dorsiflexion
JOINT_DEF = [  # name, parent, child, axis, limits key, effort (joint total: motor + parallel spring), velocity
    ('knee', 'thigh', 'shank', (0, 1, 0), 'knee', KNEE_DRIVE['peak_torque'], KNEE_DRIVE['max_speed']),
    ('ankle_pitch', 'ankle_module', 'talus', (0, -1, 0), 'ankle_pitch', ANKLE_DRIVE['peak_torque'] + PEA_MAX,
     ANKLE_DRIVE['max_speed']),
    ('ankle_roll', 'talus', 'foot', (1, 0, 0), 'ankle_roll', 60.0, 10.0),
    ('toe', 'foot', 'toe', (0, -1, 0), 'toe', 35.0, 15.0),
]


def urdf_joint(j, indent='  '):
    name, parent, child, axis, key, eff, vel = j
    lim = JOINTS[key]
    return (f'{indent}<joint name="{name}" type="revolute">\n{indent}  <parent link="capleg_{parent}"/>\n'
            f'{indent}  <child link="capleg_{child}"/>\n{indent}  <origin xyz="{vec(joint_rel(child, parent))}" rpy="0 0 0"/>\n'
            f'{indent}  <axis xyz="{vec(axis)}"/>\n'
            f'{indent}  <limit lower="{f(lim["lower"])}" upper="{f(lim["upper"])}" effort="{f(eff, 5)}" velocity="{f(vel, 4)}"/>\n'
            f'{indent}  <dynamics damping="{f(lim["damping"])}" friction="{f(lim["friction"])}"/>\n{indent}</joint>\n')


def fixed_joint(name, parent, child, xyz, indent='  '):
    return (f'{indent}<joint name="{name}" type="fixed">\n{indent}  <parent link="{parent}"/>\n'
            f'{indent}  <child link="{child}"/>\n{indent}  <origin xyz="{vec(xyz)}" rpy="0 0 0"/>\n{indent}</joint>\n')


def sensor_links(link, indent='  '):
    s = ''
    for nm, xyz in SITES.get(link, []):
        if nm == 'load_cell':
            continue
        s += f'{indent}<link name="capleg_{nm}_link"/>\n'
        s += fixed_joint(f'capleg_{nm}_joint', f'capleg_{link}', f'capleg_{nm}_link', xyz, indent)
    return s


def build_urdf_body(visuals, inertials, mesh_prefix, tt=False, xacro=False, indent='  '):
    s = ''
    if xacro:
        s += f'{indent}<xacro:if value="${{config == \'transfemoral\'}}">\n'
    if xacro or not tt:
        s += urdf_link('thigh', 'thigh', visuals, inertials, mesh_prefix, indent)
        s += sensor_links('thigh', indent)
        s += urdf_joint(JOINT_DEF[0], indent)
    if xacro:
        s += f'{indent}</xacro:if>\n'
    if xacro:
        s += f'{indent}<xacro:if value="${{config == \'transfemoral\'}}">\n'
        s += urdf_link('shank', 'shank', visuals, inertials, mesh_prefix, indent, tt=False, xacro=True)
        s += f'{indent}</xacro:if>\n{indent}<xacro:unless value="${{config == \'transfemoral\'}}">\n'
        s += urdf_link('shank', 'shank', visuals, inertials, mesh_prefix, indent, tt=True)
        s += f'{indent}</xacro:unless>\n'
    else:
        s += urdf_link('shank', 'shank', visuals, inertials, mesh_prefix, indent, tt=tt)
    s += sensor_links('shank', indent)
    s += urdf_link('ankle_module', 'ankle_module', visuals, inertials, mesh_prefix, indent)
    s += fixed_joint('load_cell_joint', 'capleg_shank', 'capleg_ankle_module', joint_rel('ankle_module', 'shank'), indent)
    for lk, jd in (('talus', JOINT_DEF[1]), ('foot', JOINT_DEF[2]), ('toe', JOINT_DEF[3])):
        s += urdf_link(lk, lk, visuals, inertials, mesh_prefix, indent)
        s += urdf_joint(jd, indent)
        s += sensor_links(lk, indent)
    return s


# ------------------------------------------------------------------ xacro files
def write_xacro(visuals, inertials):
    ud = os.path.join(DESC, 'urdf')
    os.makedirs(os.path.join(ud, 'generated'), exist_ok=True)
    body = build_urdf_body(visuals, inertials, 'package://capleg_description/meshes/visual/', xacro=True,
                           indent='    ')
    macro = f'''<?xml version="1.0"?>
<!-- CapLeg robot description macro. GENERATED by tools/build_sim_models.py from the Fusion 360 export. -->
<!-- Frames: +X anterior, +Y medial (right leg), +Z up. Joint signs: knee +flexion, ankle_pitch +dorsiflexion,
     ankle_roll +inversion, toe +dorsiflexion. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="capleg" params="config">
{materials_xml('    ')}{body}  </xacro:macro>
</robot>
'''
    write_text(os.path.join(ud, 'capleg_macro.xacro'), macro)

    rig = f'''<?xml version="1.0"?>
<!-- Treadmill hip simulator: world -> rig_x (belt progression) -> rig_z (vertical hip) -> hip (thigh angle).
     The prosthesis is attached at the hip (transfemoral) or at the residual-knee level (transtibial). -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="capleg_rig" params="config">
    <link name="world"/>
    <!-- rig_base needs mass: sdformat drops zero-mass links when converting URDF -> SDF -->
    <link name="rig_base">
      <inertial><mass value="10.0"/><inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/></inertial>
    </link>
    <joint name="rig_base_joint" type="fixed"><parent link="world"/><child link="rig_base"/>
      <origin xyz="0 0 0" rpy="0 0 0"/></joint>
    <link name="rig_carriage_x">
      <inertial><origin xyz="0 0 {f(GEOM['leg_len'])}"/><mass value="5.0"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
    </link>
    <joint name="rig_x" type="prismatic"><parent link="rig_base"/><child link="rig_carriage_x"/>
      <origin xyz="0 0 0"/><axis xyz="1 0 0"/><limit lower="-5.0" upper="500.0" effort="5000" velocity="5.0"/></joint>
    <link name="rig_hip_carriage">
      <inertial><origin xyz="0 0 0"/><mass value="2.0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
      <visual><geometry><box size="0.08 0.16 0.04"/></geometry><material name="capleg_graphite"/></visual>
    </link>
    <xacro:property name="hip_z" value="${{{f(GEOM['leg_len'])} if config == 'transfemoral' else {f(GEOM['leg_len'] - GEOM['thigh_len'])}}}"/>
    <joint name="rig_z" type="prismatic"><parent link="rig_carriage_x"/><child link="rig_hip_carriage"/>
      <origin xyz="0 0 ${{hip_z}}"/><axis xyz="0 0 1"/><limit lower="-0.25" upper="0.10" effort="3000" velocity="3.0"/>
      <dynamics damping="0.0" friction="0.0"/></joint>
    <xacro:if value="${{config == 'transfemoral'}}">
      <joint name="hip" type="revolute"><parent link="rig_hip_carriage"/><child link="capleg_thigh"/>
        <origin xyz="0 0 0"/><axis xyz="0 -1 0"/><limit lower="-0.6" upper="0.9" effort="400" velocity="8.0"/></joint>
    </xacro:if>
    <xacro:unless value="${{config == 'transfemoral'}}">
      <!-- transtibial: the rig 'hip' joint becomes the residual knee driving the shank (shank-angle phase) -->
      <joint name="hip" type="revolute"><parent link="rig_hip_carriage"/><child link="capleg_shank"/>
        <origin xyz="0 0 0"/><axis xyz="0 -1 0"/><limit lower="-1.2" upper="0.9" effort="400" velocity="8.0"/></joint>
    </xacro:unless>
  </xacro:macro>
</robot>
'''
    write_text(os.path.join(ud, 'rig.xacro'), rig)

    def jblock(name, cmd, lo, hi, init=0.0):
        return (f'      <joint name="{name}">\n        <command_interface name="{cmd}">\n'
                f'          <param name="min">{f(lo)}</param>\n          <param name="max">{f(hi)}</param>\n'
                f'        </command_interface>\n        <state_interface name="position">\n'
                f'          <param name="initial_value">{f(init)}</param>\n        </state_interface>\n'
                f'        <state_interface name="velocity"/>\n        <state_interface name="effort"/>\n      </joint>\n')
    rc = f'''<?xml version="1.0"?>
<!-- ros2_control for Gazebo Harmonic (gz_ros2_control). Prosthesis joints are torque (effort) controlled by
     capleg_control/controller_node (phase-based impedance). Rig: rig_x / hip position, rig_z effort (load servo). -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="capleg_ros2_control" params="config rig">
    <ros2_control name="CapLegSystem" type="system">
      <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        <param name="position_proportional_gain">0.5</param>
      </hardware>
      <xacro:if value="${{rig}}">
{jblock('rig_x', 'position', -5.0, 500.0)}{jblock('hip', 'position', -1.2, 0.9)}{jblock('rig_z', 'effort', -3000.0, 3000.0)}      </xacro:if>
      <xacro:if value="${{config == 'transfemoral'}}">
{jblock('knee', 'effort', -KNEE_DRIVE['peak_torque'], KNEE_DRIVE['peak_torque'])}      </xacro:if>
{jblock('ankle_pitch', 'effort', -ANKLE_DRIVE['peak_torque'] - PEA_MAX, ANKLE_DRIVE['peak_torque'])}{jblock('ankle_roll', 'effort', -60.0, 60.0)}{jblock('toe', 'effort', -35.0, 35.0)}    </ros2_control>
  </xacro:macro>
</robot>
'''
    write_text(os.path.join(ud, 'capleg.ros2_control.xacro'), rc)

    gz = '''<?xml version="1.0"?>
<!-- Gazebo Harmonic extensions: ros2_control plugin, foot friction, sensors (IMUs, 6-axis load cell). -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="capleg_gazebo" params="controllers_file config">
    <gazebo>
      <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>${controllers_file}</parameters>
      </plugin>
    </gazebo>
    <!-- DART reads friction only (kp/kd/minDepth are ODE-specific and ignored) -->
    <xacro:macro name="foot_surface" params="link">
      <gazebo reference="${link}">
        <mu1>1.0</mu1>
        <mu2>1.0</mu2>
      </gazebo>
    </xacro:macro>
    <xacro:foot_surface link="capleg_foot"/>
    <xacro:foot_surface link="capleg_toe"/>
    <!-- keep the load-cell joint so the force-torque sensor has a joint to measure -->
    <gazebo reference="load_cell_joint">
      <preserveFixedJoint>true</preserveFixedJoint>
      <sensor name="load_cell" type="force_torque">
        <always_on>true</always_on>
        <update_rate>500</update_rate>
        <topic>capleg/load_cell</topic>
        <force_torque><frame>child</frame><measure_direction>child_to_parent</measure_direction></force_torque>
      </sensor>
    </gazebo>
    <xacro:macro name="imu" params="link name">
      <gazebo reference="${link}">
        <sensor name="${name}" type="imu">
          <always_on>true</always_on>
          <update_rate>500</update_rate>
          <topic>capleg/${name}</topic>
          <imu>
            <angular_velocity>
              <x><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></x>
              <y><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></y>
              <z><noise type="gaussian"><mean>0</mean><stddev>0.002</stddev></noise></z>
            </angular_velocity>
            <linear_acceleration>
              <x><noise type="gaussian"><mean>0</mean><stddev>0.02</stddev></noise></x>
              <y><noise type="gaussian"><mean>0</mean><stddev>0.02</stddev></noise></y>
              <z><noise type="gaussian"><mean>0</mean><stddev>0.02</stddev></noise></z>
            </linear_acceleration>
          </imu>
        </sensor>
      </gazebo>
    </xacro:macro>
    <!-- sensors on the fixed IMU frames: sdformat lumps them into the parent link with the correct offset -->
    <xacro:if value="${config == 'transfemoral'}">
      <xacro:imu link="capleg_thigh_imu_link" name="thigh_imu"/>
    </xacro:if>
    <xacro:imu link="capleg_shank_imu_link" name="shank_imu"/>
    <xacro:imu link="capleg_foot_imu_link" name="foot_imu"/>
  </xacro:macro>
</robot>
'''
    write_text(os.path.join(ud, 'capleg.gazebo.xacro'), gz)

    top = '''<?xml version="1.0"?>
<!--
  CapLeg - capstan-driven powered transfemoral prosthesis (research platform)

  xacro args
    config           transfemoral | transtibial   (transtibial = ankle-foot module only)
    rig              true | false                  treadmill hip simulator (world-fixed)
    sim              none | gazebo                 adds ros2_control + Gazebo Harmonic plugins/sensors
    controllers_file path to the ros2_control controller YAML (used when sim:=gazebo)

  Examples
    xacro capleg.urdf.xacro config:=transfemoral rig:=false sim:=none
    xacro capleg.urdf.xacro rig:=true sim:=gazebo controllers_file:=/path/controllers.yaml
-->
<robot name="capleg" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="config" default="transfemoral"/>
  <xacro:arg name="rig" default="false"/>
  <xacro:arg name="sim" default="none"/>
  <xacro:arg name="controllers_file" default=""/>
  <xacro:property name="config" value="$(arg config)"/>

  <xacro:include filename="$(find capleg_description)/urdf/capleg_macro.xacro"/>
  <xacro:include filename="$(find capleg_description)/urdf/rig.xacro"/>
  <xacro:include filename="$(find capleg_description)/urdf/capleg.ros2_control.xacro"/>
  <xacro:include filename="$(find capleg_description)/urdf/capleg.gazebo.xacro"/>

  <xacro:capleg config="${config}"/>
  <xacro:if value="$(arg rig)">
    <xacro:capleg_rig config="${config}"/>
  </xacro:if>
  <xacro:if value="${'$(arg sim)' == 'gazebo'}">
    <xacro:capleg_ros2_control config="${config}" rig="$(arg rig)"/>
    <xacro:capleg_gazebo controllers_file="$(arg controllers_file)" config="${config}"/>
  </xacro:if>
</robot>
'''
    write_text(os.path.join(ud, 'capleg.urdf.xacro'), top)

    # expanded URDFs with relative mesh paths (PyBullet, Isaac Sim, Webots, MuJoCo import, ...)
    for tag, tt in (('transfemoral', False), ('transtibial', True)):
        body = build_urdf_body(visuals, inertials, '../../meshes/visual/', tt=tt)
        urdf = (f'<?xml version="1.0"?>\n<!-- CapLeg {tag} (standalone). GENERATED by tools/build_sim_models.py -->\n'
                f'<robot name="capleg_{tag}">\n{materials_xml()}{body}</robot>\n')
        write_text(os.path.join(ud, 'generated', f'capleg_{tag}.urdf'), urdf)


# ------------------------------------------------------------------ MuJoCo
def write_mjcf(visuals, inertials):
    ad = os.path.join(MJ, 'assets')
    clean_dir(ad)
    assets, mats = [], set()
    for vs in visuals.values():
        for v in vs:
            shutil.copy(os.path.join(DESC, 'meshes', 'visual', v['file']), os.path.join(ad, v['file']))
            assets.append(f'    <mesh name="{v["file"][:-4]}" file="{v["file"]}"/>')
            mats.add(v['material'])
    mat_xml = '\n'.join(f'    <material name="{slug(k)}" rgba="{vec(c, 4)}" specular="0.3" shininess="0.4"/>'
                        for k, c in COLORS.items())

    def body_inertial(link, ind):
        M, C, I = inertials[link]
        return (f'{ind}<inertial pos="{vec(C)}" mass="{f(M)}" fullinertia="{f(I[0,0])} {f(I[1,1])} {f(I[2,2])} '
                f'{f(I[0,1])} {f(I[0,2])} {f(I[1,2])}"/>\n')

    def geoms(link, ind):
        s = ''
        for v in visuals.get(link, []):
            s += f'{ind}<geom class="visual" mesh="{v["file"][:-4]}" material="{v["material"]}"/>\n'
        for c in COLLISION.get(link, []):
            cls = 'collision' if link in ('foot', 'toe') else 'nocontact'
            if c[0] == 'sphere':
                s += f'{ind}<geom class="{cls}" type="sphere" pos="{vec(c[1])}" size="{f(c[2])}"/>\n'
            else:
                (a, b), r = c[1], c[2]
                s += f'{ind}<geom class="{cls}" type="capsule" fromto="{vec(a)} {vec(b)}" size="{f(r)}"/>\n'
        for nm, xyz in SITES.get(link, []):
            s += f'{ind}<site name="{nm}" pos="{vec(xyz)}" size="0.006" rgba="0.1 0.6 1 0.6" group="4"/>\n'
        return s

    jk, ja, jr, jt = JOINTS['knee'], JOINTS['ankle_pitch'], JOINTS['ankle_roll'], JOINTS['toe']
    rel = joint_rel
    xml = f'''<mujoco model="capleg_rig">
  <!-- CapLeg on a treadmill hip simulator. GENERATED by tools/build_sim_models.py from the Fusion 360 export.
       Frames: +X anterior (walking direction), +Y medial (right leg), +Z up.
       Joints: rig_x (belt progression, position servo), rig_z (vertical hip, force), hip (thigh angle, position servo),
               knee / ankle_pitch (torque: AK80-9 + capstan, with reflected rotor inertia), ankle_roll (TPU springs),
               toe (MTP torsion springs), PEA = unilateral parallel spring on ankle dorsiflexion (tendon dead-band). -->
  <compiler angle="radian" meshdir="assets" autolimits="true"/>
  <option timestep="0.001" integrator="implicitfast" cone="elliptic" impratio="5"/>
  <size memory="20M"/>
  <visual>
    <global offwidth="1280" offheight="960" azimuth="-90" elevation="-8"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.6 0.6 0.6"/>
  </visual>
  <default>
    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/></default>
    <default class="collision"><geom contype="1" conaffinity="0" group="3" condim="3" friction="1.0 0.02 0.002"
        solref="0.006 1" solimp="0.9 0.95 0.001" rgba="0.2 0.6 1 0.4"/></default>
    <default class="nocontact"><geom contype="0" conaffinity="0" group="3" rgba="0.2 0.6 1 0.2"/></default>
  </default>
  <asset>
    <texture name="belt" type="2d" builtin="checker" rgb1="0.16 0.16 0.17" rgb2="0.22 0.22 0.23" width="512" height="512"/>
    <material name="belt" texture="belt" texrepeat="20 20" reflectance="0.02"/>
{mat_xml}
{chr(10).join(assets)}
  </asset>
  <worldbody>
    <light name="key" pos="0.5 -2.0 3.0" dir="-0.2 0.7 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="belt" contype="0" conaffinity="1" friction="1.0 0.02 0.002"/>
    <body name="rig_carriage_x">
      <joint name="rig_x" type="slide" axis="1 0 0" range="-5 500"/>
      <inertial pos="0 0 {f(GEOM['leg_len'])}" mass="5" diaginertia="0.1 0.1 0.1"/>
      <body name="rig_hip_carriage" pos="0 0 {f(GEOM['leg_len'])}">
        <joint name="rig_z" type="slide" axis="0 0 1" range="-0.25 0.10"/>
        <inertial pos="0 0 0" mass="2" diaginertia="0.01 0.01 0.01"/>
        <geom class="visual" type="box" size="0.04 0.08 0.02" material="graphite"/>
        <body name="capleg_thigh">
          <joint name="hip" type="hinge" axis="0 -1 0" range="-0.6 0.9"/>
{body_inertial('thigh', '          ')}{geoms('thigh', '          ')}          <body name="capleg_shank" pos="{vec(rel('shank', 'thigh'))}">
            <joint name="knee" type="hinge" axis="0 1 0" range="{f(jk['lower'])} {f(jk['upper'])}"
                   armature="{f(KNEE_DRIVE['armature'])}" damping="{f(jk['damping'])}" frictionloss="{f(jk['friction'])}"/>
{body_inertial('shank', '            ')}{geoms('shank', '            ')}            <body name="capleg_ankle_module" pos="{vec(rel('ankle_module', 'shank'))}">
{body_inertial('ankle_module', '              ')}{geoms('ankle_module', '              ')}              <body name="capleg_talus" pos="{vec(rel('talus', 'ankle_module'))}">
                <joint name="ankle_pitch" type="hinge" axis="0 -1 0" range="{f(ja['lower'])} {f(ja['upper'])}"
                       armature="{f(ANKLE_DRIVE['armature'])}" damping="{f(ja['damping'])}" frictionloss="{f(ja['friction'])}"/>
{body_inertial('talus', '                ')}{geoms('talus', '                ')}                <body name="capleg_foot" pos="{vec(rel('foot', 'talus'))}">
                  <joint name="ankle_roll" type="hinge" axis="1 0 0" range="{f(jr['lower'])} {f(jr['upper'])}"
                         stiffness="{f(jr['stiffness'])}" springref="0" damping="{f(jr['damping'])}" frictionloss="{f(jr['friction'])}"/>
{body_inertial('foot', '                  ')}{geoms('foot', '                  ')}                  <body name="capleg_toe" pos="{vec(rel('toe', 'foot'))}">
                    <joint name="toe" type="hinge" axis="0 -1 0" range="{f(jt['lower'])} {f(jt['upper'])}"
                           stiffness="{f(jt['stiffness'])}" springref="0" damping="{f(jt['damping'])}" frictionloss="{f(jt['friction'])}"/>
{body_inertial('toe', '                    ')}{geoms('toe', '                    ')}                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <!-- unilateral parallel-elastic actuator (PEA): springs back only when dorsiflexed beyond {PEA['engage_angle']} rad -->
    <fixed name="pea" stiffness="{f(PEA['stiffness'])}" springlength="-10 {f(PEA['engage_angle'])}">
      <joint joint="ankle_pitch" coef="1"/>
    </fixed>
  </tendon>
  <contact>
    <exclude body1="capleg_foot" body2="capleg_toe"/>
  </contact>
  <actuator>
    <!-- rig axes are force/torque driven; set-points + PD gains live in capleg_control.core.controller.RigController -->
    <motor name="rig_x" joint="rig_x" ctrlrange="-8000 8000"/>
    <motor name="rig_z" joint="rig_z" ctrlrange="-3000 3000"/>
    <motor name="hip" joint="hip" ctrlrange="-800 800"/>
    <motor name="knee" joint="knee" ctrlrange="{f(-KNEE_DRIVE['peak_torque'])} {f(KNEE_DRIVE['peak_torque'])}"/>
    <motor name="ankle" joint="ankle_pitch" ctrlrange="{f(-ANKLE_DRIVE['peak_torque'])} {f(ANKLE_DRIVE['peak_torque'])}"/>
  </actuator>
  <sensor>
    <jointpos name="q_hip" joint="hip"/>
    <jointpos name="q_knee" joint="knee"/>
    <jointpos name="q_ankle" joint="ankle_pitch"/>
    <jointpos name="q_roll" joint="ankle_roll"/>
    <jointpos name="q_toe" joint="toe"/>
    <jointvel name="qd_hip" joint="hip"/>
    <jointvel name="qd_knee" joint="knee"/>
    <jointvel name="qd_ankle" joint="ankle_pitch"/>
    <actuatorfrc name="tau_knee" actuator="knee"/>
    <actuatorfrc name="tau_ankle" actuator="ankle"/>
    <force name="load_cell_force" site="load_cell"/>
    <torque name="load_cell_torque" site="load_cell"/>
    <gyro name="thigh_gyro" site="thigh_imu"/>
    <accelerometer name="thigh_acc" site="thigh_imu"/>
    <gyro name="shank_gyro" site="shank_imu"/>
    <accelerometer name="shank_acc" site="shank_imu"/>
    <gyro name="foot_gyro" site="foot_imu"/>
    <accelerometer name="foot_acc" site="foot_imu"/>
  </sensor>
</mujoco>
'''
    write_text(os.path.join(MJ, 'capleg_rig.xml'), xml)


# ------------------------------------------------------------------ print files
def write_print_files(meta, meshes):
    pd = os.path.join(CAD, 'stl_print')
    clean_dir(pd)
    rows = []
    for b in meta['bodies']:
        code = b['name'].split('_')[0]
        if not code.startswith('P'):
            continue
        m = meshes[b['name']].copy()
        rot = PRINT_ORIENT.get(code)
        if rot:
            ax = {'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}[rot[0]]
            m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(rot[1]), ax))
        lo, hi = m.bounds
        m.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
        ext = m.extents
        m.export(os.path.join(pd, b['name'] + '.stl'))
        kind, val = material_of(b['name'])

        def fits(bx, by, bz, ext=ext):
            a, c = sorted(ext[:2])
            return bool(ext[2] <= bz and a <= min(bx, by) and c <= max(bx, by))
        rows.append(dict(part=b['name'], material=PRINT_MATERIAL.get(code, 'PA-CF or PETG-CF (structural)'),
                         x_mm=round(ext[0], 1), y_mm=round(ext[1], 1), z_mm=round(ext[2], 1),
                         volume_cm3=round(m.volume / 1000, 1), watertight=bool(m.is_watertight),
                         fits_256x256x256=fits(256, 256, 256), fits_250x210x220=fits(250, 210, 220),
                         est_mass_g=round((val * m.volume / 1000) if kind == 'density' else val * 1000, 0)))
    with open(os.path.join(pd, 'print_report.csv'), 'w', encoding='utf-8', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()), lineterminator='\n')
        w.writeheader()
        w.writerows(rows)
    return rows


def main():
    meta, meshes = load()
    visuals, inertials, table = build_visuals_and_inertials(meta, meshes)
    write_xacro(visuals, inertials)
    write_mjcf(visuals, inertials)
    rows = write_print_files(meta, meshes)
    cfg = os.path.join(DESC, 'config')
    os.makedirs(cfg, exist_ok=True)
    with open(os.path.join(cfg, 'mass_properties.yaml'), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('# GENERATED by tools/build_sim_models.py - link mass properties (SI, link frames)\n')
        tot = 0.0
        for link, (M, C, I) in inertials.items():
            fh.write(f'{link}:\n  mass: {M:.4f}\n  com: [{C[0]:.5f}, {C[1]:.5f}, {C[2]:.5f}]\n'
                     f'  inertia: [{I[0,0]:.6g}, {I[1,1]:.6g}, {I[2,2]:.6g}, {I[0,1]:.6g}, {I[0,2]:.6g}, {I[1,2]:.6g}]'
                     f'   # ixx iyy izz ixy ixz iyz\n')
            if link != 'shank_tt':
                tot += M
        fh.write(f'total_transfemoral_mass_incl_socket: {tot:.3f}\n')
        fh.write('bodies:\n')
        for r in table:
            fh.write(f'  - {{name: {r["name"]}, link: {r["link"]}, mass_kg: {r["mass_kg"]}}}\n')
    print('links:', {k: round(v[0], 3) for k, v in inertials.items()})
    print('total (TF, incl. socket): %.3f kg' % sum(v[0] for k, v in inertials.items() if k != 'shank_tt'))
    print('print parts:', len(rows))


if __name__ == '__main__':
    main()
