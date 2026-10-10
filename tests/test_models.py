"""Consistency of the generated robot descriptions (no ROS install needed).

Checks that every xacro variant expands, that the URDF, MJCF and ros2_control YAML agree on joint names, and that
every referenced mesh exists.
"""
import itertools
import os
import re
import xml.etree.ElementTree as ET

import mujoco
import pytest
from conftest import DESC, GAZEBO, MUJOCO_DIR

LEG_JOINTS = {
    'transfemoral': ['knee', 'ankle_pitch', 'ankle_roll', 'toe'],
    'transtibial': ['ankle_pitch', 'ankle_roll', 'toe'],
}
RIG_JOINTS = ['rig_x', 'rig_z', 'hip']


@pytest.fixture(scope='module')
def xacro_dir(tmp_path_factory):
    """Copy of urdf/ with $(find capleg_description) resolved, so xacro runs without ament_index."""
    xacro = pytest.importorskip('xacro')
    d = tmp_path_factory.mktemp('urdf')
    src = os.path.join(DESC, 'urdf')
    for name in os.listdir(src):
        if name.endswith('.xacro'):
            with open(os.path.join(src, name), encoding='utf-8') as fh:
                text = fh.read().replace('$(find capleg_description)/urdf', d.as_posix())
            (d / name).write_text(text, encoding='utf-8')
    return xacro, d


@pytest.mark.parametrize('config,rig,sim',
                         list(itertools.product(LEG_JOINTS, ['true', 'false'], ['none', 'gazebo'])))
def test_xacro_variants_expand(xacro_dir, config, rig, sim):
    xacro, d = xacro_dir
    doc = xacro.process_file(str(d / 'capleg.urdf.xacro'), mappings={
        'config': config, 'rig': rig, 'sim': sim, 'controllers_file': '/tmp/controllers.yaml'})
    root = ET.fromstring(doc.toxml())
    joints = [j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed']
    expected = LEG_JOINTS[config] + (RIG_JOINTS if rig == 'true' else [])
    assert sorted(joints) == sorted(expected)
    if sim == 'gazebo':
        rc = root.find('ros2_control')
        assert rc is not None
        assert {j.get('name') for j in rc.findall('joint')} == set(expected)


@pytest.mark.parametrize('config', LEG_JOINTS)
def test_generated_urdf_loads_and_meshes_exist(config):
    path = os.path.join(DESC, 'urdf', 'generated', f'capleg_{config}.urdf')
    root = ET.parse(path).getroot()
    for mesh in root.iter('mesh'):
        assert os.path.isfile(os.path.normpath(os.path.join(os.path.dirname(path), mesh.get('filename'))))
    model = mujoco.MjModel.from_xml_path(path)             # also proves it is a valid URDF
    names = {model.joint(i).name for i in range(model.njnt)}
    assert names == set(LEG_JOINTS[config])


def test_mujoco_rig_model():
    model = mujoco.MjModel.from_xml_path(os.path.join(MUJOCO_DIR, 'capleg_rig.xml'))
    names = {model.joint(i).name for i in range(model.njnt)}
    assert set(LEG_JOINTS['transfemoral'] + RIG_JOINTS) <= names
    assert {model.actuator(i).name for i in range(model.nu)} == {'rig_x', 'rig_z', 'hip', 'knee', 'ankle'}
    assert 3.5 < model.body_subtreemass[model.body('capleg_thigh').id] < 5.5      # leg + socket, kg


@pytest.mark.parametrize('config,tag', [('transfemoral', 'tf'), ('transtibial', 'tt')])
def test_controller_yaml_matches_description(config, tag):
    with open(os.path.join(GAZEBO, 'config', f'controllers_{tag}.yaml'), encoding='utf-8') as fh:
        text = fh.read()
    lists = re.findall(r'joints:\s*\[([^\]]*)\]', text)
    joints = {j.strip() for line in lists for j in line.split(',')}
    assert joints == set(LEG_JOINTS[config] + RIG_JOINTS)
