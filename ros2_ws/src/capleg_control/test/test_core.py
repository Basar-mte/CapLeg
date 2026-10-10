"""Unit tests for the simulator-agnostic CapLeg core (no ROS required):  python3 -m pytest test/"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from capleg_control.core import gait_reference as gr  # noqa: E402
from capleg_control.core import kinematics as kin  # noqa: E402
from capleg_control.core.controller import (  # noqa: E402
    ImpedanceController,
    PhaseEstimator,
    RigController,
    passive_torques,
    saturate,
    stance_weight,
)
from capleg_control.core.params import ANKLE_DRIVE, KNEE_DRIVE  # noqa: E402


def test_reference_is_periodic_and_in_range():
    for name in ('thigh', 'knee', 'ankle'):
        assert abs(gr.fourier(name, 0.0) - gr.fourier(name, 1.0)) < 1e-9
    knee = [gr.fourier('knee', i / 200) for i in range(200)]
    assert 55 < max(knee) < 70 and -1 < min(knee) < 6
    assert 0.0 <= gr.table('grf', 0.8) < 1e-6          # no load in swing


def test_phase_estimator_tracks_and_adapts():
    for T_true in (1.0, 1.2, 1.4):
        pe = PhaseEstimator(stride_time=1.2)
        dt, errs = 0.002, []
        for k in range(int(8 * T_true / dt)):
            t = k * dt
            ph = (t / T_true) % 1.0
            thigh = math.radians(gr.fourier('thigh', ph))
            thigh_vel = math.radians(gr.fourier('thigh', ph, 1)) / T_true
            est = pe.update(thigh, thigh_vel, dt)
            if t > 4 * T_true:
                e = est - ph
                errs.append(abs(e - round(e)))
        assert max(errs) < 0.02, (T_true, max(errs))
        assert abs(pe.T - T_true) < 0.06


def test_uniform_treadmill_progression():
    speeds = []
    for i in range(1, len(kin._PHIS) - 1, 20):
        dphi = kin._PHIS[i + 1] - kin._PHIS[i - 1]
        speeds.append((kin._X_STANCE[i + 1] - kin._X_STANCE[i - 1]) / dphi / 1.2)
    v = kin.treadmill_speed(1.2)
    assert all(abs(s - v) / v < 0.25 for s in speeds), speeds


def test_swing_clearance_positive():
    assert min(kin.swing_clearance(0.70 + 0.2 * i / 100) for i in range(101)) > 0.010


def test_actuator_limits_and_passives():
    assert abs(saturate('knee', 1e4, 0.0) - KNEE_DRIVE['peak_torque']) < 1e-9
    assert abs(saturate('ankle', -1e4, 0.0) + ANKLE_DRIVE['peak_torque']) < 1e-9
    assert saturate('knee', 50.0, KNEE_DRIVE['max_speed']) == 0.0            # no motoring at no-load speed
    assert passive_torques({'ankle_pitch': -0.1})['pea'] == 0.0              # PEA only in dorsiflexion
    assert passive_torques({'ankle_pitch': 0.1})['pea'] < 0.0


def test_impedance_and_rig_outputs_are_finite():
    imp, rig = ImpedanceController(), RigController()
    q = {'knee': 0.1, 'ankle_pitch': 0.0, 'ankle_roll': 0.0, 'toe': 0.0}
    qd = {'knee': 0.0, 'ankle_pitch': 0.0}
    for i in range(100):
        phi = i / 100
        tau = imp.torques(phi, 1 / 1.2, q, qd)
        rc = rig.commands(phi * 1.2, 0.0, 0.0, grf_z=500.0, x=0.0, xd=0.0, hip=0.0, hip_vel=0.0)
        assert all(math.isfinite(v) for v in tau.values())
        assert all(math.isfinite(rc[k]) for k in ('x', 'hip', 'fz', 'fx', 'tau_hip'))
        assert 0.0 <= stance_weight(phi) <= 1.0
