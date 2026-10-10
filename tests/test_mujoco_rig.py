"""Closed-loop smoke test: a short MuJoCo treadmill run must walk with sane metrics."""
import argparse

import pytest
import run_rig


@pytest.fixture(scope='module')
def short_run():
    args = argparse.Namespace(strides=5, stride_time=1.2, body_mass=75.0, phase='thigh', view=False,
                              log_rate=200.0)
    m, rows, extra = run_rig.run(args)
    return run_rig.metrics(rows, args, extra)


def test_tracking(short_run):
    assert short_run['knee_rms_err_deg_stance'] < 4.0
    assert short_run['knee_rms_err_deg_swing'] < 8.0
    assert short_run['ankle_rms_err_deg_stance'] < 5.0
    assert 50.0 < short_run['knee_peak_flexion_swing_deg'] < 75.0


def test_phase_estimator(short_run):
    assert short_run['phase_err_mean_abs'] < 0.01
    assert short_run['phase_err_max_abs'] < 0.05


def test_loads_and_clearance(short_run):
    assert 0.9 < short_run['grf_peak_BW'] < 1.6
    assert short_run['min_foot_clearance_mid_swing_mm'] > 5.0
    assert short_run['ankle_net_work_J_per_stride'] > 0.0          # net push-off
    assert short_run['knee_net_work_J_per_stride'] < 0.0           # knee absorbs, as in able-bodied gait
