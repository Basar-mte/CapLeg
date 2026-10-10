#!/usr/bin/env python3
"""CapLeg on the MuJoCo treadmill hip simulator with the phase-based impedance controller.

The rig prescribes the thigh angle (normative trajectory) and the belt progression, and presses the hip down
with a body-weight-scaled vertical GRF profile during stance (swing: hip height servo). The prosthesis
controller only sees what a real leg sees: thigh angle/velocity (for the gait phase) and its own joints.

Examples
    python3 run_rig.py                          # 12 strides, headless, writes log + metrics + plots
    python3 run_rig.py --view                   # interactive MuJoCo viewer (needs a display)
    python3 run_rig.py --stride-time 1.0 --body-mass 85 --phase time
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'capleg_control'))
from capleg_control.core import gait_reference as gr  # noqa: E402
from capleg_control.core import kinematics as kin  # noqa: E402
from capleg_control.core.controller import ImpedanceController, PhaseEstimator, RigController  # noqa: E402
from capleg_control.core.params import GEOM  # noqa: E402

FOOT_BODIES = ('capleg_foot', 'capleg_toe')


def ground_reaction(m, d, floor_id, foot_ids):
    """Total contact force of foot/toe on the floor (world frame, N)."""
    f = np.zeros(3)
    tmp = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if c.geom1 == floor_id and b2 in foot_ids or c.geom2 == floor_id and b1 in foot_ids:
            mujoco.mj_contactForce(m, d, i, tmp)
            fw = c.frame.reshape(3, 3).T @ tmp[:3]      # force of geom1 on geom2 (normal: geom1 -> geom2)
            f += fw if c.geom1 == floor_id else -fw      # force of the floor on the foot
    return f


def run(args):
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'capleg_rig.xml'))
    d = mujoco.MjData(m)
    dt = m.opt.timestep
    J = {n: m.joint(n) for n in ('rig_x', 'rig_z', 'hip', 'knee', 'ankle_pitch', 'ankle_roll', 'toe')}
    A = {n: m.actuator(n).id for n in ('rig_x', 'rig_z', 'hip', 'knee', 'ankle')}
    qa = {n: J[n].qposadr[0] for n in J}
    va = {n: J[n].dofadr[0] for n in J}
    floor_id = m.geom('floor').id
    foot_ids = {m.body(b).id for b in FOOT_BODIES}
    coll_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in foot_ids and m.geom_contype[g] == 1]

    rig = RigController(stride_time=args.stride_time, body_mass=args.body_mass,
                        supported_mass=2.0 + m.body_subtreemass[m.body('capleg_thigh').id])
    imp = ImpedanceController(body_mass=args.body_mass)
    pe = PhaseEstimator(stride_time=args.stride_time)

    # initial state: heel strike posture at the reference hip height
    th, kn, an, _ = gr.angles_rad(0.0)
    d.qpos[qa['hip']], d.qpos[qa['knee']], d.qpos[qa['ankle_pitch']] = th, kn, an
    d.qpos[qa['rig_z']] = kin.hip_height_ref(0.0) - GEOM['leg_len'] + 0.002
    mujoco.mj_forward(m, d)

    n_steps = int(args.strides * args.stride_time / dt)
    log_every = max(1, int(round(1.0 / (args.log_rate * dt))))
    rows = []
    viewer = None
    if args.view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(m, d)
    t_wall = time.time()
    for k in range(n_steps):
        t = d.time
        grf_now = ground_reaction(m, d, floor_id, foot_ids)      # treadmill force plate
        rc = rig.commands(t, d.qpos[qa['rig_z']], d.qvel[va['rig_z']], grf_z=grf_now[2],
                          x=d.qpos[qa['rig_x']], xd=d.qvel[va['rig_x']],
                          hip=d.qpos[qa['hip']], hip_vel=d.qvel[va['hip']])
        d.ctrl[A['rig_x']] = rc['fx']
        d.ctrl[A['hip']] = rc['tau_hip']
        d.ctrl[A['rig_z']] = rc['fz']
        q = {n: d.qpos[qa[n]] for n in ('knee', 'ankle_pitch', 'ankle_roll', 'toe')}
        qd = {n: d.qvel[va[n]] for n in ('knee', 'ankle_pitch')}
        if args.phase == 'thigh':
            phi = pe.update(d.qpos[qa['hip']], d.qvel[va['hip']], dt)
            rate = 1.0 / pe.T
        else:
            phi, rate = rc['phase'], 1.0 / args.stride_time
        tau = imp.torques(phi, rate, q, qd)
        d.ctrl[A['knee']] = tau['knee']
        d.ctrl[A['ankle']] = tau['ankle_pitch']
        mujoco.mj_step(m, d)
        if viewer is not None:
            viewer.sync()
            lag = d.time - (time.time() - t_wall)
            if lag > 0:
                time.sleep(lag)
            if not viewer.is_running():
                break
        if k % log_every == 0:
            grf = ground_reaction(m, d, floor_id, foot_ids)
            zmin = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in coll_geoms)
            lc = d.sensor('load_cell_force').data.copy()
            rows.append(dict(
                t=t, phase_true=rc['phase'], phase_est=phi, hip=d.qpos[qa['hip']], hip_ref=rc['hip'],
                knee=q['knee'], ankle=q['ankle_pitch'], roll=q['ankle_roll'], toe=q['toe'],
                knee_ref=gr.angles_rad(rc['phase'])[1], ankle_ref=gr.angles_rad(rc['phase'])[2],
                knee_vel=qd['knee'], ankle_vel=qd['ankle_pitch'],
                tau_knee=d.actuator_force[A['knee']], tau_ankle=d.actuator_force[A['ankle']],
                pea=d.ten_length[0], grf_x=grf[0], grf_z=grf[2], fz_rig=rc['fz'],
                hip_z=d.qpos[qa['rig_z']] + GEOM['leg_len'], hip_z_ref=rc['z_ref'] + GEOM['leg_len'],
                foot_zmin=zmin, belt_x=d.qpos[qa['rig_x']],
                load_cell_fz=lc[2], qpos=' '.join('%.5f' % v for v in d.qpos)))
    if viewer is not None:
        viewer.close()
    sim_time = d.time
    wall = time.time() - t_wall
    return m, rows, dict(sim_time=sim_time, wall_time=wall, rt_factor=sim_time / max(wall, 1e-9),
                         belt_speed=rig.v)


def metrics(rows, args, extra):
    T = args.stride_time
    R = [r for r in rows if r['t'] > 3 * T]
    deg = math.degrees

    def rms(xs):
        return math.sqrt(sum(x * x for x in xs) / max(1, len(xs)))
    st = [r for r in R if r['phase_true'] < gr.TOE_OFF]
    sw = [r for r in R if r['phase_true'] >= gr.TOE_OFF]
    ph_err = []
    for r in R:
        e = r['phase_est'] - r['phase_true']
        ph_err.append(e - round(e))
    W = args.body_mass * 9.81
    # power / work per stride
    pk = [r['tau_knee'] * r['knee_vel'] for r in R]
    pa = [r['tau_ankle'] * r['ankle_vel'] for r in R]
    n_str = (R[-1]['t'] - R[0]['t']) / T if R else 1
    dtl = R[1]['t'] - R[0]['t'] if len(R) > 1 else 0
    sw_clear = [r['foot_zmin'] for r in R if 0.70 <= r['phase_true'] <= 0.90]
    out = dict(
        strides_evaluated=round(n_str, 2),
        belt_speed_m_s=round(extra['belt_speed'], 3),
        realtime_factor=round(extra['rt_factor'], 1),
        knee_rms_err_deg_stance=round(deg(rms([r['knee'] - r['knee_ref'] for r in st])), 2),
        knee_rms_err_deg_swing=round(deg(rms([r['knee'] - r['knee_ref'] for r in sw])), 2),
        ankle_rms_err_deg_stance=round(deg(rms([r['ankle'] - r['ankle_ref'] for r in st])), 2),
        ankle_rms_err_deg_swing=round(deg(rms([r['ankle'] - r['ankle_ref'] for r in sw])), 2),
        knee_peak_flexion_swing_deg=round(deg(max(r['knee'] for r in sw)), 1),
        ankle_peak_plantarflexion_deg=round(deg(min(r['ankle'] for r in R)), 1),
        toe_peak_dorsiflexion_deg=round(deg(max(r['toe'] for r in R)), 1),
        roll_range_deg=[round(deg(min(r['roll'] for r in R)), 2), round(deg(max(r['roll'] for r in R)), 2)],
        phase_err_mean_abs=round(sum(abs(e) for e in ph_err) / len(ph_err), 4),
        phase_err_max_abs=round(max(abs(e) for e in ph_err), 4),
        knee_peak_torque_Nm=round(max(abs(r['tau_knee']) for r in R), 1),
        ankle_peak_torque_Nm=round(max(abs(r['tau_ankle']) for r in R), 1),
        knee_rms_torque_Nm=round(rms([r['tau_knee'] for r in R]), 1),
        ankle_rms_torque_Nm=round(rms([r['tau_ankle'] for r in R]), 1),
        knee_peak_speed_rad_s=round(max(abs(r['knee_vel']) for r in R), 2),
        ankle_peak_speed_rad_s=round(max(abs(r['ankle_vel']) for r in R), 2),
        ankle_peak_power_W=round(max(pa), 1),
        knee_net_work_J_per_stride=round(sum(pk) * dtl / n_str, 2),
        ankle_net_work_J_per_stride=round(sum(pa) * dtl / n_str, 2),
        grf_peak_BW=round(max(r['grf_z'] for r in R) / W, 3),
        load_cell_peak_N=round(max(abs(r['load_cell_fz']) for r in R), 1),
        min_foot_clearance_mid_swing_mm=round(1000 * min(sw_clear), 1) if sw_clear else None,
        hip_height_range_m=[round(min(r['hip_z'] for r in R), 3), round(max(r['hip_z'] for r in R), 3)],
    )
    return out


def plots(rows, args, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    T = args.stride_time
    R = [r for r in rows if r['t'] > 3 * T]
    red, blk, gry = '#D32C32', '#111111', '#9a9a9a'
    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    ph = np.array([r['phase_true'] for r in R]) * 100
    order = np.argsort(ph)

    def sc(a, key, scale, label, refkey=None):
        a.scatter(ph, np.array([r[key] for r in R]) * scale, s=1.5, c=red, label='simulated (%d strides)' % round(
            (R[-1]['t'] - R[0]['t']) / T))
        if refkey:
            a.plot(ph[order], np.array([R[i][refkey] for i in order]) * scale, c=blk, lw=1.3, label='reference')
        a.set_title(label)
        a.axvline(62, ls='--', c=gry, lw=0.8)
        a.grid(alpha=0.3)
        a.set_xlabel('gait cycle (%)')
    sc(ax[0, 0], 'knee', 180 / math.pi, 'knee flexion (deg)', 'knee_ref')
    sc(ax[0, 1], 'ankle', 180 / math.pi, 'ankle dorsiflexion (deg)', 'ankle_ref')
    sc(ax[0, 2], 'grf_z', 1 / (args.body_mass * 9.81), 'vertical GRF (BW)')
    sc(ax[1, 0], 'tau_knee', 1, 'knee torque (N m)')
    sc(ax[1, 1], 'tau_ankle', 1, 'ankle torque (N m)')
    a = ax[1, 2]
    a.plot([r['t'] for r in rows], [r['phase_est'] for r in rows], c=red, lw=1, label='estimated (thigh angle)')
    a.plot([r['t'] for r in rows], [r['phase_true'] for r in rows], c=blk, lw=0.8, ls='--', label='true')
    a.set_xlim(rows[-1]['t'] - 3 * T, rows[-1]['t'])
    a.set_title('gait phase')
    a.set_xlabel('time (s)')
    a.grid(alpha=0.3)
    a.legend(loc='lower left', fontsize=8)
    ax[0, 0].legend(fontsize=8)
    fig.suptitle('CapLeg - MuJoCo treadmill hip simulator, phase-based impedance control '
                 '(%.2f m/s, stride %.2f s, %d kg)' % (RigController(args.stride_time).v, T, args.body_mass))
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--strides', type=int, default=12)
    ap.add_argument('--stride-time', type=float, default=1.2)
    ap.add_argument('--body-mass', type=float, default=75.0)
    ap.add_argument('--phase', choices=['thigh', 'time'], default='thigh',
                    help='gait phase source: thigh-angle estimator (default) or rig clock')
    ap.add_argument('--view', action='store_true', help='open the interactive MuJoCo viewer')
    ap.add_argument('--log-rate', type=float, default=200.0)
    ap.add_argument('--out', default=os.path.join(HERE, 'results'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    m, rows, extra = run(args)
    with open(os.path.join(args.out, 'capleg_rig_log.csv'), 'w', encoding='utf-8', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()), lineterminator='\n')
        w.writeheader()
        w.writerows(rows)
    met = metrics(rows, args, extra)
    with open(os.path.join(args.out, 'metrics.json'), 'w', encoding='utf-8', newline='\n') as fh:
        json.dump(met, fh, indent=1)
        fh.write('\n')
    plots(rows, args, os.path.join(args.out, 'capleg_rig_results.png'))
    for k, v in met.items():
        print(f'{k:36s} {v}')


if __name__ == '__main__':
    main()
