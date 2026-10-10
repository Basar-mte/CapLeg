"""Sagittal-plane kinematics of CapLeg and hip-simulator (treadmill rig) references.

All positions are in the hip frame (x anterior, z up), metres; angles in radians with the
URDF conventions: thigh +flexion, knee +flexion, ankle +dorsiflexion, toe +dorsiflexion.
"""
import math

from . import gait_reference as gr
from .params import GEOM, RIG


def _r(a, x, z):
    """Rotate (x, z) by a 'flexion about -Y' angle a (positive moves points below the pivot forward)."""
    c, s = math.cos(a), math.sin(a)
    return (x * c - z * s, x * s + z * c)


def _add(p, q):
    return (p[0] + q[0], p[1] + q[1])


SOLE_X = [GEOM['heel_x'] + i * (GEOM['keel_front_x'] - GEOM['heel_x']) / 24.0 for i in range(25)]
TOE_X = [GEOM['toe_pad_x'][0] + i * (GEOM['toe_pad_x'][1] - GEOM['toe_pad_x'][0]) / 8.0 for i in range(9)]


def forward(thigh, knee, ankle, toe=0.0):
    """Joint positions and sole points of the leg (hip frame)."""
    a_th = thigh
    a_sh = thigh - knee
    a_ft = a_sh + ankle
    a_to = a_ft + toe
    p_knee = _r(a_th, 0.0, -GEOM['thigh_len'])
    p_ankle = _add(p_knee, _r(a_sh, 0.0, -GEOM['shank_len']))
    p_roll = _add(p_ankle, _r(a_ft, 0.0, -GEOM['roll_dz']))
    tx, _, tz = GEOM['toe_xyz']
    p_toe = _add(p_roll, _r(a_ft, tx, tz))
    sole = [_add(p_roll, _r(a_ft, x, GEOM['sole_z'])) for x in SOLE_X]
    toes = [_add(p_toe, _r(a_to, x, GEOM['toe_sole_z'])) for x in TOE_X]
    return dict(knee=p_knee, ankle=p_ankle, roll=p_roll, toe=p_toe, sole=sole, toe_sole=toes,
                angles=dict(thigh=a_th, shank=a_sh, foot=a_ft, toe=a_to))


def lowest_point(fk):
    pts = fk['sole'] + fk['toe_sole']
    i = min(range(len(pts)), key=lambda k: pts[k][1])
    return i, pts[i]


def _stance_track(n=400):
    """Hip height above floor and hip progression for rolling, non-slipping stance on the reference gait."""
    phis = [i / n * gr.TOE_OFF for i in range(n + 1)]
    heights, xs = [], []
    x_ground = None
    idx_prev = None
    for ph in phis:
        th, kn, an, to = gr.angles_rad(ph)
        fk = forward(th, kn, an, to)
        pts = fk['sole'] + fk['toe_sole']
        i, low = lowest_point(fk)
        heights.append(-low[1])
        if x_ground is None:
            x_ground = 0.0
        elif i != idx_prev:
            # contact transfers from point idx_prev to point i (both touching the floor at this instant)
            x_ground += pts[i][0] - pts[idx_prev][0]
        idx_prev = i
        xs.append(x_ground - pts[i][0])          # hip x in ground frame
    return phis, heights, xs


_PHIS, _H_STANCE, _X_STANCE = _stance_track()
STANCE_HIP_TRAVEL = _X_STANCE[-1] - _X_STANCE[0]


def treadmill_speed(stride_time=RIG['stride_time']):
    """Belt speed that matches the reference stance roll-over for the given stride time (m/s)."""
    return STANCE_HIP_TRAVEL / (gr.TOE_OFF * stride_time)


def _interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    f = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] * (1 - f) + ys[hi] * f


def hip_height_ref(phi, lift=RIG['swing_lift']):
    """Reference hip height above the floor (m).

    Stance: rolling, non-slipping contact of the prosthetic foot on the reference gait.
    Swing : the (virtual) contralateral leg is in stance, so the hip follows the same stance
            profile shifted by half a cycle, blended to stay continuous at toe-off and heel strike,
            plus a small extra lift (contralateral vaulting / rig margin).
    """
    phi = phi - math.floor(phi)
    if phi <= gr.TOE_OFF:
        return _interp(_PHIS, _H_STANCE, phi)
    s = (phi - gr.TOE_OFF) / (1.0 - gr.TOE_OFF)
    h_contra = _interp(_PHIS, _H_STANCE, phi - 0.5)
    e_to = _H_STANCE[-1] - _interp(_PHIS, _H_STANCE, gr.TOE_OFF - 0.5)
    e_hs = _H_STANCE[0] - _interp(_PHIS, _H_STANCE, 0.5)
    x = min(1.0, max(0.0, (s - 0.45) / 0.55))         # hip lowers only in terminal swing
    w = x * x * (3.0 - 2.0 * x)
    return h_contra + e_to * (1 - w) + e_hs * w + lift * math.sin(math.pi * s)


def swing_clearance(phi):
    """Foot clearance (m) of the reference swing at the reference hip height."""
    th, kn, an, to = gr.angles_rad(phi)
    fk = forward(th, kn, an, to)
    return hip_height_ref(phi) + lowest_point(fk)[1][1]
