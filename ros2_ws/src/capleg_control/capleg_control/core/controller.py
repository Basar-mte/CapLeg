"""Simulator-agnostic CapLeg control: phase estimation, phase-based impedance control, actuator model,
passive elements and the hip-simulator (treadmill rig) commands.

Used unchanged by the MuJoCo runner (mujoco/run_rig.py) and the ROS 2 node (capleg_control/controller_node.py).
Joint conventions (rad): knee +flexion, ankle +dorsiflexion, roll +inversion, toe +dorsiflexion, hip +flexion.
"""
import math

from . import gait_reference as gr
from . import kinematics as kin
from .params import ANKLE_DRIVE, GEOM, IMPEDANCE, JOINTS, KNEE_DRIVE, PEA, RIG

G = 9.81


def _smoothstep(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def stance_weight(phi, blend=0.04):
    """1 in stance, 0 in swing; smooth transitions centred on toe-off and heel strike."""
    phi = phi - math.floor(phi)
    to = gr.TOE_OFF
    if phi <= 0.5:                                        # loading response (heel strike at 0)
        return _smoothstep(0.5 + phi / (2 * blend))
    if phi < to + blend + 0.1:                            # around toe-off
        return 1.0 - _smoothstep((phi - (to - blend)) / (2 * blend))
    return _smoothstep((phi - (1.0 - blend)) / (2 * blend))   # terminal swing -> heel strike


# ============================================================================ phase estimation
class PhaseEstimator:
    """Gait phase from the thigh-angle phase portrait.

    At every sample the measured (thigh angle, thigh velocity) pair is matched to the nearest point of the
    reference thigh phase portrait inside a forward-looking window around the predicted phase
    (phi_prev + dt / T), with a weak prior on that prediction. This keeps the estimate continuous and
    monotonic, adapts to speed (T is refreshed at each heel strike) and needs only a thigh IMU / hip encoder
    (thigh-angle-based phase variables: cf. Quintero et al., IEEE T-RO 2018).
    """

    def __init__(self, stride_time=RIG['stride_time'], n=1000, sigma_q=2.5, sigma_v=80.0, sigma_phi=0.025,
                 window=(-0.03, 0.06), samples=48, alpha=0.2):
        self.T = stride_time
        self.alpha = alpha
        self.n = n
        self.q = [gr.fourier('thigh', i / n) for i in range(n)]            # deg
        self.dq = [gr.fourier('thigh', i / n, 1) for i in range(n)]        # deg per unit phase
        self.sq, self.sv, self.sp = sigma_q, sigma_v, sigma_phi
        self.win, self.ns = window, samples
        self.phase = 0.0
        self.t = 0.0
        self.last_wrap_t = None
        self.strides = 0

    def _cost(self, ph, q, qd, pred):
        i = int((ph - math.floor(ph)) * self.n) % self.n
        eq = (q - self.q[i]) / self.sq
        ev = (qd - self.dq[i] / self.T) / self.sv
        ep = (ph - pred) / self.sp
        return eq * eq + ev * ev + ep * ep

    def update(self, thigh_rad, thigh_vel, dt):
        """Feed thigh angle / velocity (rad, rad/s); returns the gait phase in [0, 1)."""
        self.t += dt
        q, qd = math.degrees(thigh_rad), math.degrees(thigh_vel)
        pred = self.phase + dt / self.T
        lo, hi = pred + self.win[0], pred + self.win[1]
        best, bc = pred, float('inf')
        for k in range(self.ns + 1):
            ph = lo + (hi - lo) * k / self.ns
            c = self._cost(ph, q, qd, pred)
            if c < bc:
                best, bc = ph, c
        h = (hi - lo) / self.ns                             # parabolic refinement
        c0, c1 = self._cost(best - h, q, qd, pred), self._cost(best + h, q, qd, pred)
        den = c0 - 2 * bc + c1
        if den > 1e-9:
            best += 0.5 * h * (c0 - c1) / den
        best = max(best, self.phase - 0.02)                 # essentially monotonic
        prev = self.phase
        self.phase = best - math.floor(best)
        if prev > 0.8 and self.phase < 0.2:                 # heel strike -> refresh stride time
            if self.last_wrap_t is not None:
                T_new = self.t - self.last_wrap_t
                if 0.6 < T_new < 2.5:
                    self.T = (1 - self.alpha) * self.T + self.alpha * T_new
            self.last_wrap_t = self.t
            self.strides += 1
        return self.phase


# ============================================================================ actuator model
def saturate(name, tau, omega):
    """Peak-torque and torque-speed (motoring quadrant) limits of the AK80-9 + capstan drive."""
    drv = KNEE_DRIVE if name == 'knee' else ANKLE_DRIVE
    tmax = drv['peak_torque']
    if tau * omega > 0.0:                               # motoring: linear torque-speed line
        tmax *= max(0.0, 1.0 - abs(omega) / drv['max_speed'])
    return max(-tmax, min(tmax, tau))


def passive_torques(q):
    """Software model of the passive elements (for simulators without native springs)."""
    tau = {}
    r = JOINTS['ankle_roll']
    tau['ankle_roll'] = -r['stiffness'] * q.get('ankle_roll', 0.0)
    t = JOINTS['toe']
    tau['toe'] = -t['stiffness'] * q.get('toe', 0.0)
    a = q.get('ankle_pitch', 0.0)
    tau['pea'] = -PEA['stiffness'] * (a - PEA['engage_angle']) if a > PEA['engage_angle'] else 0.0
    return tau


# ============================================================================ impedance controller
class ImpedanceController:
    """Phase-based variable-impedance control of knee and ankle.

    tau = K(phi) * (q_ref(phi) - q) + B(phi) * (qd_ref(phi) - qd) + tau_ff(phi)
    with stance/swing gain scheduling and a biomimetic feed-forward (scaled normative moments).
    """

    def __init__(self, body_mass=RIG['body_mass'], gains=None):
        self.m = body_mass
        self.g = dict(IMPEDANCE)
        if gains:
            self.g.update(gains)

    def reference(self, phi, phase_rate):
        """{joint: (q_ref, qd_ref, qdd_ref)} in rad, rad/s, rad/s^2."""
        th, kn, an, _ = gr.angles_rad(phi)
        d1 = math.radians(1.0) * phase_rate
        d2 = math.radians(1.0) * phase_rate * phase_rate
        return dict(knee=(kn, gr.fourier('knee', phi, 1) * d1, gr.fourier('knee', phi, 2) * d2),
                    ankle_pitch=(an, gr.fourier('ankle', phi, 1) * d1, gr.fourier('ankle', phi, 2) * d2),
                    thigh=(th, gr.fourier('thigh', phi, 1) * d1, gr.fourier('thigh', phi, 2) * d2))

    def torques(self, phi, phase_rate, q, qd):
        ws = stance_weight(phi, self.g['blend'])
        ref = self.reference(phi, phase_rate)
        out = {}
        for jn, key, drv in (('knee', 'knee', KNEE_DRIVE), ('ankle_pitch', 'ankle', ANKLE_DRIVE)):
            gk = self.g[key]
            K = ws * gk['k_stance'] + (1 - ws) * gk['k_swing']
            B = ws * gk['b_stance'] + (1 - ws) * gk['b_swing']
            qr, qdr, qddr = ref[jn]
            out[jn] = K * (qr - q[jn]) + B * (qdr - qd[jn])
            if self.g.get('inertia_ff', False):
                out[jn] += drv['armature'] * qddr * (1 - ws)        # swing: cancel reflected rotor inertia
        # feed-forward: + plantarflexor / extensor moments are negative in joint coordinates
        out['ankle_pitch'] += -self.g['ankle_ff_scale'] * gr.table('ankle_moment', phi) * self.m
        out['knee'] += -self.g['knee_ff_scale'] * gr.table('knee_moment', phi) * self.m * ws
        out['knee'] = saturate('knee', out['knee'], qd['knee'])
        out['ankle_pitch'] = saturate('ankle', out['ankle_pitch'], qd['ankle_pitch'])
        return out


# ============================================================================ hip-simulator rig
class RigController:
    """Treadmill hip simulator: prescribed thigh angle and belt speed, load-controlled vertical axis.

    Stance: the vertical actuator presses the hip down with the normative vertical GRF profile
            (scaled by body mass and load fraction) -> the prosthesis must carry the body.
    Swing : the vertical axis is position-servoed to the contralateral-supported hip height.
    """

    def __init__(self, stride_time=RIG['stride_time'], body_mass=RIG['body_mass'],
                 load_fraction=RIG['load_fraction'], supported_mass=None, touchdown_depth=0.006,
                 contact_threshold=30.0, load_ramp=0.03, level='hip'):
        """level='hip' (transfemoral: rig drives the thigh) or 'knee' (transtibial: rig drives the shank)."""
        self.level = level
        self.T = stride_time
        self.v = kin.treadmill_speed(stride_time)
        self.W = load_fraction * body_mass * G
        self.m_sup = supported_mass if supported_mass is not None else RIG['carriage_mass'] + 5.0
        self.td = touchdown_depth
        self.f_contact = contact_threshold
        self.ramp = load_ramp
        self.t_contact = None

    def phase(self, t):
        return (t / self.T) % 1.0

    def commands(self, t, z, zd, grf_z=None, x=None, xd=None, hip=None, hip_vel=None):
        """Rig set-points (and actuator efforts for force-driven rigs). z = rig_z position (0 = zero-pose hip height).

        grf_z (optional, N): measured vertical load (treadmill force plate / load cell). When given, the stance
        body-weight profile is only applied after foot contact (ramped over load_ramp s), and the hip is lowered
        gently (touchdown_depth) around heel strike so contact happens without an impact spike.
        x, xd, hip, hip_vel (optional): measured rig_x / hip states -> also returns 'fx' and 'tau_hip'
        (PD with velocity feed-forward, no lag) for simulators where those axes are force/torque driven.
        """
        out = self._commands(t, z, zd, grf_z)
        if x is not None:
            out['fx'] = RIG['x_kp'] * (out['x'] - x) + RIG['x_kd'] * (self.v - xd)
        if hip is not None:
            out['tau_hip'] = RIG['hip_kp'] * (out['hip'] - hip) + RIG['hip_kd'] * (out['hip_vel'] - hip_vel)
        return out

    def _commands(self, t, z, zd, grf_z):
        phi = self.phase(t)
        dphi = 1e-3

        def height(p):                      # rig_z set-point (0 = zero-pose height of the driven joint)
            h = kin.hip_height_ref(p)
            if self.level == 'knee':
                return h - GEOM['thigh_len'] * math.cos(math.radians(gr.fourier('thigh', p))) - \
                    (GEOM['leg_len'] - GEOM['thigh_len'])
            return h - GEOM['leg_len']
        if self.level == 'knee':            # transtibial: drive the shank angle (thigh - knee)
            th = math.radians(gr.fourier('thigh', phi) - gr.fourier('knee', phi))
            thd = math.radians(gr.fourier('thigh', phi, 1) - gr.fourier('knee', phi, 1)) / self.T
        else:
            th = math.radians(gr.fourier('thigh', phi))
            thd = math.radians(gr.fourier('thigh', phi, 1)) / self.T
        z_ref = height(phi)
        zd_ref = (height(phi + dphi) - height(phi - dphi)) / (2 * dphi) / self.T
        # touchdown: aim slightly below the reference over terminal swing / loading response
        td_w = _smoothstep((phi - 0.93) / 0.07) if phi > 0.5 else 1.0 - _smoothstep((phi - 0.05) / 0.05)
        z_srv = z_ref - self.td * td_w
        f_servo = RIG['swing_kp'] * (z_srv - z) + RIG['swing_kd'] * (zd_ref - zd) + self.m_sup * G
        gate = 1.0
        if grf_z is not None:
            if 0.70 < phi < 0.90:
                self.t_contact = None
            elif self.t_contact is None and grf_z > self.f_contact and (phi > 0.9 or phi < gr.TOE_OFF):
                self.t_contact = t
            gate = 0.0 if self.t_contact is None else min(1.0, (t - self.t_contact) / self.ramp)
        f_load = RIG['carriage_mass'] * G - self.W * gr.table('grf', phi)
        ws = stance_weight(phi, 0.03) * gate
        return dict(x=self.v * t, hip=th, hip_vel=thd, fz=ws * f_load + (1 - ws) * f_servo, phase=phi,
                    z_ref=z_ref, gate=gate)
