"""CapLeg physical and control parameters (SI units).

Geometry matches the Fusion 360 model (cad/fusion/CapLeg_Builder) at the zero pose.
Link frames (URDF / MJCF) are aligned with the world at the zero pose:
    capleg_thigh : hip centre            capleg_talus : ankle pitch axis
    capleg_shank : knee axis             capleg_foot  : subtalar (roll) axis
    capleg_toe   : MTP (toe) axis
Axes: +X anterior, +Y medial (right leg), +Z up.
"""
import math

# ------------------------------------------------------------------ geometry [m]
GEOM = dict(
    thigh_len=0.430,          # hip -> knee
    shank_len=0.420,          # knee -> ankle pitch axis
    roll_dz=0.044,            # ankle pitch axis -> subtalar axis
    toe_xyz=(0.120, 0.0, -0.0345),   # toe axis in the foot frame
    sole_z=-0.046,            # sole (floor contact) in the foot frame
    heel_x=-0.070,            # heel edge in the foot frame
    keel_front_x=0.116,       # front of the keel (foot frame)
    toe_pad_x=(0.001, 0.072),  # toe pad extent in the toe frame
    toe_sole_z=-0.0115,       # toe pad bottom in the toe frame
)
GEOM['leg_len'] = GEOM['thigh_len'] + GEOM['shank_len'] + 0.090     # hip -> floor at zero pose = 0.94 m

# ------------------------------------------------------------------ actuators
AK80_9 = dict(                # CubeMars AK80-9 V3.0 (KV100, 48 V) - datasheet values
    ratio=9.0, peak_torque=22.0, rated_torque=9.0,
    no_load_rpm=570.0, rated_rpm=390.0, rotor_inertia=5.79e-5, mass=0.485,
)
CAPSTAN = dict(               # effective (cable-centre) radii [m]
    drum_r=0.01225, knee_r=0.05875, ankle_r=0.06375, efficiency=0.97,
    cable='D2.5 mm UHMWPE (Dyneema) braid, >= 6 kN break (peak tension ~1.8 kN)',
)


def joint_drive(name):
    """Joint-level actuator limits for 'knee' or 'ankle' (capstan + AK80-9)."""
    r = CAPSTAN['knee_r' if name == 'knee' else 'ankle_r'] / CAPSTAN['drum_r']
    n = AK80_9['ratio'] * r
    eta = CAPSTAN['efficiency']
    return dict(
        capstan_ratio=r, total_ratio=n,
        peak_torque=AK80_9['peak_torque'] * r * eta,
        rated_torque=AK80_9['rated_torque'] * r * eta,
        max_speed=AK80_9['no_load_rpm'] * 2 * math.pi / 60.0 / r,
        rated_speed=AK80_9['rated_rpm'] * 2 * math.pi / 60.0 / r,
        armature=AK80_9['rotor_inertia'] * n * n,          # reflected rotor inertia [kg m^2]
    )


KNEE_DRIVE = joint_drive('knee')
ANKLE_DRIVE = joint_drive('ankle')

# ------------------------------------------------------------------ joints (URDF sign convention)
JOINTS = dict(
    knee=dict(lower=math.radians(-5), upper=math.radians(120), friction=0.8, damping=0.2),
    ankle_pitch=dict(lower=math.radians(-20), upper=math.radians(15), friction=0.8, damping=0.2),
    ankle_roll=dict(lower=math.radians(-10), upper=math.radians(10), friction=0.05, damping=0.5,
                    stiffness=120.0),      # TPU roll springs (tunable by durometer / size)
    toe=dict(lower=0.0, upper=math.radians(40), friction=0.02, damping=0.05,
             stiffness=45.0),              # MTP torsion springs
)
PEA = dict(stiffness=250.0, engage_angle=0.0)   # unilateral ankle parallel spring, dorsiflexion side

# ------------------------------------------------------------------ hip-simulator rig
RIG = dict(
    body_mass=75.0,           # user body mass for load scaling [kg]
    load_fraction=0.90,       # prosthetic-side vertical load relative to normative GRF
    stride_time=1.20,         # s  (100 steps/min)
    carriage_mass=2.0,        # vertical (hip) carriage [kg]
    swing_kp=30000.0,         # vertical position servo in swing [N/m]
    swing_kd=1500.0,          # [N s/m]
    swing_lift=0.012,         # extra hip lift in mid swing (contralateral vault) [m]
    x_kp=40000.0, x_kd=4000.0,      # belt-progression axis PD (force-driven rigs, e.g. MuJoCo)
    hip_kp=15000.0, hip_kd=250.0,   # thigh-angle axis PD (N m/rad, N m s/rad)
)

# ------------------------------------------------------------------ impedance controller defaults
# Per joint: stance / swing stiffness [N m/rad] and damping [N m s/rad]; feed-forward scale.
IMPEDANCE = dict(
    knee=dict(k_stance=650.0, b_stance=10.0, k_swing=140.0, b_swing=6.0),
    ankle=dict(k_stance=450.0, b_stance=8.0, k_swing=150.0, b_swing=6.0),
    ankle_ff_scale=0.55,      # fraction of normative ankle plantarflexor moment added as feed-forward
    knee_ff_scale=0.15,
    inertia_ff=True,          # add reflected-rotor-inertia x reference acceleration
    blend=0.04,               # stance/swing gain blending width (phase units)
)
