"""ROS 2 node: phase-based impedance control of CapLeg on the Gazebo treadmill hip simulator.

Subscribes
    /joint_states                      sensor_msgs/JointState   (joint_state_broadcaster)
    /capleg/load_cell                  geometry_msgs/WrenchStamped (Gazebo force-torque sensor, via ros_gz_bridge)
Publishes
    /capleg_effort_controller/commands std_msgs/Float64MultiArray  [rig_z, (knee), ankle_pitch, ankle_roll, toe]
    /rig_position_controller/commands  std_msgs/Float64MultiArray  [rig_x, hip]
    /capleg/gait_phase                 std_msgs/Float64
    /capleg/reference                  sensor_msgs/JointState   (knee / ankle references, for plotting)

All impedance gains are ROS parameters and can be changed live, e.g.
    ros2 param set /capleg_controller knee.k_stance 500.0
"""
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from .core import gait_reference as gr
from .core.controller import ImpedanceController, PhaseEstimator, RigController, passive_torques
from .core.params import ANKLE_DRIVE, IMPEDANCE, JOINTS, PEA, RIG


class CapLegController(Node):

    def __init__(self):
        super().__init__('capleg_controller')
        p = self.declare_parameter
        self.config = p('config', 'transfemoral').value
        self.rate = p('rate', 500.0).value
        self.stride_time = p('stride_time', RIG['stride_time']).value
        self.body_mass = p('body_mass', RIG['body_mass']).value
        self.phase_source = p('phase_source', 'thigh' if self.config == 'transfemoral' else 'time').value
        self.software_passive = p('software_passive', True).value     # Gazebo has no native springs / PEA
        self.start_delay = p('start_delay', 3.0).value
        for key in ('knee', 'ankle'):
            for g in ('k_stance', 'b_stance', 'k_swing', 'b_swing'):
                p(f'{key}.{g}', IMPEDANCE[key][g])
        p('ankle_ff_scale', IMPEDANCE['ankle_ff_scale'])
        p('knee_ff_scale', IMPEDANCE['knee_ff_scale'])
        p('inertia_ff', IMPEDANCE['inertia_ff'])

        self.tf = self.config == 'transfemoral'
        self.rig = RigController(stride_time=self.stride_time, body_mass=self.body_mass,
                                 level='hip' if self.tf else 'knee',
                                 supported_mass=RIG['carriage_mass'] + (4.5 if self.tf else 2.1))
        self.imp = ImpedanceController(body_mass=self.body_mass)
        self.pe = PhaseEstimator(stride_time=self.stride_time)
        self.effort_joints = ['rig_z'] + (['knee'] if self.tf else []) + ['ankle_pitch', 'ankle_roll', 'toe']

        self.q, self.qd = {}, {}
        self.grf_z = None
        self.t0 = None
        self.last_t = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, '/capleg/load_cell', self.on_load_cell, qos_profile_sensor_data)
        self.pub_eff = self.create_publisher(Float64MultiArray, '/capleg_effort_controller/commands', 10)
        self.pub_pos = self.create_publisher(Float64MultiArray, '/rig_position_controller/commands', 10)
        self.pub_phase = self.create_publisher(Float64, '/capleg/gait_phase', 10)
        self.pub_ref = self.create_publisher(JointState, '/capleg/reference', 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info(f'CapLeg controller: config={self.config}, phase={self.phase_source}, '
                               f'belt {self.rig.v:.2f} m/s, stride {self.stride_time:.2f} s')

    # ------------------------------------------------------------------ callbacks
    def on_joints(self, msg):
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                self.q[n] = msg.position[i]
            if i < len(msg.velocity):
                self.qd[n] = msg.velocity[i]

    def on_load_cell(self, msg):
        self.grf_z = abs(msg.wrench.force.z)

    def _gains(self):
        gp = self.get_parameter
        g = {k: {n: gp(f'{k}.{n}').value for n in ('k_stance', 'b_stance', 'k_swing', 'b_swing')}
             for k in ('knee', 'ankle')}
        g['ankle_ff_scale'] = gp('ankle_ff_scale').value
        g['knee_ff_scale'] = gp('knee_ff_scale').value
        g['inertia_ff'] = gp('inertia_ff').value
        return g

    # ------------------------------------------------------------------ control loop
    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        needed = ['rig_z', 'hip', 'ankle_pitch', 'ankle_roll', 'toe'] + (['knee'] if self.tf else [])
        if any(n not in self.q for n in needed):
            return
        if self.t0 is None:
            self.t_start = now
            self.t0 = now + self.start_delay
        dt = 1.0 / self.rate if self.last_t is None else max(1e-4, now - self.last_t)
        self.last_t = now
        t = max(0.0, now - self.t0)

        if now < self.t0:
            # start-up: lift the foot 3 cm, ramp the rig 'hip' joint to the heel-strike posture, then walk
            r = min(1.0, (now - self.t_start) / max(1e-3, self.start_delay))
            hip0 = self.rig.commands(0.0, self.q['rig_z'], 0.0)['hip']
            fz = RIG['swing_kp'] * (0.03 - self.q['rig_z']) - RIG['swing_kd'] * self.qd.get('rig_z', 0.0) + \
                self.rig.m_sup * 9.81
            rc = dict(x=0.0, hip=hip0 * r, fz=fz, phase=0.0)
            phi, rate = 0.0, 0.0
        else:
            rc = self.rig.commands(t, self.q['rig_z'], self.qd.get('rig_z', 0.0), grf_z=self.grf_z)
            if self.phase_source == 'thigh' and self.tf:
                phi = self.pe.update(self.q['hip'], self.qd.get('hip', 0.0), dt)
                rate = 1.0 / self.pe.T
            else:
                phi, rate = rc['phase'], 1.0 / self.stride_time

        self.imp.g.update(self._gains())
        q = {'knee': self.q.get('knee', 0.0), 'ankle_pitch': self.q['ankle_pitch'],
             'ankle_roll': self.q['ankle_roll'], 'toe': self.q['toe']}
        qd = {'knee': self.qd.get('knee', 0.0), 'ankle_pitch': self.qd.get('ankle_pitch', 0.0)}
        tau = self.imp.torques(phi, rate, q, qd)
        pas = passive_torques(q) if self.software_passive else {'ankle_roll': 0.0, 'toe': 0.0, 'pea': 0.0}
        dmp_roll = -0.5 * self.qd.get('ankle_roll', 0.0)
        dmp_toe = -0.05 * self.qd.get('toe', 0.0)

        pea_max = PEA['stiffness'] * JOINTS['ankle_pitch']['upper']        # spring torque at full dorsiflexion
        eff = {'rig_z': rc['fz'], 'knee': tau['knee'],
               'ankle_pitch': max(-ANKLE_DRIVE['peak_torque'] - pea_max,
                                  min(ANKLE_DRIVE['peak_torque'], tau['ankle_pitch'] + pas['pea'])),
               'ankle_roll': pas['ankle_roll'] + dmp_roll, 'toe': pas['toe'] + dmp_toe}
        self.pub_eff.publish(Float64MultiArray(data=[float(eff[j]) for j in self.effort_joints]))
        self.pub_pos.publish(Float64MultiArray(data=[float(rc['x']), float(rc['hip'])]))
        self.pub_phase.publish(Float64(data=float(phi)))
        ref = JointState()
        ref.header.stamp = self.get_clock().now().to_msg()
        ref.name = ['knee', 'ankle_pitch']
        _, kn, an, _ = gr.angles_rad(phi)
        ref.position = [kn, an]
        ref.effort = [float(tau['knee']), float(tau['ankle_pitch'])]
        self.pub_ref.publish(ref)


def main(args=None):
    rclpy.init(args=args)
    node = CapLegController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
