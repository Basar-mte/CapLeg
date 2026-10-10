"""Kinematic gait playback for RViz (no physics): publishes /joint_states along the reference gait.

    ros2 launch capleg_description display.launch.py rig:=true gui:=false
    ros2 run capleg_control gait_player --ros-args -p rig:=true -p stride_time:=1.2
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .core import gait_reference as gr
from .core import kinematics as kin
from .core.params import GEOM, RIG


class GaitPlayer(Node):

    def __init__(self):
        super().__init__('gait_player')
        self.T = self.declare_parameter('stride_time', RIG['stride_time']).value
        self.rig = self.declare_parameter('rig', False).value
        self.config = self.declare_parameter('config', 'transfemoral').value
        rate = self.declare_parameter('rate', 60.0).value
        self.v = kin.treadmill_speed(self.T)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        phi = (t / self.T) % 1.0
        th, kn, an, to = gr.angles_rad(phi)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names, pos = ['ankle_pitch', 'ankle_roll', 'toe'], [an, 0.0, max(0.0, to)]
        if self.config == 'transfemoral':
            names.insert(0, 'knee')
            pos.insert(0, kn)
        if self.rig:
            z = kin.hip_height_ref(phi) - GEOM['leg_len']
            hip = th
            if self.config != 'transfemoral':
                hip = th - kn
                z = kin.hip_height_ref(phi) - GEOM['thigh_len'] * math.cos(th) - (GEOM['leg_len'] - GEOM['thigh_len'])
            names += ['rig_x', 'rig_z', 'hip']
            pos += [(self.v * t) % 20.0, z, hip]
        msg.name, msg.position = names, [float(p) for p in pos]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitPlayer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
