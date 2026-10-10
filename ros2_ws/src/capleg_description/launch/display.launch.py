"""View CapLeg in RViz.

    ros2 launch capleg_description display.launch.py                       # sliders for every joint
    ros2 launch capleg_description display.launch.py config:=transtibial
    ros2 launch capleg_description display.launch.py rig:=true gui:=false  # then, in a second terminal:
    ros2 run capleg_control gait_player --ros-args -p rig:=true
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory('capleg_description')
    config = LaunchConfiguration('config').perform(context)
    rig = LaunchConfiguration('rig').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    doc = xacro.process_file(os.path.join(share, 'urdf', 'capleg.urdf.xacro'),
                             mappings={'config': config, 'rig': rig, 'sim': 'none'})
    fixed = 'world' if rig == 'true' else ('capleg_thigh' if config == 'transfemoral' else 'capleg_shank')
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': doc.toxml()}], output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'capleg_rig.rviz' if rig == 'true' else 'capleg.rviz'),
                        '-f', fixed]),
    ]
    if gui == 'true':
        nodes.append(Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='transfemoral',
                              choices=['transfemoral', 'transtibial']),
        DeclareLaunchArgument('rig', default_value='false', choices=['true', 'false'],
                              description='include the treadmill hip-simulator joints'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='joint_state_publisher_gui sliders (false: use gait_player)'),
        OpaqueFunction(function=_setup),
    ])
