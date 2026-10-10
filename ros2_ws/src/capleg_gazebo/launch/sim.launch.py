"""CapLeg on the Gazebo Harmonic treadmill hip simulator with ros2_control + phase-based impedance control.

    ros2 launch capleg_gazebo sim.launch.py                         # transfemoral, Gazebo GUI
    ros2 launch capleg_gazebo sim.launch.py gui:=false              # headless server
    ros2 launch capleg_gazebo sim.launch.py config:=transtibial
    ros2 launch capleg_gazebo sim.launch.py stride_time:=1.0 body_mass:=85.0 rviz:=true

Start-up order: gz sim -> spawn robot -> joint_state_broadcaster -> rig position controller + capleg_controller
-> (1 s later, once capleg_controller is streaming) effort controller. Until the effort controller is active,
gz_ros2_control holds the unclaimed joints, so the leg never goes limp.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    desc = get_package_share_directory('capleg_description')
    here = get_package_share_directory('capleg_gazebo')
    config = LaunchConfiguration('config').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    rviz = LaunchConfiguration('rviz').perform(context) == 'true'
    world = LaunchConfiguration('world').perform(context) or os.path.join(here, 'worlds', 'capleg_treadmill.sdf')
    stride_time = float(LaunchConfiguration('stride_time').perform(context))
    body_mass = float(LaunchConfiguration('body_mass').perform(context))
    tf = config == 'transfemoral'
    controllers = os.path.join(here, 'config', 'controllers_tf.yaml' if tf else 'controllers_tt.yaml')

    robot_description = xacro.process_file(
        os.path.join(desc, 'urdf', 'capleg.urdf.xacro'),
        mappings={'config': config, 'rig': 'true', 'sim': 'gazebo', 'controllers_file': controllers}).toxml()

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                                                   'gz_sim.launch.py')),
        launch_arguments={'gz_args': ('-r -v 3 ' if gui else '-r -s -v 3 ') + world,
                          'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'capleg', '-z', '0.0'])
    topics = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
              '/capleg/load_cell@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
              '/capleg/shank_imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
              '/capleg/foot_imu@sensor_msgs/msg/Imu[gz.msgs.IMU']
    if tf:
        topics.append('/capleg/thigh_imu@sensor_msgs/msg/Imu[gz.msgs.IMU')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=topics, output='screen',
                  parameters=[{'use_sim_time': True}])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager'])
    jsb = spawner('joint_state_broadcaster')
    effort = spawner('capleg_effort_controller')
    rigpos = spawner('rig_position_controller')
    ctrl = Node(package='capleg_control', executable='capleg_controller', output='screen',
                parameters=[os.path.join(get_package_share_directory('capleg_control'), 'config', 'controller.yaml'),
                            {'use_sim_time': True, 'config': config, 'stride_time': stride_time,
                             'body_mass': body_mass, 'phase_source': 'thigh' if tf else 'time'}])
    actions = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc)),   # resolves package:// meshes
        gz, rsp, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[rigpos, ctrl])),
        RegisterEventHandler(OnProcessStart(target_action=ctrl,
                                            on_start=[TimerAction(period=1.0, actions=[effort])])),
    ]
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', os.path.join(desc, 'rviz', 'capleg_rig.rviz')],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='transfemoral', choices=['transfemoral', 'transtibial']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='', description='SDF world (default: treadmill world)'),
        DeclareLaunchArgument('stride_time', default_value='1.2'),
        DeclareLaunchArgument('body_mass', default_value='75.0'),
        OpaqueFunction(function=_setup),
    ])
