import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """题目2（1）：四轮麦轮全向底盘仿真（空场地，专用于遥操作演示）。"""
    desc_share = get_package_share_directory('diff_bot_description')
    xacro_file = os.path.join(desc_share, 'urdf', 'omni_bot.urdf.xacro')
    rviz_cfg = os.path.join(desc_share, 'rviz', 'diff_bot.rviz')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    sw_render = LaunchConfiguration('sw_render')
    teleop = LaunchConfiguration('teleop')

    declare = [
        DeclareLaunchArgument('gui', default_value='true', description='是否显示 Gazebo 界面'),
        DeclareLaunchArgument('rviz', default_value='false', description='是否启动 RViz2'),
        DeclareLaunchArgument('sw_render', default_value='1', description='WSL2 上强制软件渲染'),
        DeclareLaunchArgument('teleop', default_value='false', description='是否同时启动 WASDQE 键盘遥控节点'),
    ]

    set_sw_render = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', sw_render)

    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' sim:=gz']), value_type=str)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    gz_args = ['-r ', PythonExpression(["'--headless-rendering -s ' if '", gui, "' == 'false' else ''"]),
               '-v 2 ', 'empty.sdf']
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                   'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items())

    spawn = TimerAction(period=6.0, actions=[
        Node(package='ros_gz_sim', executable='create', name='spawn_omni_bot', output='screen',
             arguments=['-world', 'empty', '-topic', 'robot_description', '-name', 'omni_bot',
                        '-x', '0.0', '-y', '0.0', '-z', '0.1'])])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
                  output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                             '/omni_cmd_vel_world@geometry_msgs/msg/Twist]gz.msgs.Twist',
                             '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                             '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                             '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                             '/world/empty/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
                  parameters=[{'use_sim_time': True}],
                  remappings=[('/world/empty/pose/info', '/gz_ground_truth')])

    omni_bridge = TimerAction(period=8.0, actions=[
        Node(package='diff_bot_control', executable='omni_bridge', name='omni_bridge',
             output='screen', parameters=[{'use_sim_time': True}])])

    teleop_node = TimerAction(period=10.0, condition=IfCondition(teleop), actions=[
        Node(package='diff_bot_control', executable='wasdqe_teleop', name='wasdqe_teleop',
             output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': True, 'mode': 'omni'}])])

    rviz_node = TimerAction(period=10.0, condition=IfCondition(rviz), actions=[
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}], output='log')])

    return LaunchDescription(declare + [set_sw_render, rsp, gz_sim, bridge, spawn,
                                        omni_bridge, teleop_node, rviz_node])
