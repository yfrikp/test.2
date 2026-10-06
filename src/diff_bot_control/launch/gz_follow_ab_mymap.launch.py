import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    desc_share = get_package_share_directory('diff_bot_description')
    my_world_sdf = os.path.join(desc_share, 'worlds', 'my_map_gz.sdf')
    my_world_dat = os.path.join(desc_share, 'worlds', 'my_map.world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    lookahead = LaunchConfiguration('lookahead')
    v_max = LaunchConfiguration('v_max')
    start_delay = LaunchConfiguration('start_delay')
    declare = [DeclareLaunchArgument('gui', default_value='true'), DeclareLaunchArgument('rviz', default_value='true'), DeclareLaunchArgument('lookahead', default_value='0.45'), DeclareLaunchArgument('v_max', default_value='0.22'), DeclareLaunchArgument('start_delay', default_value='30.0', description='等多少秒后开始跑（新版启动较慢）')]
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'gz_sim.launch.py')), launch_arguments={'gui': gui, 'rviz': rviz, 'world': my_world_sdf}.items())
    follower = TimerAction(period=start_delay, actions=[Node(package='diff_bot_control', executable='path_follower', name='path_follower', output='screen', parameters=[{'use_sim_time': True, 'odom_is_world': False, 'spawn_x': -2.0, 'spawn_y': -2.0, 'spawn_yaw': 0.785, 'goal_x': 2.0, 'goal_y': 2.0, 'world_file': my_world_dat, 'lookahead': lookahead, 'v_max': v_max}])])
    return LaunchDescription(declare + [sim, follower])
