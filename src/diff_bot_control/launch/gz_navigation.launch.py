import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    desc_share = get_package_share_directory('diff_bot_description')
    ctrl_share = get_package_share_directory('diff_bot_control')
    nav2_share = get_package_share_directory('nav2_bringup')
    nav_params = os.path.join(ctrl_share, 'config', 'nav2_params_gz.yaml')
    nav_rviz = os.path.join(desc_share, 'rviz', 'nav2.rviz')
    default_map = os.path.expanduser('~/vision_ws/models/map_gz.yaml')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    map_yaml = LaunchConfiguration('map')
    declare = [DeclareLaunchArgument('gui', default_value='true'), DeclareLaunchArgument('rviz', default_value='true'), DeclareLaunchArgument('map', default_value=default_map, description='静态地图 yaml（默认用新版建的 map_gz）')]
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'gz_sim.launch.py')), launch_arguments={'gui': gui, 'rviz': 'false'}.items())
    nav2 = TimerAction(period=25.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')), launch_arguments={'slam': 'False', 'map': map_yaml, 'use_sim_time': 'true', 'params_file': nav_params, 'autostart': 'true', 'use_composition': 'True', 'use_respawn': 'False'}.items())])
    rviz_node = TimerAction(period=35.0, condition=IfCondition(rviz), actions=[Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', nav_rviz], parameters=[{'use_sim_time': True}], output='log')])
    return LaunchDescription(declare + [sim, nav2, rviz_node])
