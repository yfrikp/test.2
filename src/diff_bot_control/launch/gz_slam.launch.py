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
    slam_params = os.path.join(ctrl_share, 'config', 'mapper_params_online_async.yaml')
    slam_rviz = os.path.join(desc_share, 'rviz', 'slam.rviz')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    patrol = LaunchConfiguration('patrol')
    patrol_delay = LaunchConfiguration('patrol_delay')
    declare = [DeclareLaunchArgument('gui', default_value='true'), DeclareLaunchArgument('rviz', default_value='true'), DeclareLaunchArgument('patrol', default_value='false', description='是否自动启动巡逻节点绕场建图'), DeclareLaunchArgument('patrol_delay', default_value='35.0', description='等多少秒后开始自动巡逻（新版启动比 Classic 慢，留足时间）')]
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'gz_sim.launch.py')), launch_arguments={'gui': gui, 'rviz': 'false'}.items())
    slam = Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox', output='screen', parameters=[slam_params, {'use_sim_time': True}])
    rviz_node = TimerAction(period=25.0, condition=IfCondition(rviz), actions=[Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', slam_rviz], parameters=[{'use_sim_time': True}], output='log')])
    patrol_node = TimerAction(period=patrol_delay, condition=IfCondition(patrol), actions=[Node(package='diff_bot_control', executable='mapping_patrol', name='mapping_patrol', output='screen', parameters=[{'use_sim_time': True, 'odom_is_world': False, 'spawn_x': -2.0, 'spawn_y': -2.0, 'spawn_yaw': 0.785, 'track_file': '/tmp/slam_track.csv', 'track_ground_truth': True}])])
    return LaunchDescription(declare + [sim, slam, rviz_node, patrol_node])
