import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """题目2（1）一键启动：麦轮全向底盘仿真 + WASDQE 键盘遥控。"""
    desc_share = get_package_share_directory('diff_bot_description')
    gui = LaunchConfiguration('gui')
    declare = [DeclareLaunchArgument('gui', default_value='true', description='是否显示 Gazebo 界面')]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'omni_sim.launch.py')),
        launch_arguments={'gui': gui, 'teleop': 'false'}.items())
    return LaunchDescription(declare + [sim])
