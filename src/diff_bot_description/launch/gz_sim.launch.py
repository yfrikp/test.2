import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    desc_share = get_package_share_directory('diff_bot_description')
    xacro_file = os.path.join(desc_share, 'urdf', 'diff_bot.urdf.xacro')
    default_world = os.path.join(desc_share, 'worlds', 'my_map_gz.sdf')
    rviz_cfg = os.path.join(desc_share, 'rviz', 'diff_bot.rviz')
    spawn_x, spawn_y, spawn_z, spawn_yaw = ('-2.0', '-2.0', '0.1', '0.785')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    sw_render = LaunchConfiguration('sw_render')
    declare = [DeclareLaunchArgument('gui', default_value='true', description='是否显示 Gazebo Sim 界面'), DeclareLaunchArgument('rviz', default_value='true', description='是否启动 RViz2'), DeclareLaunchArgument('world', default_value=default_world, description='world 文件（.sdf）路径'), DeclareLaunchArgument('sw_render', default_value='1', description='强制软件渲染（WSL2 上必须为 1，见下方说明）')]
    set_sw_render = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', sw_render)
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' sim:=gz']), value_type=str)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', output='screen', parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    gz_args = ['-r ', PythonExpression(["'--headless-rendering -s ' if '", gui, "' == 'false' else ''"]), '-v 3 ', world]
    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')), launch_arguments={'gz_args': gz_args}.items())
    spawn = TimerAction(period=6.0, actions=[Node(package='ros_gz_sim', executable='create', name='spawn_diff_bot', output='screen', arguments=['-world', 'obstacles', '-topic', 'robot_description', '-name', 'diff_bot', '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw])])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge', output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry', '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V', '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'], parameters=[{'use_sim_time': True}], remappings=[('/scan', '/scan_raw')])
    scan_filter = Node(package='diff_bot_control', executable='scan_filter', name='scan_filter', output='screen', parameters=[{'use_sim_time': True, 'in_topic': '/scan_raw', 'out_topic': '/scan', 'min_valid_range': 0.6, 'frame_id': 'laser_link'}])
    gt_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_ground_truth', output='screen', arguments=['/world/obstacles/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'], remappings=[('/world/obstacles/dynamic_pose/info', '/gz_ground_truth')], parameters=[{'use_sim_time': True}])
    rviz_node = TimerAction(period=12.0, condition=IfCondition(rviz), actions=[Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}], output='log')])
    return LaunchDescription(declare + [set_sw_render, rsp, gz_sim, bridge, gt_bridge, scan_filter, spawn, rviz_node])
