import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg      = get_package_share_directory('follower_sim')
    world    = os.path.join(pkg, 'worlds', 'follower_world.sdf')
    urdf     = os.path.join(pkg, 'urdf', 'trolley.urdf.xacro')
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf,
                 ' enable_kinect:=', LaunchConfiguration('kinect')]),
        value_type=str,
    )
    cfg      = os.path.join(get_package_share_directory('robot_follower'),
                             'config', 'follower_params.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'trolley.rviz')
    gui_cfg  = os.path.join(pkg, 'config', 'gazebo_minimal.gui.config')
    ekf_cfg  = os.path.join(pkg, 'config', 'ekf.yaml')
    kinect_bridge_cfg = os.path.join(pkg, 'config', 'kinect_sim_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_follower', default_value='false',
            description='Start the real perception/follower node.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2.'),
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Fuse wheel odometry and IMU with robot_localization.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run only the Gazebo server for automated tests.'),
        DeclareLaunchArgument(
            'kinect', default_value='false',
            description='Enable the lightweight simulated RGB-D Kinect.'),

        # 1. Gazebo Sim
        # Start the server first. Delaying the GUI avoids missing a robot that
        # is inserted dynamically from /robot_description.
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world], output='screen',
                       ),

        # 2. Robot state publisher (URDF → TF)
        TimerAction(period=3.0, actions=[
            Node(package='robot_state_publisher',
                 executable='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': robot_description,
                              'use_sim_time': True}]),

            # 3. Spawn robot en Gazebo
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'trolley',
                             '-topic', 'robot_description',
                             '-x', '0', '-y', '0', '-z', '0.45'],
                 output='screen'),
        ]),

        TimerAction(period=7.0, actions=[
            ExecuteProcess(cmd=['gz', 'sim', '-g', '--gui-config', gui_cfg],
                           output='screen',
                           additional_env={
                               'GALLIUM_DRIVER': 'd3d12',
                               'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'NVIDIA',
                           },
                           condition=UnlessCondition(LaunchConfiguration('headless'))),
        ]),

        # 4. ROS↔Gazebo bridge
        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='gz_bridge', output='screen',
                 arguments=[
                     '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                     '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
                     '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                     '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                     '/world/follower_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 ]),
        ]),

        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='kinect_gz_bridge', output='screen',
                 parameters=[{'config_file': kinect_bridge_cfg}],
                 condition=IfCondition(LaunchConfiguration('kinect'))),
        ]),

        TimerAction(period=5.0, actions=[
            Node(package='robot_localization', executable='ekf_node',
                 name='ekf_filter_node', output='screen',
                 parameters=[ekf_cfg, {'use_sim_time': True}],
                 remappings=[('odometry/filtered', '/odometry/filtered')],
                 condition=IfCondition(LaunchConfiguration('localization'))),
        ]),

        TimerAction(period=5.0, actions=[
            Node(package='rviz2', executable='rviz2', output='screen',
                 arguments=['-d', rviz_cfg],
                 parameters=[{'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('rviz'))),
        ]),

        # 5. Follower node (después de que el bridge esté listo)
        TimerAction(period=6.0, actions=[
            Node(package='robot_follower', executable='follower_node',
                 name='robot_follower', output='screen',
                 parameters=[cfg],
                 condition=IfCondition(LaunchConfiguration('enable_follower'))),
        ]),
    ])
