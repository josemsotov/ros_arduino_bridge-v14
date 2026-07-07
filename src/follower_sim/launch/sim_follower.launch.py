import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg      = get_package_share_directory('follower_sim')
    world    = os.path.join(pkg, 'worlds', 'follower_world.sdf')
    urdf     = os.path.join(pkg, 'urdf', 'trolley.urdf.xacro')
    cfg      = os.path.join(get_package_share_directory('robot_follower'),
                             'config', 'follower_params.yaml')

    return LaunchDescription([

        # 1. Gazebo Sim
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen'),

        # 2. Robot state publisher (URDF → TF)
        TimerAction(period=3.0, actions=[
            Node(package='robot_state_publisher',
                 executable='robot_state_publisher', output='screen',
                 parameters=[{'robot_description':
                     open(urdf).read() if os.path.exists(urdf) else ''}]),

            # 3. Spawn robot en Gazebo
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'trolley',
                             '-file', urdf,
                             '-x', '0', '-y', '0', '-z', '0.15'],
                 output='screen'),
        ]),

        # 4. ROS↔Gazebo bridge
        TimerAction(period=4.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='gz_bridge', output='screen',
                 arguments=[
                     '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                     '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                     '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                     '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 ]),
        ]),

        # 5. Follower node (después de que el bridge esté listo)
        TimerAction(period=6.0, actions=[
            Node(package='robot_follower', executable='follower_node',
                 name='robot_follower', output='screen',
                 parameters=[cfg]),
        ]),
    ])
