import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    follower_sim = get_package_share_directory('follower_sim')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follower_sim, 'launch', 'sim_follower.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'localization': 'true',
            'enable_follower': 'false',
            'headless': LaunchConfiguration('headless'),
            'kinect': LaunchConfiguration('kinect'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(follower_sim, 'launch', 'nav2_minimal.launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open the lightweight RViz profile.'),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run without the Gazebo graphical client.'),
        DeclareLaunchArgument(
            'kinect', default_value='false',
            description='Enable the simulated Kinect RGB-D topics.'),
        simulation,
        TimerAction(period=12.0, actions=[navigation]),
    ])
