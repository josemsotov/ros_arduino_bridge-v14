import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    follower_sim = get_package_share_directory('follower_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    params = os.path.join(nav2_bringup, 'params', 'nav2_params.yaml')
    overrides = os.path.join(
        follower_sim, 'config', 'nav2_trolley_overrides.yaml')
    map_yaml = os.path.join(follower_sim, 'maps', 'follower_world.yaml')
    common = [params, overrides, {'use_sim_time': True}]
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator'
    ]

    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='both',
             parameters=[params, overrides, {'use_sim_time': True,
                                  'yaml_filename': map_yaml}],
             remappings=tf_remaps),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='both',
             parameters=[params, overrides, {
                 'use_sim_time': True,
                 'set_initial_pose': True,
                 'initial_pose.x': 0.0,
                 'initial_pose.y': 0.0,
                 'initial_pose.z': 0.0,
                 'initial_pose.yaw': 0.0,
             }],
             remappings=tf_remaps),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': localization_nodes}]),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='both',
             parameters=common, remappings=tf_remaps),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='both',
             parameters=common, remappings=tf_remaps),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='both',
             parameters=common, remappings=tf_remaps),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='both',
             parameters=common, remappings=tf_remaps),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='both',
             parameters=common, remappings=tf_remaps),
        TimerAction(period=10.0, actions=[
            Node(package='nav2_lifecycle_manager',
                 executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': navigation_nodes}]),
        ]),
    ])
