from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD06',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD06'},
                {'topic_name': 'scan'},
                {'frame_id': 'base_laser'},
                {'port_name': '/dev/ttyUSB0'},
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': True},
                {'angle_crop_min': -0.0436},
                {'angle_crop_max': 0.0436}
            ]
        )
    ])
