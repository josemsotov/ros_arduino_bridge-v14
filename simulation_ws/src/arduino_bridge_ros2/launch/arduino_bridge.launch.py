from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arduino_bridge_ros2',
            executable='arduino_node',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'port':       '/dev/ttyACM0',
                'baud':       115200,
                'wheel_base': 0.82,
                'wheel_dia':  0.27,
                'ppr':        60,
            }],
        ),
    ])
