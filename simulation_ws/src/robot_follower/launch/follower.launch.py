import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('robot_follower')
    default_cfg = os.path.join(pkg, 'config', 'follower_params.yaml')
    cfg = LaunchConfiguration('params_file')
    enable_arduino = LaunchConfiguration('enable_arduino')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_kinect = LaunchConfiguration('enable_kinect')
    enable_stadia  = LaunchConfiguration('enable_stadia')
    enable_gesture = LaunchConfiguration('enable_gesture')
    enable_follower = LaunchConfiguration('enable_follower')
    enable_field_supervisor = LaunchConfiguration('enable_field_supervisor')
    use_kinect = LaunchConfiguration('use_kinect')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg,
                              description='Path to follower params YAML'),
        DeclareLaunchArgument('enable_arduino', default_value='true',
                              description='Start Arduino serial bridge'),
        DeclareLaunchArgument('enable_lidar', default_value='true',
                              description='Start LD19 LiDAR driver'),
        DeclareLaunchArgument('enable_kinect', default_value='true',
                              description='Start Kinect RGB/depth camera node'),
        DeclareLaunchArgument('enable_stadia',
            default_value='true',
            description='Launch Stadia ROS2 node'),
        DeclareLaunchArgument('enable_gesture', default_value='true',
                              description='Start open-palm gesture detector'),
        DeclareLaunchArgument('enable_follower', default_value='true',
                              description='Start follower control node'),
        DeclareLaunchArgument('enable_field_supervisor', default_value='true',
                              description='Start monitor-only field supervisor'),
        DeclareLaunchArgument('use_kinect', default_value='true',
                              description='Use Kinect depth inside follower node'),
        Node(package='arduino_bridge_ros2', executable='arduino_node',
             name='arduino_bridge', output='screen',
             condition=IfCondition(enable_arduino),
             parameters=[{'port':'/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_85438333036351A040D0-if00',
                          'baud':115200,
                          'wheel_base':0.82,'wheel_dia':0.20,'ppr':45,
                          'cmd_linear_sign':1.0,
                          'cmd_angular_sign':1.0}]),
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ldlidar', output='screen',
             condition=IfCondition(enable_lidar),
             parameters=[{'product_name':'LDLiDAR_LD19','topic_name':'scan',
                          'port_name':'/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                          'port_baudrate':230400,
                          'frame_id':'base_laser','laser_scan_dir':True,
                          'enable_angle_crop_func':False}]),
        Node(package='kinect_ros2', executable='kinect_node',
             name='kinect_node', output='screen',
             condition=IfCondition(enable_kinect)),
        Node(package='robot_follower', executable='open_palm_node',
             name='open_palm_detector', output='screen',
             condition=IfCondition(enable_gesture),
             parameters=[{'image_topic':'/camera/rgb/image_raw',
                          'process_rate':8.0,
                          'gesture_hold_time':0.7,
                          'cooldown':4.0,
                          'require_raised_hand':True,
                          'max_wrist_y':0.78,
                          'publish_image':True}]),
        Node(package='arduino_bridge_ros2', executable='stadia_node',
             name='stadia_node', output='screen',
             condition=IfCondition(enable_stadia),
             parameters=[cfg]),
        Node(package='robot_follower', executable='follower_node',
             name='robot_follower', output='screen',
             condition=IfCondition(enable_follower),
             parameters=[cfg, {'use_kinect': ParameterValue(use_kinect, value_type=bool)}]),
        Node(package='robot_follower', executable='field_supervisor',
             name='field_supervisor', output='screen',
             condition=IfCondition(enable_field_supervisor),
             parameters=[cfg]),
    ])
