from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sick_tim_usb_ros2',
            executable='usb_lidar_node',
            name='sick_tim_usb',
            output='screen',
            parameters=[{
                'frame_id': 'laser',
                'topic': '/scan',
                'angular_resolution': '1.0',  # '0.33'|'0.5'|'1.0'
                'range_min': 0.1,
                'range_max': 10.0,
            }],
        ),
    ])
