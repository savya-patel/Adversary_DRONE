"""
Launch Google Cartographer SLAM with SICK TiM561 USB LiDAR
Cartographer works WITHOUT odometry - uses scan matching only!
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the map (meters per pixel)'
    )

    # USB LiDAR node (publishes /scan)
    usb_lidar_node = Node(
        package='sick_tim_usb_ros2',
        executable='usb_lidar_node',
        name='usb_tim_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'laser_frame',
            'range_min': 0.1,
            'range_max': 10.0,
            'angular_resolution': '1.0',
            'log_throttle_sec': 2.0,
        }]
    )

    # base_link -> laser_frame static transform
    # LiDAR is at same position/orientation as base_link (no offset for stationary testing)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',  # No height offset for now (can adjust later)
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame'
        ]
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', PathJoinSubstitution([
                FindPackageShare('slam_config'),
                'config'
            ]),
            '-configuration_basename', 'cartographer_2d_lidar.lua'
        ],
        remappings=[
            ('/scan', '/scan'),
        ]
    )

    # Occupancy grid node (converts cartographer's submaps to nav_msgs/OccupancyGrid /map)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': resolution,
        }],
        arguments=['-resolution', resolution]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_resolution,
        usb_lidar_node,
        base_to_laser_tf,
        cartographer_node,
        occupancy_grid_node,
    ])
