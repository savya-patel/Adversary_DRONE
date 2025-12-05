"""
Launch SLAM Toolbox with SICK TiM561 USB LiDAR
Uses sick_tim_usb_ros2 for direct USB connection (no sick_scan_xd needed)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_config'),
            'config',
            'mapper_params_online_async.yaml'
        ]),
        description='Full path to the SLAM Toolbox parameters file'
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
            'range_max': 10.0,  # TIM561 max range
            'angular_resolution': '1.0',
            'log_throttle_sec': 2.0,
        }]
    )
    
    # Static transforms for TF tree: odom -> base_link -> laser_frame
    # SLAM Toolbox will publish map->odom dynamically. We publish an identity
    # odom->base_link to close the TF tree in lidar-only setups (this is NOT an
    # odometry sensor, just a fixed transform so frames are connected).
    # Note: Using --frame-id parent_frame --child-frame-id child_frame format (ROS 2 Humble)

    # odom -> base_link (identity transform)
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link'
        ]
    )
    # base_link -> laser_frame (LiDAR mounting position)
    # Adjust x/y/z to match your robot's LiDAR mount (currently 0.1m above base_link)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.1',
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame'
        ]
    )
    
    # SLAM Toolbox node
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params,
        usb_lidar_node,
        odom_to_base_tf,
        base_to_laser_tf,
        slam_toolbox_node,
    ])
