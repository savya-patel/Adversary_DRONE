"""
⚠️⚠️⚠️ WARNING - DO NOT USE THIS FILE ⚠️⚠️⚠️

slam_toolbox has known issues on ROS 2 Humble:
- TF message filter deadlock (drops ALL scans with "queue is full" error)
- Incompatible with high-frequency LiDAR (20 Hz SICK TiM561)
- Will NOT work properly - use Cartographer instead!

✅ USE CARTOGRAPHER INSTEAD:
  cd ~/Adversary_DRONE/slam_ws
  ./run_cartographer.sh

This file is kept for reference only per user request "don't delete slam_toolbox files"

Original description:
Launch SLAM Toolbox with SICK TiM561 USB LiDAR and rf2o_laser_odometry
This variant uses rf2o to publish odom->base_link and /odom, and slam_toolbox
to publish map->odom. Do NOT publish an identity odom->base_link here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
            'mapper_params_online_async_odom.yaml'
        ]),
        description='Full path to the SLAM Toolbox parameters file (odom-enabled)'
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

    # laser_frame -> base_link (INVERTED to help message filter)
    # Message filter seems to struggle with parent-to-child lookups
    laser_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '-0.1',  # Inverted Z
            '--qx', '0.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '1.0',
            '--frame-id', 'laser_frame',
            '--child-frame-id', 'base_link'
        ]
    )

    # rf2o_laser_odometry: publishes odom->base_link and nav_msgs/Odometry on /odom
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'freq': 20.0,
            'init_pose_from_topic': ''  # Start from zero pose, don't wait for GT
        }],
        remappings=[
            ('/scan', '/scan'),  # ensure correct topic
        ]
    )

    # SLAM Toolbox node (uses odom and TF scan transformation)
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
        laser_to_base_tf,
        rf2o_node,
        slam_toolbox_node,
    ])
