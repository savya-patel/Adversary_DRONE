import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Launch SICK TIM561 LiDAR with SLAM Toolbox for real-time mapping
    Optimized for NVIDIA Jetson
    """
    
    # Get package directories
    slam_config_dir = get_package_share_directory('slam_config')
    sick_scan_dir = get_package_share_directory('sick_scan_xd')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(slam_config_dir, 'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file for SLAM Toolbox')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start SLAM')
    
    # SICK TIM561 LiDAR Node
    # Note: You'll need to modify the hostname parameter to match your LiDAR's IP
    sick_tim561_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_tim561',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.0.1',  # Change to your LiDAR's IP address
            'port': '2112',
            'min_ang': -2.35619,  # -135 degrees (TIM561 has 270° FoV)
            'max_ang': 2.35619,   # +135 degrees
            'use_binary_protocol': True,
            'intensity': True,
            'frame_id': 'laser',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/cloud', '/sick_cloud'),
        ]
    )
    
    # Static transform from base_link to laser frame
    # Adjust xyz and rpy based on your sensor mounting
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )
    
    # SLAM Toolbox Node - Async mapping mode for best performance
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan')
        ]
    )
    
    # RViz2 for visualization (optional, comment out if running headless)
    rviz_config_file = os.path.join(slam_config_dir, 'rviz', 'slam_tim561.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add nodes
    ld.add_action(sick_tim561_node)
    ld.add_action(base_to_laser_tf)
    ld.add_action(slam_toolbox_node)
    # ld.add_action(rviz_node)  # Uncomment to enable RViz
    
    return ld
