import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='Scan topic'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame'),
        DeclareLaunchArgument('gmap', default_value='true', description='Launch GMAP RViz config'),
        DeclareLaunchArgument('explore', default_value='false', description='Launch Explore RViz config'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),

        # Conditional launch of GMAP RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('gmap')],
            condition=IfCondition(LaunchConfiguration('gmap')),
            required=True
        ),
        
        # Conditional launch of Explore RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('explore')],
            condition=IfCondition(LaunchConfiguration('explore')),
            required=True
        ),
        
        # SLAM using Slam Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'map_update_interval': 5.0,
                'maxUrange': 6.0,
                'maxRange': 8.0
            }]
        ),
    ])

