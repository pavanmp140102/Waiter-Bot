import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('map_file', default_value=FindPackageShare('waiter_bot_description').find('map/map1.yaml'), description='Path to the map file'),
        DeclareLaunchArgument('move_forward_only', default_value='true', description='Allow move forward only'),

        # Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            arguments=['--map-file', LaunchConfiguration('map_file')]
        ),

        # Include Bot AMCL Launch File
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                FindPackageShare('waiter_bot_description').find('navigation/bot_amcl.launch.py')
            )
        ),

        # Include Move Base Launch File
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                FindPackageShare('waiter_bot_description').find('navigation/movebase.launch.py')
            )
        ),

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', FindPackageShare('waiter_bot_description').find('rviz_config/nav2.rviz')]
        ),
    ])

