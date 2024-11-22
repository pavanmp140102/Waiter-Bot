from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_name', default_value=os.path.join(get_package_share_directory('waiter_bot_description'), 'world', 'cafe3.world')),

        # Spawn URDF Model
        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_urdf',
            arguments=['-param', 'robot_description', '-urdf', '-model', 'waiter_bot'],
            output='screen'
        ),

        # Gazebo Empty World
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo',
            arguments=['-s', 'libgazebo_ros_factory.so'],
            parameters=[{'use_sim_time': 'true'}],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('waiter_bot_description'), 'launch', 'urdf.rviz')]
        ),

        # Control Panel
        Node(
            package='waiter_bot_description',
            executable='control_panel.py',
            name='control_panel',
            condition=IfCondition(LaunchConfiguration('ui'))
        ),
    ])

