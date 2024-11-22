from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='$(find waiter_bot_description)/urdf/waiter_bot.xacro'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rvizconfig', default_value='$(find waiter_bot_description)/launch/urdf.rviz'),

        # Publish the robot description
        Node(
            package='xacro',
            executable='xacro',
            name='xacro',
            output='screen',
            arguments=[LaunchConfiguration('model')],
            remappings=[('/robot_description', '/robot_description')]
        ),

        # Joint State Publisher and Robot State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]
        ),
    ])

