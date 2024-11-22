from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, PushRosNamespace, LogInfo
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Load the controller.yaml as parameters
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            namespace='waiter_bot',
            output='screen',
            arguments=[
                'RF_W_position_controller',
                'LF_W_position_controller',
                'LB_W_position_controller',
                'RB_w_position_controller',
                'joint_state_controller'
            ],
            parameters=[os.path.join(get_package_share_directory('waiter_bot_description'), 'launch', 'controller.yaml')],
            respawn=False
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': 'true'
            }],
            remappings=[('/joint_states', '/waiter_bot/joint_states')]
        )
    ])

