import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel', description='Cmd Vel topic'),
        DeclareLaunchArgument('odom_topic', default_value='odom', description='Odometry topic'),
        DeclareLaunchArgument('move_forward_only', default_value='true', description='Move forward only'),

        # Move Base Node
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='move_base',
            output='screen',
            parameters=[
                {
                    'base_local_planner': 'dwa_local_planner/DWAPlannerROS',
                    'DWAPlannerROS/min_vel_x': 0.0 if LaunchConfiguration('move_forward_only') == 'true' else None
                },
                # Load ROS parameters from YAML files
                FindPackageShare('waiter_bot_description').find('config/planner.yaml'),
                FindPackageShare('defect_bot_description').find('param/costmap_common_params.yaml'),
                FindPackageShare('defect_bot_description').find('param/costmap_common_params.yaml'),
                FindPackageShare('defect_bot_description').find('param/local_costmap_params.yaml'),
                FindPackageShare('defect_bot_description').find('param/global_costmap_params.yaml'),
                FindPackageShare('defect_bot_description').find('param/move_base_params.yaml')
            ],
            remappings=[
                ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                ('odom', LaunchConfiguration('odom_topic'))
            ]
        ),
    ])

