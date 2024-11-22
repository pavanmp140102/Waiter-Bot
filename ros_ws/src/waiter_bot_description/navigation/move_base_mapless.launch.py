import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('no_static_map', default_value='true', description='Use dynamic map'),
        DeclareLaunchArgument('base_global_planner', default_value='navfn/NavfnROS', description='Global planner'),
        DeclareLaunchArgument('base_local_planner', default_value='dwa_local_planner/DWAPlannerROS', description='Local planner'),

        # Move Base Node
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='move_base',
            output='screen',
            parameters=[
                {
                    'base_global_planner': LaunchConfiguration('base_global_planner'),
                    'base_local_planner': LaunchConfiguration('base_local_planner')
                },
                # Load ROS parameters from YAML files
                FindPackageShare('waiter_bot_description').find('config/planner.yaml'),
                FindPackageShare('waiter_bot_description').find('config/costmap_common.yaml'),
                FindPackageShare('waiter_bot_description').find('config/costmap_common.yaml'),
                FindPackageShare('waiter_bot_description').find('config/costmap_local.yaml'),
                # Conditional parameters for global costmap
                FindPackageShare('waiter_bot_description').find('config/costmap_global_static.yaml'),
                FindPackageShare('waiter_bot_description').find('config/costmap_global_laser.yaml'),
            ],
            remappings=[
                # You can add topic remappings here if needed
            ]
        ),

        # Conditional load for Static Global Costmap (if no_static_map is false)
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='move_base_static_costmap',
            output='screen',
            parameters=[
                {
                    'global_costmap/width': 100.0,
                    'global_costmap/height': 100.0
                }
            ],
            condition=IfCondition(LaunchConfiguration('no_static_map'))
        ),

        # Conditional load for Dynamic Global Costmap (if no_static_map is true)
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='move_base_dynamic_costmap',
            output='screen',
            parameters=[
                {
                    'global_costmap/width': 100.0,
                    'global_costmap/height': 100.0
                }
            ],
            condition=UnlessCondition(LaunchConfiguration('no_static_map'))
        ),
    ])

