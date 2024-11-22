import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='Scan topic'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0', description='Initial pose x'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0', description='Initial pose y'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0', description='Initial pose angle'),

        # AMCL (Nav2 version)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'min_particles': 100,
                'max_particles': 4000,
                'kld_err': 0.02,
                'update_min_d': 0.20,
                'update_min_a': 0.20,
                'resample_interval': 1,
                'transform_tolerance': 0.5,
                'recovery_alpha_slow': 0.00,
                'recovery_alpha_fast': 0.00,
                'initial_pose_x': launch.substitutions.LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': launch.substitutions.LaunchConfiguration('initial_pose_y'),
                'initial_pose_a': launch.substitutions.LaunchConfiguration('initial_pose_a'),
                'gui_publish_rate': 50.0,
                'laser_max_range': 3.5,
                'laser_max_beams': 180
            }],
            remappings=[('scan', launch.substitutions.LaunchConfiguration('scan_topic'))]
        ),
    ])

