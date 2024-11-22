import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Gmapping launch
        IncludeLaunchDescription(
            FindPackageShare('waiter_bot_description'),
            launch_file='navigation/gmapping_auto.launch.py'
        ),

        # Nav2 move_base mapless launch
        IncludeLaunchDescription(
            FindPackageShare('waiter_bot_description'),
            launch_file='navigation/move_base_mapless.launch.py'
        )
    ])

