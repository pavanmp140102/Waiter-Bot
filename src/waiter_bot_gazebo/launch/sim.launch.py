# Copyright (c) 2026 Pavan Patil
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Start Gazebo Harmonic with a world, spawn the waiter bot and bridge its topics to ROS."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown)
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Describe the launch: simulator, robot description, spawner and topic bridge."""
    gazebo_share = get_package_share_directory('waiter_bot_gazebo')
    description_share = get_package_share_directory('waiter_bot_description')

    world = PathJoinSubstitution(
        [gazebo_share, 'worlds', [LaunchConfiguration('world'), '.sdf']])
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(description_share, 'urdf', 'waiter_bot.urdf.xacro'),
        ' camera:=', LaunchConfiguration('camera')]), value_type=str)
    sim_time = {'use_sim_time': True}

    def simulator(context):
        """
        Run gz sim directly, as a server only or with the GUI, from the `headless` argument.

        It is started without a shell so that stopping the launch stops the simulator itself;
        a shell would die and leave the server running.
        """
        headless = LaunchConfiguration('headless').perform(context) == 'true'
        flags = ['-r', '-s', '--headless-rendering'] if headless else ['-r']
        return [ExecuteProcess(cmd=['gz', 'sim', *flags, world.perform(context)],
                               output='screen', sigterm_timeout='15', sigkill_timeout='10',
                               on_exit=[Shutdown()])]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='World file name in waiter_bot_gazebo/worlds'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the server only, without the Gazebo GUI'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Include the front camera'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # Lets Gazebo find the worlds' models and resolve package:// mesh URIs of the robot
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description_share)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models')),
        OpaqueFunction(function=simulator),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim_time],
             output='screen'),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'waiter_bot', '-topic', 'robot_description',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', '0.02', '-Y', LaunchConfiguration('yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gazebo_share, 'config', 'bridge.yaml')},
                         sim_time]),
    ])
