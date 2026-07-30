# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from pal_sea_arm_description.pal_sea_arm_utils import get_pal_sea_arm_hw_suffix
from pal_sea_arm_description.launch_arguments import SEAArmArgs
from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    wrist_model: DeclareLaunchArgument = SEAArmArgs.wrist_model
    end_effector: DeclareLaunchArgument = SEAArmArgs.end_effector
    ft_sensor: DeclareLaunchArgument = SEAArmArgs.ft_sensor
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    arm_model: DeclareLaunchArgument = DeclareLaunchArgument(
        'arm_model', default_value='pal-sea-arm-standalone',
        choices=['pal-sea-arm-standalone', 'tiago-pro', 'tiago-sea', 'tiago-sea-dual'],
        description='The arm model')


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(function=start_rviz))
    return


def start_rviz(context, *args, **kwargs):

    end_effector = read_launch_argument('end_effector', context)
    ft_sensor = read_launch_argument('ft_sensor', context)

    hw_suffix = get_pal_sea_arm_hw_suffix(
        end_effector=end_effector,
        ft_sensor=ft_sensor
    )

    robot_description_semantic = (f'config/srdf/pal_sea_arm{hw_suffix}.srdf')

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        f'config/controllers/controllers{hw_suffix}.yaml')

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder('pal_sea_arm')
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'pal_sea_arm_moveit_config'), 'config', 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        emulate_tty=True,
        parameters=[
            {},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return [rviz_node]


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
