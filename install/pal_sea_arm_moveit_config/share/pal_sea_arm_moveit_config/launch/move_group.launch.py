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
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from pal_sea_arm_description.pal_sea_arm_utils import get_pal_sea_arm_hw_suffix
from pal_sea_arm_description.launch_arguments import SEAArmArgs
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    wrist_model: DeclareLaunchArgument = SEAArmArgs.wrist_model
    end_effector: DeclareLaunchArgument = SEAArmArgs.end_effector
    ft_sensor: DeclareLaunchArgument = SEAArmArgs.ft_sensor
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    arm_type: DeclareLaunchArgument = DeclareLaunchArgument(
        'arm_type', default_value='pal-sea-arm-standalone',
        choices=['pal-sea-arm-standalone', 'tiago-pro', 'tiago-sea', 'tiago-sea-dual'],
        description='The arm model')


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(function=start_move_group))
    return


def start_move_group(context, *args, **kwargs):

    end_effector = read_launch_argument('end_effector', context)
    ft_sensor = read_launch_argument('ft_sensor', context)

    if end_effector == "no-end-effector":
        end_effector = "no-ee"

    hw_suffix = get_pal_sea_arm_hw_suffix(
        end_effector=end_effector,
        ft_sensor=ft_sensor
    )
    srdf_file_path = Path(
        os.path.join(
            get_package_share_directory("pal_sea_arm_moveit_config"),
            "config", "srdf",
            "pal_sea_arm.srdf.xacro",
        )
    )

    srdf_input_args = {
        'arm_type': read_launch_argument('arm_type', context),
        'end_effector': read_launch_argument('end_effector', context),
        'ft_sensor': read_launch_argument('ft_sensor', context)
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        f'config/controllers/controllers{hw_suffix}.yaml')

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder('pal_sea_arm')
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .joint_limits(file_path=os.path.join('config', 'joint_limits.yaml'))
        .planning_pipelines(pipelines=['ompl', 'chomp'], default_planning_pipeline='ompl')
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
    )

    moveit_config.to_moveit_configs()

    move_group_configuration = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'publish_robot_description_semantic': True,
        'robot_description_timeout': 60.0,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        emulate_tty=True,
        parameters=move_group_params,
    )

    return [run_move_group_node]


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
