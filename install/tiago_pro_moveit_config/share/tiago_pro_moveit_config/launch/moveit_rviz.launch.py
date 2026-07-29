# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from tiago_pro_description.launch_arguments import TiagoProArgs
from launch_pal.arg_utils import LaunchArgumentsBase
from tiago_pro_description.tiago_pro_launch_utils import get_tiago_pro_hw_suffix
from dataclasses import dataclass
from launch_pal.robot_arguments import CommonArgs


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoProArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoProArgs.ft_sensor_left
    base_type: DeclareLaunchArgument = TiagoProArgs.base_type
    wrist_model_right: DeclareLaunchArgument = TiagoProArgs.wrist_model_right
    wrist_model_left: DeclareLaunchArgument = TiagoProArgs.wrist_model_left

    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    use_sensor_manager_arg: DeclareLaunchArgument = CommonArgs.use_sensor_manager


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(function=start_rviz))
    return


def start_rviz(context, *args, **kwargs):

    arm_type_right = read_launch_argument('arm_type_right', context)
    arm_type_left = read_launch_argument('arm_type_left', context)
    end_effector_right = read_launch_argument('end_effector_right', context)
    end_effector_left = read_launch_argument('end_effector_left', context)

    hw_suffix = get_tiago_pro_hw_suffix(
        arm_right=arm_type_right,
        arm_left=arm_type_left,
        end_effector_right=end_effector_right,
        end_effector_left=end_effector_left)

    srdf_file_path = Path(
        os.path.join(
            get_package_share_directory("tiago_pro_moveit_config"),
            "config", "srdf",
            "tiago_pro.srdf.xacro",
        )
    )

    srdf_input_args = {
        'arm_type_right': read_launch_argument('arm_type_right', context),
        'arm_type_left': read_launch_argument('arm_type_left', context),
        'end_effector_right': read_launch_argument('end_effector_right', context),
        'end_effector_left': read_launch_argument('end_effector_left', context),
        'ft_sensor_right': read_launch_argument('ft_sensor_right', context),
        'ft_sensor_left': read_launch_argument('ft_sensor_left', context),
        "base_type": read_launch_argument("base_type", context),
        'wrist_model_right': read_launch_argument('wrist_model_right', context),
        'wrist_model_left': read_launch_argument('wrist_model_left', context),
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
        MoveItConfigsBuilder('tiago_pro')
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .joint_limits(file_path=os.path.join('config', 'joint_limits.yaml'))
        .planning_pipelines(pipelines=['ompl', 'chomp'], default_planning_pipeline='ompl')
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(file_path=os.path.join('config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'tiago_pro_moveit_config'), 'config', 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        emulate_tty=True,
        parameters=[
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
