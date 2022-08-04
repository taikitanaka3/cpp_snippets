# Copyright 2021 Tier IV, Inc. All rights reserved.
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

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # delay_estimator_param param path
    delay_estimator_param = os.path.join(
        get_package_share_directory("delay_estimator_param"), "config"
    )
    with open(delay_estimator_param, "r") as f:
        delay_estimator_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    delay_estimator_component = ComposableNode(
        package="delay_estimator",
        plugin="ObstacleAvoidancePlanner",
        name="delay_estimator",
        namespace="",
        remappings=[
            ("~/input/command", "/control/command/control_cmd"),
            ("~/input/status", "/vehicle/status/steering_status"),
        ],
        parameters=[
            delay_estimator_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # obstacle stop planner
    obstacle_stop_planner_param_path = os.path.join(
        get_package_share_directory("tier4_planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "motion_planning",
        "obstacle_stop_planner",
        "obstacle_stop_planner.param.yaml",
    )
    obstacle_stop_planner_acc_param_path = os.path.join(
        get_package_share_directory("tier4_planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "motion_planning",
        "obstacle_stop_planner",
        "adaptive_cruise_control.param.yaml",
    )
    with open(obstacle_stop_planner_param_path, "r") as f:
        obstacle_stop_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(obstacle_stop_planner_acc_param_path, "r") as f:
        obstacle_stop_planner_acc_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    obstacle_stop_planner_component = ComposableNode(
        package="obstacle_stop_planner",
        plugin="motion_planning::ObstacleStopPlannerNode",
        name="obstacle_stop_planner",
        namespace="",
        remappings=[
            ("~/output/stop_reason", "/planning/scenario_planning/status/stop_reason"),
            ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
            ("~/output/max_velocity", "/planning/scenario_planning/max_velocity_candidates"),
            (
                "~/output/velocity_limit_clear_command",
                "/planning/scenario_planning/clear_velocity_limit",
            ),
            ("~/output/trajectory", "/planning/scenario_planning/lane_driving/trajectory"),
            (
                "~/input/pointcloud",
                "/perception/obstacle_segmentation/pointcloud",
            ),
            ("~/input/objects", "/perception/object_recognition/objects"),
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/input/trajectory", "obstacle_avoidance_planner/trajectory"),
        ],
        parameters=[
            common_param,
            obstacle_stop_planner_param,
            obstacle_stop_planner_acc_param,
            vehicle_info_param,
            {"enable_slow_down": False},
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # obstacle cruise planner
    obstacle_cruise_planner_param_path = os.path.join(
        get_package_share_directory("tier4_planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "motion_planning",
        "obstacle_cruise_planner",
        "obstacle_cruise_planner.param.yaml",
    )
    with open(obstacle_cruise_planner_param_path, "r") as f:
        obstacle_cruise_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    obstacle_cruise_planner_component = ComposableNode(
        package="obstacle_cruise_planner",
        plugin="motion_planning::ObstacleCruisePlannerNode",
        name="obstacle_cruise_planner",
        namespace="",
        remappings=[
            ("~/input/trajectory", "obstacle_avoidance_planner/trajectory"),
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/input/objects", "/perception/object_recognition/objects"),
            ("~/output/trajectory", "/planning/scenario_planning/lane_driving/trajectory"),
            ("~/output/velocity_limit", "/planning/scenario_planning/max_velocity_candidates"),
            ("~/output/clear_velocity_limit", "/planning/scenario_planning/clear_velocity_limit"),
            ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
        ],
        parameters=[
            common_param,
            obstacle_cruise_planner_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    delay_estimator_component = ComposableNode(
        package="topic_tools",
        plugin="topic_tools::RelayNode",
        name="delay_estimator_relay",
        namespace="",
        parameters=[
            {"~/input/command": "/control/command/control_cmd"},
            {"~/input/status": "/vehicle/status/steering_status"},
            {"type": "autoware_auto_planning_msgs/msg/Trajectory"},
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="motion_planning_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            obstacle_avoidance_planner_component,
        ],
    )

    obstacle_stop_planner_loader = LoadComposableNodes(
        composable_node_descriptions=[obstacle_stop_planner_component],
        target_container=container,
        condition=LaunchConfigurationEquals("cruise_planner", "obstacle_stop_planner"),
    )

    obstacle_cruise_planner_loader = LoadComposableNodes(
        composable_node_descriptions=[obstacle_cruise_planner_component],
        target_container=container,
        condition=LaunchConfigurationEquals("cruise_planner", "obstacle_cruise_planner"),
    )

    obstacle_cruise_planner_relay_loader = LoadComposableNodes(
        composable_node_descriptions=[obstacle_cruise_planner_relay_component],
        target_container=container,
        condition=LaunchConfigurationEquals("cruise_planner", "none"),
    )

    surround_obstacle_checker_loader = LoadComposableNodes(
        composable_node_descriptions=[surround_obstacle_checker_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("use_surround_obstacle_check")),
    )

    group = GroupAction(
        [
            container,
            obstacle_stop_planner_loader,
            obstacle_cruise_planner_loader,
            obstacle_cruise_planner_relay_loader,
            surround_obstacle_checker_loader,
        ]
    )
    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # vehicle information parameter file
    add_launch_arg(
        "vehicle_info_param_file",
        [
            FindPackageShare("vehicle_info_util"),
            "/config/vehicle_info.param.yaml",
        ],
        "path to the parameter file of vehicle information",
    )

    # obstacle_avoidance_planner
    add_launch_arg(
        "input_path_topic",
        "/planning/scenario_planning/lane_driving/behavior_planning/path",
        "input path topic of obstacle_avoidance_planner",
    )

    # surround obstacle checker
    add_launch_arg("use_surround_obstacle_check", "true", "launch surround_obstacle_checker or not")
    add_launch_arg(
        "cruise_planner", "obstacle_stop_planner", "cruise planner type"
    )  # select from "obstacle_stop_planner", "obstacle_cruise_planner", "none"

    add_launch_arg("use_intra_process", "false", "use ROS2 component container communication")
    add_launch_arg("use_multithread", "false", "use multithread")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )
    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
