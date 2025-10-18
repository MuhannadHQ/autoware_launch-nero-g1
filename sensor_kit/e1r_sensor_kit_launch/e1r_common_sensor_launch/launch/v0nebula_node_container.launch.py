# Copyright 2023 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# See original license for details.

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
import yaml


def get_vehicle_info(context):
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    nodes = []

    # --- CropBox filter for vehicle ---
    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_vehicle",
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # --- Distortion correction ---
    distortion_corrector_node_param = ParameterFile(
        param_file=LaunchConfiguration("distortion_correction_node_param_path").perform(context),
        allow_substs=True,
    )
    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            parameters=[distortion_corrector_node_param],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # --- Ring Outlier Filter ---
    ring_outlier_filter_node_param = ParameterFile(
        param_file=LaunchConfiguration("ring_outlier_filter_node_param_path").perform(context),
        allow_substs=True,
    )

    if LaunchConfiguration("output_as_sensor_frame").perform(context).lower() == "true":
        ring_outlier_output_frame = {"output_frame": LaunchConfiguration("frame_id")}
    else:
        ring_outlier_output_frame = {"output_frame": ""}

    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "pointcloud_final"),
            ],
            parameters=[ring_outlier_filter_node_param, ring_outlier_output_frame],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # --- Container for all nodes ---
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="both",
    )

    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    common_sensor_share_dir = get_package_share_directory("common_sensor_launch")

    # --- Keep only relevant launch args ---
    add_launch_arg("input_frame", "base_link", "use for cropbox")
    add_launch_arg("output_frame", "base_link", "use for cropbox")
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("container_name", "nebula_node_container")
    add_launch_arg("output_as_sensor_frame", "True", "output final pointcloud in sensor frame")
    add_launch_arg(
        "distortion_correction_node_param_path",
        os.path.join(
            common_sensor_share_dir,
            "config",
            "distortion_corrector_node.param.yaml",
        ),
        description="path to distortion correction params",
    )
    add_launch_arg(
        "ring_outlier_filter_node_param_path",
        os.path.join(
            common_sensor_share_dir,
            "config",
            "ring_outlier_filter_node.param.yaml",
        ),
        description="path to ring outlier filter params",
    )

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
        launch_arguments + [set_container_executable, set_container_mt_executable] + [OpaqueFunction(function=launch_setup)]
    )
