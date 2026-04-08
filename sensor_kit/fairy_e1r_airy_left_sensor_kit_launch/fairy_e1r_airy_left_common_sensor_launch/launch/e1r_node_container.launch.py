"""Composable-node container launch for the RoboSense E1R Nebula-style driver.

Follows the same pattern as nebula_node_container.launch.py used for the
Helios sensor, adapted for the E1R solid-state LiDAR.

Includes a CropBoxFilter for vehicle self-crop (negative crop).
CropBox geometry is loaded from the shared config/crop_box_filter_self.param.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("fairy_e1r_airy_left_common_sensor_launch")
    cropbox_param_file = os.path.join(pkg_dir, "config", "crop_box_filter_self.param.yaml")

    driver_params = {
        "host_ip": LaunchConfiguration("host_ip"),
        "sensor_ip": LaunchConfiguration("sensor_ip"),
        "data_port": LaunchConfiguration("data_port"),
        "difop_port": LaunchConfiguration("difop_port"),
        "frame_id": LaunchConfiguration("frame_id"),
        "return_mode": LaunchConfiguration("return_mode"),
    }

    e1r_node = ComposableNode(
        package="e1r_nebula_driver",
        plugin="e1r_nebula_driver::E1RRosWrapper",
        name="e1r_ros_wrapper_node",
        parameters=[driver_params],
        remappings=[
            ("e1r_points", "e1r_points"),
            ("aw_points", "aw_points_e1r"),
            ("aw_points_ex", "aw_points_ex_e1r"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    frame_overrides = {
        "input_frame": LaunchConfiguration("input_frame"),
        "output_frame": LaunchConfiguration("output_frame"),
    }

    crop_box_node = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_self",
        remappings=[
            ("input", "e1r_points"),
            ("output", "pointcloud_before_sync"),
        ],
        parameters=[cropbox_param_file, frame_overrides],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="e1r_driver_container",
        namespace="pointcloud_driver",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[e1r_node, crop_box_node],
        output="screen",
    )

    return [container]


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument("host_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("sensor_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("data_port", default_value="6699"),
        DeclareLaunchArgument("difop_port", default_value="7788"),
        DeclareLaunchArgument("frame_id", default_value="e1r"),
        DeclareLaunchArgument("return_mode", default_value="Strongest"),
        DeclareLaunchArgument("input_frame", default_value="base_link"),
        DeclareLaunchArgument("output_frame", default_value="base_link"),
    ]

    return LaunchDescription(
        [*launch_arguments, OpaqueFunction(function=launch_setup)]
    )
