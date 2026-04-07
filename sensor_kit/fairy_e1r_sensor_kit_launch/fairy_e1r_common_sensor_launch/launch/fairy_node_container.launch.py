"""Composable-node container launch for the RoboSense Fairy (AIRY) Nebula-style driver.

Follows the same pattern as nebula_node_container.launch.py / e1r_node_container.launch.py,
adapted for the Fairy mechanical LiDAR.  Includes a CropBoxFilter that removes points
belonging to the vehicle body (negative crop) — identical to the self-crop stage in the
Autoware preprocessing pipeline.

CropBox geometry is loaded from the shared config/crop_box_filter_self.param.yaml;
sensor-specific frame IDs are overlaid via launch arguments.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("fairy_e1r_common_sensor_launch")
    cropbox_param_file = os.path.join(pkg_dir, "config", "crop_box_filter_self.param.yaml")

    driver_params = {
        "host_ip": LaunchConfiguration("host_ip"),
        "sensor_ip": LaunchConfiguration("sensor_ip"),
        "data_port": LaunchConfiguration("data_port"),
        "difop_port": LaunchConfiguration("difop_port"),
        "frame_id": LaunchConfiguration("frame_id"),
        "return_mode": LaunchConfiguration("return_mode"),
    }

    fairy_node = ComposableNode(
        package="fairy_nebula_driver",
        plugin="fairy_nebula_driver::FairyRosWrapper",
        name="fairy_ros_wrapper_node",
        parameters=[driver_params],
        remappings=[
            ("fairy_points", "fairy_points"),
            ("aw_points", "aw_points"),
            ("aw_points_ex", "aw_points_ex"),
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
            ("input", "fairy_points"),
            ("output", "pointcloud_before_sync"),
        ],
        parameters=[cropbox_param_file, frame_overrides],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="fairy_driver_container",
        namespace="pointcloud_driver",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[fairy_node, crop_box_node],
        output="screen",
    )

    return [container]


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument("host_ip", default_value="192.168.1.102"),
        DeclareLaunchArgument("sensor_ip", default_value="192.168.1.202"),
        DeclareLaunchArgument("data_port", default_value="6677"),
        DeclareLaunchArgument("difop_port", default_value="7766"),
        DeclareLaunchArgument("frame_id", default_value="fairy"),
        DeclareLaunchArgument("return_mode", default_value="Strongest"),
        DeclareLaunchArgument("input_frame", default_value="sensor_kit_base_link"),
        DeclareLaunchArgument("output_frame", default_value="sensor_kit_base_link"),
    ]

    return LaunchDescription(
        [*launch_arguments, OpaqueFunction(function=launch_setup)]
    )
