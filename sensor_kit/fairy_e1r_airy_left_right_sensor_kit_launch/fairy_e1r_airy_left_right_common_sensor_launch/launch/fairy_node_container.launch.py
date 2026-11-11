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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("fairy_e1r_airy_left_right_common_sensor_launch")
    cropbox_param_file = os.path.join(pkg_dir, "config", "crop_box_filter_self.param.yaml")
    dc_param_file = os.path.join(pkg_dir, "config", "distortion_corrector_node.param.yaml")
    # [nero 2026-11-05 jolt] NERO_LIDAR_MOTION_3D=1: correct each sweep in 3D with the IMU's roll / pitch / yaw rate
    # (upstream DistortionCorrector3D: use_3d_distortion_correction). Unset / any other value: the param file as is.
    dc_params = [dc_param_file]
    if os.environ.get("NERO_LIDAR_MOTION_3D", "").strip() == "1":
        dc_params.append({"use_3d_distortion_correction": True})

    driver_params = {
        "host_ip": LaunchConfiguration("host_ip"),
        "sensor_ip": LaunchConfiguration("sensor_ip"),
        "data_port": LaunchConfiguration("data_port"),
        "difop_port": LaunchConfiguration("difop_port"),
        "frame_id": LaunchConfiguration("frame_id"),
        "return_mode": LaunchConfiguration("return_mode"),
        "cut_angle": float(LaunchConfiguration("cut_angle").perform(context)),
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
            ("output", "self_cropped/pointcloud"),
        ],
        parameters=[cropbox_param_file, frame_overrides],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Motion (distortion) correction of each sweep with the ego twist + IMU yaw rate (2026-10-22).
    distortion_corrector_node = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
        name="distortion_corrector_node",
        remappings=[
            ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
            ("~/input/imu", "/sensing/imu/imu_data"),
            ("~/input/pointcloud", "self_cropped/pointcloud"),
            ("~/output/pointcloud", "pointcloud_before_sync"),
        ],
        parameters=dc_params,
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    nodes = [fairy_node, crop_box_node, distortion_corrector_node]

    # Cloudini 1 mm compression of the motion-corrected crop-box output, recorded instead of the
    # raw cloud (~6x smaller bags). Relative topics, so it lands next to pointcloud_before_sync.
    if IfCondition(LaunchConfiguration("enable_cloudini")).evaluate(context):
        nodes.append(
            ComposableNode(
                package="cloudini_ros",
                plugin="CloudiniPointcloudConverter",
                name="cloudini_before_sync",
                parameters=[
                    {
                        "topic_input": "pointcloud_before_sync",
                        "topic_output": "pointcloud_before_sync/cloudini",
                        "compressing": True,
                        "resolution": float(LaunchConfiguration("cloudini_resolution").perform(context)),
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    # Multithreaded by default so compression does not delay the driver and crop box callbacks.
    use_mt = IfCondition(LaunchConfiguration("use_multithread_container")).evaluate(context)

    container = ComposableNodeContainer(
        name="fairy_driver_container",
        namespace="pointcloud_driver",
        package="rclcpp_components",
        executable="component_container_mt" if use_mt else "component_container",
        composable_node_descriptions=nodes,
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
        # [nero 2026-11-11 PHASELOCK] frame seam (driver cut_angle, deg, sensor encoder frame) = the azimuth that points to the
        # vehicle rear (sensor_kit_calibration.yaml v0.44.3). The lidar's Phase Lock (web PhaLockSet) puts it there on every
        # whole second, so each cloud is stamped on the PTP 100 ms grid and all lidars look forward together at +50 ms.
        # nero-drivers-g1 docs/LIDAR_PHASE_LOCK.md (recompute after a re-mount / re-calibration). 0 = old azimuth-wrap cut.
        DeclareLaunchArgument("cut_angle", default_value="177.75"),
        DeclareLaunchArgument("return_mode", default_value="Strongest"),
        DeclareLaunchArgument("input_frame", default_value="base_link"),
        DeclareLaunchArgument("output_frame", default_value="base_link"),
        DeclareLaunchArgument("use_multithread_container", default_value="true"),
        DeclareLaunchArgument("enable_cloudini", default_value="true"),
        DeclareLaunchArgument("cloudini_resolution", default_value="0.001"),
    ]

    return LaunchDescription(
        [*launch_arguments, OpaqueFunction(function=launch_setup)]
    )
