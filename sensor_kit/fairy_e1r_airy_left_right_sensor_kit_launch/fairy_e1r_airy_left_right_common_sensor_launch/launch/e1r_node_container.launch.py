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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _near_weak_params(value):
    """NERO_E1R_NEAR_WEAK=<max_distance m>,<max_intensity>,<neighbor_radius_ratio>,<max_neighbors> or None."""
    value = value.strip()
    if not value:
        return None
    try:
        r, i, k, n = value.split(",")
        return {
            "max_distance": float(r),
            "max_intensity": int(i),
            "neighbor_radius_ratio": float(k),
            "max_neighbors": int(n),
        }
    except ValueError:
        print(f"[e1r_node_container] NERO_E1R_NEAR_WEAK='{value}' is not 4 numbers: E1R path unchanged")
        return None


_DUST_KEYS = {
    "max_distance": float, "max_intensity": int, "min_z": float, "max_z": float, "min_x": float, "max_abs_y": float,
    "angle_radius": float, "range_tol_abs": float, "range_tol_rel": float, "surface_core": int, "surface_min": int,
    "line_core": int, "line_min": int, "line_extent": float, "segment_range": float, "local_radius": float,
    "local_share": float, "planar_max": float, "planar_min_points": int, "frame_min": int, "gate_max_intensity": int,
    "gate_weak_share": float, "hold_min": int, "hold_sec": float,
    "gate_max_abs_y": float, "segment_margin": float,   # aebdust_1106
}


def _near_dust_params(value):
    """NERO_E1R_NEAR_DUST=1 (the package defaults) or <key>=<value>,... (overrides) or None."""
    value = value.strip()
    if not value or value.lower() in ("0", "false", "off"):
        return None
    params = {"dust.enabled": True}
    if value.lower() in ("1", "true", "on"):
        return params
    try:
        for kv in value.split(","):
            k, v = kv.split("=")
            params["dust." + k.strip()] = _DUST_KEYS[k.strip()](v)
        return params
    except (ValueError, KeyError):
        print(f"[e1r_node_container] NERO_E1R_NEAR_DUST='{value}' is not 1 or key=value pairs: dust stage off")
        return None


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("fairy_e1r_airy_left_right_common_sensor_launch")
    cropbox_param_file = os.path.join(pkg_dir, "config", "crop_box_filter_self.param.yaml")
    dc_param_file = os.path.join(pkg_dir, "config", "distortion_corrector_node.param.yaml")
    # [nero 2026-11-05 jolt] NERO_LIDAR_MOTION_3D=1: correct each sweep in 3D with the IMU's roll / pitch / yaw rate
    # (upstream DistortionCorrector3D: use_3d_distortion_correction). Unset / any other value: the param file as is.
    dc_params = [dc_param_file]
    if os.environ.get("NERO_LIDAR_MOTION_3D", "").strip() == "1":
        dc_params.append({"use_3d_distortion_correction": True})

    # [nero 2026-10-28 AEBD] NERO_E1R_NEAR_WEAK (e.g. 1.5,3,0.07,10) puts autoware_near_weak_return_filter after the
    # distortion corrector: it drops E1R returns that are near the sensor, weak AND isolated (the dust / near-range
    # noise behind the 10-27 20:47:49.7 phantom AEB stop). Unset: this container is exactly the car's.
    near_weak = _near_weak_params(os.environ.get("NERO_E1R_NEAR_WEAK", ""))
    # [nero 2026-11-05 AEBD2] NERO_E1R_NEAR_DUST=1 also runs the component's dust stage (dense dust in front of the
    # bumper that the weak-return stage keeps: the 10-29 / 11-01 / 11-04 phantom AEB stops). It lives in the same
    # component, so it needs NERO_E1R_NEAR_WEAK. Unset: the component's parameters are exactly as before.
    near_dust = _near_dust_params(os.environ.get("NERO_E1R_NEAR_DUST", ""))
    if near_dust and not near_weak:
        print("[e1r_node_container] NERO_E1R_NEAR_DUST needs NERO_E1R_NEAR_WEAK (the same component): dust stage off")
        near_dust = None
    dc_output = "near_weak_filter/input" if near_weak else "pointcloud_before_sync"

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
            ("~/output/pointcloud", dc_output),
        ],
        parameters=dc_params,
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    nodes = [e1r_node, crop_box_node, distortion_corrector_node]

    if near_weak:
        nodes.append(
            ComposableNode(
                package="autoware_near_weak_return_filter",
                plugin="autoware::near_weak_return_filter::NearWeakReturnFilterComponent",
                name="near_weak_return_filter",
                remappings=[("input", dc_output), ("output", "pointcloud_before_sync")],
                parameters=[dict(near_weak, **near_dust) if near_dust else near_weak],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

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
        name="e1r_driver_container",
        namespace="pointcloud_driver",
        package="rclcpp_components",
        executable="component_container_mt" if use_mt else "component_container",
        composable_node_descriptions=nodes,
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
        DeclareLaunchArgument("use_multithread_container", default_value="true"),
        DeclareLaunchArgument("enable_cloudini", default_value="true"),
        DeclareLaunchArgument("cloudini_resolution", default_value="0.001"),
    ]

    return LaunchDescription(
        [*launch_arguments, OpaqueFunction(function=launch_setup)]
    )
