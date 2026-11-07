"""Composable-node container launch for the RIGHT RoboSense AIRY LiDAR.

Designed for use inside the multi-sensor kit's lidar.launch.xml, which
controls the sensor-position namespace via <push-ros-namespace>.

When used standalone, pass a namespace arg to get properly isolated topics:
    ros2 launch fairy_e1r_airy_left_right_common_sensor_launch airy_right_node_container.launch.py namespace:=right

Sensor parameters (from sensor_details.txt):
    Airy-Right:  192.168.1.204  MSOP 6655  DIFOP 7744  IMU 6644

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


def _bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes")


# [nero 2026-11-06 AEBD2, aebdust_1106] The dust cloud in front of the bumper is also seen by the side Airys (11-04 14:46:32 /
# 18:21:19: the LEFT Airy's view of it made obstacle_stop stop the robot for 0.7-1.4 s after the E1R's dust stage removed its own
# part). NERO_AIRY_NEAR_DUST=1 runs the near weak return filter's dust tests on this Airy in FOLLOWER mode: it has no gate of its
# own and drops returns only within follow_sec of an open frame of the front E1R's dust gate (that stage publishes it), only in
# front of the bumper (x > 1.0, |y| <= 1.5, 0.08 < z <= 0.6 base_link, < 3.5 m from this Airy) and only off every surface / line of
# this Airy's range image in a mostly-dust, non-planar neighbourhood. It needs NERO_E1R_NEAR_DUST (and so NERO_E1R_NEAR_WEAK).
# Unset (or without the E1R stage): this container is exactly the car's.
_AIRY_DUST_DEFAULTS = {
    "dust.enabled": True, "dust.follow": True, "dust.follow_sec": 0.3, "dust.max_abs_y": 1.5, "dust.max_distance": 3.5,
    "dust.angle_radius": 0.025, "dust.segment_range": 4.5, "dust.segment_margin": 0.5, "dust.planar_min_points": 30,
}
_AIRY_DUST_KEYS = {
    "follow_sec": float, "max_abs_y": float, "max_distance": float, "angle_radius": float, "segment_range": float,
    "segment_margin": float, "planar_min_points": int, "max_intensity": int, "min_z": float, "max_z": float, "min_x": float, "range_tol_abs": float,
    "range_tol_rel": float, "surface_core": int, "surface_min": int, "line_core": int, "line_min": int, "line_extent": float,
    "local_radius": float, "local_share": float, "planar_max": float,
}
_DUST_GATE_TOPIC = "/sensing/lidar/front/near_weak_return_filter/dust_gate"   # the front E1R stage's ~/dust_gate


def _airy_dust_params(value):
    """NERO_AIRY_NEAR_DUST=1 (the defaults) or <key>=<value>,... (overrides), only with the E1R dust stage; else None."""
    value = value.strip()
    if not value or value.lower() in ("0", "false", "off"):
        return None
    e1r_dust = os.environ.get("NERO_E1R_NEAR_DUST", "").strip().lower()
    e1r_weak = os.environ.get("NERO_E1R_NEAR_WEAK", "").strip()
    if not e1r_dust or e1r_dust in ("0", "false", "off") or not e1r_weak:
        print("[airy_right_node_container] NERO_AIRY_NEAR_DUST needs NERO_E1R_NEAR_DUST + NERO_E1R_NEAR_WEAK (the E1R stage leads it): off")
        return None
    params = dict(_AIRY_DUST_DEFAULTS)
    # the first stage of the component (the E1R's weak isolated returns) stays off on an Airy
    params.update({"max_distance": 0.0, "max_intensity": 3, "neighbor_radius_ratio": 0.07, "max_neighbors": 10})
    if value.lower() in ("1", "true", "on"):
        return params
    try:
        for kv in value.split(","):
            k, v = kv.split("=")
            params["dust." + k.strip()] = _AIRY_DUST_KEYS[k.strip()](v)
        return params
    except (ValueError, KeyError):
        print(f"[airy_right_node_container] NERO_AIRY_NEAR_DUST='{value}' is not 1 or key=value pairs: off")
        return None


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("fairy_e1r_airy_left_right_common_sensor_launch")
    cropbox_param_file = os.path.join(pkg_dir, "config", "crop_box_filter_self_right.param.yaml")
    dc_param_file = os.path.join(pkg_dir, "config", "distortion_corrector_node.param.yaml")
    # [nero 2026-11-05 jolt] NERO_LIDAR_MOTION_3D=1: correct each sweep in 3D with the IMU's roll / pitch / yaw rate
    # (upstream DistortionCorrector3D: use_3d_distortion_correction). Unset / any other value: the param file as is.
    dc_params = [dc_param_file]
    if os.environ.get("NERO_LIDAR_MOTION_3D", "").strip() == "1":
        dc_params.append({"use_3d_distortion_correction": True})

    container_name = LaunchConfiguration("container_name").perform(context)
    node_name = LaunchConfiguration("node_name").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    use_mt = _bool(LaunchConfiguration("use_multithread_container").perform(context))
    use_ipc = _bool(LaunchConfiguration("use_intra_process_comms").perform(context))

    host_ip = LaunchConfiguration("host_ip").perform(context)
    sensor_ip = LaunchConfiguration("sensor_ip").perform(context)
    msop_port = int(LaunchConfiguration("msop_port").perform(context))
    difop_port = int(LaunchConfiguration("difop_port").perform(context))
    imu_port = int(LaunchConfiguration("imu_port").perform(context))
    frame_id = LaunchConfiguration("frame_id").perform(context)
    return_mode = LaunchConfiguration("return_mode").perform(context)
    recv_buf = int(LaunchConfiguration("receive_buffer_size").perform(context))

    driver_params = {
        "host_ip": host_ip,
        "sensor_ip": sensor_ip,
        "msop_port": msop_port,
        "difop_port": difop_port,
        "imu_port": imu_port,
        "frame_id": frame_id,
        "return_mode": return_mode,
        "receive_buffer_size": recv_buf,
    }

    node_ns = namespace if namespace else None

    airy_node = ComposableNode(
        package="airy_nebula_driver",
        plugin="airy_nebula_driver::AiryRosWrapper",
        name=node_name,
        namespace=node_ns,
        parameters=[driver_params],
        remappings=[
            ("airy_points", "airy_points"),
            ("aw_points", "aw_points"),
            ("aw_points_ex", "aw_points_ex"),
            ("imu_raw", "imu_raw"),
        ],
        extra_arguments=[{"use_intra_process_comms": use_ipc}],
    )

    frame_overrides = {
        "input_frame": LaunchConfiguration("input_frame"),
        "output_frame": LaunchConfiguration("output_frame"),
    }

    crop_box_node = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_self",
        namespace=node_ns,
        remappings=[
            ("input", "airy_points"),
            ("output", "self_cropped/pointcloud"),
        ],
        parameters=[cropbox_param_file, frame_overrides],
        extra_arguments=[{"use_intra_process_comms": use_ipc}],
    )

    # Motion (distortion) correction of each sweep with the ego twist + IMU yaw rate (2026-10-22).
    distortion_corrector_node = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
        name="distortion_corrector_node",
        namespace=node_ns,
        remappings=[
            ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
            ("~/input/imu", "/sensing/imu/imu_data"),
            ("~/input/pointcloud", "self_cropped/pointcloud"),
            ("~/output/pointcloud", "pointcloud_before_sync"),
        ],
        parameters=dc_params,
        extra_arguments=[{"use_intra_process_comms": use_ipc}],
    )

    nodes = [airy_node, crop_box_node, distortion_corrector_node]

    # [nero 2026-11-06 AEBD2] NERO_AIRY_NEAR_DUST: the dust follower between the distortion corrector and pointcloud_before_sync
    # (the corrector is re-declared with the follower's input as its output; its parameters as declared above)
    airy_dust = _airy_dust_params(os.environ.get("NERO_AIRY_NEAR_DUST", ""))
    if airy_dust:
        nodes[2] = ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            namespace=node_ns,
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "self_cropped/pointcloud"),
                ("~/output/pointcloud", "near_dust_filter/input"),
            ],
            parameters=distortion_corrector_node.parameters,
            extra_arguments=[{"use_intra_process_comms": use_ipc}],
        )
        nodes.append(
            ComposableNode(
                package="autoware_near_weak_return_filter",
                plugin="autoware::near_weak_return_filter::NearWeakReturnFilterComponent",
                name="near_weak_return_filter",
                namespace=node_ns,
                remappings=[
                    ("input", "near_dust_filter/input"),
                    ("output", "pointcloud_before_sync"),
                    ("dust_gate_in", _DUST_GATE_TOPIC),
                ],
                parameters=[airy_dust],
                extra_arguments=[{"use_intra_process_comms": use_ipc}],
            )
        )

    # Cloudini 1 mm compression of the motion-corrected crop-box output, recorded instead of the
    # raw cloud (~6x smaller bags). Relative topics and the crop box's namespace, so it lands next
    # to pointcloud_before_sync.
    if IfCondition(LaunchConfiguration("enable_cloudini")).evaluate(context):
        nodes.append(
            ComposableNode(
                package="cloudini_ros",
                plugin="CloudiniPointcloudConverter",
                name="cloudini_before_sync",
                namespace=node_ns,
                parameters=[
                    {
                        "topic_input": "pointcloud_before_sync",
                        "topic_output": "pointcloud_before_sync/cloudini",
                        "compressing": True,
                        "resolution": float(LaunchConfiguration("cloudini_resolution").perform(context)),
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": use_ipc}],
            )
        )

    container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt" if use_mt else "component_container",
        composable_node_descriptions=nodes,
        output="both",
    )

    return [container]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "container_name",
            default_value="airy_right_driver_container",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="airy_right_ros_wrapper_node",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Empty when called from XML (push-ros-namespace handles it); "
                        "set to e.g. 'right' for standalone use",
        ),
        DeclareLaunchArgument(
            "use_multithread_container",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_intra_process_comms",
            default_value="true",
        ),
        DeclareLaunchArgument("host_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("sensor_ip", default_value="192.168.1.204"),
        DeclareLaunchArgument("msop_port", default_value="6655"),
        DeclareLaunchArgument("difop_port", default_value="7744"),
        DeclareLaunchArgument("imu_port", default_value="8844"),
        DeclareLaunchArgument("frame_id", default_value="airy_right"),
        DeclareLaunchArgument("return_mode", default_value="Strongest"),
        DeclareLaunchArgument("receive_buffer_size", default_value="4194304"),
        DeclareLaunchArgument("input_frame", default_value="base_link"),
        DeclareLaunchArgument("output_frame", default_value="base_link"),
        DeclareLaunchArgument("enable_cloudini", default_value="true"),
        DeclareLaunchArgument("cloudini_resolution", default_value="0.001"),
    ]

    return LaunchDescription(
        [*declared, OpaqueFunction(function=launch_setup)]
    )
