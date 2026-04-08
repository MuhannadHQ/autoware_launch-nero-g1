"""Composable-node container launch for the LEFT RoboSense AIRY LiDAR.

Designed for use inside the multi-sensor kit's lidar.launch.xml, which
controls the sensor-position namespace via <push-ros-namespace>.

When used standalone, pass a namespace arg to get properly isolated topics:
    ros2 launch fairy_e1r_airy_left_common_sensor_launch airy_left_node_container.launch.py namespace:=left

Sensor parameters (from sensor_details.txt):
    Airy-Left:  192.168.1.203  MSOP 6666  DIFOP 7755  IMU 6655

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


def _bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes")


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("fairy_e1r_airy_left_common_sensor_launch")
    cropbox_param_file = os.path.join(pkg_dir, "config", "crop_box_filter_self.param.yaml")

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
            ("output", "pointcloud_before_sync"),
        ],
        parameters=[cropbox_param_file, frame_overrides],
        extra_arguments=[{"use_intra_process_comms": use_ipc}],
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt" if use_mt else "component_container",
        composable_node_descriptions=[airy_node, crop_box_node],
        output="both",
    )

    return [container]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "container_name",
            default_value="airy_left_driver_container",
        ),
        DeclareLaunchArgument(
            "node_name",
            default_value="airy_left_ros_wrapper_node",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Empty when called from XML (push-ros-namespace handles it); "
                        "set to e.g. 'left' for standalone use",
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
        DeclareLaunchArgument("sensor_ip", default_value="192.168.1.203"),
        DeclareLaunchArgument("msop_port", default_value="6666"),
        DeclareLaunchArgument("difop_port", default_value="7755"),
        DeclareLaunchArgument("imu_port", default_value="8855"),
        DeclareLaunchArgument("frame_id", default_value="airy_left"),
        DeclareLaunchArgument("return_mode", default_value="Strongest"),
        DeclareLaunchArgument("receive_buffer_size", default_value="4194304"),
        DeclareLaunchArgument("input_frame", default_value="base_link"),
        DeclareLaunchArgument("output_frame", default_value="base_link"),
    ]

    return LaunchDescription(
        [*declared, OpaqueFunction(function=launch_setup)]
    )
