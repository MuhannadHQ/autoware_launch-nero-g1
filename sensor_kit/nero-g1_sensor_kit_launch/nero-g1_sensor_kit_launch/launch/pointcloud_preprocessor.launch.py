import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from rclpy.qos import QoSProfile, ReliabilityPolicy

def launch_setup(context, *args, **kwargs):
    nodes = []

    # CropBox filter node (works without external data)
    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_test",
            remappings=[
                ("input", "/points_in"),   # check names
                ("output", "/points_out"),
            ],
            parameters=[{
                "min_x": -1.0, "max_x": 1.0,
                "min_y": -1.0, "max_y": 1.0,
                "min_z": -1.0, "max_z": 1.0,
                "negative": True,
                "input_frame": "rslidar",   # adjust later
                "output_frame": "rslidar"   # adjust later
            }],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="autoware_pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        output="screen",
    )

    return [container]

def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument("container_name", default_value="test_container"),
        DeclareLaunchArgument("use_intra_process", default_value="true"),
        DeclareLaunchArgument("use_multithread", default_value="false"),
    ]

    # Container executable setup
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
        launch_arguments + [set_container_executable, set_container_mt_executable, OpaqueFunction(function=launch_setup)]
    )