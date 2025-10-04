import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    nodes = []

    # VoxelGrid filter node
    nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::VoxelGridFilterComponent",
            name="voxel_grid_filter",
            remappings=[
                ("input", "lidar/points"),
                ("output", "lidar/points_voxel"),
            ],
            parameters=[{
                "leaf_size_x": 0.1,
                "leaf_size_y": 0.1,
                "leaf_size_z": 0.1,
            }],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # Container to run the node
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
    launch_arguments = []

    # Basic launch args
    launch_arguments.append(
        DeclareLaunchArgument("container_name", default_value="pointcloud_container")
    )
    launch_arguments.append(
        DeclareLaunchArgument("use_intra_process", default_value="true")
    )
    launch_arguments.append(
        DeclareLaunchArgument("use_multithread", default_value="false")
    )

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
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
