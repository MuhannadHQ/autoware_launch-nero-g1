# Copyright 2020 Tier IV, Inc. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    # concatenate node parameters
    concatenate_and_time_sync_node_param = ParameterFile(
        param_file=LaunchConfiguration("concatenate_and_time_sync_node_param_path").perform(
            context
        ),
        allow_substs=True,
    )

    concat_parameters = [concatenate_and_time_sync_node_param]
    # [nero 2026-11-04 LIDPHASE / CONCATPHASE] NERO_LIDAR_CONCAT_MATCH=advanced,<noise_s>[,<offset_s> x 4]: group the four
    # lidar scans by their header stamps (matching_strategy "advanced": stamp - offset within the summed noise windows)
    # instead of by arrival (naive), so one late / lost scan no longer pairs every later cloud with that lidar's NEXT
    # scan. noise_s per lidar (0.04 = +-80 ms between two lidars: wider than any free-running phase spread seen, 32-62 ms,
    # and narrower than the 100 ms sweep, so a next-cycle scan never joins). Unset / empty / invalid: the param file as is.
    nero_match = os.environ.get("NERO_LIDAR_CONCAT_MATCH", "").strip()
    if nero_match:
        fields = [f.strip() for f in nero_match.split(",")]
        try:
            if fields[0] != "advanced" or len(fields) not in (2, 6):
                raise ValueError("expected advanced,<noise_s>[,<offset_s> x 4]")
            noise = float(fields[1])
            offsets = [float(f) for f in fields[2:]] if len(fields) == 6 else [0.0, 0.0, 0.0, 0.0]
            if not (0.0 < noise < 0.05) or any(abs(o) > 0.1 for o in offsets):
                raise ValueError("noise_s must be in (0, 0.05), offsets within +-0.1 s")
            concat_parameters.append(
                {
                    "matching_strategy.type": "advanced",
                    "matching_strategy.lidar_timestamp_offsets": offsets,
                    "matching_strategy.lidar_timestamp_noise_window": [noise] * 4,
                }
            )
            print(f"[NERO_LIDAR_CONCAT_MATCH] advanced matching, noise {noise} s, offsets {offsets}")
        except (ValueError, IndexError) as e:
            print(f"[NERO_LIDAR_CONCAT_MATCH] '{nero_match}' ignored ({e}): the param file's matching as is")

    # [nero 2026-11-05 jolt] NERO_LIDAR_MOTION_3D=1: bring each scan from its own reference time (stamp + its first point's
    # time: the distortion corrector's reference) to the group's stamp in 3D with the IMU's roll / pitch / yaw rate
    # (nero_imu_3d) instead of speed + yaw rate between the stamps. Unset / any other value: as is.
    concat_remappings = [
        ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
        ("output", "concatenated/pointcloud"),
        ("output_info", "concatenated/pointcloud_info"),
    ]
    # [jolt-6] the front E1R is taken at its STAMP: its time_stamp starts 13.3 ms after the stamp in every frame (since 11-03)
    # while its corrected cloud lines up with the other lidars at the stamp (recorded drives, 0..+5 ms); the Airys line up at
    # stamp + first point.
    if os.environ.get("NERO_LIDAR_MOTION_3D", "").strip() == "1":
        concat_parameters.append(
            {
                "nero_imu_3d": True,
                "nero_imu_3d_header_reference_topics": ["/sensing/lidar/front/pointcloud_before_sync"],
            }
        )
        concat_remappings.append(("~/input/imu", "/sensing/imu/imu_data"))
        print("[NERO_LIDAR_MOTION_3D] 3D motion compensation in the concatenation (IMU)")

    # set concat filter as a component
    concat_component = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_data",
        remappings=concat_remappings,
        parameters=concat_parameters,
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        composable_node_descriptions=[concat_component],
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_concat_filter")),
    )

    return [concat_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    fairy_e1r_airy_left_right_sensor_kit_launch_share_dir = get_package_share_directory("fairy_e1r_airy_left_right_sensor_kit_launch")

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")
    add_launch_arg(
        "concatenate_and_time_sync_node_param_path",
        os.path.join(
            fairy_e1r_airy_left_right_sensor_kit_launch_share_dir,
            "config",
            "concatenate_and_time_sync_node.param.yaml",
        ),
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
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
