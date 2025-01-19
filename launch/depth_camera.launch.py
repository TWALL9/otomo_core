import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share_dir = get_package_share_directory("realsense2_camera")

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share_dir, "launch", "rs_launch.py")]
        ),
        launch_arguments={
            "pointcloud.enable": "true",
            "enable_infra1": "true",
            "enable_infra2": "true",
        }.items(),
    )

    return LaunchDescription([realsense_node])
