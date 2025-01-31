#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    package_share_directory = get_package_share_directory("doom_ros")
    config_file_path = f"{package_share_directory}/config/doom_ros.yaml"
    return LaunchDescription(
        [
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="rqt_image_view",
                arguments=["/doom_image"],
            ),
            Node(
                package="joy",
                executable="game_controller_node",
                name="joy",
            ),
            Node(
                package="doom_ros",
                executable="doom_ros.py",
                name="doom_ros_node",
                parameters=[config_file_path],
            ),
        ]
    )
