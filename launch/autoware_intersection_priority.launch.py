# Copyright 2026 Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="autoware_intersection_priority",
            executable="autoware_intersection_priority_node",
            name="autoware_intersection_priority",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("autoware_intersection_priority"),
                "config",
                "autoware_intersection_priority.param.yaml",
            ])],
        ),
    ])
