# Copyright 2026 Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="autoware_intersection_priority",
            executable="autoware_intersection_priority_node",
            name="autoware_intersection_priority",
            output="screen",
        ),
    ])
