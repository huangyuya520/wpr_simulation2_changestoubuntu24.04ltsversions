#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def spawn_model(model_name, entity_name, x, y, yaw=0.0):
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": "default",
                "file": os.path.join(
                    get_package_share_directory("wpr_simulation2"), "models", model_name
                ),
                "name": entity_name,
                "allow_renaming": True,
                "x": x,
                "y": y,
                "z": 0.0,
                "Y": yaw,
            }
        ],
    )


def generate_launch_description():
    spawn_man_1 = spawn_model("man_1.model", "man_1", 4.0, 1.0, yaw=3.1415926)
    spawn_man_2 = spawn_model("man_2.model", "man_2", 4.0, 1.0, yaw=3.1415926)
    spawn_woman = spawn_model("woman.model", "woman", 4.0, 1.0, yaw=3.1415926)

    return LaunchDescription(
        [
            TimerAction(period=6.0, actions=[spawn_man_1]),
            TimerAction(period=7.0, actions=[spawn_man_2]),
            TimerAction(period=8.0, actions=[spawn_woman]),
        ]
    )
