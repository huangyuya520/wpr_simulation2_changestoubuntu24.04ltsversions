#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("wpr_simulation2"), "launch", "world.launch.py"])
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "worlds", "light.world"]
            )
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_wpb_head_up.launch.py"]
            )
        ),
        launch_arguments={
            "pose_x": "0.0",
            "pose_y": "0.0",
            "pose_theta": "3.1415926",
        }.items(),
    )

    spawn_kai = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_kai",
        output="screen",
        parameters=[
            {
                "world": "default",
                "file": os.path.join(
                    get_package_share_directory("wpr_simulation2"),
                    "models",
                    "kai_standing.model",
                ),
                "name": "kai",
                "allow_renaming": False,
                "x": -2.0,
                "y": 0.0,
                "z": 0.0,
                "Y": 1.57,
            }
        ],
    )

    return LaunchDescription(
        [
            world,
            spawn_robot,
            TimerAction(period=2.0, actions=[spawn_kai]),
        ]
    )
