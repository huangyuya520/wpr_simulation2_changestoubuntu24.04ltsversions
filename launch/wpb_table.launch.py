#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

WORLD_READY_DELAY = 4.0
SCENE_OBJECT_DELAY = 6.0


def spawn_model(relative_path, entity_name, x, y, z=0.0, yaw=0.0):
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{entity_name}",
        output="screen",
        parameters=[
            {
                "world": "default",
                "file": os.path.join(
                    get_package_share_directory("wpr_simulation2"), *relative_path
                ),
                "name": entity_name,
                "allow_renaming": False,
                "x": x,
                "y": y,
                "z": z,
                "Y": yaw,
            }
        ],
    )


def generate_launch_description():
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("wpr_simulation2"), "launch", "world.launch.py"])
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "worlds", "table.world"]
            )
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_wpb_mani.launch.py"]
            )
        ),
        launch_arguments={"spawn_delay": str(WORLD_READY_DELAY)}.items(),
    )

    spawn_table = spawn_model(("models", "table.model"), "table", 1.2, 0.0)
    spawn_red_bottle = spawn_model(
        ("models", "bottles", "red_bottle.model"), "red_bottle", 1.1, 0.3, z=2.0
    )
    spawn_green_bottle = spawn_model(
        ("models", "bottles", "green_bottle.model"), "green_bottle", 1.1, -0.2, z=2.0
    )

    return LaunchDescription(
        [
            world,
            spawn_robot,
            TimerAction(
                period=SCENE_OBJECT_DELAY,
                actions=[spawn_table, spawn_red_bottle, spawn_green_bottle],
            ),
        ]
    )
