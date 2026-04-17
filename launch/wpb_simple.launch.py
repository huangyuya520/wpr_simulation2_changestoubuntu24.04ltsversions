#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

WORLD_READY_DELAY = 4.0
SCENE_OBJECT_DELAY = 6.0


def generate_launch_description():
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("wpr_simulation2"), "launch", "world.launch.py"])
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "worlds", "wpb_simple.world"]
            )
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_wpb_lidar.launch.py"]
            )
        ),
        launch_arguments={"spawn_delay": str(WORLD_READY_DELAY)}.items(),
    )

    spawn_bookshelft = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": "default",
                "file": PathJoinSubstitution(
                    [FindPackageShare("wpr_simulation2"), "models", "bookshelft.model"]
                ),
                "name": "bookshelft_01",
                "allow_renaming": True,
                "x": 3.0,
                "y": 0.0,
                "Y": 3.1415926,
            }
        ],
    )

    return LaunchDescription(
        [
            world,
            spawn_robot,
            TimerAction(period=SCENE_OBJECT_DELAY, actions=[spawn_bookshelft]),
        ]
    )
