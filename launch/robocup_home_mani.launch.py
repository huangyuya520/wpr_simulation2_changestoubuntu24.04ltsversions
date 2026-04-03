#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("wpr_simulation2"), "launch", "world.launch.py"])
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "worlds", "robocup_home.world"]
            )
        }.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_wpb_mani.launch.py"]
            )
        ),
        launch_arguments={"pose_x": "-4.0", "pose_y": "-0.5", "pose_theta": "0.0"}.items(),
    )

    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_objects.launch.py"]
            )
        )
    )

    spawn_persons = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "launch", "spawn_persons.launch.py"]
            )
        )
    )

    return LaunchDescription([world, spawn_robot, spawn_objects, spawn_persons])
