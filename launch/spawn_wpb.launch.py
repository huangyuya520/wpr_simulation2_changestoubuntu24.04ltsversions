#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("pose_x", default_value="0.0"),
            DeclareLaunchArgument("pose_y", default_value="0.0"),
            DeclareLaunchArgument("pose_theta", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("wpr_simulation2"),
                            "launch",
                            "spawn_wpb_lidar.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "pose_x": LaunchConfiguration("pose_x"),
                    "pose_y": LaunchConfiguration("pose_y"),
                    "pose_theta": LaunchConfiguration("pose_theta"),
                }.items(),
            ),
        ]
    )
