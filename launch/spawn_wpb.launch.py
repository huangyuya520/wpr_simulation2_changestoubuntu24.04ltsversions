#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("wpr_simulation2"), "models", "wpb_home.model"]
                ),
            ),
            DeclareLaunchArgument("entity_name", default_value="wpb_home"),
            DeclareLaunchArgument("pose_x", default_value="0.0"),
            DeclareLaunchArgument("pose_y", default_value="0.0"),
            DeclareLaunchArgument("pose_theta", default_value="0.0"),
            DeclareLaunchArgument("spawn_delay", default_value="3.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("wpr_simulation2"),
                            "launch",
                            "spawn_wpb_mobile.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "model_file": LaunchConfiguration("model_file"),
                    "entity_name": LaunchConfiguration("entity_name"),
                    "pose_x": LaunchConfiguration("pose_x"),
                    "pose_y": LaunchConfiguration("pose_y"),
                    "pose_theta": LaunchConfiguration("pose_theta"),
                    "spawn_delay": LaunchConfiguration("spawn_delay"),
                }.items(),
            ),
        ]
    )
