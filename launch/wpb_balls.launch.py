#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_ball(entity_name, x, y):
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{entity_name}",
        output="screen",
        parameters=[
            {
                "world": "default",
                "file": os.path.join(
                    get_package_share_directory("wpr_simulation2"),
                    "models",
                    "balls",
                    f"{entity_name}.model",
                ),
                "name": entity_name,
                "allow_renaming": False,
                "x": x,
                "y": y,
                "z": 0.0,
                "Y": 0.0,
            }
        ],
    )


def ball_planar_move(entity_name, x, y):
    return Node(
        package="wpr_simulation2",
        executable="gz_planar_move.py",
        name=f"{entity_name}_planar_move",
        output="screen",
        parameters=[
            {
                "entity_name": entity_name,
                "world_name": "default",
                "cmd_vel_topic": f"/{entity_name}/cmd_vel",
                "odom_topic": f"/{entity_name}/odom",
                "publish_odom": False,
                "publish_odom_tf": False,
                "odom_frame": f"{entity_name}/odom",
                "base_frame": f"{entity_name}/base_link",
                "initial_x": x,
                "initial_y": y,
                "initial_z": 0.0,
                "initial_yaw": 0.0,
            }
        ],
    )


def ball_random_move(entity_name):
    return Node(
        package="wpr_simulation2",
        executable="ball_random_move",
        name=f"{entity_name}_random_move",
        output="screen",
        arguments=[entity_name],
    )


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
            PathJoinSubstitution([FindPackageShare("wpr_simulation2"), "launch", "spawn_wpb.launch.py"])
        )
    )

    spawn_orange_ball = spawn_ball("orange_ball", 2.0, 0.0)
    spawn_red_ball = spawn_ball("red_ball", 2.0, 0.5)
    spawn_green_ball = spawn_ball("green_ball", 2.0, -0.5)
    spawn_blue_ball = spawn_ball("blue_ball", 3.0, 0.0)

    orange_planar_move = ball_planar_move("orange_ball", 2.0, 0.0)
    red_planar_move = ball_planar_move("red_ball", 2.0, 0.5)
    green_planar_move = ball_planar_move("green_ball", 2.0, -0.5)
    blue_planar_move = ball_planar_move("blue_ball", 3.0, 0.0)

    orange_random_move = ball_random_move("orange_ball")
    red_random_move = ball_random_move("red_ball")
    green_random_move = ball_random_move("green_ball")
    blue_random_move = ball_random_move("blue_ball")

    return LaunchDescription(
        [
            world,
            spawn_robot,
            TimerAction(
                period=2.0,
                actions=[
                    spawn_orange_ball,
                    spawn_red_ball,
                    spawn_green_ball,
                    spawn_blue_ball,
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    orange_planar_move,
                    red_planar_move,
                    green_planar_move,
                    blue_planar_move,
                    orange_random_move,
                    red_random_move,
                    green_random_move,
                    blue_random_move,
                ],
            ),
        ]
    )
