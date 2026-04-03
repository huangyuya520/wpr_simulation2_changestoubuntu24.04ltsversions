#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def spawn_model(relative_path, entity_name, x, y, z=0.0, yaw=0.0):
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": "default",
                "file": os.path.join(
                    get_package_share_directory("wpr_simulation2"), *relative_path
                ),
                "name": entity_name,
                "allow_renaming": True,
                "x": x,
                "y": y,
                "z": z,
                "Y": yaw,
            }
        ],
    )


def generate_launch_description():
    spawn_bed = spawn_model(("models", "bed.model"), "bed", 5.0, -3.9, yaw=3.1415926)
    spawn_sofa = spawn_model(("models", "sofa.model"), "sofa", -1.0, -3.9, yaw=1.57)
    spawn_tea_table = spawn_model(
        ("models", "tea_table.model"), "tea_table", -2.1, -2.2, yaw=1.57
    )
    spawn_bookshelft = spawn_model(
        ("models", "bookshelft.model"), "bookshelft", 2.0, -0.55, yaw=-1.57
    )

    spawn_kitchen_table = spawn_model(
        ("models", "table.model"), "kitchen_table", -3.5, 3.7, yaw=1.57
    )
    spawn_red_bottle = spawn_model(
        ("models", "bottles", "red_bottle.model"), "red_bottle", -3.3, 3.55, z=2.0
    )
    spawn_green_bottle = spawn_model(
        ("models", "bottles", "green_bottle.model"), "green_bottle", -3.6, 3.55, z=2.0
    )

    spawn_cupboard_0 = spawn_model(
        ("models", "cupboard.model"), "cupboard_0", -2.0, 0.7, yaw=1.57
    )
    spawn_cupboard_1 = spawn_model(
        ("models", "cupboard.model"), "cupboard_1", -1.3, 3.7, yaw=-1.57
    )

    spawn_dinning_table_0 = spawn_model(
        ("models", "table.model"), "dinning_table_0", 1.5, 1.5, yaw=1.57
    )
    spawn_dinning_table_1 = spawn_model(
        ("models", "table.model"), "dinning_table_1", 1.5, 2.0, yaw=1.57
    )
    spawn_dinning_table_2 = spawn_model(
        ("models", "table.model"), "dinning_table_2", 2.7, 1.5, yaw=1.57
    )
    spawn_dinning_table_3 = spawn_model(
        ("models", "table.model"), "dinning_table_3", 2.7, 2.0, yaw=1.57
    )

    spawn_chair_0 = spawn_model(
        ("models", "chair.model"), "chair_0", 1.5, 1.2, yaw=1.57
    )
    spawn_chair_1 = spawn_model(
        ("models", "chair.model"), "chair_1", 1.5, 2.3, yaw=-1.57
    )
    spawn_chair_2 = spawn_model(
        ("models", "chair.model"), "chair_2", 2.7, 1.2, yaw=1.57
    )
    spawn_chair_3 = spawn_model(
        ("models", "chair.model"), "chair_3", 2.7, 2.3, yaw=-1.57
    )

    return LaunchDescription(
        [
            TimerAction(
                period=1.0,
                actions=[spawn_bed, spawn_sofa, spawn_tea_table, spawn_bookshelft],
            ),
            TimerAction(
                period=2.0,
                actions=[spawn_kitchen_table, spawn_cupboard_0, spawn_cupboard_1],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    spawn_dinning_table_0,
                    spawn_dinning_table_1,
                    spawn_dinning_table_2,
                    spawn_dinning_table_3,
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[spawn_chair_0, spawn_chair_1, spawn_chair_2, spawn_chair_3],
            ),
            TimerAction(period=5.0, actions=[spawn_red_bottle, spawn_green_bottle]),
        ]
    )
