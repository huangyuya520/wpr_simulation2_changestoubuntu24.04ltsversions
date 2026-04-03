#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pose_x = LaunchConfiguration("pose_x")
    pose_y = LaunchConfiguration("pose_y")
    pose_theta = LaunchConfiguration("pose_theta")
    controller_params = PathJoinSubstitution(
        [FindPackageShare("wpr_simulation2"), "config", "wpb_home_controller.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("wpr_simulation2"), "models", "wpb_home_mani.model"]
            ),
            " ",
            "controller_config:=",
            controller_params,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/kinect2/qhd/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/kinect2/sd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "world": "default",
                "topic": "robot_description",
                "name": "wpb_home_mani",
                "allow_renaming": False,
                "x": pose_x,
                "y": pose_y,
                "z": 0.0,
                "Y": pose_theta,
            }
        ],
    )

    planar_move = Node(
        package="wpr_simulation2",
        executable="gz_planar_move.py",
        name="wpb_home_planar_move",
        output="screen",
        parameters=[
            {
                "entity_name": "wpb_home_mani",
                "world_name": "default",
                "initial_x": pose_x,
                "initial_y": pose_y,
                "initial_yaw": pose_theta,
                "initial_z": 0.0,
            }
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    manipulator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "--param-file", controller_params],
        output="screen",
    )

    wpb_home_mani_sim = Node(
        package="wpr_simulation2",
        executable="wpb_home_mani_sim",
        name="wpb_home_mani_sim",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("pose_x", default_value="0.0"),
            DeclareLaunchArgument("pose_y", default_value="0.0"),
            DeclareLaunchArgument("pose_theta", default_value="0.0"),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[planar_move, joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[manipulator_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=manipulator_controller_spawner,
                    on_exit=[wpb_home_mani_sim],
                )
            ),
            bridge,
            robot_state_publisher,
            spawn_robot,
        ]
    )
