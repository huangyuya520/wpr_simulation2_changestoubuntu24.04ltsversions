#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _continue_on_success(success_actions, process_label):
    def _handler(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg=f"[wpr_simulation2] {process_label} completed successfully."),
                *success_actions,
            ]

        return [
            LogInfo(
                msg=(
                    f"[wpr_simulation2] {process_label} failed with exit code "
                    f"{event.returncode}. Stopping remaining startup actions."
                )
            ),
            Shutdown(reason=f"{process_label} failed"),
        ]

    return _handler


def generate_launch_description():
    model_file = LaunchConfiguration("model_file")
    entity_name = LaunchConfiguration("entity_name")
    pose_x = LaunchConfiguration("pose_x")
    pose_y = LaunchConfiguration("pose_y")
    pose_theta = LaunchConfiguration("pose_theta")
    spawn_delay = LaunchConfiguration("spawn_delay")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            model_file,
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
                "name": entity_name,
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
        name="wpb_mobile_planar_move",
        output="screen",
        parameters=[
            {
                "entity_name": entity_name,
                "world_name": "default",
                "initial_x": pose_x,
                "initial_y": pose_y,
                "initial_yaw": pose_theta,
                "initial_z": 0.0,
            }
        ],
    )

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
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=_continue_on_success([planar_move], "robot spawn"),
                )
            ),
            bridge,
            robot_state_publisher,
            # Give Gazebo a short warm-up window so model creation doesn't race the world startup.
            TimerAction(period=spawn_delay, actions=[spawn_robot]),
        ]
    )
