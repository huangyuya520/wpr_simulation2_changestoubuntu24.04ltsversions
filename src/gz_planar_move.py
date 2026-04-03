#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class GzPlanarMove(Node):
    def __init__(self) -> None:
        super().__init__("gz_planar_move")

        self.declare_parameter("entity_name", "wpb_home_mani")
        self.declare_parameter("world_name", "default")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.entity_name = self.get_parameter("entity_name").value
        self.world_name = self.get_parameter("world_name").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_odom = self.get_parameter("publish_odom").value
        self.publish_odom_tf = self.get_parameter("publish_odom_tf").value
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.cmd_vel_timeout = Duration(
            seconds=float(self.get_parameter("cmd_vel_timeout").value)
        )

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.z = float(self.get_parameter("initial_z").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)

        self.cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.pending_request = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        service_name = f"/world/{self.world_name}/set_pose"
        self.set_pose_client = self.create_client(SetEntityPose, service_name)
        self.create_timer(1.0 / self.update_rate, self.on_timer)

    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        self.last_update_time = now
        cmd = Twist()
        if now - self.last_cmd_time <= self.cmd_vel_timeout:
            cmd = self.cmd_vel

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        world_vx = cmd.linear.x * cos_yaw - cmd.linear.y * sin_yaw
        world_vy = cmd.linear.x * sin_yaw + cmd.linear.y * cos_yaw

        self.x += world_vx * dt
        self.y += world_vy * dt
        self.yaw += cmd.angular.z * dt

        self.send_pose_request()
        self.publish_odom(now, cmd)

    def send_pose_request(self) -> None:
        if not self.set_pose_client.service_is_ready():
            return

        if self.pending_request is not None and not self.pending_request.done():
            return

        request = SetEntityPose.Request()
        request.entity = Entity(name=self.entity_name, type=Entity.MODEL)
        request.pose.position.x = self.x
        request.pose.position.y = self.y
        request.pose.position.z = self.z
        (
            request.pose.orientation.x,
            request.pose.orientation.y,
            request.pose.orientation.z,
            request.pose.orientation.w,
        ) = quaternion_from_yaw(self.yaw)

        self.pending_request = self.set_pose_client.call_async(request)

    def publish_odom(self, now, cmd: Twist) -> None:
        quat = quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist = cmd
        if self.publish_odom:
            self.odom_pub.publish(odom)

        if not self.publish_odom_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzPlanarMove()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
