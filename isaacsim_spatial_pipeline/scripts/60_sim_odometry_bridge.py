#!/usr/bin/env python3
"""Publish opt-in simulation-only odometry derived from live Isaac TF motion."""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


@dataclass
class PoseSample:
    stamp_sec: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw: float


@dataclass
class VelocitySample:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wz: float = 0.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


class SimOdometryBridge(Node):
    def __init__(self) -> None:
        super().__init__("m06_sim_odometry_bridge")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("enable_sim_odom", False)
        self.declare_parameter("publish_odom_topic", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("source_parent_frame", "world")
        self.declare_parameter("source_child_frame", "body")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("planar_only", True)

        self.enable_sim_odom = self.get_parameter("enable_sim_odom").value
        self.publish_odom_topic = self.get_parameter("publish_odom_topic").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.source_parent_frame = self.get_parameter("source_parent_frame").value
        self.source_child_frame = self.get_parameter("source_child_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.planar_only = self.get_parameter("planar_only").value

        self.latest_clock = None
        self.latest_pose: PoseSample | None = None
        self.previous_pose: PoseSample | None = None
        self.velocity = VelocitySample()
        self.warned_no_source_tf = False

        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Clock, "/clock", self._on_clock, volatile_qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, volatile_qos)

        self.odom_pub = None
        self.tf_pub = None
        if self.enable_sim_odom and self.publish_odom_topic:
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        if self.enable_sim_odom and self.publish_tf:
            self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)

        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.05
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().warn(
            "Milestone #6 odometry is simulation-only and derived from live Isaac TF; "
            "it is not real wheel or leg odometry."
        )
        if not self.enable_sim_odom:
            self.get_logger().warn(
                "enable_sim_odom is false; this node will observe world -> body but publish nothing."
            )
        else:
            self.get_logger().warn(
                "enable_sim_odom is true; publishing simulation-derived odometry only from observed TF motion."
            )
        self.get_logger().info(
            f"Source TF: {self.source_parent_frame} -> {self.source_child_frame}; "
            f"output: {self.odom_frame} -> {self.base_frame}, topic {self.odom_topic}"
        )

    def _on_clock(self, message: Clock) -> None:
        self.latest_clock = message.clock

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id.strip()
            child = transform.child_frame_id.strip()
            if parent == self.source_parent_frame and child == self.source_child_frame:
                self._update_pose(transform)

    def _update_pose(self, transform: TransformStamped) -> None:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp_sec = stamp_to_sec(transform.header.stamp)
        if stamp_sec <= 0.0 and self.latest_clock is not None:
            stamp_sec = stamp_to_sec(self.latest_clock)

        yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        z = 0.0 if self.planar_only else translation.z
        qx, qy, qz, qw = (
            quaternion_from_yaw(yaw)
            if self.planar_only
            else (rotation.x, rotation.y, rotation.z, rotation.w)
        )
        pose = PoseSample(
            stamp_sec=stamp_sec,
            x=translation.x,
            y=translation.y,
            z=z,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            yaw=yaw,
        )

        if self.latest_pose is not None:
            self.previous_pose = self.latest_pose
            self._update_velocity(self.previous_pose, pose)
        self.latest_pose = pose
        self.warned_no_source_tf = False

    def _update_velocity(self, previous: PoseSample, current: PoseSample) -> None:
        dt = current.stamp_sec - previous.stamp_sec
        if dt <= 1e-6:
            return
        self.velocity = VelocitySample(
            vx=(current.x - previous.x) / dt,
            vy=(current.y - previous.y) / dt,
            vz=(current.z - previous.z) / dt,
            wz=normalize_angle(current.yaw - previous.yaw) / dt,
        )

    def _on_timer(self) -> None:
        if self.latest_pose is None:
            if not self.warned_no_source_tf:
                self.get_logger().warn(
                    f"No {self.source_parent_frame} -> {self.source_child_frame} TF observed; publishing nothing."
                )
                self.warned_no_source_tf = True
            return

        if not self.enable_sim_odom:
            return

        if self.publish_odom_topic and self.odom_pub is not None:
            self.odom_pub.publish(self._make_odom_message())
        if self.publish_tf and self.tf_pub is not None:
            self.tf_pub.publish(TFMessage(transforms=[self._make_tf_message()]))

    def _make_odom_message(self) -> Odometry:
        pose = self.latest_pose
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = pose.z
        message.pose.pose.orientation.x = pose.qx
        message.pose.pose.orientation.y = pose.qy
        message.pose.pose.orientation.z = pose.qz
        message.pose.pose.orientation.w = pose.qw
        message.twist.twist.linear.x = self.velocity.vx
        message.twist.twist.linear.y = self.velocity.vy
        message.twist.twist.linear.z = self.velocity.vz
        message.twist.twist.angular.z = self.velocity.wz
        return message

    def _make_tf_message(self) -> TransformStamped:
        pose = self.latest_pose
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.translation.z = pose.z
        transform.transform.rotation.x = pose.qx
        transform.transform.rotation.y = pose.qy
        transform.transform.rotation.z = pose.qz
        transform.transform.rotation.w = pose.qw
        return transform


def main() -> int:
    rclpy.init()
    node = SimOdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
