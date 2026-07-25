#!/usr/bin/env python3
"""Relay LaserScan messages from best-effort input to reliable output."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LaserScanQosRelay(Node):
    def __init__(self) -> None:
        super().__init__("m07_laserscan_qos_relay")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("publish_every_n", 1)
        self.declare_parameter("drop_after_sec", -1.0)
        self.declare_parameter("drop_duration_sec", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.publish_every_n = int(self.get_parameter("publish_every_n").value)
        self.drop_after_sec = float(self.get_parameter("drop_after_sec").value)
        self.drop_duration_sec = float(self.get_parameter("drop_duration_sec").value)
        if self.publish_every_n < 1 or self.drop_duration_sec < 0.0:
            raise ValueError("publish_every_n must be >= 1 and drop_duration_sec must be >= 0")
        self.started = time.monotonic()
        self.received = 0

        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(LaserScan, output_topic, reliable_qos)
        self.create_subscription(LaserScan, input_topic, self._relay, best_effort_qos)
        self.get_logger().info(
            f"Relaying LaserScan {input_topic} best-effort input to {output_topic} reliable output."
        )

    def _relay(self, message: LaserScan) -> None:
        self.received += 1
        elapsed = time.monotonic() - self.started
        in_dropout = (
            self.drop_after_sec >= 0.0
            and self.drop_after_sec <= elapsed < self.drop_after_sec + self.drop_duration_sec
        )
        if in_dropout or self.received % self.publish_every_n:
            return
        self.publisher.publish(message)


def main() -> int:
    rclpy.init()
    node = LaserScanQosRelay()
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
