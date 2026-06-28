#!/usr/bin/env python3
"""Validate Milestone #4 LiDAR SLAM bring-up topics and frames.

Run while the Milestone #3 bag is replaying with --clock and the Milestone #4
alias, pointcloud_to_laserscan, and slam_toolbox launch files are active.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


EXPECTED_TOPIC_TYPES = {
    "/scan": "sensor_msgs/msg/LaserScan",
    "/map": "nav_msgs/msg/OccupancyGrid",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
}

REQUIRED_FRAMES = {
    "base_link",
    "os1_frame",
}

SLAM_FRAMES = {
    "map",
    "odom",
}


class Level(IntEnum):
    PASS = 0
    WARN = 1
    FAIL = 2


@dataclass
class CheckResult:
    level: Level
    label: str
    detail: str


@dataclass
class ValidationState:
    scan_seen: bool = False
    scan_frame_id: str | None = None
    map_seen: bool = False
    tf_messages_seen: int = 0
    tf_static_messages_seen: int = 0
    dynamic_tf_frames: set[str] = field(default_factory=set)
    static_tf_frames: set[str] = field(default_factory=set)

    @property
    def tf_frames(self) -> set[str]:
        return self.dynamic_tf_frames | self.static_tf_frames


class SlamTopicValidator(Node):
    def __init__(self) -> None:
        super().__init__("m04_slam_topic_validator")
        self.state = ValidationState()

        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(LaserScan, "/scan", self._on_scan, volatile_qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, volatile_qos)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, static_tf_qos)

    def _on_scan(self, message: LaserScan) -> None:
        self.state.scan_seen = True
        if self.state.scan_frame_id is None:
            self.state.scan_frame_id = message.header.frame_id

    def _on_map(self, _: OccupancyGrid) -> None:
        self.state.map_seen = True

    def _on_tf(self, message: TFMessage) -> None:
        self.state.tf_messages_seen += 1
        self._collect_tf_frames(message, self.state.dynamic_tf_frames)

    def _on_tf_static(self, message: TFMessage) -> None:
        self.state.tf_static_messages_seen += 1
        self._collect_tf_frames(message, self.state.static_tf_frames)

    @staticmethod
    def _collect_tf_frames(message: TFMessage, frame_set: set[str]) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id.strip()
            child = transform.child_frame_id.strip()
            if parent:
                frame_set.add(parent)
            if child:
                frame_set.add(child)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate Milestone #4 /scan, /map, and SLAM TF frames.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=8.0,
        help="Seconds to collect messages before reporting.",
    )
    parser.add_argument(
        "--spin-timeout",
        type=float,
        default=0.1,
        help="ROS2 spin timeout in seconds.",
    )
    return parser.parse_args()


def topic_type_map(node: Node) -> dict[str, list[str]]:
    return {
        topic_name: topic_types
        for topic_name, topic_types in node.get_topic_names_and_types()
    }


def check_topic_types(topic_types: dict[str, list[str]]) -> list[CheckResult]:
    results = []
    for topic, expected_type in EXPECTED_TOPIC_TYPES.items():
        observed_types = topic_types.get(topic)
        if not observed_types:
            level = Level.FAIL if topic == "/scan" else Level.WARN
            results.append(CheckResult(level, topic, "topic not observed"))
        elif expected_type in observed_types:
            results.append(CheckResult(Level.PASS, topic, f"type {expected_type}"))
        else:
            results.append(
                CheckResult(
                    Level.FAIL,
                    topic,
                    f"expected {expected_type}, observed {', '.join(observed_types)}",
                )
            )
    return results


def check_runtime_state(state: ValidationState) -> list[CheckResult]:
    results = []
    if state.scan_seen:
        results.append(
            CheckResult(
                Level.PASS,
                "/scan",
                f"messages received with frame_id {state.scan_frame_id}",
            )
        )
    else:
        results.append(CheckResult(Level.FAIL, "/scan", "no LaserScan messages received"))

    if state.map_seen:
        results.append(CheckResult(Level.PASS, "/map", "OccupancyGrid messages received"))
    else:
        results.append(
            CheckResult(
                Level.WARN,
                "/map",
                "no map received yet; slam_toolbox may be waiting on odom/base TF",
            )
        )

    if state.tf_messages_seen:
        results.append(
            CheckResult(
                Level.PASS,
                "/tf",
                f"received {state.tf_messages_seen} messages",
            )
        )
    else:
        results.append(CheckResult(Level.FAIL, "/tf", "no TF messages received"))

    if state.tf_static_messages_seen:
        results.append(
            CheckResult(
                Level.PASS,
                "/tf_static",
                f"received {state.tf_static_messages_seen} messages",
            )
        )
    else:
        results.append(
            CheckResult(
                Level.WARN,
                "/tf_static",
                "no static TF messages received; alias frames may be missing",
            )
        )

    return results


def check_tf_frames(state: ValidationState) -> list[CheckResult]:
    results = []
    for frame in sorted(REQUIRED_FRAMES):
        if frame in state.tf_frames:
            source = "/tf_static" if frame in state.static_tf_frames else "/tf"
            results.append(
                CheckResult(
                    Level.PASS,
                    frame,
                    f"Milestone #4 alias frame observed on {source}",
                )
            )
        else:
            results.append(CheckResult(Level.FAIL, frame, "Milestone #4 alias frame missing"))

    for frame in sorted(SLAM_FRAMES):
        if frame in state.tf_frames:
            results.append(CheckResult(Level.PASS, frame, "SLAM frame observed"))
        else:
            results.append(
                CheckResult(
                    Level.WARN,
                    frame,
                    "not observed yet; expected only after SLAM publishes transforms",
                )
            )

    return results


def print_section(title: str, results: Iterable[CheckResult]) -> Level:
    print(f"\n{title}")
    worst = Level.PASS
    for result in results:
        worst = max(worst, result.level)
        print(f"  {result.level.name:<4} {result.label}: {result.detail}")
    return worst


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = SlamTopicValidator()
    start_time = node.get_clock().now()

    try:
        while rclpy.ok():
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1_000_000_000
            if elapsed >= args.duration:
                break
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)

        topic_results = check_topic_types(topic_type_map(node))
        runtime_results = check_runtime_state(node.state)
        tf_results = check_tf_frames(node.state)

        print("Milestone #4 LiDAR SLAM Bring-up Validation")
        print(f"Collection duration: {args.duration:.1f} seconds")

        worst = Level.PASS
        worst = max(worst, print_section("Topic type checks", topic_results))
        worst = max(worst, print_section("Runtime message checks", runtime_results))
        worst = max(worst, print_section("TF frame checks", tf_results))

        if node.state.tf_frames:
            print("\nObserved TF frames")
            for frame in sorted(node.state.tf_frames):
                print(f"  {frame}")

        print("\nContract notes")
        print("  /scan should come from /spot/lidar/points through pointcloud_to_laserscan.")
        print("  base_link and os1_frame are compatibility aliases, not Isaac frame replacements.")
        print("  map and odom are SLAM frames; they are not part of the Milestone #3 contract.")
        print("  Camera, depth, and IMU fusion are out of scope for Milestone #4.")

        print(f"\nOverall result: {worst.name}")
        return 1 if worst == Level.FAIL else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
