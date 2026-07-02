#!/usr/bin/env python3
"""Inspect available odometry inputs without publishing anything."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


TOPICS_OF_INTEREST = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    "/spot/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
}

FRAMES_OF_INTEREST = [
    "world",
    "body",
    "base_link",
    "sensor",
    "os1_frame",
    "odom",
    "map",
]

CHAINS_OF_INTEREST = [
    ("world", "body"),
    ("body", "base_link"),
    ("sensor", "os1_frame"),
    ("odom", "base_link"),
    ("map", "odom"),
]


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
class InspectionState:
    pointcloud_seen: bool = False
    scan_seen: bool = False
    scan_frame_id: str | None = None
    odom_messages: dict[str, int] = field(default_factory=lambda: defaultdict(int))
    odom_frames: dict[str, set[str]] = field(default_factory=lambda: defaultdict(set))
    tf_messages_seen: int = 0
    tf_static_messages_seen: int = 0
    dynamic_edges: set[tuple[str, str]] = field(default_factory=set)
    static_edges: set[tuple[str, str]] = field(default_factory=set)

    @property
    def edges(self) -> set[tuple[str, str]]:
        return self.dynamic_edges | self.static_edges

    @property
    def frames(self) -> set[str]:
        frames = set()
        for parent, child in self.edges:
            frames.add(parent)
            frames.add(child)
        return frames


class OdometryInputInspector(Node):
    def __init__(self) -> None:
        super().__init__("m05_odometry_input_inspector")
        self.state = InspectionState()
        self._odom_subscriptions = []
        self._odom_topic_names = set()

        self.volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(PointCloud2, "/spot/lidar/points", self._on_pointcloud, self.volatile_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, self.volatile_qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, self.volatile_qos)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, static_tf_qos)
        self.refresh_odom_subscriptions()

    def refresh_odom_subscriptions(self) -> None:
        for topic_name, topic_types in self.get_topic_names_and_types():
            if "nav_msgs/msg/Odometry" in topic_types and topic_name not in self._odom_topic_names:
                subscription = self.create_subscription(
                    Odometry,
                    topic_name,
                    self._odom_callback(topic_name),
                    self.volatile_qos,
                )
                self._odom_subscriptions.append(subscription)
                self._odom_topic_names.add(topic_name)

    def _on_pointcloud(self, _: PointCloud2) -> None:
        self.state.pointcloud_seen = True

    def _on_scan(self, message: LaserScan) -> None:
        self.state.scan_seen = True
        if self.state.scan_frame_id is None:
            self.state.scan_frame_id = message.header.frame_id

    def _on_tf(self, message: TFMessage) -> None:
        self.state.tf_messages_seen += 1
        self._collect_tf_edges(message, self.state.dynamic_edges)

    def _on_tf_static(self, message: TFMessage) -> None:
        self.state.tf_static_messages_seen += 1
        self._collect_tf_edges(message, self.state.static_edges)

    def _odom_callback(self, topic_name: str):
        def callback(message: Odometry) -> None:
            self.state.odom_messages[topic_name] += 1
            if message.header.frame_id:
                self.state.odom_frames[topic_name].add(message.header.frame_id)
            if message.child_frame_id:
                self.state.odom_frames[topic_name].add(message.child_frame_id)

        return callback

    @staticmethod
    def _collect_tf_edges(message: TFMessage, edge_set: set[tuple[str, str]]) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id.strip()
            child = transform.child_frame_id.strip()
            if parent and child:
                edge_set.add((parent, child))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect Milestone #5 odometry inputs without publishing anything.",
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


def graph_has_chain(edges: set[tuple[str, str]], source: str, target: str) -> bool:
    graph: dict[str, set[str]] = defaultdict(set)
    for parent, child in edges:
        graph[parent].add(child)
        graph[child].add(parent)

    queue = deque([source])
    visited = {source}
    while queue:
        current = queue.popleft()
        if current == target:
            return True
        for next_frame in graph[current]:
            if next_frame not in visited:
                visited.add(next_frame)
                queue.append(next_frame)
    return False


def check_topics(topic_types: dict[str, list[str]]) -> list[CheckResult]:
    results = []
    for topic, expected_type in TOPICS_OF_INTEREST.items():
        observed_types = topic_types.get(topic)
        if not observed_types:
            level = Level.WARN if topic in {"/odom", "/scan"} else Level.FAIL
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


def check_runtime_state(state: InspectionState) -> list[CheckResult]:
    results = []
    results.append(
        CheckResult(
            Level.PASS if state.pointcloud_seen else Level.WARN,
            "/spot/lidar/points",
            "PointCloud2 messages received" if state.pointcloud_seen else "no messages received",
        )
    )
    results.append(
        CheckResult(
            Level.PASS if state.scan_seen else Level.WARN,
            "/scan",
            (
                f"LaserScan messages received with frame_id {state.scan_frame_id}"
                if state.scan_seen
                else "no LaserScan messages received; Milestone #4 conversion may be inactive"
            ),
        )
    )
    results.append(
        CheckResult(
            Level.PASS if state.tf_messages_seen else Level.FAIL,
            "/tf",
            f"received {state.tf_messages_seen} messages" if state.tf_messages_seen else "no TF messages received",
        )
    )
    results.append(
        CheckResult(
            Level.PASS if state.tf_static_messages_seen else Level.FAIL,
            "/tf_static",
            (
                f"received {state.tf_static_messages_seen} messages"
                if state.tf_static_messages_seen
                else "no static TF messages received; Milestone #4 aliases may be missing"
            ),
        )
    )

    if state.odom_messages:
        for topic_name in sorted(state.odom_messages):
            frames = ", ".join(sorted(state.odom_frames[topic_name])) or "no frame ids observed"
            results.append(
                CheckResult(
                    Level.PASS,
                    topic_name,
                    f"received {state.odom_messages[topic_name]} Odometry messages; frames: {frames}",
                )
            )
    else:
        results.append(
            CheckResult(
                Level.WARN,
                "Odometry topics",
                "no nav_msgs/msg/Odometry messages received",
            )
        )
    return results


def check_frames(state: InspectionState) -> list[CheckResult]:
    results = []
    for frame in FRAMES_OF_INTEREST:
        if frame in state.frames:
            results.append(CheckResult(Level.PASS, frame, "observed in TF"))
        else:
            level = Level.WARN if frame in {"odom", "map"} else Level.FAIL
            results.append(CheckResult(level, frame, "not observed in TF"))
    return results


def check_chains(state: InspectionState) -> list[CheckResult]:
    results = []
    for parent, child in CHAINS_OF_INTEREST:
        if graph_has_chain(state.edges, parent, child):
            results.append(CheckResult(Level.PASS, f"{parent} -> {child}", "TF chain observed"))
        else:
            level = Level.WARN if parent in {"odom", "map"} else Level.FAIL
            results.append(CheckResult(level, f"{parent} -> {child}", "TF chain not observed"))
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
    node = OdometryInputInspector()
    start_time = node.get_clock().now()

    try:
        while rclpy.ok():
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1_000_000_000
            if elapsed >= args.duration:
                break
            node.refresh_odom_subscriptions()
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)

        topic_results = check_topics(topic_type_map(node))
        runtime_results = check_runtime_state(node.state)
        frame_results = check_frames(node.state)
        chain_results = check_chains(node.state)

        print("Milestone #5 Odometry Input Inspection")
        print(f"Collection duration: {args.duration:.1f} seconds")
        print("This tool is read-only and does not publish topics or TF.")

        worst = Level.PASS
        worst = max(worst, print_section("Topic type checks", topic_results))
        worst = max(worst, print_section("Runtime message checks", runtime_results))
        worst = max(worst, print_section("TF frame checks", frame_results))
        worst = max(worst, print_section("TF chain checks", chain_results))

        if node.state.edges:
            print("\nObserved TF edges")
            for parent, child in sorted(node.state.edges):
                print(f"  {parent} -> {child}")

        print("\nStrategy note")
        print("  Missing /odom or odom -> base_link is WARN in inspection mode.")
        print("  Do not treat this as full SLAM readiness until a defensible odometry source exists.")

        print(f"\nOverall result: {worst.name}")
        return 1 if worst == Level.FAIL else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
