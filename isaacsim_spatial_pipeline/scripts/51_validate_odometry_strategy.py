#!/usr/bin/env python3
"""Validate the Milestone #5 odometry strategy without assuming SLAM success."""

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
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


REQUIRED_TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
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


class OdometryStrategyValidator(Node):
    def __init__(self) -> None:
        super().__init__("m05_odometry_strategy_validator")
        self.state = ValidationState()
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
        description="Validate Milestone #5 odometry strategy status.",
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


def check_required_topics(topic_types: dict[str, list[str]]) -> list[CheckResult]:
    results = []
    for topic, expected_type in REQUIRED_TOPIC_TYPES.items():
        observed_types = topic_types.get(topic)
        if not observed_types:
            results.append(CheckResult(Level.FAIL, topic, "topic not observed"))
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

    scan_types = topic_types.get("/scan")
    if not scan_types:
        results.append(
            CheckResult(
                Level.WARN,
                "/scan",
                "topic not observed; expected only when Milestone #4 conversion is active",
            )
        )
    elif "sensor_msgs/msg/LaserScan" in scan_types:
        results.append(CheckResult(Level.PASS, "/scan", "type sensor_msgs/msg/LaserScan"))
    else:
        results.append(
            CheckResult(
                Level.FAIL,
                "/scan",
                f"expected sensor_msgs/msg/LaserScan, observed {', '.join(scan_types)}",
            )
        )

    odom_types = topic_types.get("/odom")
    if not odom_types:
        results.append(CheckResult(Level.WARN, "/odom", "topic not observed"))
    elif "nav_msgs/msg/Odometry" in odom_types:
        results.append(CheckResult(Level.PASS, "/odom", "type nav_msgs/msg/Odometry"))
    else:
        results.append(
            CheckResult(
                Level.FAIL,
                "/odom",
                f"expected nav_msgs/msg/Odometry, observed {', '.join(odom_types)}",
            )
        )
    return results


def check_runtime_state(state: ValidationState) -> list[CheckResult]:
    results = []
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
                "/odom",
                "no nav_msgs/msg/Odometry messages received",
            )
        )
    return results


def check_frames_and_chains(state: ValidationState) -> list[CheckResult]:
    results = []
    for frame in ["base_link", "os1_frame"]:
        if frame in state.frames:
            source = "/tf_static" if any(child == frame for _, child in state.static_edges) else "/tf"
            results.append(CheckResult(Level.PASS, frame, f"observed through {source}"))
        else:
            results.append(CheckResult(Level.FAIL, frame, "Milestone #4 alias frame missing"))

    if "odom" in state.frames:
        results.append(CheckResult(Level.PASS, "odom", "frame observed in TF"))
    else:
        results.append(CheckResult(Level.WARN, "odom", "frame not observed in TF"))

    if "map" in state.frames:
        results.append(CheckResult(Level.PASS, "map", "frame observed in TF"))
    else:
        results.append(CheckResult(Level.WARN, "map", "allowed absent unless SLAM genuinely publishes it"))

    if graph_has_chain(state.edges, "odom", "base_link"):
        results.append(CheckResult(Level.PASS, "odom -> base_link", "TF chain observed"))
    else:
        results.append(
            CheckResult(
                Level.WARN,
                "odom -> base_link",
                "TF chain not observed; inspection-only strategy is not full odometry readiness",
            )
        )

    return results


def final_level(results: Iterable[CheckResult], state: ValidationState) -> Level:
    worst = max((result.level for result in results), default=Level.PASS)
    if worst == Level.FAIL:
        return Level.FAIL
    if graph_has_chain(state.edges, "odom", "base_link"):
        return Level.PASS
    return Level.WARN


def print_section(title: str, results: Iterable[CheckResult]) -> None:
    print(f"\n{title}")
    for result in results:
        print(f"  {result.level.name:<4} {result.label}: {result.detail}")


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = OdometryStrategyValidator()
    start_time = node.get_clock().now()

    try:
        while rclpy.ok():
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1_000_000_000
            if elapsed >= args.duration:
                break
            node.refresh_odom_subscriptions()
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)

        results = []
        topic_results = check_required_topics(topic_type_map(node))
        runtime_results = check_runtime_state(node.state)
        tf_results = check_frames_and_chains(node.state)
        results.extend(topic_results)
        results.extend(runtime_results)
        results.extend(tf_results)

        print("Milestone #5 Odometry Strategy Validation")
        print(f"Collection duration: {args.duration:.1f} seconds")
        print("This validator does not publish odometry, map, or localization transforms.")

        print_section("Topic type checks", topic_results)
        print_section("Runtime message checks", runtime_results)
        print_section("TF frame and chain checks", tf_results)

        print("\nContract notes")
        print("  PASS requires an observed odom -> base_link TF chain.")
        print("  WARN is expected for the current Milestone #3 replay if no odometry exists.")
        print("  /map and map are allowed to be absent unless SLAM genuinely publishes them.")

        overall = final_level(results, node.state)
        print(f"\nOverall result: {overall.name}")
        return 1 if overall == Level.FAIL else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
