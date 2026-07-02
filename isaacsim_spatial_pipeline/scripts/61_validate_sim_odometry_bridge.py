#!/usr/bin/env python3
"""Validate the Milestone #6 live simulation odometry bridge."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


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
    clock_seen: bool = False
    scan_seen: bool = False
    scan_frame_ids: set[str] = field(default_factory=set)
    odom_messages: int = 0
    odom_frame_ids: set[str] = field(default_factory=set)
    odom_child_frame_ids: set[str] = field(default_factory=set)
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


class SimOdometryBridgeValidator(Node):
    def __init__(self, odom_topic: str) -> None:
        super().__init__("m06_sim_odometry_bridge_validator")
        self.state = ValidationState()
        self.odom_topic = odom_topic

        volatile_qos = QoSProfile(
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

        self.create_subscription(Clock, "/clock", self._on_clock, volatile_qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, volatile_qos)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, static_tf_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, volatile_qos)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, volatile_qos)

    def _on_clock(self, _: Clock) -> None:
        self.state.clock_seen = True

    def _on_scan(self, message: LaserScan) -> None:
        self.state.scan_seen = True
        if message.header.frame_id:
            self.state.scan_frame_ids.add(message.header.frame_id)

    def _on_odom(self, message: Odometry) -> None:
        self.state.odom_messages += 1
        if message.header.frame_id:
            self.state.odom_frame_ids.add(message.header.frame_id)
        if message.child_frame_id:
            self.state.odom_child_frame_ids.add(message.child_frame_id)

    def _on_tf(self, message: TFMessage) -> None:
        self.state.tf_messages_seen += 1
        self._collect_edges(message, self.state.dynamic_edges)

    def _on_tf_static(self, message: TFMessage) -> None:
        self.state.tf_static_messages_seen += 1
        self._collect_edges(message, self.state.static_edges)

    @staticmethod
    def _collect_edges(message: TFMessage, edge_set: set[tuple[str, str]]) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id.strip()
            child = transform.child_frame_id.strip()
            if parent and child:
                edge_set.add((parent, child))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the Milestone #6 simulation-only odometry bridge.",
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--spin-timeout", type=float, default=0.1)
    parser.add_argument("--bridge-enabled", action="store_true")
    parser.add_argument("--publish-tf-enabled", action="store_true")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--source-parent-frame", default="world")
    parser.add_argument("--source-child-frame", default="body")
    parser.add_argument("--scan-frame", default="os1_frame")
    return parser.parse_args()


def topic_type_map(node: Node) -> dict[str, list[str]]:
    return {
        topic_name: topic_types
        for topic_name, topic_types in node.get_topic_names_and_types()
    }


def edge_exists(edges: set[tuple[str, str]], parent: str, child: str) -> bool:
    return (parent, child) in edges


def check_topic_types(topic_types: dict[str, list[str]], args: argparse.Namespace) -> list[CheckResult]:
    results = []
    expected = {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
    }
    for topic, topic_type in expected.items():
        observed = topic_types.get(topic)
        if not observed:
            results.append(CheckResult(Level.FAIL, topic, "topic not observed"))
        elif topic_type in observed:
            results.append(CheckResult(Level.PASS, topic, f"type {topic_type}"))
        else:
            results.append(CheckResult(Level.FAIL, topic, f"expected {topic_type}, observed {', '.join(observed)}"))

    scan_types = topic_types.get("/scan")
    if not scan_types:
        results.append(CheckResult(Level.WARN, "/scan", "not observed; expected only when conversion is active"))
    elif "sensor_msgs/msg/LaserScan" in scan_types:
        results.append(CheckResult(Level.PASS, "/scan", "type sensor_msgs/msg/LaserScan"))
    else:
        results.append(CheckResult(Level.FAIL, "/scan", f"expected LaserScan, observed {', '.join(scan_types)}"))

    odom_types = topic_types.get(args.odom_topic)
    if not odom_types:
        level = Level.FAIL if args.bridge_enabled else Level.WARN
        detail = "not observed while bridge is enabled" if args.bridge_enabled else "not observed; bridge may be disabled"
        results.append(CheckResult(level, args.odom_topic, detail))
    elif "nav_msgs/msg/Odometry" in odom_types:
        level = Level.PASS if args.bridge_enabled else Level.WARN
        detail = (
            "type nav_msgs/msg/Odometry"
            if args.bridge_enabled
            else "Odometry observed while bridge is declared disabled; verify this is an external validated source"
        )
        results.append(CheckResult(level, args.odom_topic, detail))
    else:
        results.append(CheckResult(Level.FAIL, args.odom_topic, f"expected Odometry, observed {', '.join(odom_types)}"))

    return results


def check_runtime_state(state: ValidationState, args: argparse.Namespace) -> list[CheckResult]:
    results = [
        CheckResult(Level.PASS if state.clock_seen else Level.FAIL, "/clock", "messages received" if state.clock_seen else "no messages received"),
        CheckResult(Level.PASS if state.tf_messages_seen else Level.FAIL, "/tf", f"received {state.tf_messages_seen} messages" if state.tf_messages_seen else "no TF messages received"),
        CheckResult(Level.PASS if state.tf_static_messages_seen else Level.FAIL, "/tf_static", f"received {state.tf_static_messages_seen} messages" if state.tf_static_messages_seen else "no static TF messages received"),
    ]

    if state.scan_seen and args.scan_frame in state.scan_frame_ids:
        results.append(CheckResult(Level.PASS, "/scan", f"messages received with frame_id {args.scan_frame}"))
    elif state.scan_seen:
        results.append(CheckResult(Level.FAIL, "/scan", f"expected frame_id {args.scan_frame}, observed {', '.join(sorted(state.scan_frame_ids))}"))
    else:
        results.append(CheckResult(Level.WARN, "/scan", "no messages received; conversion may be inactive"))

    if state.odom_messages:
        frame_ok = state.odom_frame_ids == {args.odom_frame}
        child_ok = state.odom_child_frame_ids == {args.base_frame}
        if frame_ok and child_ok and args.bridge_enabled:
            results.append(CheckResult(Level.PASS, args.odom_topic, f"received {state.odom_messages} messages with correct frame IDs"))
        elif frame_ok and child_ok:
            results.append(
                CheckResult(
                    Level.WARN,
                    args.odom_topic,
                    (
                        f"received {state.odom_messages} messages with correct frame IDs while bridge is declared disabled; "
                        "verify this is an external validated source"
                    ),
                )
            )
        else:
            results.append(
                CheckResult(
                    Level.FAIL,
                    args.odom_topic,
                    (
                        f"expected header.frame_id {args.odom_frame} and child_frame_id {args.base_frame}; "
                        f"observed headers {sorted(state.odom_frame_ids)}, children {sorted(state.odom_child_frame_ids)}"
                    ),
                )
            )
    else:
        level = Level.FAIL if args.bridge_enabled else Level.WARN
        detail = "no Odometry messages while bridge is enabled" if args.bridge_enabled else "no Odometry messages; bridge disabled is allowed"
        results.append(CheckResult(level, args.odom_topic, detail))

    return results


def check_frames_and_edges(state: ValidationState, args: argparse.Namespace) -> list[CheckResult]:
    results = []
    if edge_exists(state.dynamic_edges, args.source_parent_frame, args.source_child_frame):
        results.append(CheckResult(Level.PASS, f"{args.source_parent_frame} -> {args.source_child_frame}", "source TF observed on /tf"))
    else:
        results.append(CheckResult(Level.FAIL, f"{args.source_parent_frame} -> {args.source_child_frame}", "source TF not observed on /tf"))

    if edge_exists(state.static_edges, "body", args.base_frame):
        results.append(CheckResult(Level.PASS, f"body -> {args.base_frame}", "Milestone #4 alias preserved on /tf_static"))
    else:
        results.append(CheckResult(Level.FAIL, f"body -> {args.base_frame}", "Milestone #4 alias missing"))

    if edge_exists(state.static_edges, "sensor", "os1_frame"):
        results.append(CheckResult(Level.PASS, "sensor -> os1_frame", "Milestone #4 alias preserved on /tf_static"))
    else:
        results.append(CheckResult(Level.FAIL, "sensor -> os1_frame", "Milestone #4 alias missing"))

    if edge_exists(state.dynamic_edges, args.odom_frame, args.base_frame):
        results.append(CheckResult(Level.PASS, f"{args.odom_frame} -> {args.base_frame}", "odometry TF observed on /tf"))
    else:
        level = Level.FAIL if args.bridge_enabled and args.publish_tf_enabled else Level.WARN
        detail = (
            "missing while bridge TF publishing is enabled"
            if level == Level.FAIL
            else "not observed; allowed when bridge or TF publishing is disabled"
        )
        results.append(CheckResult(level, f"{args.odom_frame} -> {args.base_frame}", detail))

    if "map" in state.frames or edge_exists(state.edges, "map", args.odom_frame):
        results.append(
            CheckResult(
                Level.WARN,
                "map",
                "map frame observed; verify this came from SLAM/localization, not the Milestone #6 bridge",
            )
        )
    else:
        results.append(CheckResult(Level.PASS, "map", "not required and not observed"))

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
    node = SimOdometryBridgeValidator(args.odom_topic)
    start_time = node.get_clock().now()

    try:
        while rclpy.ok():
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1_000_000_000
            if elapsed >= args.duration:
                break
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)

        topic_results = check_topic_types(topic_type_map(node), args)
        runtime_results = check_runtime_state(node.state, args)
        tf_results = check_frames_and_edges(node.state, args)

        print("Milestone #6 Simulation-Only Odometry Bridge Validation")
        print(f"Collection duration: {args.duration:.1f} seconds")
        print(f"Bridge expected enabled: {args.bridge_enabled}")
        print("This validator does not publish odometry, map, or localization transforms.")

        worst = Level.PASS
        worst = max(worst, print_section("Topic type checks", topic_results))
        worst = max(worst, print_section("Runtime message checks", runtime_results))
        worst = max(worst, print_section("TF checks", tf_results))

        if not args.bridge_enabled and worst == Level.PASS:
            worst = Level.WARN

        print("\nContract notes")
        print("  PASS requires live Isaac world -> body, preserved aliases, /odom, and odom -> base_link.")
        print("  WARN is expected when the bridge is disabled and inherited pieces are healthy.")
        print("  map -> odom remains the responsibility of SLAM/localization, not this bridge.")

        print(f"\nOverall result: {worst.name}")
        return 1 if worst == Level.FAIL else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
