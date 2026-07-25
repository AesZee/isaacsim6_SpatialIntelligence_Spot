#!/usr/bin/env python3
"""Validate Milestone #7 LiDAR SLAM with simulation-only odometry.

This is read-only. It subscribes to live ROS2 topics and inspects the graph; it
does not publish odometry, map, or TF.
"""

from __future__ import annotations

import argparse
import math
from collections import defaultdict
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
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
    lidar_seen: bool = False
    lidar_frame_ids: set[str] = field(default_factory=set)
    scan_seen: bool = False
    scan_frame_ids: set[str] = field(default_factory=set)
    scan_finite_ranges: int = 0
    odom_messages: int = 0
    odom_frame_ids: set[str] = field(default_factory=set)
    odom_child_frame_ids: set[str] = field(default_factory=set)
    map_seen: bool = False
    map_frame_ids: set[str] = field(default_factory=set)
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
        for frame_id in self.map_frame_ids:
            frames.add(frame_id)
        return frames


class LidarSlamWithSimOdomValidator(Node):
    def __init__(self, odom_topic: str, map_topic: str) -> None:
        super().__init__("m07_lidar_slam_with_sim_odom_validator")
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

        self.create_subscription(Clock, "/clock", self._on_clock, volatile_qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, volatile_qos)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, static_tf_qos)
        self.create_subscription(PointCloud2, "/spot/lidar/points", self._on_lidar, volatile_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, volatile_qos)
        self.create_subscription(Odometry, odom_topic, self._on_odom, volatile_qos)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)

    def _on_clock(self, _: Clock) -> None:
        self.state.clock_seen = True

    def _on_lidar(self, message: PointCloud2) -> None:
        self.state.lidar_seen = True
        if message.header.frame_id:
            self.state.lidar_frame_ids.add(message.header.frame_id)

    def _on_scan(self, message: LaserScan) -> None:
        self.state.scan_seen = True
        self.state.scan_finite_ranges += sum(math.isfinite(value) for value in message.ranges)
        if message.header.frame_id:
            self.state.scan_frame_ids.add(message.header.frame_id)

    def _on_odom(self, message: Odometry) -> None:
        self.state.odom_messages += 1
        if message.header.frame_id:
            self.state.odom_frame_ids.add(message.header.frame_id)
        if message.child_frame_id:
            self.state.odom_child_frame_ids.add(message.child_frame_id)

    def _on_map(self, message: OccupancyGrid) -> None:
        self.state.map_seen = True
        if message.header.frame_id:
            self.state.map_frame_ids.add(message.header.frame_id)

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
        description="Validate Milestone #7 live Isaac LiDAR SLAM with simulation-only odometry.",
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--spin-timeout", type=float, default=0.1)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--scan-frame", default="os1_frame")
    parser.add_argument("--lidar-frame", default="sensor")
    parser.add_argument("--exit-when-ready", action="store_true")
    return parser.parse_args()


def topic_type_map(node: Node) -> dict[str, list[str]]:
    return {
        topic_name: topic_types
        for topic_name, topic_types in node.get_topic_names_and_types()
    }


def publisher_nodes(node: Node, topic: str) -> set[str]:
    return {
        info.node_name
        for info in node.get_publishers_info_by_topic(topic)
        if info.node_name
    }


def edge_exists(edges: set[tuple[str, str]], parent: str, child: str) -> bool:
    return (parent, child) in edges


def graph_has_chain(edges: set[tuple[str, str]], parent: str, child: str) -> bool:
    if parent == child:
        return True
    children_by_parent: dict[str, set[str]] = defaultdict(set)
    for edge_parent, edge_child in edges:
        children_by_parent[edge_parent].add(edge_child)
    visited = set()
    frontier = [parent]
    while frontier:
        current = frontier.pop()
        if current in visited:
            continue
        visited.add(current)
        for next_child in children_by_parent.get(current, set()):
            if next_child == child:
                return True
            frontier.append(next_child)
    return False


def parent_map(edges: set[tuple[str, str]]) -> dict[str, set[str]]:
    parents: dict[str, set[str]] = defaultdict(set)
    for parent, child in edges:
        parents[child].add(parent)
    return parents


def check_topic_types(topic_types: dict[str, list[str]], args: argparse.Namespace) -> list[CheckResult]:
    results = []
    expected = {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
        "/spot/lidar/points": "sensor_msgs/msg/PointCloud2",
        "/scan": "sensor_msgs/msg/LaserScan",
        args.odom_topic: "nav_msgs/msg/Odometry",
        args.map_topic: "nav_msgs/msg/OccupancyGrid",
    }
    for topic, topic_type in expected.items():
        observed = topic_types.get(topic)
        if not observed:
            level = Level.WARN if topic == args.map_topic else Level.FAIL
            detail = "topic not observed yet" if topic == args.map_topic else "topic not observed"
            results.append(CheckResult(level, topic, detail))
        elif topic_type in observed:
            results.append(CheckResult(Level.PASS, topic, f"type {topic_type}"))
        else:
            results.append(CheckResult(Level.FAIL, topic, f"expected {topic_type}, observed {', '.join(observed)}"))
    return results


def check_runtime_state(state: ValidationState, args: argparse.Namespace) -> list[CheckResult]:
    results = [
        CheckResult(Level.PASS if state.clock_seen else Level.FAIL, "/clock", "messages received" if state.clock_seen else "no messages received"),
        CheckResult(Level.PASS if state.tf_messages_seen else Level.FAIL, "/tf", f"received {state.tf_messages_seen} messages" if state.tf_messages_seen else "no TF messages received"),
        CheckResult(Level.PASS if state.tf_static_messages_seen else Level.FAIL, "/tf_static", f"received {state.tf_static_messages_seen} messages" if state.tf_static_messages_seen else "no static TF messages received"),
    ]

    if state.lidar_seen and state.lidar_frame_ids == {args.lidar_frame}:
        results.append(CheckResult(Level.PASS, "/spot/lidar/points", f"messages received with frame IDs {sorted(state.lidar_frame_ids)}"))
    elif state.lidar_seen:
        results.append(CheckResult(Level.FAIL, "/spot/lidar/points", f"expected frame_id {args.lidar_frame}, observed {sorted(state.lidar_frame_ids)}"))
    else:
        results.append(CheckResult(Level.FAIL, "/spot/lidar/points", "no PointCloud2 messages received"))

    if state.scan_seen and state.scan_frame_ids == {args.scan_frame} and state.scan_finite_ranges:
        results.append(CheckResult(Level.PASS, "/scan", f"messages received with frame_id {args.scan_frame} and {state.scan_finite_ranges} finite returns"))
    elif state.scan_seen and state.scan_frame_ids == {args.scan_frame}:
        results.append(CheckResult(Level.FAIL, "/scan", "messages received but no finite ranges were observed"))
    elif state.scan_seen:
        results.append(CheckResult(Level.FAIL, "/scan", f"expected frame_id {args.scan_frame}, observed {sorted(state.scan_frame_ids)}"))
    else:
        results.append(CheckResult(Level.FAIL, "/scan", "no LaserScan messages received"))

    if state.odom_messages:
        frame_ok = state.odom_frame_ids == {args.odom_frame}
        child_ok = state.odom_child_frame_ids == {args.base_frame}
        if frame_ok and child_ok:
            results.append(CheckResult(Level.PASS, args.odom_topic, f"received {state.odom_messages} messages with correct frame IDs"))
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
        results.append(CheckResult(Level.FAIL, args.odom_topic, "no Odometry messages while simulation odometry is expected"))

    if state.map_seen:
        if state.map_frame_ids == {args.map_frame}:
            results.append(CheckResult(Level.PASS, args.map_topic, f"OccupancyGrid received with frame_id {args.map_frame}"))
        else:
            results.append(CheckResult(Level.FAIL, args.map_topic, f"expected frame_id {args.map_frame}, observed {sorted(state.map_frame_ids)}"))
    else:
        results.append(CheckResult(Level.WARN, args.map_topic, "no OccupancyGrid received within validation window"))

    return results


def check_tf(state: ValidationState, args: argparse.Namespace) -> list[CheckResult]:
    results = []
    if edge_exists(state.static_edges, "body", args.base_frame):
        results.append(CheckResult(Level.PASS, f"body -> {args.base_frame}", "base alias observed on /tf_static"))
    else:
        results.append(CheckResult(Level.FAIL, f"body -> {args.base_frame}", "base alias missing"))

    if edge_exists(state.static_edges, "sensor", args.scan_frame):
        results.append(CheckResult(Level.PASS, f"sensor -> {args.scan_frame}", "Milestone #4 alias observed on /tf_static"))
    else:
        results.append(CheckResult(Level.FAIL, f"sensor -> {args.scan_frame}", "Milestone #4 alias missing"))

    if graph_has_chain(state.edges, args.odom_frame, args.base_frame):
        results.append(CheckResult(Level.PASS, f"{args.odom_frame} -> {args.base_frame}", "TF chain observed"))
    else:
        results.append(CheckResult(Level.FAIL, f"{args.odom_frame} -> {args.base_frame}", "TF chain missing while simulation odometry is expected"))

    tf_frames = set()
    for parent, child in state.edges:
        tf_frames.add(parent)
        tf_frames.add(child)

    for frame in (args.map_frame, args.odom_frame):
        if frame in tf_frames:
            results.append(CheckResult(Level.PASS, frame, "frame observed in TF"))
        else:
            level = Level.WARN if frame == args.map_frame else Level.FAIL
            results.append(CheckResult(level, frame, "frame not observed in TF"))

    if edge_exists(state.dynamic_edges, args.map_frame, args.odom_frame):
        results.append(CheckResult(Level.PASS, f"{args.map_frame} -> {args.odom_frame}", "SLAM transform observed on /tf"))
    else:
        results.append(CheckResult(Level.WARN, f"{args.map_frame} -> {args.odom_frame}", "not observed within validation window"))

    child_to_parents = parent_map(state.edges)
    conflicts = {
        child: parents
        for child, parents in child_to_parents.items()
        if len(parents) > 1 and child in {args.base_frame, args.odom_frame, args.scan_frame}
    }
    if conflicts:
        detail = "; ".join(
            f"{child} has parents {sorted(parents)}"
            for child, parents in sorted(conflicts.items())
        )
        results.append(CheckResult(Level.FAIL, "TF parent conflict", detail))
    else:
        results.append(CheckResult(Level.PASS, "TF parent conflict", "no severe parent conflict detected for SLAM frames"))

    return results


def check_slam_publishers(node: Node, args: argparse.Namespace) -> list[CheckResult]:
    results = []
    node_names = set(node.get_node_names())
    map_publishers = publisher_nodes(node, args.map_topic)
    tf_publishers = publisher_nodes(node, "/tf")

    if "slam_toolbox" in node_names:
        results.append(CheckResult(Level.PASS, "slam_toolbox", "node observed"))
    else:
        results.append(CheckResult(Level.FAIL, "slam_toolbox", f"node not observed; active nodes: {sorted(node_names)}"))

    if map_publishers:
        if "slam_toolbox" in map_publishers:
            results.append(CheckResult(Level.PASS, args.map_topic, f"publisher includes slam_toolbox: {sorted(map_publishers)}"))
        else:
            results.append(CheckResult(Level.FAIL, args.map_topic, f"publisher is not slam_toolbox: {sorted(map_publishers)}"))
    else:
        results.append(CheckResult(Level.WARN, args.map_topic, "no publishers observed yet"))

    if "slam_toolbox" in tf_publishers:
        results.append(CheckResult(Level.PASS, "/tf slam_toolbox", "slam_toolbox is publishing TF"))
    else:
        results.append(CheckResult(Level.WARN, "/tf slam_toolbox", f"slam_toolbox TF publisher not observed; TF publishers: {sorted(tf_publishers)}"))

    return results


def ready_for_motion(node: LidarSlamWithSimOdomValidator, args: argparse.Namespace) -> bool:
    state = node.state
    return (
        state.clock_seen
        and state.tf_messages_seen > 0
        and state.tf_static_messages_seen > 0
        and state.lidar_frame_ids == {args.lidar_frame}
        and state.scan_frame_ids == {args.scan_frame}
        and state.scan_finite_ranges > 0
        and state.odom_frame_ids == {args.odom_frame}
        and state.odom_child_frame_ids == {args.base_frame}
        and edge_exists(state.static_edges, "body", args.base_frame)
        and edge_exists(state.static_edges, "sensor", args.scan_frame)
        and graph_has_chain(state.edges, args.odom_frame, args.base_frame)
        and "slam_toolbox" in node.get_node_names()
    )


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
    node = LidarSlamWithSimOdomValidator(args.odom_topic, args.map_topic)
    start_time = node.get_clock().now()

    try:
        while rclpy.ok():
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1_000_000_000
            if elapsed >= args.duration or (args.exit_when_ready and ready_for_motion(node, args)):
                break
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)

        topic_results = check_topic_types(topic_type_map(node), args)
        runtime_results = check_runtime_state(node.state, args)
        tf_results = check_tf(node.state, args)
        publisher_results = check_slam_publishers(node, args)

        print("Milestone #7 LiDAR SLAM With Simulation-Only Odometry Validation")
        print(f"Collection duration: {args.duration:.1f} seconds")
        print("This validator is read-only and does not publish map, odometry, or TF.")

        worst = Level.PASS
        worst = max(worst, print_section("Topic type checks", topic_results))
        worst = max(worst, print_section("Runtime message checks", runtime_results))
        worst = max(worst, print_section("TF checks", tf_results))
        worst = max(worst, print_section("SLAM publisher checks", publisher_results))

        if node.state.edges:
            print("\nObserved TF edges")
            for parent, child in sorted(node.state.edges):
                print(f"  {parent} -> {child}")

        print("\nContract notes")
        print("  /odom must come from the Milestone #6 simulation-only bridge.")
        print("  Milestone #7 declares odom -> world so Isaac TF provides the odom -> base_link chain.")
        print("  /scan must come from Milestone #4 PointCloud2-to-LaserScan conversion.")
        print("  /map and map -> odom must come from slam_toolbox, not custom fake publishers.")
        print("  A child frame with multiple parents is reported as a severe TF conflict.")

        print(f"\nOverall result: {worst.name}")
        return 1 if worst == Level.FAIL else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
