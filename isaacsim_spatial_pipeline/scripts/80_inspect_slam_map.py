#!/usr/bin/env python3
"""Inspect live Milestone #7 SLAM map output without publishing anything."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


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
class MapStats:
    total_cells: int
    unknown_cells: int
    free_cells: int
    occupied_cells: int

    @property
    def unknown_ratio(self) -> float:
        return ratio(self.unknown_cells, self.total_cells)

    @property
    def free_ratio(self) -> float:
        return ratio(self.free_cells, self.total_cells)

    @property
    def occupied_ratio(self) -> float:
        return ratio(self.occupied_cells, self.total_cells)

    @property
    def known_ratio(self) -> float:
        return ratio(self.free_cells + self.occupied_cells, self.total_cells)


class SlamMapInspector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("m08_slam_map_inspector")
        self.args = args
        self.clock_seen = False
        self.scan_seen = False
        self.scan_frame_ids: set[str] = set()
        self.odom_seen = False
        self.odom_frame_ids: set[str] = set()
        self.odom_child_frame_ids: set[str] = set()
        self.map_message: OccupancyGrid | None = None

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Clock, "/clock", self._on_clock, volatile_qos)
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan, volatile_qos)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, volatile_qos)
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, map_qos)

    def _on_clock(self, _: Clock) -> None:
        self.clock_seen = True

    def _on_scan(self, message: LaserScan) -> None:
        self.scan_seen = True
        if message.header.frame_id:
            self.scan_frame_ids.add(message.header.frame_id)

    def _on_odom(self, message: Odometry) -> None:
        self.odom_seen = True
        if message.header.frame_id:
            self.odom_frame_ids.add(message.header.frame_id)
        if message.child_frame_id:
            self.odom_child_frame_ids.add(message.child_frame_id)

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message


def ratio(count: int, total: int) -> float:
    return float(count) / float(total) if total else 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect live Milestone #8 SLAM map output.")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--spin-timeout", type=float, default=0.1)
    return parser.parse_args()


def topic_type_map(node: Node) -> dict[str, list[str]]:
    return {
        topic_name: topic_types
        for topic_name, topic_types in node.get_topic_names_and_types()
    }


def check_topic_types(topic_types: dict[str, list[str]], args: argparse.Namespace) -> list[CheckResult]:
    expected = {
        "/clock": "rosgraph_msgs/msg/Clock",
        args.scan_topic: "sensor_msgs/msg/LaserScan",
        args.odom_topic: "nav_msgs/msg/Odometry",
        args.map_topic: "nav_msgs/msg/OccupancyGrid",
    }
    results = []
    for topic, topic_type in expected.items():
        observed = topic_types.get(topic)
        if not observed:
            if topic == args.map_topic:
                results.append(CheckResult(Level.WARN, topic, "topic not observed yet"))
            else:
                results.append(CheckResult(Level.FAIL, topic, "topic not observed"))
        elif topic_type in observed:
            results.append(CheckResult(Level.PASS, topic, f"type {topic_type}"))
        else:
            results.append(CheckResult(Level.FAIL, topic, f"expected {topic_type}, observed {', '.join(observed)}"))
    return results


def map_stats(message: OccupancyGrid) -> MapStats:
    data = list(message.data)
    return MapStats(
        total_cells=len(data),
        unknown_cells=sum(1 for value in data if value == -1),
        free_cells=sum(1 for value in data if value == 0),
        occupied_cells=sum(1 for value in data if value > 0),
    )


def check_runtime(node: SlamMapInspector, args: argparse.Namespace) -> list[CheckResult]:
    results = []
    results.append(
        CheckResult(
            Level.PASS if node.clock_seen else Level.FAIL,
            "/clock",
            "messages received" if node.clock_seen else "no messages received",
        )
    )
    if node.scan_seen:
        expected = {"os1_frame"}
        level = Level.PASS if node.scan_frame_ids == expected else Level.FAIL
        results.append(CheckResult(level, args.scan_topic, f"LaserScan frame IDs {sorted(node.scan_frame_ids)}"))
    else:
        results.append(CheckResult(Level.FAIL, args.scan_topic, "no LaserScan messages received"))

    if node.odom_seen:
        frame_ok = node.odom_frame_ids == {args.odom_frame}
        child_ok = node.odom_child_frame_ids == {args.base_frame}
        level = Level.PASS if frame_ok and child_ok else Level.FAIL
        results.append(
            CheckResult(
                level,
                args.odom_topic,
                f"headers {sorted(node.odom_frame_ids)}, children {sorted(node.odom_child_frame_ids)}",
            )
        )
    else:
        results.append(CheckResult(Level.FAIL, args.odom_topic, "no Odometry messages received"))

    if node.map_message is None:
        results.append(CheckResult(Level.WARN, args.map_topic, "no OccupancyGrid received within validation window"))
    else:
        frame_id = node.map_message.header.frame_id
        level = Level.PASS if frame_id == args.map_frame else Level.FAIL
        results.append(CheckResult(level, args.map_topic, f"OccupancyGrid frame_id {frame_id!r}"))
    return results


def check_map_metadata(message: OccupancyGrid | None, args: argparse.Namespace) -> tuple[list[CheckResult], MapStats | None]:
    if message is None:
        return [CheckResult(Level.WARN, "map metadata", "no map received")], None

    info = message.info
    expected_len = int(info.width) * int(info.height)
    stats = map_stats(message)
    results = [
        CheckResult(Level.PASS if info.resolution > 0.0 and math.isfinite(info.resolution) else Level.FAIL, "resolution", str(info.resolution)),
        CheckResult(Level.PASS if info.width > 0 else Level.FAIL, "width", str(info.width)),
        CheckResult(Level.PASS if info.height > 0 else Level.FAIL, "height", str(info.height)),
        CheckResult(Level.PASS, "origin", f"position=({info.origin.position.x:.3f}, {info.origin.position.y:.3f}, {info.origin.position.z:.3f})"),
        CheckResult(
            Level.PASS if len(message.data) == expected_len and expected_len > 0 else Level.FAIL,
            "data length",
            f"{len(message.data)} cells, expected {expected_len}",
        ),
    ]

    if stats.total_cells == 0:
        results.append(CheckResult(Level.FAIL, "degenerate map", "empty data"))
    elif stats.unknown_cells == stats.total_cells:
        results.append(CheckResult(Level.WARN, "degenerate map", "all cells are unknown"))
    elif stats.unknown_ratio > 0.98:
        results.append(CheckResult(Level.WARN, "known coverage", f"unknown_ratio {stats.unknown_ratio:.4f} > 0.98"))
    else:
        results.append(CheckResult(Level.PASS, "known coverage", f"unknown_ratio {stats.unknown_ratio:.4f}"))

    if stats.occupied_cells == 0 and stats.total_cells:
        results.append(CheckResult(Level.WARN, "occupied cells", "no occupied cells observed"))
    elif stats.occupied_ratio <= 0.001 and stats.total_cells:
        results.append(CheckResult(Level.WARN, "occupied ratio", f"occupied_ratio {stats.occupied_ratio:.6f} <= 0.001"))
    else:
        results.append(CheckResult(Level.PASS, "occupied ratio", f"occupied_ratio {stats.occupied_ratio:.6f}"))

    results.extend(
        [
            CheckResult(Level.PASS, "unknown cells", f"{stats.unknown_cells} ({stats.unknown_ratio:.4f})"),
            CheckResult(Level.PASS, "free cells", f"{stats.free_cells} ({stats.free_ratio:.4f})"),
            CheckResult(Level.PASS, "occupied cells", f"{stats.occupied_cells} ({stats.occupied_ratio:.6f})"),
        ]
    )
    return results, stats


def check_tf(node: SlamMapInspector, args: argparse.Namespace) -> list[CheckResult]:
    now = Time()
    timeout = Duration(seconds=0.1)
    results = []
    map_to_odom = node.tf_buffer.can_transform(args.map_frame, args.odom_frame, now, timeout)
    odom_to_base = node.tf_buffer.can_transform(args.odom_frame, args.base_frame, now, timeout)
    results.append(
        CheckResult(
            Level.PASS if map_to_odom else Level.WARN,
            f"{args.map_frame} -> {args.odom_frame}",
            "available in TF" if map_to_odom else "not observed within validation window",
        )
    )
    results.append(
        CheckResult(
            Level.PASS if odom_to_base else Level.FAIL,
            f"{args.odom_frame} -> {args.base_frame}",
            "TF chain available" if odom_to_base else "TF chain missing",
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
    node = SlamMapInspector(args)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)

        topic_results = check_topic_types(topic_type_map(node), args)
        runtime_results = check_runtime(node, args)
        metadata_results, _ = check_map_metadata(node.map_message, args)
        tf_results = check_tf(node, args)

        print("Milestone #8 Live SLAM Map Inspection")
        print(f"Collection duration: {args.duration:.1f} seconds")
        print("This inspector is read-only and does not publish map, odometry, or TF.")

        worst = Level.PASS
        worst = max(worst, print_section("Topic type checks", topic_results))
        worst = max(worst, print_section("Runtime checks", runtime_results))
        worst = max(worst, print_section("Map metadata and quality checks", metadata_results))
        worst = max(worst, print_section("TF checks", tf_results))

        print(f"\nOverall result: {worst.name}")
        return 1 if worst == Level.FAIL else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
