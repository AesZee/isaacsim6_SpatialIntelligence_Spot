#!/usr/bin/env python3
"""Read-only PointCloud2 and LaserScan density diagnostics for Milestone #9."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2


FLOAT_FORMATS = {7: "f", 8: "d"}  # sensor_msgs/PointField FLOAT32/FLOAT64


@dataclass(frozen=True)
class ProjectionSettings:
    min_height: float
    max_height: float
    range_min: float
    range_max: float
    angle_min: float
    angle_max: float
    angle_increment: float

    def validate(self) -> None:
        if self.min_height >= self.max_height:
            raise ValueError("min_height must be less than max_height")
        if self.range_min < 0.0 or self.range_min >= self.range_max:
            raise ValueError("range_min must be non-negative and less than range_max")
        if self.angle_min >= self.angle_max:
            raise ValueError("angle_min must be less than angle_max")
        if self.angle_increment <= 0.0:
            raise ValueError("angle_increment must be positive")

    @property
    def beam_capacity(self) -> int:
        return int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment))


@dataclass(frozen=True)
class CloudCounts:
    total: int
    finite_xyz: int
    height_slice: int
    projected_beams: int


def iter_xyz(message: PointCloud2) -> Iterable[tuple[float, float, float]]:
    fields = {field.name: field for field in message.fields}
    missing = [name for name in ("x", "y", "z") if name not in fields]
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {', '.join(missing)}")
    byte_order = ">" if message.is_bigendian else "<"
    unpackers = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.count != 1 or field.datatype not in FLOAT_FORMATS:
            raise ValueError(
                f"PointCloud2 field {name} must be scalar FLOAT32/FLOAT64; "
                f"datatype={field.datatype}, count={field.count}"
            )
        unpackers.append((field.offset, struct.Struct(byte_order + FLOAT_FORMATS[field.datatype])))

    if message.point_step <= 0 or message.row_step < message.width * message.point_step:
        raise ValueError("PointCloud2 point_step/row_step is invalid")
    required_bytes = message.row_step * message.height
    if len(message.data) < required_bytes:
        raise ValueError(f"PointCloud2 data has {len(message.data)} bytes; expected at least {required_bytes}")

    for row in range(message.height):
        row_offset = row * message.row_step
        for column in range(message.width):
            point_offset = row_offset + column * message.point_step
            yield tuple(unpacker.unpack_from(message.data, point_offset + offset)[0] for offset, unpacker in unpackers)


def analyze_points(
    points: Iterable[tuple[float, float, float]],
    total: int,
    settings: ProjectionSettings,
) -> CloudCounts:
    finite_xyz = 0
    height_slice = 0
    beams: set[int] = set()
    for x, y, z in points:
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        finite_xyz += 1
        if not settings.min_height <= z <= settings.max_height:
            continue
        height_slice += 1
        distance = math.hypot(x, y)
        angle = math.atan2(y, x)
        if not settings.range_min <= distance <= settings.range_max:
            continue
        if not settings.angle_min <= angle <= settings.angle_max:
            continue
        index = int((angle - settings.angle_min) / settings.angle_increment)
        if 0 <= index < settings.beam_capacity:
            beams.add(index)
    return CloudCounts(total, finite_xyz, height_slice, len(beams))


def frequency_hz(arrivals: list[float]) -> float | None:
    if len(arrivals) < 2 or arrivals[-1] <= arrivals[0]:
        return None
    return (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])


def triplet(values: list[int]) -> str:
    if not values:
        return "n/a"
    return f"min={min(values)}, median={statistics.median(values):g}, max={max(values)}"


class ScanQualityDiagnostics(Node):
    def __init__(self, args: argparse.Namespace, settings: ProjectionSettings) -> None:
        super().__init__("m09_laserscan_quality_diagnostics")
        self.args = args
        self.settings = settings
        self.cloud_arrivals: list[float] = []
        self.scan_arrivals: list[float] = []
        self.cloud_frames: set[str] = set()
        self.scan_frames: set[str] = set()
        self.cloud_counts: list[CloudCounts] = []
        self.scan_finite_counts: list[int] = []
        self.scan_infinite_counts: list[int] = []
        self.scan_invalid_counts: list[int] = []
        self.scan_total_counts: list[int] = []
        self.finite_ranges: list[float] = []
        self.cloud_error: str | None = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, sensor_qos)
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan, sensor_qos)

    def _on_cloud(self, message: PointCloud2) -> None:
        self.cloud_arrivals.append(time.monotonic())
        self.cloud_frames.add(message.header.frame_id)
        try:
            self.cloud_counts.append(
                analyze_points(
                    iter_xyz(message),
                    int(message.width) * int(message.height),
                    self.settings,
                )
            )
        except (ValueError, struct.error) as error:
            self.cloud_error = str(error)

    def _on_scan(self, message: LaserScan) -> None:
        self.scan_arrivals.append(time.monotonic())
        self.scan_frames.add(message.header.frame_id)
        finite = [float(value) for value in message.ranges if math.isfinite(value)]
        infinite = sum(1 for value in message.ranges if math.isinf(value))
        invalid = len(message.ranges) - len(finite) - infinite
        self.scan_total_counts.append(len(message.ranges))
        self.scan_finite_counts.append(len(finite))
        self.scan_infinite_counts.append(infinite)
        self.scan_invalid_counts.append(invalid)
        self.finite_ranges.extend(finite)


def format_frequency(arrivals: list[float]) -> str:
    value = frequency_hz(arrivals)
    return f"{value:.3f} Hz" if value is not None else "n/a (need at least two messages)"


def report(node: ScanQualityDiagnostics) -> None:
    counts = node.cloud_counts
    print("Milestone #9 LaserScan Quality Diagnostics")
    print("Read-only: no topics, transforms, maps, or odometry were published.")
    print("\nSelected projection parameters")
    for name in ("min_height", "max_height", "range_min", "range_max", "angle_min", "angle_max", "angle_increment"):
        print(f"  {name}: {getattr(node.settings, name)}")

    print("\nPointCloud2 input")
    print(f"  topic: {node.args.cloud_topic}")
    print(f"  messages: {len(node.cloud_arrivals)}")
    print(f"  frequency: {format_frequency(node.cloud_arrivals)}")
    print(f"  frame_ids: {sorted(node.cloud_frames)}")
    print(f"  total points/cloud: {triplet([item.total for item in counts])}")
    print(f"  finite XYZ/cloud: {triplet([item.finite_xyz for item in counts])}")
    print(f"  selected height slice/cloud: {triplet([item.height_slice for item in counts])}")
    print(f"  valid projected beams/cloud: {triplet([item.projected_beams for item in counts])}")
    print(f"  projected beam capacity: {node.settings.beam_capacity}")
    coverage = [
        item.projected_beams / node.settings.beam_capacity
        for item in counts
        if node.settings.beam_capacity
    ]
    print(f"  angular coverage: {statistics.median(coverage):.4f}" if coverage else "  angular coverage: n/a")
    if node.cloud_error:
        print(f"  parse error: {node.cloud_error}")
    if node.cloud_frames - {"sensor", "os1_frame"}:
        print("  WARN: projection estimate assumes the cloud frame is sensor or identity-equivalent os1_frame.")

    print("\nLaserScan output")
    print(f"  topic: {node.args.scan_topic}")
    print(f"  messages: {len(node.scan_arrivals)}")
    print(f"  frequency: {format_frequency(node.scan_arrivals)}")
    print(f"  frame_ids: {sorted(node.scan_frames)}")
    print(f"  finite ranges/scan: {triplet(node.scan_finite_counts)}")
    print(f"  infinite ranges/scan: {triplet(node.scan_infinite_counts)}")
    print(f"  other invalid ranges/scan: {triplet(node.scan_invalid_counts)}")
    scan_coverage = [
        finite / total for finite, total in zip(node.scan_finite_counts, node.scan_total_counts) if total
    ]
    print(
        f"  finite-beam angular coverage: {statistics.median(scan_coverage):.4f}"
        if scan_coverage
        else "  finite-beam angular coverage: n/a"
    )
    if node.finite_ranges:
        print(
            "  finite range min/median/max: "
            f"{min(node.finite_ranges):.3f} / "
            f"{statistics.median(node.finite_ranges):.3f} / "
            f"{max(node.finite_ranges):.3f} m"
        )
    else:
        print("  finite range min/median/max: n/a")
    if node.scan_frames and node.scan_frames != {"os1_frame"}:
        print("  FAIL: /scan frame_id contract is os1_frame.")


def numeric_summary(values: list[float | int]) -> dict | None:
    if not values:
        return None
    return {"min": min(values), "median": statistics.median(values), "max": max(values)}


def diagnostics_dict(node: ScanQualityDiagnostics) -> dict:
    counts = node.cloud_counts
    capacity = node.settings.beam_capacity
    cloud_coverage = [item.projected_beams / capacity for item in counts if capacity]
    scan_coverage = [
        finite / total for finite, total in zip(node.scan_finite_counts, node.scan_total_counts) if total
    ]
    valid_scan_frame = node.scan_frames == {"os1_frame"}
    return {
        "result": "PASS" if counts and node.scan_arrivals and not node.cloud_error and valid_scan_frame else "FAIL",
        "parameters": {
            name: getattr(node.settings, name)
            for name in ("min_height", "max_height", "range_min", "range_max", "angle_min", "angle_max", "angle_increment")
        },
        "pointcloud": {
            "topic": node.args.cloud_topic,
            "messages": len(node.cloud_arrivals),
            "frequency_hz": frequency_hz(node.cloud_arrivals),
            "frame_ids": sorted(node.cloud_frames),
            "total_points": numeric_summary([item.total for item in counts]),
            "finite_xyz": numeric_summary([item.finite_xyz for item in counts]),
            "height_slice_points": numeric_summary([item.height_slice for item in counts]),
            "projected_beams": numeric_summary([item.projected_beams for item in counts]),
            "angular_coverage_ratio": numeric_summary(cloud_coverage),
            "parse_error": node.cloud_error,
        },
        "laserscan": {
            "topic": node.args.scan_topic,
            "messages": len(node.scan_arrivals),
            "frequency_hz": frequency_hz(node.scan_arrivals),
            "frame_ids": sorted(node.scan_frames),
            "finite_ranges": numeric_summary(node.scan_finite_counts),
            "infinite_ranges": numeric_summary(node.scan_infinite_counts),
            "invalid_ranges": numeric_summary(node.scan_invalid_counts),
            "finite_range_m": numeric_summary(node.finite_ranges),
            "angular_coverage_ratio": numeric_summary(scan_coverage),
        },
    }


def self_check() -> None:
    settings = ProjectionSettings(-0.2, 0.2, 0.2, 20.0, -math.pi / 2, math.pi / 2, math.pi / 4)
    settings.validate()
    points = [
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (-1.0, 0.0, 0.0),
        (1.0, 0.0, 1.0),
        (math.inf, 0.0, 0.0),
    ]
    counts = analyze_points(points, len(points), settings)
    assert counts == CloudCounts(total=6, finite_xyz=5, height_slice=4, projected_beams=2)

    packed = struct.pack("<fff", 1.0, 2.0, 3.0)
    fields = [
        SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
        for name, offset in (("x", 0), ("y", 4), ("z", 8))
    ]
    message = SimpleNamespace(
        fields=fields,
        is_bigendian=False,
        point_step=12,
        row_step=12,
        width=1,
        height=1,
        data=packed,
    )
    assert list(iter_xyz(message)) == [(1.0, 2.0, 3.0)]
    assert frequency_hz([1.0, 1.5, 2.0]) == 2.0
    print("PASS: LaserScan diagnostic self-check")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--spin-timeout", type=float, default=0.1)
    parser.add_argument("--cloud-topic", default="/spot/lidar/points")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--min-height", type=float, default=-0.20)
    parser.add_argument("--max-height", type=float, default=0.20)
    parser.add_argument("--range-min", type=float, default=0.20)
    parser.add_argument("--range-max", type=float, default=20.0)
    parser.add_argument("--angle-min", type=float, default=-3.14159)
    parser.add_argument("--angle-max", type=float, default=3.14159)
    parser.add_argument("--angle-increment", type=float, default=0.0087)
    parser.add_argument("--json-output")
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_check:
        self_check()
        return 0
    if args.duration <= 0.0:
        raise ValueError("duration must be positive")
    settings = ProjectionSettings(
        args.min_height,
        args.max_height,
        args.range_min,
        args.range_max,
        args.angle_min,
        args.angle_max,
        args.angle_increment,
    )
    settings.validate()

    rclpy.init()
    node = ScanQualityDiagnostics(args, settings)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(args.spin_timeout, max(0.0, deadline - time.monotonic())))
        report(node)
        diagnostics = diagnostics_dict(node)
        if args.json_output:
            output = Path(args.json_output)
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open("x", encoding="utf-8") as stream:
                json.dump(diagnostics, stream, indent=2, sort_keys=True)
                stream.write("\n")
        return 0 if diagnostics["result"] == "PASS" else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
