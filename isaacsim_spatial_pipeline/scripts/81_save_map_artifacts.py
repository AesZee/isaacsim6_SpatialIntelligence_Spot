#!/usr/bin/env python3
"""Save Milestone #8 map artifacts from a real live /map message."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapReceiver(Node):
    def __init__(self, map_topic: str) -> None:
        super().__init__("m08_map_artifact_saver")
        self.map_message: OccupancyGrid | None = None
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Save map artifacts from a live OccupancyGrid.")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--output-root", default="/home/aes/isaac_ws/maps")
    parser.add_argument("--prefix", default="m08")
    parser.add_argument("--spin-timeout", type=float, default=0.1)
    return parser.parse_args()


def validate_metadata(message: OccupancyGrid) -> list[str]:
    errors = []
    info = message.info
    expected_len = int(info.width) * int(info.height)
    if not message.header.frame_id:
        errors.append("map header.frame_id is empty")
    if info.resolution <= 0.0 or not math.isfinite(info.resolution):
        errors.append(f"invalid resolution {info.resolution}")
    if info.width == 0:
        errors.append("map width is zero")
    if info.height == 0:
        errors.append("map height is zero")
    if len(message.data) != expected_len:
        errors.append(f"data length {len(message.data)} does not match width*height {expected_len}")
    return errors


def cell_counts(message: OccupancyGrid) -> dict[str, int]:
    data = list(message.data)
    return {
        "total_cells": len(data),
        "unknown_cells": sum(1 for value in data if value == -1),
        "free_cells": sum(1 for value in data if value == 0),
        "occupied_cells": sum(1 for value in data if value > 0),
    }


def ratio(count: int, total: int) -> float:
    return float(count) / float(total) if total else 0.0


def origin_list(message: OccupancyGrid) -> list[float]:
    origin = message.info.origin
    return [
        float(origin.position.x),
        float(origin.position.y),
        yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        ),
    ]


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def metadata_json(message: OccupancyGrid) -> dict:
    info = message.info
    origin = info.origin
    return {
        "frame_id": message.header.frame_id,
        "stamp": {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        },
        "resolution": float(info.resolution),
        "width": int(info.width),
        "height": int(info.height),
        "origin": {
            "position": {
                "x": float(origin.position.x),
                "y": float(origin.position.y),
                "z": float(origin.position.z),
            },
            "orientation": {
                "x": float(origin.orientation.x),
                "y": float(origin.orientation.y),
                "z": float(origin.orientation.z),
                "w": float(origin.orientation.w),
            },
        },
    }


def stats_json(message: OccupancyGrid, output_directory: Path) -> dict:
    counts = cell_counts(message)
    total = counts["total_cells"]
    known_cells = counts["free_cells"] + counts["occupied_cells"]
    resolution = float(message.info.resolution)
    cell_area = resolution * resolution
    return {
        **counts,
        "known_cells": known_cells,
        "unknown_ratio": ratio(counts["unknown_cells"], total),
        "known_ratio": ratio(known_cells, total),
        "free_ratio": ratio(counts["free_cells"], total),
        "occupied_ratio": ratio(counts["occupied_cells"], total),
        "map_area_m2": total * cell_area,
        "known_area_m2": known_cells * cell_area,
        "occupied_area_m2": counts["occupied_cells"] * cell_area,
        "output_directory": str(output_directory),
    }


def write_yaml(path: Path, message: OccupancyGrid) -> None:
    lines = [
        "image: map.pgm",
        "mode: trinary",
        f"resolution: {float(message.info.resolution):.9g}",
        f"origin: [{origin_list(message)[0]:.9g}, {origin_list(message)[1]:.9g}, {origin_list(message)[2]:.9g}]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.25",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def occupancy_to_pgm_value(value: int) -> int:
    if value == -1:
        return 205
    if value == 0:
        return 254
    return 0


def write_pgm(path: Path, message: OccupancyGrid) -> None:
    width = int(message.info.width)
    height = int(message.info.height)
    data = list(message.data)
    pixels = bytearray()
    for y in range(height - 1, -1, -1):
        row_start = y * width
        for x in range(width):
            pixels.append(occupancy_to_pgm_value(data[row_start + x]))
    with path.open("wb") as output:
        output.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        output.write(pixels)


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def make_output_dir(output_root: str, prefix: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(output_root) / f"{prefix}_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    return output_dir


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MapReceiver(args.map_topic)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration and node.map_message is None:
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)

        if node.map_message is None:
            print("WARN /map: no OccupancyGrid received within timeout; no files were created.")
            return 0

        errors = validate_metadata(node.map_message)
        if errors:
            print("FAIL map metadata: invalid map; no files were created.")
            for error in errors:
                print(f"  {error}")
            return 1

        output_dir = make_output_dir(args.output_root, args.prefix)
        paths = {
            "map_yaml": output_dir / "map.yaml",
            "map_pgm": output_dir / "map.pgm",
            "metadata_json": output_dir / "map_metadata.json",
            "stats_json": output_dir / "map_stats.json",
        }

        write_yaml(paths["map_yaml"], node.map_message)
        write_pgm(paths["map_pgm"], node.map_message)
        write_json(paths["metadata_json"], metadata_json(node.map_message))
        write_json(paths["stats_json"], stats_json(node.map_message, output_dir))

        print("PASS saved map artifacts from live /map")
        for label, path in paths.items():
            print(f"  {label}: {path}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
