#!/usr/bin/env python3
"""Read-only rosbag2 contract checker for the Isaac Sim sensor baseline.

Run after manually recording a bag:

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0
    python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/30_check_bag_contract.py /path/to/bag_dir

The script inspects metadata and, when rosbag2_py is available, reads messages
to verify the first observed frame_id for each sensor topic. It never writes to
the bag.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Iterable


EXPECTED_TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/spot/lidar/points": "sensor_msgs/msg/PointCloud2",
    "/spot/d455/color/image": "sensor_msgs/msg/Image",
    "/spot/d455/color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/spot/d455/depth/image": "sensor_msgs/msg/Image",
    "/spot/d455/imu": "sensor_msgs/msg/Imu",
}

EXPECTED_SENSOR_FRAMES = {
    "/spot/lidar/points": "sensor",
    "/spot/d455/color/image": "Camera_OmniVision_OV9782_Color",
    "/spot/d455/color/camera_info": "Camera_OmniVision_OV9782_Color",
    "/spot/d455/depth/image": "Camera_Pseudo_Depth",
    "/spot/d455/imu": "Imu_Sensor",
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect a rosbag2 directory against the Milestone #3 topic/frame contract.",
    )
    parser.add_argument("bag", type=Path, help="Path to a rosbag2 directory.")
    parser.add_argument(
        "--max-messages",
        type=int,
        default=20000,
        help="Maximum messages to scan while searching for sensor frame_ids.",
    )
    return parser.parse_args()


def parse_storage_identifier(metadata_text: str) -> str:
    for line in metadata_text.splitlines():
        stripped = line.strip()
        if stripped.startswith("storage_identifier:"):
            return stripped.split(":", 1)[1].strip().strip("'\"")
    return ""


def bag_metadata_path(bag_dir: Path) -> Path:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yaml not found in {bag_dir}")
    return metadata_path


def get_bag_topic_types(bag_dir: Path, storage_id: str) -> dict[str, str]:
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    return {
        topic_metadata.name: topic_metadata.type
        for topic_metadata in reader.get_all_topics_and_types()
    }


def read_sensor_frame_ids(
    bag_dir: Path,
    storage_id: str,
    topic_types: dict[str, str],
    max_messages: int,
) -> dict[str, str]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    message_types = {
        topic: get_message(topic_type)
        for topic, topic_type in topic_types.items()
        if topic in EXPECTED_SENSOR_FRAMES
    }

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    frame_ids: dict[str, str] = {}
    scanned = 0
    while reader.has_next() and scanned < max_messages:
        topic, data, _ = reader.read_next()
        scanned += 1
        if topic not in message_types or topic in frame_ids:
            continue

        message = deserialize_message(data, message_types[topic])
        frame_ids[topic] = message.header.frame_id

        if set(frame_ids) == set(EXPECTED_SENSOR_FRAMES):
            break

    return frame_ids


def check_topic_types(topic_types: dict[str, str]) -> list[CheckResult]:
    results = []
    for topic, expected_type in EXPECTED_TOPIC_TYPES.items():
        observed_type = topic_types.get(topic)
        if observed_type is None:
            results.append(CheckResult(Level.FAIL, topic, "topic missing from bag"))
        elif observed_type == expected_type:
            results.append(CheckResult(Level.PASS, topic, f"type {observed_type}"))
        else:
            results.append(
                CheckResult(
                    Level.FAIL,
                    topic,
                    f"expected {expected_type}, observed {observed_type}",
                )
            )
    return results


def check_sensor_frames(frame_ids: dict[str, str]) -> list[CheckResult]:
    results = []
    for topic, expected_frame in EXPECTED_SENSOR_FRAMES.items():
        observed_frame = frame_ids.get(topic)
        if observed_frame is None:
            results.append(CheckResult(Level.FAIL, topic, "no message frame_id found"))
        elif observed_frame == expected_frame:
            results.append(CheckResult(Level.PASS, topic, f"frame_id {observed_frame}"))
        else:
            results.append(
                CheckResult(
                    Level.FAIL,
                    topic,
                    f"expected frame_id {expected_frame}, observed {observed_frame}",
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
    bag_dir = args.bag.expanduser().resolve()

    if not bag_dir.is_dir():
        raise SystemExit(f"Bag path is not a directory: {bag_dir}")

    metadata_text = bag_metadata_path(bag_dir).read_text(encoding="utf-8")
    storage_id = parse_storage_identifier(metadata_text)

    if not storage_id:
        print("WARN metadata.yaml does not declare storage_identifier; trying default rosbag2_py behavior.")

    try:
        topic_types = get_bag_topic_types(bag_dir, storage_id)
    except ImportError as exc:
        raise SystemExit(
            "rosbag2_py is not available. Source ROS2 Jazzy first:\n"
            "  source /opt/ros/jazzy/setup.bash"
        ) from exc

    print("Milestone #3 rosbag2 Contract Check")
    print(f"Bag: {bag_dir}")
    print(f"Storage: {storage_id or '<default>'}")

    worst = print_section("Topic type checks", check_topic_types(topic_types))

    try:
        frame_ids = read_sensor_frame_ids(
            bag_dir,
            storage_id,
            topic_types,
            args.max_messages,
        )
    except ImportError as exc:
        raise SystemExit(
            "Message deserialization dependencies are unavailable. Source ROS2 Jazzy first:\n"
            "  source /opt/ros/jazzy/setup.bash"
        ) from exc

    worst = max(worst, print_section("Sensor frame_id checks", check_sensor_frames(frame_ids)))

    print("\nContract notes")
    print("  This check is read-only and does not replay or modify the bag.")
    print("  Current frame naming is Isaac-derived.")
    print("  base_link, odom, map, and static aliases are not required for Milestone #3.")
    print("  SLAM/localization validation belongs to Milestone #4.")

    print(f"\nOverall result: {worst.name}")
    return 1 if worst == Level.FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
