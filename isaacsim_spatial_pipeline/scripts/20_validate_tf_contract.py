#!/usr/bin/env python3
"""Validate the current Isaac-derived ROS2 topic and TF frame contract.

Run this from a ROS2-sourced terminal while Isaac Sim is already running and
the stage playback is active:

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0
    python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/20_validate_tf_contract.py

This script is read-only. It does not save USD files and does not record bags.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage


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

SENSOR_MESSAGE_TYPES = {
    "/spot/lidar/points": PointCloud2,
    "/spot/d455/color/image": Image,
    "/spot/d455/color/camera_info": CameraInfo,
    "/spot/d455/depth/image": Image,
    "/spot/d455/imu": Imu,
}

REQUIRED_SENSOR_TF_FRAMES = {
    "sensor",
    "Camera_OmniVision_OV9782_Color",
    "Camera_Pseudo_Depth",
    "Imu_Sensor",
}

IMPORTANT_TF_FRAMES = {
    "world",
    "body",
    "lidar_link",
    "OS1",
    "rsd455_link",
    "RSD455",
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
    clock_seen: bool = False
    tf_messages_seen: int = 0
    tf_frames: set[str] = field(default_factory=set)
    sensor_frame_ids: dict[str, str] = field(default_factory=dict)


class TfContractValidator(Node):
    def __init__(self) -> None:
        super().__init__("tf_contract_validator")
        self.state = ValidationState()

        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Clock, "/clock", self._on_clock, volatile_qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, volatile_qos)

        for topic, message_type in SENSOR_MESSAGE_TYPES.items():
            self.create_subscription(
                message_type,
                topic,
                self._sensor_callback(topic),
                volatile_qos,
            )

    def _on_clock(self, _: Clock) -> None:
        self.state.clock_seen = True

    def _on_tf(self, message: TFMessage) -> None:
        self.state.tf_messages_seen += 1
        for transform in message.transforms:
            parent = transform.header.frame_id.strip()
            child = transform.child_frame_id.strip()
            if parent:
                self.state.tf_frames.add(parent)
            if child:
                self.state.tf_frames.add(child)

    def _sensor_callback(self, topic: str):
        def callback(message) -> None:
            if topic not in self.state.sensor_frame_ids:
                self.state.sensor_frame_ids[topic] = message.header.frame_id

        return callback


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate Isaac Sim ROS2 topic types, sensor frame IDs, and TF frames.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Seconds to collect /clock, /tf, and sensor messages before reporting.",
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
            results.append(CheckResult(Level.FAIL, topic, "topic is missing"))
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


def check_sensor_frame_ids(state: ValidationState) -> list[CheckResult]:
    results = []
    for topic, expected_frame in EXPECTED_SENSOR_FRAMES.items():
        observed_frame = state.sensor_frame_ids.get(topic)
        if observed_frame is None:
            results.append(CheckResult(Level.FAIL, topic, "no message received"))
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


def check_tf_frames(state: ValidationState) -> list[CheckResult]:
    results = []
    if state.tf_messages_seen == 0:
        results.append(CheckResult(Level.FAIL, "/tf", "no TF messages received"))
    else:
        results.append(
            CheckResult(
                Level.PASS,
                "/tf",
                f"received {state.tf_messages_seen} messages with {len(state.tf_frames)} frames",
            )
        )

    for frame in sorted(REQUIRED_SENSOR_TF_FRAMES):
        if frame in state.tf_frames:
            results.append(CheckResult(Level.PASS, frame, "required sensor frame is in TF"))
        else:
            results.append(CheckResult(Level.FAIL, frame, "required sensor frame missing from TF"))

    for frame in sorted(IMPORTANT_TF_FRAMES):
        if frame in state.tf_frames:
            results.append(CheckResult(Level.PASS, frame, "important robot frame is in TF"))
        else:
            results.append(CheckResult(Level.WARN, frame, "important robot frame not observed"))

    for frame in ("base_link", "odom", "map"):
        if frame in state.tf_frames:
            results.append(
                CheckResult(
                    Level.WARN,
                    frame,
                    "not expected at Milestone #2; verify this was introduced intentionally",
                )
            )
        else:
            results.append(CheckResult(Level.PASS, frame, "not expected at this stage"))

    return results


def check_runtime_presence(state: ValidationState) -> list[CheckResult]:
    return [
        CheckResult(
            Level.PASS if state.clock_seen else Level.FAIL,
            "/clock",
            "messages received" if state.clock_seen else "no messages received",
        )
    ]


def print_section(title: str, results: Iterable[CheckResult]) -> Level:
    print(f"\n{title}")
    worst = Level.PASS
    for result in results:
        worst = max(worst, result.level)
        print(f"  {result.level.name:<4} {result.label}: {result.detail}")
    return worst


def print_contract_notes() -> None:
    print("\nContract notes")
    print("  Current TF is Isaac-derived and uses Isaac prim names.")
    print("  body is currently the base-like frame.")
    print("  base_link, odom, and map are not expected at this stage.")
    print("  SLAM/localization may later introduce map and odom.")
    print("  Add frame aliases only after this contract is stable.")


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = TfContractValidator()
    start_time = node.get_clock().now()

    try:
        while rclpy.ok():
            elapsed = (node.get_clock().now() - start_time).nanoseconds / 1_000_000_000
            if elapsed >= args.duration:
                break
            rclpy.spin_once(node, timeout_sec=args.spin_timeout)

        topic_results = check_topic_types(topic_type_map(node))
        runtime_results = check_runtime_presence(node.state)
        sensor_results = check_sensor_frame_ids(node.state)
        tf_results = check_tf_frames(node.state)

        print("Isaac Sim ROS2 TF Contract Validation")
        print(f"Collection duration: {args.duration:.1f} seconds")

        worst = Level.PASS
        worst = max(worst, print_section("Topic type checks", topic_results))
        worst = max(worst, print_section("Runtime message checks", runtime_results))
        worst = max(worst, print_section("Sensor frame_id checks", sensor_results))
        worst = max(worst, print_section("TF frame checks", tf_results))

        if node.state.tf_frames:
            print("\nObserved TF frames")
            for frame in sorted(node.state.tf_frames):
                print(f"  {frame}")

        print_contract_notes()

        print(f"\nOverall result: {worst.name}")
        return 1 if worst == Level.FAIL else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
