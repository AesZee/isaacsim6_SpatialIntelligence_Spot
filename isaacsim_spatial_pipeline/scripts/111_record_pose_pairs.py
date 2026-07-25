#!/usr/bin/env python3
"""Record bounded Isaac TF ground truth and simulation-odometry pose CSVs."""

import argparse
import csv
import json
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


FIELDS = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]


def seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class PoseRecorder(Node):
    def __init__(self) -> None:
        super().__init__("m11_pose_pair_recorder")
        self.ground_truth = []
        self.estimate = []
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id == "world" and transform.child_frame_id == "body":
                translation, rotation = transform.transform.translation, transform.transform.rotation
                self.ground_truth.append(
                    [seconds(transform.header.stamp), translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
                )

    def on_odom(self, message: Odometry) -> None:
        position, rotation = message.pose.pose.position, message.pose.pose.orientation
        self.estimate.append(
            [seconds(message.header.stamp), position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w]
        )


def write_csv(path: Path, rows: list[list[float]]) -> None:
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--trajectory-id", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    if args.duration <= 0:
        raise ValueError("duration must be positive")
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = PoseRecorder()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    write_csv(output / "ground_truth.csv", node.ground_truth)
    write_csv(output / "estimate.csv", node.estimate)
    result = "PASS" if len(node.ground_truth) >= 2 and len(node.estimate) >= 2 else "FAIL"
    (output / "metadata.json").write_text(
        json.dumps(
            {
                "result": result,
                "trajectory_id": args.trajectory_id,
                "duration_sec": args.duration,
                "ground_truth_frame": "world -> body",
                "estimate_topic": "/odom",
                "ground_truth_samples": len(node.ground_truth),
                "estimate_samples": len(node.estimate),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"Overall result: {result}")
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
