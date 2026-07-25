#!/usr/bin/env python3
"""Measure stationary map-to-odom localization convergence."""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class LocalizationObserver(Node):
    def __init__(self) -> None:
        super().__init__("m12_localization_observer")
        self.started = time.monotonic()
        self.samples = []
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos_profile_sensor_data)

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                t, q = transform.transform.translation, transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                stamp = transform.header.stamp
                self.samples.append(
                    {
                        "wall_sec": time.monotonic() - self.started,
                        "sim_sec": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
                        "x": t.x,
                        "y": t.y,
                        "yaw": yaw,
                    }
                )


def measure(
    samples: list[dict],
    duration: float,
    expected: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> dict:
    final = samples[len(samples) // 2 :] if samples else []
    median = {
        key: statistics.median(sample[key] for sample in final)
        for key in ("x", "y", "yaw")
    } if final else {"x": 0.0, "y": 0.0, "yaw": 0.0}
    drift_translation = max(
        (math.hypot(sample["x"] - median["x"], sample["y"] - median["y"]) for sample in final),
        default=math.inf,
    )
    drift_yaw = max(
        (abs(wrap(sample["yaw"] - median["yaw"])) for sample in final),
        default=math.inf,
    )
    residual_translation = math.hypot(median["x"] - expected[0], median["y"] - expected[1])
    residual_yaw = abs(wrap(median["yaw"] - expected[2]))
    result = (
        "PASS"
        if len(samples) >= 5
        and samples[0]["wall_sec"] <= min(5.0, duration)
        and samples[-1]["sim_sec"] > samples[0]["sim_sec"]
        and drift_translation <= 0.10
        and drift_yaw <= 0.10
        and residual_translation <= 0.25
        and residual_yaw <= 0.25
        else "FAIL"
    )
    return {
        "result": result,
        "criteria": {
            "first_transform_sec_max": min(5.0, duration),
            "stationary_translation_drift_m_max": 0.10,
            "stationary_yaw_drift_rad_max": 0.10,
            "residual_translation_m_max": 0.25,
            "residual_yaw_rad_max": 0.25,
        },
        "sample_count": len(samples),
        "expected_transform": {"x": expected[0], "y": expected[1], "yaw": expected[2]},
        "first_transform_wall_sec": samples[0]["wall_sec"] if samples else None,
        "simulation_span_sec": (
            samples[-1]["sim_sec"] - samples[0]["sim_sec"] if len(samples) >= 2 else 0.0
        ),
        "final_median": median,
        "stationary_translation_drift_m": drift_translation,
        "stationary_yaw_drift_rad": drift_yaw,
        "residual_translation_m": residual_translation,
        "residual_yaw_rad": residual_yaw,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--expected-x", type=float, default=0.0)
    parser.add_argument("--expected-y", type=float, default=0.0)
    parser.add_argument("--expected-yaw", type=float, default=0.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        raise ValueError("duration must be positive")
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    rclpy.init()
    node = LocalizationObserver()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    result = measure(
        node.samples,
        args.duration,
        (args.expected_x, args.expected_y, args.expected_yaw),
    )
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(f"Overall result: {result['result']}")
    return 0 if result["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
