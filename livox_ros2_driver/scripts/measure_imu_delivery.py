#!/usr/bin/env python3
"""Measure source timing and local ROS delivery using independent clocks.

Run on the driver host with its ROS environment sourced. Cloud callbacks use
serialized messages so point deserialization cannot delay IMU callbacks.
"""

import argparse
import csv
import json
import math
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from livox_interfaces.msg import CustomMsg

Sample = tuple[int, int, int]


def distribution(values: list[float]) -> dict[str, float]:
    if not values:
        return {}
    ordered = sorted(values)
    result = {"min": ordered[0], "max": ordered[-1]}
    for percentile in (50, 95, 99):
        index = math.ceil(len(ordered) * percentile / 100) - 1
        result[f"p{percentile}"] = ordered[index]
    return result


def summarize(
    samples: list[Sample], period_ns: int = 5_000_000
) -> dict[str, object]:
    if len(samples) < 2:
        return {"count": len(samples)}
    source = [sample[2] for sample in samples]
    source_gaps = [b - a for a, b in zip(source, source[1:])]
    wall_gaps = [(b[0] - a[0]) / 1e6 for a, b in zip(samples, samples[1:])]
    return {
        "count": len(samples),
        "callback_rate_hz": (len(samples) - 1)
        * 1e9
        / (samples[-1][0] - samples[0][0]),
        "source_rate_hz": (len(samples) - 1) * 1e9 / (source[-1] - source[0]),
        "callback_gap_ms": distribution(wall_gaps),
        "source_gap_ms": distribution([gap / 1e6 for gap in source_gaps]),
        "age_ms": distribution(
            [(sample[1] - sample[2]) / 1e6 for sample in samples]
        ),
        "pauses": {
            f"over_{limit}_ms": sum(gap > limit for gap in wall_gaps)
            for limit in (10, 15, 25, 50)
        },
        "duplicates": len(source) - len(set(source)),
        "nonmonotonic": sum(gap <= 0 for gap in source_gaps),
        "missing_nominal_slots": sum(
            max(0, round(gap / period_ns) - 1) for gap in source_gaps
        ),
    }


class DeliveryProbe(Node):
    def __init__(self) -> None:
        super().__init__("livox_delivery_probe")
        self.imu: list[Sample] = []
        self.clouds: list[Sample] = []
        self.clocks: list[str] = []
        self.cloud_example = b""
        self.subscriptions_held = [
            self.create_subscription(
                Imu, "/livox/imu", self.on_imu, QoSProfile(depth=2048)
            ),
            self.create_subscription(
                CustomMsg,
                "/livox/lidar",
                self.on_cloud,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
                raw=True,
            ),
            self.create_subscription(
                String, "/livox/clock_status", self.on_clock, 100
            ),
        ]

    def on_imu(self, message: Imu) -> None:
        self.imu.append(
            (
                time.monotonic_ns(),
                time.time_ns(),
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec,
            )
        )

    def on_cloud(self, message: bytes) -> None:
        # CDR encapsulation is followed by Header.stamp (seconds, nanoseconds).
        byte_order = "<" if message[1] & 1 else ">"
        seconds, nanoseconds = struct.unpack_from(
            byte_order + "iI", message, 4
        )
        self.clouds.append(
            (
                time.monotonic_ns(),
                time.time_ns(),
                seconds * 1_000_000_000 + nanoseconds,
            )
        )
        self.cloud_example = message

    def on_clock(self, message: String) -> None:
        self.clocks.append(message.data)


def save_results(probe: DeliveryProbe, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=False)
    for name, samples in (("imu", probe.imu), ("cloud", probe.clouds)):
        with (output / f"{name}.csv").open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(("monotonic_ns", "wall_ns", "source_ns"))
            writer.writerows(samples)
    (output / "clock.jsonl").write_text("\n".join(probe.clocks) + "\n")
    summary = {
        "imu": summarize(probe.imu),
        "cloud": summarize(probe.clouds, period_ns=100_000_000),
    }
    if probe.cloud_example:
        cloud = deserialize_message(probe.cloud_example, CustomMsg)
        summary["cloud"]["example"] = {
            "point_num": cloud.point_num,
            "first_offset_ns": cloud.points[0].offset_time,
            "last_offset_ns": cloud.points[-1].offset_time,
        }
    encoded = json.dumps(summary, indent=2)
    (output / "summary.json").write_text(encoded + "\n")
    print(encoded, flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--warmup-seconds", type=float, default=3)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    probe = DeliveryProbe()
    try:
        warmup_end = time.monotonic() + args.warmup_seconds
        while time.monotonic() < warmup_end:
            rclpy.spin_once(probe, timeout_sec=0.05)
        probe.imu.clear()
        probe.clouds.clear()
        probe.clocks.clear()
        end = time.monotonic() + args.seconds
        print("MEASUREMENT_STARTED", flush=True)
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.05)
        save_results(probe, args.output)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
