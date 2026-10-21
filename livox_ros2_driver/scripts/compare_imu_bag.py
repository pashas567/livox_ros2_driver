#!/usr/bin/env python3
"""Compare IMU timestamps in a delivery probe CSV and an independent MCAP bag."""

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def read_bag_stamps(path: Path) -> list[int]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    stamps: list[int] = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/livox/imu":
            message = deserialize_message(data, Imu)
            stamps.append(
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
    return stamps


def compare(probe: list[int], bag: list[int]) -> dict[str, int]:
    if not probe or not bag:
        raise ValueError("Both probe and bag must contain IMU samples")
    start = max(min(probe), min(bag))
    end = min(max(probe), max(bag))
    probe_counts = Counter(stamp for stamp in probe if start <= stamp <= end)
    bag_counts = Counter(stamp for stamp in bag if start <= stamp <= end)
    return {
        "probe_in_overlap": sum(probe_counts.values()),
        "bag_in_overlap": sum(bag_counts.values()),
        "missing_from_probe": sum((bag_counts - probe_counts).values()),
        "missing_from_bag": sum((probe_counts - bag_counts).values()),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with args.csv.open() as stream:
        probe = [int(row["source_ns"]) for row in csv.DictReader(stream)]
    result = compare(probe, read_bag_stamps(args.bag))
    encoded = json.dumps(result, indent=2)
    args.output.write_text(encoded + "\n")
    print(encoded)


if __name__ == "__main__":
    main()
