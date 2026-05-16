#!/usr/bin/env python3
import argparse
import os
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
import rosbag2_py

from auto_aim_interfaces.msg import Send
from rcl_interfaces.msg import Log
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32, UInt8


TOPIC_TYPES = {
    "/base/image_raw/compressed": (CompressedImage, "sensor_msgs/msg/CompressedImage"),
    "/outpost/image_raw/compressed": (CompressedImage, "sensor_msgs/msg/CompressedImage"),
    "/base/camera_info": (CameraInfo, "sensor_msgs/msg/CameraInfo"),
    "/outpost/camera_info": (CameraInfo, "sensor_msgs/msg/CameraInfo"),
    "/base/Send_pnp": (Send, "auto_aim_interfaces/msg/Send"),
    "/outpost/Send_pnp": (Send, "auto_aim_interfaces/msg/Send"),
    "/base/Send_fused": (Send, "auto_aim_interfaces/msg/Send"),
    "/outpost/Send_fused": (Send, "auto_aim_interfaces/msg/Send"),
    "/Send": (Send, "auto_aim_interfaces/msg/Send"),
    "/target_id": (UInt8, "std_msgs/msg/UInt8"),
    "/current_dart_id": (UInt8, "std_msgs/msg/UInt8"),
    "/offset": (Float32, "std_msgs/msg/Float32"),
    "/competition_mode": (UInt8, "std_msgs/msg/UInt8"),
    "/rosout": (Log, "rcl_interfaces/msg/Log"),
}

IMAGE_TOPIC_BY_ROLE = {
    "base": "/base/image_raw/compressed",
    "outpost": "/outpost/image_raw/compressed",
}

SMALL_TOPICS = [
    "/base/camera_info",
    "/outpost/camera_info",
    "/base/Send_pnp",
    "/outpost/Send_pnp",
    "/base/Send_fused",
    "/outpost/Send_fused",
    "/Send",
    "/target_id",
    "/current_dart_id",
    "/offset",
    "/competition_mode",
    "/rosout",
]


def stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SelectiveRosbagRecorder(Node):
    def __init__(self, output_dir: str, mode: str):
        super().__init__("selective_rosbag_recorder")
        self.output_dir = output_dir
        self.mode = mode
        self.active_role = "base"
        self.counts = {topic: 0 for topic in TOPIC_TYPES}

        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=output_dir, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        self.writer.open(storage_options, converter_options)

        for topic, (_, type_name) in TOPIC_TYPES.items():
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic,
                    type=type_name,
                    serialization_format="cdr",
                    offered_qos_profiles="",
                )
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        for topic in self.topics_to_subscribe():
            msg_type = TOPIC_TYPES[topic][0]
            self.create_subscription(msg_type, topic, self.make_callback(topic), qos)

        self.create_timer(10.0, self.report_counts)
        self.get_logger().info(
            f"Selective rosbag recorder started: mode={mode}, output={output_dir}"
        )

    def topics_to_subscribe(self):
        topics = list(SMALL_TOPICS)
        if self.mode == "base_only":
            topics.append(IMAGE_TOPIC_BY_ROLE["base"])
        elif self.mode == "outpost_only":
            topics.append(IMAGE_TOPIC_BY_ROLE["outpost"])
        else:
            topics.extend(IMAGE_TOPIC_BY_ROLE.values())
        return topics

    def make_callback(self, topic: str) -> Callable:
        if topic == "/target_id":
            return self.target_id_callback
        if topic == IMAGE_TOPIC_BY_ROLE["base"]:
            return lambda msg: self.image_callback("base", topic, msg)
        if topic == IMAGE_TOPIC_BY_ROLE["outpost"]:
            return lambda msg: self.image_callback("outpost", topic, msg)
        return lambda msg: self.write_message(topic, msg)

    def target_id_callback(self, msg: UInt8):
        self.write_message("/target_id", msg)
        if msg.data == 1:
            self.active_role = "base"
        elif msg.data == 0:
            self.active_role = "outpost"
        else:
            self.get_logger().warn(f"Unknown target_id={msg.data}; keeping {self.active_role}")

    def image_callback(self, role: str, topic: str, msg: CompressedImage):
        if self.should_record_image(role):
            self.write_message(topic, msg)

    def should_record_image(self, role: str) -> bool:
        if self.mode == "full":
            return True
        if self.mode == "active":
            return role == self.active_role
        if self.mode == "base_only":
            return role == "base"
        if self.mode == "outpost_only":
            return role == "outpost"
        return True

    def message_time(self, msg) -> int:
        if hasattr(msg, "header"):
            timestamp = stamp_to_nanoseconds(msg.header.stamp)
            if timestamp > 0:
                return timestamp
        if hasattr(msg, "stamp"):
            timestamp = stamp_to_nanoseconds(msg.stamp)
            if timestamp > 0:
                return timestamp
        return self.get_clock().now().nanoseconds

    def write_message(self, topic: str, msg):
        self.writer.write(topic, serialize_message(msg), self.message_time(msg))
        self.counts[topic] += 1

    def report_counts(self):
        nonzero = {topic: count for topic, count in self.counts.items() if count}
        image_counts = {
            role: nonzero.get(topic, 0)
            for role, topic in IMAGE_TOPIC_BY_ROLE.items()
        }
        self.get_logger().info(
            f"recorded images base={image_counts['base']} "
            f"outpost={image_counts['outpost']} active={self.active_role}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--mode",
        choices=["full", "active", "base_only", "outpost_only"],
        default="active",
    )
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    rclpy.init()
    node = SelectiveRosbagRecorder(args.output, args.mode)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
