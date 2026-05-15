#!/usr/bin/env python3
import os
import shutil
import sys
from bisect import bisect_right
from pathlib import Path

import cv2
import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
except ImportError:
    print("Error: failed to import ROS 2 libraries. Source ROS 2 and this workspace first.")
    sys.exit(1)


IMAGE_TOPICS = {
    "base": ["/base/image_raw/compressed", "/base/image_raw"],
    "outpost": ["/outpost/image_raw/compressed", "/outpost/image_raw"],
}

SEND_TOPIC_PRIORITY = {
    "base": ["/base/Send_fused", "/base/Send_pnp", "/Send"],
    "outpost": ["/outpost/Send_fused", "/outpost/Send_pnp", "/Send"],
}

SEND_TYPE = "auto_aim_interfaces/msg/Send"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
RAW_IMAGE_TYPE = "sensor_msgs/msg/Image"


def build_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    return reader, type_map


def decode_image(msg, msg_type):
    if msg_type == COMPRESSED_IMAGE_TYPE:
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if msg_type != RAW_IMAGE_TYPE:
        return None

    height = int(msg.height)
    width = int(msg.width)
    encoding = msg.encoding.lower()
    data = np.frombuffer(msg.data, np.uint8)

    if encoding in ("bgr8", "rgb8"):
        image = data.reshape((height, width, 3))
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    if encoding in ("mono8", "8uc1"):
        image = data.reshape((height, width))
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    return None


def is_no_target(send_msg):
    return (
        abs(float(send_msg.distance) - 666.0) < 1e-3
        and (
            abs(float(send_msg.angle) - 1234.0) < 1e-3
            or abs(float(send_msg.pixel_angle) - 1234.0) < 1e-3
        )
    )


def latest_before(buffer, timestamp):
    if not buffer:
        return None
    times, messages = buffer
    index = bisect_right(times, timestamp) - 1
    if index < 0:
        return None
    return messages[index]


def choose_send_topic(role, type_map):
    for topic in SEND_TOPIC_PRIORITY[role]:
        if type_map.get(topic) == SEND_TYPE:
            return topic
    return None


def draw_send_overlay(image, send_msg, role, send_topic):
    annotated = image.copy()
    height, width = annotated.shape[:2]

    if send_msg is None:
        cv2.putText(
            annotated,
            f"{role}: no Send",
            (24, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    no_target = is_no_target(send_msg)
    stable = int(send_msg.stability) != 0 and not no_target
    color = (0, 220, 0) if stable else ((0, 200, 255) if not no_target else (0, 0, 255))

    status = "stable" if stable else ("tracking" if not no_target else "no target")
    cv2.putText(
        annotated,
        f"{role} {status}",
        (24, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        color,
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        annotated,
        f"dist={float(send_msg.distance):.2f} angle={float(send_msg.angle):.3f} pix={float(send_msg.pixel_angle):.3f}",
        (24, 78),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        annotated,
        send_topic,
        (24, height - 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )

    if no_target:
        return annotated

    center_x = int(round(float(send_msg.u)))
    center_y = int(round(float(send_msg.v)))
    radius = int(round(float(send_msg.roi_radius)))
    if 0 <= center_x < width and 0 <= center_y < height:
        draw_radius = max(radius, 8)
        cv2.circle(annotated, (center_x, center_y), draw_radius, color, 2)
        cv2.circle(annotated, (center_x, center_y), 3, (0, 0, 255), -1)
        cv2.line(annotated, (center_x - 12, center_y), (center_x + 12, center_y), color, 1)
        cv2.line(annotated, (center_x, center_y - 12), (center_x, center_y + 12), color, 1)

    return annotated


def write_rosout(log_file, msg):
    levels = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
    level_str = levels.get(msg.level, "UNKNOWN")
    log_file.write(
        f"[{msg.stamp.sec}.{msg.stamp.nanosec:09d}] "
        f"[{level_str}] [{msg.name}]: {msg.msg}\n"
    )


def process_bag(bag_path):
    if not os.path.exists(bag_path):
        print(f"Error: bag path '{bag_path}' does not exist.")
        sys.exit(1)

    extract_fps = float(os.environ.get("EXTRACT_FPS", "60.0"))
    progress_interval = int(os.environ.get("EXTRACT_PROGRESS_FRAMES", "300"))
    if progress_interval <= 0:
        progress_interval = 300
    final_out_dir = Path(bag_path.rstrip("/") + "_extracted")
    out_dir = Path(bag_path.rstrip("/") + "_extracting")
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = os.path.join(out_dir, "rosout.txt")

    print(f"Processing bag: {bag_path}", flush=True)
    print("Pass 1: buffering Send messages...", flush=True)

    reader_pass1, type_map = build_reader(bag_path)
    send_buffers = {
        topic: []
        for topic, topic_type in type_map.items()
        if topic_type == SEND_TYPE
    }

    message_types = {}
    for topic, topic_type in type_map.items():
        try:
            message_types[topic] = get_message(topic_type)
        except (AttributeError, ModuleNotFoundError, ValueError):
            print(f"Warning: cannot load message type {topic_type} for {topic}")

    while reader_pass1.has_next():
        topic, data, timestamp = reader_pass1.read_next()
        if topic not in send_buffers or topic not in message_types:
            continue
        msg = deserialize_message(data, message_types[topic])
        send_buffers[topic].append((timestamp, msg))

    for buffer in send_buffers.values():
        buffer.sort(key=lambda item: item[0])

    indexed_send_buffers = {
        topic: (
            [item[0] for item in buffer],
            [item[1] for item in buffer],
        )
        for topic, buffer in send_buffers.items()
    }

    for topic, buffer in send_buffers.items():
        print(f"  {topic}: {len(buffer)} messages", flush=True)

    send_topic_for_role = {
        role: choose_send_topic(role, type_map)
        for role in IMAGE_TOPICS
    }
    for role, send_topic in send_topic_for_role.items():
        print(f"  {role} overlay source: {send_topic or 'no Send topic'}", flush=True)

    print("Pass 2: extracting images, overlays, and rosout...", flush=True)
    reader_pass2, type_map = build_reader(bag_path)
    video_writers = {}
    frame_counts = {role: 0 for role in IMAGE_TOPICS}
    processed_frames = 0
    log_count = 0

    image_topic_to_role = {}
    for role, topics in IMAGE_TOPICS.items():
        for topic in topics:
            if topic in type_map:
                image_topic_to_role[topic] = role

    try:
        with open(log_path, "w", encoding="utf-8") as log_file:
            while reader_pass2.has_next():
                topic, data, timestamp = reader_pass2.read_next()
                if topic not in message_types:
                    continue

                msg = deserialize_message(data, message_types[topic])

                if topic in image_topic_to_role:
                    role = image_topic_to_role[topic]
                    image = decode_image(msg, type_map[topic])
                    if image is None:
                        continue

                    height, width = image.shape[:2]
                    if role not in video_writers:
                        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                        raw_path = os.path.join(out_dir, f"video_{role}_raw.mp4")
                        annotated_path = os.path.join(out_dir, f"video_{role}_annotated.mp4")
                        video_writers[role] = (
                            cv2.VideoWriter(raw_path, fourcc, extract_fps, (width, height)),
                            cv2.VideoWriter(annotated_path, fourcc, extract_fps, (width, height)),
                        )
                        if not video_writers[role][0].isOpened() or not video_writers[role][1].isOpened():
                            raise RuntimeError(f"failed to open video writers for {role}")

                    send_topic = send_topic_for_role.get(role)
                    send_msg = latest_before(indexed_send_buffers.get(send_topic), timestamp)
                    annotated = draw_send_overlay(image, send_msg, role, send_topic or "no Send topic")

                    raw_writer, annotated_writer = video_writers[role]
                    raw_writer.write(image)
                    annotated_writer.write(annotated)
                    frame_counts[role] += 1
                    processed_frames += 1

                    if processed_frames % progress_interval == 0:
                        print(
                            "  extracting frames: "
                            f"base={frame_counts['base']}, "
                            f"outpost={frame_counts['outpost']}, "
                            f"rosout={log_count}",
                            flush=True,
                        )

                elif topic == "/rosout":
                    write_rosout(log_file, msg)
                    log_count += 1
    finally:
        for raw_writer, annotated_writer in video_writers.values():
            raw_writer.release()
            annotated_writer.release()

    if final_out_dir.exists():
        backup_dir = Path(str(final_out_dir) + "_replaced")
        if backup_dir.exists():
            shutil.rmtree(backup_dir)
        final_out_dir.rename(backup_dir)
        shutil.rmtree(backup_dir)
    out_dir.rename(final_out_dir)

    print("Extraction complete.", flush=True)
    for role, frame_count in frame_counts.items():
        if frame_count:
            print(f" - {role}: extracted {frame_count} frames", flush=True)
    print(f" - extracted {log_count} log entries to {final_out_dir / 'rosout.txt'}", flush=True)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_bag.py <path_to_bag_dir>")
        print("Example: python3 extract_bag.py /home/pnx/rosbag/rmvision_dart/rmvision_dart_20260515_182905")
        sys.exit(1)
    process_bag(sys.argv[1])
