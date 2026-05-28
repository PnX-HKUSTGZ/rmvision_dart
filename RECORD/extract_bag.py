#!/usr/bin/env python3
import os
import shutil
import sys
from bisect import bisect_right
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
    import sensor_msgs_py.point_cloud2 as point_cloud2
except ImportError:
    print("Error: failed to import ROS 2 libraries. Source ROS 2 and this workspace first.")
    sys.exit(1)


IMAGE_TOPICS = {
    "base": ["/base/image_raw/compressed", "/base/image_raw"],
    "outpost": ["/outpost/image_raw/compressed", "/outpost/image_raw"],
}

RESULT_IMAGE_TOPICS = {
    "base": ["/base/detector/result_img/compressed", "/base/detector/result_img"],
    "outpost": ["/outpost/detector/result_img/compressed", "/outpost/detector/result_img"],
}

SEND_TOPIC_PRIORITY = {
    "base": ["/base/Send_fused", "/base/Send_pnp", "/Send"],
    "outpost": ["/outpost/Send_fused", "/outpost/Send_pnp", "/Send"],
}

SEND_TYPE = "auto_aim_interfaces/msg/Send"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
RAW_IMAGE_TYPE = "sensor_msgs/msg/Image"
POINT_CLOUD_TYPE = "sensor_msgs/msg/PointCloud2"
CLOUD_TOPIC = "/livox/accum_points"

REPO_ROOT = Path(__file__).resolve().parents[1]
BRINGUP_CONFIG_DIR = REPO_ROOT / "src/vision_bringup/rm_vision_bringup/config"


def bag_uses_compression(bag_path):
    metadata_path = Path(bag_path) / "metadata.yaml"
    if not metadata_path.exists():
        return False
    metadata = load_yaml(metadata_path).get("rosbag2_bagfile_information", {})
    return bool(metadata.get("compression_format") or metadata.get("compression_mode"))


def build_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = (
        rosbag2_py.SequentialCompressionReader()
        if bag_uses_compression(bag_path)
        else rosbag2_py.SequentialReader()
    )
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


def rpy_to_quaternion(rpy):
    roll, pitch, yaw = [float(value) for value in rpy.split()]
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ], dtype=np.float64)


def quaternion_to_matrix(q):
    x, y, z, w = [float(value) for value in q]
    norm = x * x + y * y + z * z + w * w
    if norm <= 0.0:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx = x * x * scale
    yy = y * y * scale
    zz = z * z * scale
    xy = x * y * scale
    xz = x * z * scale
    yz = y * z * scale
    wx = w * x * scale
    wy = w * y * scale
    wz = w * z * scale
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ], dtype=np.float64)


def transform_matrix(transform):
    xyz = [float(value) for value in transform.get("xyz", "0 0 0").split()]
    if "q" in transform:
        q = [float(value) for value in transform["q"].split()]
    else:
        q = rpy_to_quaternion(transform.get("rpy", "0 0 0"))
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_matrix(q)
    matrix[:3, 3] = xyz
    return matrix


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def camera_to_livox_transform(camera_config, active_lens_config):
    return transform_matrix(camera_config.get("camera_to_livox", active_lens_config["camera_to_livox"]))


def load_projection_configs():
    launch_params = load_yaml(BRINGUP_CONFIG_DIR / "launch_params.yaml")
    cameras = launch_params.get("cameras", {})
    active_lens = launch_params["active_lens_profile"]
    active_lens_config = launch_params["lens_profiles"][active_lens]
    base_camera = cameras.get("base", {})
    outpost_camera = cameras.get("outpost", {})

    odom_to_gimbal = transform_matrix({"xyz": "0.12 0.06 0.04", "rpy": "0 0 0"})
    gimbal_to_base = transform_matrix(launch_params["odom2camera"])
    base_to_livox = camera_to_livox_transform(base_camera, active_lens_config)
    outpost_to_livox = camera_to_livox_transform(outpost_camera, active_lens_config)
    gimbal_to_livox = gimbal_to_base @ base_to_livox
    gimbal_to_outpost = gimbal_to_livox @ np.linalg.inv(outpost_to_livox)
    link_to_optical = transform_matrix({
        "xyz": "0 0 0",
        "rpy": "-1.5707963267948966 0 -1.5707963267948966",
    })

    role_transforms = {
        "base": np.linalg.inv(odom_to_gimbal @ gimbal_to_base @ link_to_optical),
        "outpost": np.linalg.inv(odom_to_gimbal @ gimbal_to_outpost @ link_to_optical),
    }

    configs = {}
    for role, camera_config in (("base", base_camera), ("outpost", outpost_camera)):
        camera_info_url = camera_config.get("camera_info_url", "")
        camera_info_name = camera_info_url.rsplit("/", 1)[-1]
        camera_info = load_yaml(BRINGUP_CONFIG_DIR / camera_info_name)
        configs[role] = {
            "odom_to_optical_inv": role_transforms[role],
            "camera_matrix": np.array(
                camera_info["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3),
        }
    return configs


def cloud_to_xyz_array(cloud_msg):
    points = point_cloud2.read_points(
        cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    if isinstance(points, np.ndarray):
        if points.dtype.names:
            return np.column_stack((
                points["x"].astype(np.float64),
                points["y"].astype(np.float64),
                points["z"].astype(np.float64),
            ))
        return points.astype(np.float64).reshape(-1, 3)
    return np.array(list(points), dtype=np.float64).reshape(-1, 3)


def project_cloud_points(cloud_msg, role, projection_configs, max_points):
    if cloud_msg is None or role not in projection_configs:
        return None
    if cloud_msg.header.frame_id and cloud_msg.header.frame_id != "odom":
        return None

    xyz = cloud_to_xyz_array(cloud_msg)
    if xyz.size == 0:
        return None
    if max_points > 0 and xyz.shape[0] > max_points:
        step = int(np.ceil(xyz.shape[0] / max_points))
        xyz = xyz[::step]

    transform = projection_configs[role]["odom_to_optical_inv"]
    homogeneous = np.ones((xyz.shape[0], 4), dtype=np.float64)
    homogeneous[:, :3] = xyz
    camera_points = (transform @ homogeneous.T).T[:, :3]

    z = camera_points[:, 2]
    valid = z > 0.0
    if not np.any(valid):
        return None
    camera_points = camera_points[valid]
    z = z[valid]

    camera_matrix = projection_configs[role]["camera_matrix"]
    u = camera_matrix[0, 0] * camera_points[:, 0] / z + camera_matrix[0, 2]
    v = camera_matrix[1, 1] * camera_points[:, 1] / z + camera_matrix[1, 2]
    return np.column_stack((u, v, z))


def draw_cloud_overlay(image, projected_points):
    annotated = image.copy()
    if projected_points is None:
        cv2.putText(
            annotated,
            "no cloud",
            (24, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    height, width = annotated.shape[:2]
    in_view = (
        (projected_points[:, 0] >= 0.0)
        & (projected_points[:, 1] >= 0.0)
        & (projected_points[:, 0] < width)
        & (projected_points[:, 1] < height)
    )
    for u, v, depth in projected_points[in_view]:
        if depth < 10.0:
            color = (0, 255, 255)
        elif depth < 20.0:
            color = (0, 255, 0)
        else:
            color = (255, 180, 0)
        cv2.circle(annotated, (int(u), int(v)), 1, color, -1)
    cv2.putText(
        annotated,
        f"cloud points={int(np.count_nonzero(in_view))}",
        (24, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return annotated


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


def env_bool(name, default):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in ("1", "true", "yes", "on")


def load_topic_message_counts(bag_path):
    metadata_path = Path(bag_path) / "metadata.yaml"
    if not metadata_path.exists():
        return {}
    metadata = load_yaml(metadata_path).get("rosbag2_bagfile_information", {})
    counts = {}
    for entry in metadata.get("topics_with_message_count", []):
        topic_metadata = entry.get("topic_metadata", {})
        name = topic_metadata.get("name")
        if name:
            counts[name] = int(entry.get("message_count", 0))
    return counts


def metadata_time_to_ns(value):
    if isinstance(value, dict):
        for key in ("nanoseconds_since_epoch", "nanoseconds"):
            if key in value:
                return int(value[key])
    if value is None:
        return None
    return int(value)


def load_bag_start_time_ns(bag_path):
    metadata_path = Path(bag_path) / "metadata.yaml"
    if metadata_path.exists():
        metadata = load_yaml(metadata_path).get("rosbag2_bagfile_information", {})
        start_ns = metadata_time_to_ns(metadata.get("starting_time"))
        if start_ns is not None:
            return start_ns

    reader, _ = build_reader(bag_path)
    if reader.has_next():
        _, _, timestamp = reader.read_next()
        return int(timestamp)
    return None


def env_float(name):
    value = os.environ.get(name)
    if value is None or value == "":
        return None
    return float(value)


def format_seconds_label(seconds):
    label = f"{seconds:.3f}".rstrip("0").rstrip(".")
    return label.replace(".", "p")


def build_output_suffix(range_enabled, start_sec, end_sec):
    explicit_suffix = os.environ.get("EXTRACT_OUTPUT_SUFFIX")
    if explicit_suffix:
        return explicit_suffix if explicit_suffix.startswith("_") else f"_{explicit_suffix}"
    if not range_enabled:
        return ""

    start_label = format_seconds_label(start_sec)
    if end_sec is None:
        return f"_start_{start_label}s"
    duration_label = format_seconds_label(max(0.0, end_sec - start_sec))
    return f"_start_{start_label}s_dur_{duration_label}s"


def seek_reader(reader, timestamp_ns):
    if timestamp_ns is None:
        return
    try:
        reader.seek(int(timestamp_ns))
    except AttributeError:
        print("Warning: this rosbag2 reader does not support seek; scanning from bag start.", flush=True)
    except RuntimeError as exc:
        print(f"Warning: failed to seek reader to {timestamp_ns}: {exc}", flush=True)


def process_bag(bag_path):
    if not os.path.exists(bag_path):
        print(f"Error: bag path '{bag_path}' does not exist.")
        sys.exit(1)

    start_sec = env_float("EXTRACT_START_SEC")
    duration_sec = env_float("EXTRACT_DURATION_SEC")
    end_sec = env_float("EXTRACT_END_SEC")
    if start_sec is None:
        start_sec = 0.0
    if duration_sec is not None:
        end_sec = start_sec + duration_sec
    range_enabled = (
        os.environ.get("EXTRACT_START_SEC") is not None
        or duration_sec is not None
        or os.environ.get("EXTRACT_END_SEC") is not None
    )
    if end_sec is not None and end_sec <= start_sec:
        print("Error: extraction end time must be greater than start time.")
        sys.exit(1)

    bag_start_ns = load_bag_start_time_ns(bag_path) if range_enabled else None
    range_start_ns = None
    range_end_ns = None
    if range_enabled:
        if bag_start_ns is None:
            print("Error: failed to determine bag start time for ranged extraction.")
            sys.exit(1)
        range_start_ns = bag_start_ns + int(start_sec * 1_000_000_000)
        if end_sec is not None:
            range_end_ns = bag_start_ns + int(end_sec * 1_000_000_000)

    extract_fps = float(os.environ.get("EXTRACT_FPS", "60.0"))
    extract_cloud_max_points = int(os.environ.get("EXTRACT_CLOUD_MAX_POINTS", "50000"))
    extract_cloud_every_n_frames = int(os.environ.get("EXTRACT_CLOUD_EVERY_N_FRAMES", "1"))
    if extract_cloud_every_n_frames <= 0:
        extract_cloud_every_n_frames = 1
    write_raw_video = env_bool("EXTRACT_RAW_VIDEO", True)
    write_annotated_video = env_bool("EXTRACT_ANNOTATED_VIDEO", True)
    write_cloud_video = env_bool("EXTRACT_CLOUD_VIDEO", True)
    write_result_video = env_bool("EXTRACT_RESULT_VIDEO", True)
    write_rosout_log = env_bool("EXTRACT_ROSOUT", True)
    process_image_messages = write_raw_video or write_annotated_video or write_cloud_video
    progress_interval = int(os.environ.get("EXTRACT_PROGRESS_FRAMES", "300"))
    if progress_interval <= 0:
        progress_interval = 300
    output_suffix = build_output_suffix(range_enabled, start_sec, end_sec)
    final_out_dir = Path(bag_path.rstrip("/") + "_extracted" + output_suffix)
    out_dir = Path(bag_path.rstrip("/") + "_extracting" + output_suffix)
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = os.path.join(out_dir, "rosout.txt")

    print(f"Processing bag: {bag_path}", flush=True)
    print(
        "Extraction options: "
        f"raw={write_raw_video}, annotated={write_annotated_video}, "
        f"cloud={write_cloud_video}, result={write_result_video}, "
        f"rosout={write_rosout_log}, cloud_every_n_frames={extract_cloud_every_n_frames}",
        flush=True,
    )
    if range_enabled:
        end_label = "bag end" if end_sec is None else f"{end_sec:.3f}s"
        print(
            f"Extraction range: start={start_sec:.3f}s, end={end_label}, "
            f"output_suffix={output_suffix}",
            flush=True,
        )
    print("Pass 1: reading metadata and buffering Send messages if needed...", flush=True)

    reader_pass1, type_map = build_reader(bag_path)
    send_buffers = {
        topic: []
        for topic, topic_type in type_map.items()
        if topic_type == SEND_TYPE
    }
    if not write_annotated_video:
        send_buffers = {}

    message_types = {}
    for topic, topic_type in type_map.items():
        try:
            message_types[topic] = get_message(topic_type)
        except (AttributeError, ModuleNotFoundError, ValueError):
            print(f"Warning: cannot load message type {topic_type} for {topic}")

    image_topics_in_bag = {
        topic
        for topics in IMAGE_TOPICS.values()
        for topic in topics
        if topic in type_map
    }
    result_image_topics_in_bag = {
        topic
        for topics in RESULT_IMAGE_TOPICS.values()
        for topic in topics
        if topic in type_map
    }
    pass2_totals = {
        "images": 0,
        "result_images": 0,
        "clouds": 0,
        "rosout": 0,
    }
    topic_message_counts = load_topic_message_counts(bag_path)
    if topic_message_counts and not range_enabled:
        if process_image_messages:
            pass2_totals["images"] = sum(
                topic_message_counts.get(topic, 0) for topic in image_topics_in_bag)
        if write_result_video:
            pass2_totals["result_images"] = sum(
                topic_message_counts.get(topic, 0) for topic in result_image_topics_in_bag)
        if write_cloud_video:
            pass2_totals["clouds"] = topic_message_counts.get(CLOUD_TOPIC, 0)
        if write_rosout_log:
            pass2_totals["rosout"] = topic_message_counts.get("/rosout", 0)

    need_pass1_scan = bool(send_buffers) or (not topic_message_counts and not range_enabled)
    count_messages_in_pass1 = need_pass1_scan and (range_enabled or not topic_message_counts)

    seek_reader(reader_pass1, range_start_ns)
    while need_pass1_scan and reader_pass1.has_next():
        topic, data, timestamp = reader_pass1.read_next()
        if range_start_ns is not None and timestamp < range_start_ns:
            continue
        if range_end_ns is not None and timestamp >= range_end_ns:
            break

        if count_messages_in_pass1:
            if process_image_messages and topic in image_topics_in_bag:
                pass2_totals["images"] += 1
            elif write_result_video and topic in result_image_topics_in_bag:
                pass2_totals["result_images"] += 1
            elif write_cloud_video and topic == CLOUD_TOPIC:
                pass2_totals["clouds"] += 1
            elif write_rosout_log and topic == "/rosout":
                pass2_totals["rosout"] += 1

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

    if send_buffers:
        for topic, buffer in send_buffers.items():
            print(f"  {topic}: {len(buffer)} messages", flush=True)
    else:
        print("  Send buffering skipped", flush=True)
    pass2_total_messages = sum(pass2_totals.values())
    print(
        "  extraction totals: "
        f"images={pass2_totals['images']}, "
        f"result_images={pass2_totals['result_images']}, "
        f"cloud_msgs={pass2_totals['clouds']}, "
        f"rosout={pass2_totals['rosout']}, "
        f"total={pass2_total_messages}",
        flush=True,
    )

    send_topic_for_role = {
        role: choose_send_topic(role, type_map)
        for role in IMAGE_TOPICS
    } if write_annotated_video else {}
    for role, send_topic in send_topic_for_role.items():
        print(f"  {role} overlay source: {send_topic or 'no Send topic'}", flush=True)

    projection_configs = {}
    if write_cloud_video and type_map.get(CLOUD_TOPIC) == POINT_CLOUD_TYPE:
        try:
            projection_configs = load_projection_configs()
            print(
                f"  cloud overlay source: {CLOUD_TOPIC}, max_draw_points={extract_cloud_max_points}",
                flush=True,
            )
        except Exception as exc:
            print(f"Warning: failed to load cloud projection config: {exc}", flush=True)

    print("Pass 2: extracting images, overlays, cloud overlays, and rosout...", flush=True)
    reader_pass2, type_map = build_reader(bag_path)
    video_writers = {}
    raw_video_writers = {}
    annotated_video_writers = {}
    result_video_writers = {}
    cloud_video_writers = {}
    frame_counts = {role: 0 for role in IMAGE_TOPICS}
    result_frame_counts = {role: 0 for role in RESULT_IMAGE_TOPICS}
    cloud_frame_counts = {role: 0 for role in IMAGE_TOPICS}
    processed_frames = 0
    log_count = 0
    cloud_count = 0
    processed_pass2_messages = 0
    next_progress_message = progress_interval
    latest_cloud_msg = None
    projected_cloud_cache = {}
    image_frame_indices = {role: 0 for role in IMAGE_TOPICS}

    def report_progress(force=False):
        nonlocal next_progress_message
        if not force and processed_pass2_messages < next_progress_message:
            return
        if pass2_total_messages > 0:
            percent = min(100.0, processed_pass2_messages * 100.0 / pass2_total_messages)
            progress_prefix = (
                f"  extracting progress: {percent:5.1f}% "
                f"({processed_pass2_messages}/{pass2_total_messages})"
            )
        else:
            progress_prefix = f"  extracting progress: {processed_pass2_messages} messages"
        print(
            progress_prefix + " "
            f"base={frame_counts['base']}, "
            f"outpost={frame_counts['outpost']}, "
            f"result_base={result_frame_counts['base']}, "
            f"result_outpost={result_frame_counts['outpost']}, "
            f"cloud_frames={cloud_frame_counts['base'] + cloud_frame_counts['outpost']}, "
            f"cloud_msgs={cloud_count}, "
            f"rosout={log_count}",
            flush=True,
        )
        while next_progress_message <= processed_pass2_messages:
            next_progress_message += progress_interval

    image_topic_to_role = {}
    for role, topics in IMAGE_TOPICS.items():
        for topic in topics:
            if topic in type_map:
                image_topic_to_role[topic] = role

    result_image_topic_to_role = {}
    for role, topics in RESULT_IMAGE_TOPICS.items():
        for topic in topics:
            if topic in type_map:
                result_image_topic_to_role[topic] = role

    try:
        with open(log_path, "w", encoding="utf-8") as log_file:
            seek_reader(reader_pass2, range_start_ns)
            while reader_pass2.has_next():
                topic, data, timestamp = reader_pass2.read_next()
                if range_start_ns is not None and timestamp < range_start_ns:
                    continue
                if range_end_ns is not None and timestamp >= range_end_ns:
                    break
                if topic not in message_types:
                    continue

                msg = deserialize_message(data, message_types[topic])

                if topic == CLOUD_TOPIC:
                    if not write_cloud_video:
                        continue
                    processed_pass2_messages += 1
                    latest_cloud_msg = msg
                    projected_cloud_cache = {}
                    cloud_count += 1
                    report_progress()

                elif topic in image_topic_to_role:
                    if not process_image_messages:
                        continue
                    processed_pass2_messages += 1
                    role = image_topic_to_role[topic]
                    image_frame_indices[role] += 1
                    image = decode_image(msg, type_map[topic])
                    if image is None:
                        report_progress()
                        continue

                    height, width = image.shape[:2]
                    if role not in video_writers:
                        video_writers[role] = True
                        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                        if write_raw_video:
                            raw_path = os.path.join(out_dir, f"video_{role}_raw.mp4")
                            raw_video_writers[role] = cv2.VideoWriter(
                                raw_path, fourcc, extract_fps, (width, height))
                            if not raw_video_writers[role].isOpened():
                                raise RuntimeError(f"failed to open raw video writer for {role}")
                        if write_annotated_video:
                            annotated_path = os.path.join(out_dir, f"video_{role}_annotated.mp4")
                            annotated_video_writers[role] = cv2.VideoWriter(
                                annotated_path, fourcc, extract_fps, (width, height))
                            if not annotated_video_writers[role].isOpened():
                                raise RuntimeError(f"failed to open annotated video writer for {role}")

                    if (
                        projection_configs
                        and latest_cloud_msg is not None
                        and image_frame_indices[role] % extract_cloud_every_n_frames == 0
                    ):
                        if role not in cloud_video_writers:
                            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                            cloud_path = os.path.join(out_dir, f"video_{role}_cloud.mp4")
                            cloud_video_writers[role] = cv2.VideoWriter(
                                cloud_path, fourcc, extract_fps, (width, height))
                            if not cloud_video_writers[role].isOpened():
                                raise RuntimeError(f"failed to open cloud video writer for {role}")

                        if role not in projected_cloud_cache:
                            projected_cloud_cache[role] = project_cloud_points(
                                latest_cloud_msg, role, projection_configs, extract_cloud_max_points)
                        cloud_overlay = draw_cloud_overlay(image, projected_cloud_cache[role])
                        cloud_video_writers[role].write(cloud_overlay)
                        cloud_frame_counts[role] += 1

                    send_topic = send_topic_for_role.get(role)
                    if write_raw_video:
                        raw_video_writers[role].write(image)
                    if write_annotated_video:
                        send_msg = latest_before(indexed_send_buffers.get(send_topic), timestamp)
                        annotated = draw_send_overlay(image, send_msg, role, send_topic or "no Send topic")
                        annotated_video_writers[role].write(annotated)
                    frame_counts[role] += 1
                    processed_frames += 1
                    report_progress()

                elif topic in result_image_topic_to_role:
                    if not write_result_video:
                        continue
                    processed_pass2_messages += 1
                    role = result_image_topic_to_role[topic]
                    image = decode_image(msg, type_map[topic])
                    if image is None:
                        report_progress()
                        continue

                    height, width = image.shape[:2]
                    if role not in result_video_writers:
                        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                        result_path = os.path.join(out_dir, f"video_{role}_result.mp4")
                        result_video_writers[role] = cv2.VideoWriter(
                            result_path, fourcc, extract_fps, (width, height))
                        if not result_video_writers[role].isOpened():
                            raise RuntimeError(f"failed to open result video writer for {role}")

                    result_video_writers[role].write(image)
                    result_frame_counts[role] += 1
                    processed_frames += 1
                    report_progress()

                elif topic == "/rosout":
                    if not write_rosout_log:
                        continue
                    processed_pass2_messages += 1
                    write_rosout(log_file, msg)
                    log_count += 1
                    report_progress()
    finally:
        for raw_writer in raw_video_writers.values():
            raw_writer.release()
        for annotated_writer in annotated_video_writers.values():
            annotated_writer.release()
        for result_writer in result_video_writers.values():
            result_writer.release()
        for cloud_writer in cloud_video_writers.values():
            cloud_writer.release()

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
    for role, frame_count in result_frame_counts.items():
        if frame_count:
            print(f" - {role}: extracted {frame_count} result frames", flush=True)
    for role, frame_count in cloud_frame_counts.items():
        if frame_count:
            print(f" - {role}: extracted {frame_count} cloud overlay frames", flush=True)
    if cloud_count:
        print(f" - read {cloud_count} cloud messages from {CLOUD_TOPIC}", flush=True)
    print(f" - extracted {log_count} log entries to {final_out_dir / 'rosout.txt'}", flush=True)
    report_progress(force=True)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_bag.py <path_to_bag_dir>")
        print("Example: python3 extract_bag.py /home/pnx/rosbag/rmvision_dart/rmvision_dart_20260515_182905")
        sys.exit(1)
    process_bag(sys.argv[1])
