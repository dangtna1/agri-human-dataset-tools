#!/usr/bin/env python3
"""
check_and_make_rosbag2.py (ROS2 Humble)

Writes a rosbag2 from the dataset folder with:
Sensors:
  /dataset/cam_fish_front/image     sensor_msgs/msg/Image (rgb8)
  /dataset/cam_fish_left/image
  /dataset/cam_fish_right/image
  /dataset/cam_zed_rgb/image
  /dataset/cam_zed_depth/image      sensor_msgs/msg/Image (32FC1 or 16UC1 from .npy)
  /dataset/lidar/points             sensor_msgs/msg/PointCloud2 (XYZ only from binary PCD)

Labels (structured, standard):
  /dataset/labels/cam_fish_front    vision_msgs/msg/Detection2DArray
  /dataset/labels/cam_fish_left
  /dataset/labels/cam_fish_right
  /dataset/labels/cam_zed_rgb
  /dataset/labels/lidar             vision_msgs/msg/Detection3DArray

Optional viz (derived from 3D detections; requires TF if RViz fixed frame != 'lidar'):
  /dataset/viz/lidar_boxes          visualization_msgs/msg/MarkerArray

Optional TF:
  /tf  tf2_msgs/msg/TFMessage (map -> lidar by default), published periodically from bag start to bag end.

Why /tf not /tf_static:
- avoids rosbag2 Humble QoS YAML parsing errors.

Usage:
  source /opt/ros/humble/setup.bash
  rm -rf merged_check_bag1
  python3 check_and_make_rosbag2.py \
    --bag-dir out_straw_3push_st_11_07_2024_1_label_trimmed \
    --make-rosbag \
    --rosbag-out merged_check_bag1 \
    --write-tf \
    --tf-parent map --tf-child lidar --tf-xyzrpy 0,0,0,0,0,0 \
    --tf-period-sec 0.5 \
    --write-lidar-viz-markers

  ros2 bag play merged_check_bag1 --clock

RViz tips:
- If you don't want TF: set Fixed Frame = 'lidar', show /dataset/lidar/points and /dataset/viz/lidar_boxes.
- If you want Fixed Frame = 'map': enable --write-tf so map->lidar exists during playback.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import zipfile
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

TS_NAME_RE = re.compile(r"^(?P<sec>\d{9,})(?:_(?P<frac>\d+))?\.(?P<ext>png|npy|pcd)$", re.IGNORECASE)
ZED_RGB_MODALITY = "cam_zed_rgb"
ZED_RGB_IMAGE_TOPIC = "/dataset/cam_zed_rgb/image"
ZED_RGB_CAMERA_INFO_TOPIC = "/dataset/cam_zed_rgb/camera_info"
LIDAR_TOPIC = "/dataset/lidar/points"
ZED_RGB_OPTICAL_FRAME = "front_left_camera_optical_frame"
FRONT_LIDAR_FRAME = "front_lidar_link"
CAMERA_INFO_TOPICS = {
    "cam_zed_rgb": ZED_RGB_CAMERA_INFO_TOPIC,
    "cam_fish_front": "/dataset/cam_fish_front/camera_info",
    "cam_fish_left": "/dataset/cam_fish_left/camera_info",
    "cam_fish_right": "/dataset/cam_fish_right/camera_info",
}
CALIBRATED_CAMERA_FRAMES = {
    "cam_zed_rgb": ZED_RGB_OPTICAL_FRAME,
    "cam_fish_front": "fish_front_camera_link_optical",
    "cam_fish_left": "fish_left_camera_link_optical",
    "cam_fish_right": "fish_right_camera_link_optical",
}

# ---- PCD header parsing ----
_PCD_HEADER_KEYS = {
    "version", "fields", "size", "type", "count", "width", "height", "points", "data", "viewpoint"
}
_TYPE_MAP = {
    ("F", 4): "float32",
    ("F", 8): "float64",
    ("I", 1): "int8",
    ("I", 2): "int16",
    ("I", 4): "int32",
    ("I", 8): "int64",
    ("U", 1): "uint8",
    ("U", 2): "uint16",
    ("U", 4): "uint32",
    ("U", 8): "uint64",
}


# ---------------------------
# Generic helpers
# ---------------------------

def parse_ts_from_filename(fn: str) -> Optional[Decimal]:
    m = TS_NAME_RE.match(fn)
    if not m:
        return None
    sec = m.group("sec")
    frac = m.group("frac") or "0"
    try:
        return Decimal(f"{sec}.{frac}")
    except InvalidOperation:
        return None


def iter_files(root: Path) -> Iterable[Path]:
    for d, _, files in os.walk(root):
        for f in files:
            yield Path(d) / f


def load_json(p: Path) -> Any:
    with p.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_calibration_json(path: Path, member: str) -> Any:
    if path.suffix.lower() == ".zip":
        with zipfile.ZipFile(path) as archive:
            with archive.open(member) as file:
                return json.loads(file.read().decode("utf-8"))
    candidate = path / member if path.is_dir() else path
    return load_json(candidate)


def read_jsonl_objs(p: Path) -> List[Any]:
    out = []
    with p.open("r", encoding="utf-8") as f:
        for i, line in enumerate(f, start=1):
            s = line.strip()
            if not s:
                continue
            try:
                out.append(json.loads(s))
            except json.JSONDecodeError as e:
                raise ValueError(f"JSONL parse error in {p} at line {i}: {e}") from e
    return out


def try_extract_ts(obj: Any) -> Optional[Decimal]:
    if not isinstance(obj, dict):
        return None
    for k in ("timestamp", "Timestamp", "stamp", "time", "t"):
        if k in obj:
            try:
                return Decimal(str(obj[k]))
            except InvalidOperation:
                return None
    hdr = obj.get("header") if isinstance(obj.get("header"), dict) else None
    if hdr:
        st = hdr.get("stamp")
        if isinstance(st, dict) and "sec" in st:
            sec = str(st.get("sec"))
            nsec = str(st.get("nanosec", "0"))
            try:
                return Decimal(f"{sec}.{nsec}")
            except InvalidOperation:
                return None
    return None


def ns_from_decimal_seconds(ts: Decimal) -> int:
    return int(ts * Decimal(1_000_000_000))


# ---------------------------
# Validation
# ---------------------------

@dataclass
class CheckReport:
    sensor_file_count: int = 0
    sensor_modality_counts: Dict[str, int] = None
    sensor_non_ts_files: List[str] = None

    ann_files_checked: int = 0
    ann_records_checked: int = 0
    ann_missing_file_refs: List[str] = None
    ann_ts_mismatch: List[str] = None

    jsonl_files_checked: int = 0
    jsonl_lines_total: int = 0
    jsonl_nonmonotonic_events: int = 0

    warnings: List[str] = None

    def __post_init__(self):
        self.sensor_modality_counts = self.sensor_modality_counts or {}
        self.sensor_non_ts_files = self.sensor_non_ts_files or []
        self.ann_missing_file_refs = self.ann_missing_file_refs or []
        self.ann_ts_mismatch = self.ann_ts_mismatch or []
        self.warnings = self.warnings or []


def build_sensor_index(sensor_root: Path) -> Tuple[Dict[str, List[Path]], List[Tuple[Decimal, str, Path]], CheckReport]:
    report = CheckReport()
    name_index: Dict[str, List[Path]] = {}
    timeline: List[Tuple[Decimal, str, Path]] = []

    if not sensor_root.exists():
        report.warnings.append(f"Missing sensor_data folder: {sensor_root}")
        return name_index, timeline, report

    for modality_dir in sorted([p for p in sensor_root.iterdir() if p.is_dir()]):
        modality = modality_dir.name
        count = 0
        for p in iter_files(modality_dir):
            if p.is_dir():
                continue
            count += 1
            report.sensor_file_count += 1
            name_index.setdefault(p.name, []).append(p)

            ts = parse_ts_from_filename(p.name)
            if ts is None:
                report.sensor_non_ts_files.append(str(p.relative_to(sensor_root)))
            else:
                timeline.append((ts, modality, p))

        report.sensor_modality_counts[modality] = count

    timeline.sort(key=lambda x: x[0])
    return name_index, timeline, report


def check_annotations(ann_dir: Path, name_index: Dict[str, List[Path]], report: CheckReport, ts_match_tol_ms: float) -> None:
    if not ann_dir.exists():
        report.warnings.append(f"Missing annotations folder: {ann_dir}")
        return

    tol = Decimal(str(ts_match_tol_ms)) / Decimal("1000")

    for ann_path in sorted(ann_dir.glob("*_ann.json")):
        report.ann_files_checked += 1
        data = load_json(ann_path)
        if not isinstance(data, list):
            raise ValueError(f"{ann_path} is not a list")

        for i, rec in enumerate(data):
            if not isinstance(rec, dict):
                continue
            report.ann_records_checked += 1
            f = rec.get("File")
            t = rec.get("Timestamp")

            if not isinstance(f, str):
                report.ann_missing_file_refs.append(f"{ann_path.name}: rec[{i}] missing/invalid File")
                continue
            if f not in name_index:
                report.ann_missing_file_refs.append(f"{ann_path.name}: rec[{i}] File not in sensor_data -> {f}")
                continue

            file_ts = parse_ts_from_filename(f)
            if file_ts is None or t is None:
                continue
            try:
                rec_ts = Decimal(str(t))
            except InvalidOperation:
                continue

            if (rec_ts - file_ts).copy_abs() > tol:
                report.ann_ts_mismatch.append(
                    f"{ann_path.name}: rec[{i}] Timestamp {rec_ts} != file_ts {file_ts} (File={f})"
                )


def check_metadata(meta_dir: Path, report: CheckReport) -> None:
    if not meta_dir.exists():
        report.warnings.append(f"Missing metadata folder: {meta_dir}")
        return

    jsonl_paths = list(meta_dir.glob("*.jsonl"))
    tf_dir = meta_dir / "tf"
    if tf_dir.exists():
        jsonl_paths += list(tf_dir.glob("*.jsonl"))

    for p in sorted(jsonl_paths):
        report.jsonl_files_checked += 1
        rows = read_jsonl_objs(p)
        report.jsonl_lines_total += len(rows)

        last: Optional[Decimal] = None
        for obj in rows:
            ts = try_extract_ts(obj)
            if ts is None:
                continue
            if last is not None and ts < last:
                report.jsonl_nonmonotonic_events += 1
            last = ts


def check_sync(sync_path: Path, report: CheckReport) -> None:
    if not sync_path.exists():
        report.warnings.append("sync.json not found (ok if you don’t rely on it).")
        return
    _ = load_json(sync_path)


def run_checks(bag_dir: Path, ts_match_tol_ms: float) -> Tuple[CheckReport, List[Tuple[Decimal, str, Path]]]:
    sensor_root = bag_dir / "sensor_data"
    ann_dir = bag_dir / "annotations"
    meta_dir = bag_dir / "metadata"
    sync_path = bag_dir / "sync.json"

    name_index, timeline, report = build_sensor_index(sensor_root)
    check_annotations(ann_dir, name_index, report, ts_match_tol_ms=ts_match_tol_ms)
    check_metadata(meta_dir, report)
    check_sync(sync_path, report)

    if report.ann_missing_file_refs:
        sample = "\n".join(report.ann_missing_file_refs[:50])
        raise RuntimeError(f"Annotation references missing sensor files (showing up to 50):\n{sample}")
    if report.ann_ts_mismatch:
        sample = "\n".join(report.ann_ts_mismatch[:50])
        raise RuntimeError(f"Annotation Timestamp != File timestamp (showing up to 50):\n{sample}")

    return report, timeline


# ---------------------------
# PCD binary reader (XYZ only)
# ---------------------------

def _parse_pcd_header_and_offset(p: Path) -> Tuple[Dict[str, str], int]:
    header: Dict[str, str] = {}
    offset = 0
    with p.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"Unexpected EOF while reading PCD header: {p}")
            offset += len(line)
            s = line.decode("utf-8", errors="ignore").strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split(None, 1)
            key = parts[0].lower()
            val = parts[1] if len(parts) > 1 else ""
            if key in _PCD_HEADER_KEYS:
                header[key] = val
            if key == "data":
                break
    return header, offset


def read_pcd_binary_xyz(p: Path) -> "np.ndarray":
    import numpy as np

    header, data_offset = _parse_pcd_header_and_offset(p)
    data_type = header.get("data", "").strip().lower()
    if data_type != "binary":
        raise ValueError(f"Unsupported PCD DATA type: {data_type} (only 'binary' supported). File: {p}")

    fields = header.get("fields", "").split()
    sizes = [int(x) for x in header.get("size", "").split()]
    types = header.get("type", "").split()
    counts = [int(x) for x in header.get("count", "").split()] if "count" in header else [1] * len(fields)

    if not (fields and sizes and types) or len(fields) != len(sizes) or len(fields) != len(types) or len(counts) != len(fields):
        raise ValueError(f"Malformed PCD header (FIELDS/SIZE/TYPE/COUNT mismatch): {p}")

    if "points" in header:
        n = int(header["points"])
    elif "width" in header and "height" in header:
        n = int(header["width"]) * int(header["height"])
    else:
        raise ValueError(f"PCD header missing POINTS and WIDTH/HEIGHT: {p}")

    dtype_fields = []
    for name, sz, ty, cnt in zip(fields, sizes, types, counts):
        key = (ty.upper(), sz)
        if key not in _TYPE_MAP:
            raise ValueError(f"Unsupported PCD TYPE/SIZE combo: TYPE={ty}, SIZE={sz} in {p}")
        np_base = getattr(np, _TYPE_MAP[key])
        if cnt == 1:
            dtype_fields.append((name, np_base))
        else:
            dtype_fields.append((name, np_base, (cnt,)))
    dtype = np.dtype(dtype_fields)

    expected_bytes = n * dtype.itemsize
    with p.open("rb") as f:
        f.seek(data_offset)
        raw = f.read(expected_bytes)

    if len(raw) < expected_bytes:
        raise ValueError(f"PCD payload too short: got {len(raw)} bytes, expected {expected_bytes}. File: {p}")

    arr = np.frombuffer(raw, dtype=dtype, count=n)

    for k in ("x", "y", "z"):
        if k not in arr.dtype.names:
            raise ValueError(f"PCD missing required field '{k}': {p}")

    x, y, z = arr["x"], arr["y"], arr["z"]
    if getattr(x, "ndim", 1) > 1:
        x = x[:, 0]
    if getattr(y, "ndim", 1) > 1:
        y = y[:, 0]
    if getattr(z, "ndim", 1) > 1:
        z = z[:, 0]

    import numpy as np
    return np.column_stack([x.astype(np.float32, copy=False),
                            y.astype(np.float32, copy=False),
                            z.astype(np.float32, copy=False)])


# ---------------------------
# Math helpers
# ---------------------------

def euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    import math
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def _list_from_intrinsics_entry(entry: Dict[str, Any], direct_key: str, ros_key: str) -> List[float]:
    if direct_key in entry:
        values = entry[direct_key]
    else:
        matrix = entry.get(ros_key)
        if not isinstance(matrix, dict) or "data" not in matrix:
            raise KeyError(f"Missing intrinsics field '{direct_key}'/'{ros_key}.data'")
        values = matrix["data"]
    return [float(v) for v in values]


def camera_info_values_from_intrinsics(intrinsics: Dict[str, Any], camera_name: str) -> Dict[str, Any]:
    if camera_name not in intrinsics:
        raise KeyError(f"Camera '{camera_name}' not found in intrinsics")
    entry = intrinsics[camera_name]
    if not isinstance(entry, dict):
        raise ValueError(f"Camera '{camera_name}' intrinsics entry is not an object")

    width = entry.get("width", entry.get("image_width"))
    height = entry.get("height", entry.get("image_height"))
    if width is None or height is None:
        raise KeyError(f"Camera '{camera_name}' missing width/height")

    d = entry.get("d")
    if d is None:
        distortion = entry.get("distortion_coefficients")
        if not isinstance(distortion, dict) or "data" not in distortion:
            raise KeyError(f"Camera '{camera_name}' missing distortion coefficients")
        d = distortion["data"]

    k = _list_from_intrinsics_entry(entry, "k", "camera_matrix")
    r = _list_from_intrinsics_entry(entry, "r", "rectification_matrix")
    p = _list_from_intrinsics_entry(entry, "p", "projection_matrix")
    if len(k) != 9:
        raise ValueError(f"Camera '{camera_name}' K must contain 9 values")
    if len(r) != 9:
        raise ValueError(f"Camera '{camera_name}' R must contain 9 values")
    if len(p) != 12:
        raise ValueError(f"Camera '{camera_name}' P must contain 12 values")

    frame_id = (entry.get("header") or {}).get("frame_id")
    if not frame_id:
        raise KeyError(f"Camera '{camera_name}' missing header.frame_id")

    return {
        "width": int(width),
        "height": int(height),
        "distortion_model": str(entry.get("distortion_model", "")),
        "d": [float(v) for v in d],
        "k": k,
        "r": r,
        "p": p,
        "frame_id": str(frame_id),
    }


def _normalise_quaternion(rotation: Dict[str, Any]) -> Tuple[float, float, float, float]:
    import math

    x = float(rotation["x"])
    y = float(rotation["y"])
    z = float(rotation["z"])
    w = float(rotation["w"])
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("Static transform quaternion has zero norm")
    return x / norm, y / norm, z / norm, w / norm


def static_transform_entries_from_extrinsics(extrinsics: Dict[str, Any]) -> List[Dict[str, Any]]:
    transforms = extrinsics.get("transforms")
    if not isinstance(transforms, list):
        raise ValueError("Extrinsics JSON must contain a 'transforms' list")

    entries = []
    for idx, edge in enumerate(transforms):
        header = edge.get("header") or {}
        parent = header.get("frame_id")
        child = edge.get("child_frame_id")
        transform = edge.get("transform") or {}
        translation = transform.get("translation") or {}
        rotation = transform.get("rotation") or {}
        if not parent or not child:
            raise ValueError(f"Static transform {idx} is missing parent or child frame")
        qx, qy, qz, qw = _normalise_quaternion(rotation)
        entries.append({
            "parent": str(parent),
            "child": str(child),
            "translation": (
                float(translation["x"]),
                float(translation["y"]),
                float(translation["z"]),
            ),
            "rotation": (qx, qy, qz, qw),
        })
    return entries


# ---------------------------
# Annotation parsing to vision_msgs
# ---------------------------

def class_to_int_id(class_name: str) -> int:
    """
    Deterministic mapping for RViz/consumers without a pre-defined taxonomy.
    You can swap this for a fixed dict if you have a known label set.
    """
    # stable 32-bit hash -> positive int
    return (hash(class_name) & 0x7FFFFFFF)


# ---------------------------
# ROS2 bag writer
# ---------------------------

def make_rosbag2(
    bag_dir: Path,
    timeline: List[Tuple[Decimal, str, Path]],
    rosbag_out: Path,
    include_modalities: Optional[set],
    write_tf: bool,
    tf_parent: str,
    tf_child: str,
    tf_xyzrpy: Tuple[float, float, float, float, float, float],
    tf_period_sec: float,
    write_lidar_viz_markers: bool,
    write_calibration: bool,
    calibration_path: Optional[Path],
    intrinsics_member: str,
    extrinsics_member: str,
    camera_name: str,
    camera_info_topic: str,
    camera_info_cameras: List[str],
) -> None:
    try:
        import numpy as np
        from PIL import Image as PILImage

        from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
        from rclpy.serialization import serialize_message

        from std_msgs.msg import Header
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        from sensor_msgs_py import point_cloud2

        from geometry_msgs.msg import Pose, Point as GPoint, Quaternion, Vector3, TransformStamped
        from tf2_msgs.msg import TFMessage

        from vision_msgs.msg import Detection2DArray, Detection2D, Detection3DArray, Detection3D
        from vision_msgs.msg import ObjectHypothesisWithPose, BoundingBox2D, BoundingBox3D

        from visualization_msgs.msg import Marker, MarkerArray
    except Exception as e:
        raise RuntimeError("ROS2 writing requires ROS2 Humble sourced + numpy + pillow + vision_msgs.") from e

    # Topics
    cam_topics = {
        "cam_fish_front": "/dataset/cam_fish_front/image",
        "cam_fish_left": "/dataset/cam_fish_left/image",
        "cam_fish_right": "/dataset/cam_fish_right/image",
        "cam_zed_rgb": ZED_RGB_IMAGE_TOPIC,
        "cam_zed_depth": "/dataset/cam_zed_depth/image",
    }
    lidar_topic = LIDAR_TOPIC

    # Label topics (vision_msgs)
    label2d_topics = {
        "cam_fish_front_ann.json": "/dataset/labels/cam_fish_front",
        "cam_fish_left_ann.json": "/dataset/labels/cam_fish_left",
        "cam_fish_right_ann.json": "/dataset/labels/cam_fish_right",
        "cam_zed_rgb_ann.json": "/dataset/labels/cam_zed_rgb",
    }
    label3d_topic = "/dataset/labels/lidar"

    viz_lidar_markers_topic = "/dataset/viz/lidar_boxes"
    camera_info_by_modality: Dict[str, Dict[str, Any]] = {}
    camera_info_topics: Dict[str, str] = {}
    static_transforms: List[Dict[str, Any]] = []
    if write_calibration:
        if calibration_path is None:
            raise ValueError("--write-calibration requires --calibration-path")
        intrinsics = load_calibration_json(calibration_path, intrinsics_member)
        extrinsics = load_calibration_json(calibration_path, extrinsics_member)
        for info_camera in camera_info_cameras:
            if info_camera not in cam_topics or info_camera == "cam_zed_depth":
                raise ValueError(f"Unsupported CameraInfo camera: {info_camera}")
            values = camera_info_values_from_intrinsics(intrinsics, info_camera)
            expected_frame = CALIBRATED_CAMERA_FRAMES.get(info_camera)
            if expected_frame and values["frame_id"] != expected_frame:
                raise ValueError(
                    f"{info_camera} CameraInfo frame_id is {values['frame_id']}, "
                    f"expected {expected_frame}"
                )
            camera_info_by_modality[info_camera] = values
            camera_info_topics[info_camera] = (
                camera_info_topic if info_camera == camera_name
                else CAMERA_INFO_TOPICS[info_camera]
            )
        static_transforms = static_transform_entries_from_extrinsics(extrinsics)

    # Writer
    out = rosbag_out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    storage_options = StorageOptions(uri=str(out), storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    created: Dict[str, str] = {}

    def ensure_topic(topic: str, msg_type: str, offered_qos_profiles: str = "") -> None:
        if topic in created:
            if created[topic] != msg_type:
                raise RuntimeError(f"Topic {topic} already created with type {created[topic]}, requested {msg_type}")
            return
        writer.create_topic(TopicMetadata(
            name=topic,
            type=msg_type,
            serialization_format="cdr",
            offered_qos_profiles=offered_qos_profiles,
        ))
        created[topic] = msg_type

    # ---- Sensor encoders ----

    def image_from_png(path: Path, stamp_ns: int, frame_id: str) -> Image:
        img = PILImage.open(path).convert("RGB")
        arr = np.asarray(img)
        msg = Image()
        msg.header.frame_id = frame_id
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.height = int(arr.shape[0])
        msg.width = int(arr.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = arr.tobytes()
        return msg

    def camera_info_msg(stamp_ns: int, values: Dict[str, Any]) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = values["frame_id"]
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.width = values["width"]
        msg.height = values["height"]
        msg.distortion_model = values["distortion_model"]
        msg.d = values["d"]
        msg.k = values["k"]
        msg.r = values["r"]
        msg.p = values["p"]
        return msg

    def image_from_depth_npy(path: Path, stamp_ns: int, frame_id: str) -> Image:
        depth = np.load(path)
        if depth.ndim != 2:
            raise ValueError(f"Depth .npy must be HxW, got {depth.shape} at {path}")
        msg = Image()
        msg.header.frame_id = frame_id
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.height = int(depth.shape[0])
        msg.width = int(depth.shape[1])
        msg.is_bigendian = False

        if depth.dtype == np.float32:
            msg.encoding = "32FC1"
            msg.step = msg.width * 4
        elif depth.dtype == np.uint16:
            msg.encoding = "16UC1"
            msg.step = msg.width * 2
        else:
            depth = depth.astype(np.float32)
            msg.encoding = "32FC1"
            msg.step = msg.width * 4

        msg.data = depth.tobytes()
        return msg

    def cloud_from_pcd(path: Path, stamp_ns: int) -> PointCloud2:
        xyz = read_pcd_binary_xyz(path)
        hdr = Header()
        hdr.frame_id = FRONT_LIDAR_FRAME if write_calibration else "lidar"
        hdr.stamp.sec = stamp_ns // 1_000_000_000
        hdr.stamp.nanosec = stamp_ns % 1_000_000_000
        return point_cloud2.create_cloud_xyz32(header=hdr, points=xyz.tolist())

    # ---- TF (/tf) ----

    def write_tf_at_all_timestamps() -> None:
        """
        Writes a TFMessage at every lidar timestamp found in `timeline`.
        This guarantees TF coverage for all PointCloud2 + lidar viz markers timestamps,
        avoiding RViz extrapolation into past/future when using a map-level frame.
        """
        if not write_tf:
            return

        ensure_topic("/tf", "tf2_msgs/msg/TFMessage")

        x, y, z, roll, pitch, yaw = tf_xyzrpy
        qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)

        # Publish at lidar times (recommended). If you want *all* sensor times, remove the modality filter.
        for ts, modality, _p in timeline:
            if modality != "lidar":
                continue

            stamp_ns = ns_from_decimal_seconds(ts)

            tfs = TransformStamped()
            tfs.header.frame_id = tf_parent          # e.g. "map"
            tfs.child_frame_id = tf_child            # e.g. "base_link"
            tfs.header.stamp.sec = stamp_ns // 1_000_000_000
            tfs.header.stamp.nanosec = stamp_ns % 1_000_000_000

            tfs.transform.translation.x = float(x)
            tfs.transform.translation.y = float(y)
            tfs.transform.translation.z = float(z)
            tfs.transform.rotation.x = float(qx)
            tfs.transform.rotation.y = float(qy)
            tfs.transform.rotation.z = float(qz)
            tfs.transform.rotation.w = float(qw)

            msg = TFMessage()
            msg.transforms = [tfs]
            writer.write("/tf", serialize_message(msg), stamp_ns)

    def write_static_transforms() -> None:
        if not write_calibration:
            return
        if not static_transforms:
            raise RuntimeError("No static transforms loaded from calibration")
        ensure_topic("/tf_static", "tf2_msgs/msg/TFMessage")
        stamp_ns = ns_from_decimal_seconds(timeline[0][0])
        msg = TFMessage()
        for entry in static_transforms:
            tfs = TransformStamped()
            tfs.header.frame_id = entry["parent"]
            tfs.child_frame_id = entry["child"]
            tfs.header.stamp.sec = stamp_ns // 1_000_000_000
            tfs.header.stamp.nanosec = stamp_ns % 1_000_000_000
            x, y, z = entry["translation"]
            qx, qy, qz, qw = entry["rotation"]
            tfs.transform.translation.x = x
            tfs.transform.translation.y = y
            tfs.transform.translation.z = z
            tfs.transform.rotation.x = qx
            tfs.transform.rotation.y = qy
            tfs.transform.rotation.z = qz
            tfs.transform.rotation.w = qw
            msg.transforms.append(tfs)
        writer.write("/tf_static", serialize_message(msg), stamp_ns)

    # ---- Labels (vision_msgs) ----

    def rec_to_detection2d_array(rec: Dict[str, Any], frame_id: str, stamp_ns: int) -> Detection2DArray:
        arr = Detection2DArray()
        arr.header.frame_id = frame_id
        arr.header.stamp.sec = stamp_ns // 1_000_000_000
        arr.header.stamp.nanosec = stamp_ns % 1_000_000_000

        labels = rec.get("Labels")
        if not isinstance(labels, list):
            return arr

        for i, lb in enumerate(labels):
            if not isinstance(lb, dict):
                continue
            cls = lb.get("Class")
            bb = lb.get("BoundingBoxes")
            if not isinstance(cls, str) or not isinstance(bb, list) or len(bb) < 4:
                continue
            try:
                x, y, w, h = [float(v) for v in bb[:4]]
            except Exception:
                continue

            det = Detection2D()
            det.id = str(i)

            bbox = BoundingBox2D()
            # vision_msgs uses center + size
            bbox.center.position.x = x + w * 0.5
            bbox.center.position.y = y + h * 0.5
            bbox.center.theta = 0.0
            bbox.size_x = w
            bbox.size_y = h
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_to_int_id(cls))
            hyp.hypothesis.score = 1.0
            # pose unused for 2D
            det.results.append(hyp)

            arr.detections.append(det)

        return arr

    def rec_to_detection3d_array(rec: Dict[str, Any], frame_id: str, stamp_ns: int) -> Detection3DArray:
        arr = Detection3DArray()
        arr.header.frame_id = frame_id
        arr.header.stamp.sec = stamp_ns // 1_000_000_000
        arr.header.stamp.nanosec = stamp_ns % 1_000_000_000

        labels = rec.get("Labels")
        if not isinstance(labels, list):
            return arr

        for i, lb in enumerate(labels):
            if not isinstance(lb, dict):
                continue
            cls = lb.get("Class")
            bb = lb.get("BoundingBoxes")
            if not isinstance(cls, str) or not isinstance(bb, list) or len(bb) < 9:
                continue
            try:
                x, y, z, dx, dy, dz, roll, pitch, yaw = [float(v) for v in bb[:9]]
            except Exception:
                continue

            det = Detection3D()
            det.id = str(i)

            bbox = BoundingBox3D()
            bbox.center.position.x = x
            bbox.center.position.y = y
            bbox.center.position.z = z
            qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
            bbox.center.orientation.x = qx
            bbox.center.orientation.y = qy
            bbox.center.orientation.z = qz
            bbox.center.orientation.w = qw
            bbox.size.x = max(1e-6, dx)
            bbox.size.y = max(1e-6, dy)
            bbox.size.z = max(1e-6, dz)
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_to_int_id(cls))
            hyp.hypothesis.score = 1.0
            # Use pose to store bbox center pose (optional, but consistent)
            hyp.pose.pose = Pose()
            hyp.pose.pose.position = GPoint(x=float(x), y=float(y), z=float(z))
            hyp.pose.pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
            det.results.append(hyp)

            arr.detections.append(det)

        return arr

    def detection3d_array_to_markers(d3: Detection3DArray, stamp_ns: int) -> MarkerArray:
        # For RViz visualization
        ma = MarkerArray()
        mid = 0
        for det in d3.detections:
            m = Marker()
            m.header = d3.header
            m.ns = "lidar_boxes"
            m.id = mid
            mid += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = det.bbox.center
            m.scale = Vector3(x=det.bbox.size.x, y=det.bbox.size.y, z=det.bbox.size.z)
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 1.0
            m.color.a = 0.6
            ma.markers.append(m)
        return ma

    def write_labels() -> None:
        ann_dir = bag_dir / "annotations"
        if not ann_dir.exists():
            return

        # Create topics
        for _, t in label2d_topics.items():
            ensure_topic(t, "vision_msgs/msg/Detection2DArray")
        ensure_topic(label3d_topic, "vision_msgs/msg/Detection3DArray")
        if write_lidar_viz_markers:
            ensure_topic(viz_lidar_markers_topic, "visualization_msgs/msg/MarkerArray")

        # 2D cameras
        for ann_file, topic in label2d_topics.items():
            p = ann_dir / ann_file
            if not p.exists():
                continue
            data = load_json(p)
            if not isinstance(data, list):
                raise ValueError(f"{p} is not a list")

            if ann_file.startswith("cam_zed_rgb"):
                frame_id = ZED_RGB_OPTICAL_FRAME if write_calibration else "cam_zed_rgb"
            elif ann_file.startswith("cam_fish_front"):
                frame_id = CALIBRATED_CAMERA_FRAMES["cam_fish_front"] if write_calibration else "cam_fish_front"
            elif ann_file.startswith("cam_fish_left"):
                frame_id = CALIBRATED_CAMERA_FRAMES["cam_fish_left"] if write_calibration else "cam_fish_left"
            elif ann_file.startswith("cam_fish_right"):
                frame_id = CALIBRATED_CAMERA_FRAMES["cam_fish_right"] if write_calibration else "cam_fish_right"
            else:
                frame_id = "camera"

            for rec in data:
                if not isinstance(rec, dict):
                    continue
                t = rec.get("Timestamp")
                if t is None:
                    continue
                try:
                    ts = Decimal(str(t))
                except Exception:
                    continue
                stamp_ns = ns_from_decimal_seconds(ts)
                msg = rec_to_detection2d_array(rec, frame_id=frame_id, stamp_ns=stamp_ns)
                writer.write(topic, serialize_message(msg), stamp_ns)

        # 3D lidar
        p3 = ann_dir / "lidar_ann.json"
        if p3.exists():
            data3 = load_json(p3)
            if not isinstance(data3, list):
                raise ValueError(f"{p3} is not a list")

            for rec in data3:
                if not isinstance(rec, dict):
                    continue
                t = rec.get("Timestamp")
                if t is None:
                    continue
                try:
                    ts = Decimal(str(t))
                except Exception:
                    continue
                stamp_ns = ns_from_decimal_seconds(ts)

                d3 = rec_to_detection3d_array(
                    rec,
                    frame_id=FRONT_LIDAR_FRAME if write_calibration else "lidar",
                    stamp_ns=stamp_ns,
                )
                writer.write(label3d_topic, serialize_message(d3), stamp_ns)

                if write_lidar_viz_markers:
                    ma = detection3d_array_to_markers(d3, stamp_ns)
                    writer.write(viz_lidar_markers_topic, serialize_message(ma), stamp_ns)

    # ---- Write order: TF -> labels -> sensors ----
    write_static_transforms()
    write_tf_at_all_timestamps()
    write_labels()

    # Sensors
    for ts, modality, p in timeline:
        if include_modalities is not None and modality not in include_modalities:
            continue
        stamp_ns = ns_from_decimal_seconds(ts)

        if modality in cam_topics:
            topic = cam_topics[modality]
            ensure_topic(topic, "sensor_msgs/msg/Image")
            if modality == "cam_zed_depth":
                msg = image_from_depth_npy(p, stamp_ns, frame_id=modality)
            else:
                frame_id = CALIBRATED_CAMERA_FRAMES.get(modality, modality) if write_calibration else modality
                msg = image_from_png(p, stamp_ns, frame_id=frame_id)
            writer.write(topic, serialize_message(msg), stamp_ns)
            if write_calibration and modality in camera_info_by_modality:
                info_topic = camera_info_topics[modality]
                ensure_topic(info_topic, "sensor_msgs/msg/CameraInfo")
                writer.write(
                    info_topic,
                    serialize_message(camera_info_msg(stamp_ns, camera_info_by_modality[modality])),
                    stamp_ns,
                )

        elif modality == "lidar":
            ensure_topic(lidar_topic, "sensor_msgs/msg/PointCloud2")
            msg = cloud_from_pcd(p, stamp_ns)
            writer.write(lidar_topic, serialize_message(msg), stamp_ns)

    print(f"[rosbag2] wrote: {out}")


# ---------------------------
# Main
# ---------------------------

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag-dir", type=Path, required=True)
    ap.add_argument("--ts-match-tol-ms", type=float, default=5.0)

    ap.add_argument("--make-rosbag", action="store_true")
    ap.add_argument("--rosbag-out", type=Path, default=Path("./rosbag_out"))
    ap.add_argument("--only-modalities", type=str, default="")

    ap.add_argument("--write-tf", action="store_true")
    ap.add_argument("--tf-parent", type=str, default="map")
    ap.add_argument("--tf-child", type=str, default="lidar")
    ap.add_argument("--tf-xyzrpy", type=str, default="0,0,0,0,0,0")
    ap.add_argument("--tf-period-sec", type=float, default=0.5)

    ap.add_argument("--write-lidar-viz-markers", action="store_true")
    ap.add_argument(
        "--write-calibration",
        action="store_true",
        help="Write ZED RGB CameraInfo and extrinsics-derived /tf_static.",
    )
    ap.add_argument(
        "--calibration-path",
        type=Path,
        default=None,
        help="Calibration directory or ZIP containing calibration/intrinsics.json and calibration/extrinsics.json.",
    )
    ap.add_argument("--intrinsics-member", type=str, default="calibration/intrinsics.json")
    ap.add_argument("--extrinsics-member", type=str, default="calibration/extrinsics.json")
    ap.add_argument("--camera-name", type=str, default=ZED_RGB_MODALITY)
    ap.add_argument("--camera-info-topic", type=str, default=ZED_RGB_CAMERA_INFO_TOPIC)
    ap.add_argument(
        "--camera-info-cameras",
        type=str,
        default="cam_zed_rgb,cam_fish_front,cam_fish_left,cam_fish_right",
        help="Comma-separated camera modalities to publish CameraInfo for.",
    )

    args = ap.parse_args()

    bag_dir = args.bag_dir
    if not bag_dir.exists():
        raise SystemExit(f"bag-dir does not exist: {bag_dir}")

    report, timeline = run_checks(bag_dir, ts_match_tol_ms=args.ts_match_tol_ms)

    print("=== CHECK REPORT ===")
    print(f"Bag: {bag_dir}")
    print(f"Sensor files total: {report.sensor_file_count}")
    print("Sensor modality counts:")
    for k, v in sorted(report.sensor_modality_counts.items()):
        print(f"  - {k}: {v}")
    print(f"Annotation JSON files checked: {report.ann_files_checked}")
    print(f"Annotation records checked: {report.ann_records_checked}")
    print(f"Metadata JSONL files checked: {report.jsonl_files_checked}")
    print(f"Metadata JSONL total lines: {report.jsonl_lines_total}")
    print(f"Metadata JSONL non-monotonic timestamp events: {report.jsonl_nonmonotonic_events}")

    if report.warnings:
        print("Warnings:")
        for w in report.warnings:
            print(f"  - {w}")

    print("Checks: PASS")

    if args.make_rosbag:
        if not timeline:
            raise SystemExit("No timestamped sensor files found to build a timeline.")

        include = None
        if args.only_modalities.strip():
            include = {x.strip() for x in args.only_modalities.split(",") if x.strip()}

        xyzrpy = tuple(float(x) for x in args.tf_xyzrpy.split(","))
        if len(xyzrpy) != 6:
            raise SystemExit("--tf-xyzrpy must be 6 comma-separated numbers: x,y,z,roll,pitch,yaw")
        camera_info_cameras = [
            item.strip()
            for item in args.camera_info_cameras.split(",")
            if item.strip()
        ]

        print(f"Writing rosbag2 to: {args.rosbag_out.resolve()}")
        make_rosbag2(
            bag_dir=bag_dir,
            timeline=timeline,
            rosbag_out=args.rosbag_out,
            include_modalities=include,
            write_tf=args.write_tf,
            tf_parent=args.tf_parent,
            tf_child=args.tf_child,
            tf_xyzrpy=xyzrpy,
            tf_period_sec=args.tf_period_sec,
            write_lidar_viz_markers=args.write_lidar_viz_markers,
            write_calibration=args.write_calibration,
            calibration_path=args.calibration_path,
            intrinsics_member=args.intrinsics_member,
            extrinsics_member=args.extrinsics_member,
            camera_name=args.camera_name,
            camera_info_topic=args.camera_info_topic,
            camera_info_cameras=camera_info_cameras,
        )
        print("rosbag2: DONE")


if __name__ == "__main__":
    main()
