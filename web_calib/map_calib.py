#!/usr/bin/env python3
"""带 RTK 的地图标定。网页选择雷达，RViz 查看起点，确认后再逐帧优化。"""

import hashlib
import os
import re
import struct
import threading
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from scipy.spatial import cKDTree
import rosbag2_py
import yaml
from flask import Flask, Response, jsonify, render_template, request
from pyproj import CRS, Transformer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import long_pair_calib as pair
import small_gicp

OUTPUT = Path(os.environ.get("OUTPUT", "/out/map_calib_result.yaml"))
PORT = int(os.environ.get("PORT", "8766"))
MARGIN = 90.0
MATCH_VOXEL = 0.30
BASELINE_Z = 1.0
COLORS = ("#ff3c3c", "#3cc850", "#468cff", "#ffc828", "#50dcdc", "#dc78dc", "#ff783c")
LABELS = (
    ("/long/left_front/", "远距左前", "hesai_lidar_left_front"),
    ("/long/right_end/", "远距右后", "hesai_lidar_right_end"),
    ("/blind/left_front/", "补盲左前", "suteng_lidar_left_front"),
    ("/blind/right_end/", "补盲右后", "suteng_lidar_right_end"),
    ("/front_mid_top/", "前向中顶", "hesai_lidar_front_mid_top"),
    ("/left_front/", "左前", "hesai_lidar_left_front"),
    ("/right_front/", "右前", "hesai_lidar_right_front"),
)
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def topic_meta(topic):
    for needle, label, frame in LABELS:
        if needle in topic:
            return label, frame
    tail = topic.strip("/").split("/")[-2] if topic.count("/") >= 2 else topic
    return tail, re.sub(r"[^0-9A-Za-z_]+", "_", tail)


def preferred_reference(topics):
    for needle in ("/long/left_front/", "/front_mid_top/"):
        for topic in topics:
            if needle in topic["name"]:
                return topic["name"]
    return topics[0]["name"] if topics else ""


def pcd_layout(path: Path):
    fields, sizes, types = [], [], []
    with path.open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise SystemExit(f"PCD 没有 DATA: {path}")
            if line.startswith(b"DATA"):
                kind = line.decode("ascii", "replace").strip().split()[-1]
                if kind != "binary":
                    raise SystemExit(f"只读取 binary PCD，当前是 {kind}")
                offset = handle.tell()
                break
            text = line.decode("ascii", "replace").strip()
            if not text or text.startswith("#"):
                continue
            key, value = text.split(None, 1)
            if key == "FIELDS":
                fields = value.split()
            elif key == "SIZE":
                sizes = [int(item) for item in value.split()]
            elif key == "TYPE":
                types = value.split()
    if fields[:3] != ["x", "y", "z"] or len(sizes) < 3 or len(types) < 3:
        raise SystemExit(f"PCD 缺少 x y z: {path}")
    return {"sizes": sizes, "types": types, "step": sum(sizes), "offset": offset}


def chunk_xyz(raw, layout):
    step = layout["step"]
    count = len(raw) // step
    packed = np.frombuffer(raw, dtype=np.uint8, count=count * step).reshape(count, step)
    if layout["types"][0] == "F" and layout["sizes"][0] == 8:
        view = np.ascontiguousarray(packed[:, :24]).view(np.float64)
        return view.reshape(count, 3)
    if layout["types"][0] == "F" and layout["sizes"][0] == 4:
        view = np.ascontiguousarray(packed[:, :12]).view(np.float32)
        return view.reshape(count, 3).astype(np.float64)
    raise SystemExit(f"不支持的坐标类型 {layout['types'][0]} / {layout['sizes'][0]}")


def load_points(path: Path, bounds, voxel, status):
    layout = pcd_layout(path)
    kept = []
    step = layout["step"]
    chunk_points = 1_000_000
    seen = 0
    with path.open("rb") as handle:
        handle.seek(layout["offset"])
        while True:
            raw = handle.read(chunk_points * step)
            if not raw:
                break
            xyz = chunk_xyz(raw, layout)
            finite = np.isfinite(xyz).all(axis=1)
            if bounds is not None:
                xmin, xmax, ymin, ymax = bounds
                finite &= (xyz[:, 0] >= xmin) & (xyz[:, 0] <= xmax)
                finite &= (xyz[:, 1] >= ymin) & (xyz[:, 1] <= ymax)
            if np.any(finite):
                kept.append(pair.voxel_down(xyz[finite], voxel))
            seen += len(xyz)
            status(f"正在读取地图，已扫描 {seen / 1e6:.0f} 百万点")
    if not kept:
        raise SystemExit("地图范围内没有点")
    return pair.voxel_down(np.vstack(kept), voxel)


def sample_median(path: Path):
    layout = pcd_layout(path)
    with path.open("rb") as handle:
        handle.seek(layout["offset"])
        raw = handle.read(20000 * layout["step"])
    xyz = chunk_xyz(raw, layout)
    finite = np.isfinite(xyz).all(axis=1)
    return np.median(np.abs(xyz[finite, :2]), axis=0)


def projector_of(path: Path):
    info = yaml.safe_load(path.read_text(encoding="utf-8"))
    origin = info["map_origin"]
    return {
        "kind": "mercator",
        "latitude": float(origin["latitude"]),
        "longitude": float(origin["longitude"]),
        "altitude": float(origin.get("altitude", 0.0)),
        "scale": float(info.get("scale_factor", 0.9996)),
    }


def traj_grid_origin(path: Path):
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = [item.strip() for item in line.split(",")]
            if len(parts) < 10:
                continue
            return np.array([float(parts[7]), float(parts[8]), float(parts[9])], dtype=np.float64)
    return None


def utm_epsg(directory: Path):
    for traj in sorted(directory.glob("*.traj")):
        head = traj.read_text(encoding="utf-8", errors="replace")[:4000]
        found = re.search(r'AUTHORITY\["EPSG","(326\d{2})"\]', head)
        if found:
            return found.group(1), traj
    return "32650", None


def read_gnss(mcap: Path):
    reader = pair.open_reader(mcap, [pair.GNSS_FIX, pair.GNSS_POSE])
    fixes, quats = [], []
    while reader.has_next():
        topic, raw, _stamp = reader.read_next()
        if topic.endswith("nav_sat_fix"):
            from rclpy.serialization import deserialize_message
            from sensor_msgs.msg import NavSatFix

            msg = deserialize_message(raw, NavSatFix)
            fixes.append((pair.stamp_sec(msg), msg.latitude, msg.longitude, msg.altitude))
        elif topic.endswith("/pose"):
            from geometry_msgs.msg import PoseStamped
            from rclpy.serialization import deserialize_message

            msg = deserialize_message(raw, PoseStamped)
            quat = msg.pose.orientation
            quats.append((pair.stamp_sec(msg), quat.x, quat.y, quat.z, quat.w))
    if len(fixes) < 2 or len(quats) < 2:
        raise SystemExit("数据包里没有足够的 RTK 位置或姿态")
    fixes = np.asarray(fixes, dtype=np.float64)
    quats = np.asarray(quats, dtype=np.float64)
    order = np.argsort(quats[:, 0])
    quats = quats[order]
    for index in range(1, len(quats)):
        if float(np.dot(quats[index, 1:], quats[index - 1, 1:])) < 0.0:
            quats[index, 1:] *= -1.0
    return fixes, quats


POSE_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
SKIP_EXTRINSIC = {"base_link", "gnss_base_link"}
GENERIC_TOKENS = {
    "base", "link", "lidar", "points", "sutpc", "sensor", "driver", "msg", "sensing",
}


def load_extrinsic_yaml(path: Path):
    values = {}
    current = None
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.endswith(":") and not line.startswith("-"):
            current = line[:-1].strip()
            values[current] = {}
            continue
        if current is None or ":" not in line:
            continue
        key, value = line.split(":", 1)
        try:
            values[current][key.strip()] = float(value.strip())
        except ValueError:
            continue
    poses = {}
    for name, block in values.items():
        if name in SKIP_EXTRINSIC or any(field not in block for field in POSE_FIELDS):
            continue
        poses[name] = {field: float(block[field]) for field in POSE_FIELDS}
    if not poses:
        raise SystemExit(f"{path.name} 里没有雷达外参")
    return poses


def name_tokens(text):
    return {
        token for token in re.split(r"[^0-9A-Za-z]+", text.lower())
        if token and token not in GENERIC_TOKENS
    }


def extrinsic_score(topic, key):
    frame = topic.get("frame_id") or topic.get("frame") or ""
    if frame and (key == f"{frame}_base_link" or key == frame):
        return 100
    topic_tokens = name_tokens(f"{topic['name']} {frame}")
    stem = key[: -len("_base_link")] if key.endswith("_base_link") else key
    key_tokens = name_tokens(stem)
    shared = topic_tokens & key_tokens
    score = 0
    for token in shared:
        score += 8 if token in {"left", "right", "front", "end", "mid", "top", "rear"} else 3
    name = topic["name"]
    if "/long/" in name and "hesai" in key_tokens:
        score += 6
    if "/blind/" in name and "suteng" in key_tokens:
        score += 6
    if "/long/" in name and "suteng" in key_tokens:
        score -= 12
    if "/blind/" in name and "hesai" in key_tokens:
        score -= 12
    return score


def assign_extrinsics(topics, keys):
    pairs = []
    for topic in topics:
        topic["extrinsic"] = ""
        for key in keys:
            score = extrinsic_score(topic, key)
            if score >= 12:
                pairs.append((score, topic["name"], key))
    pairs.sort(reverse=True)
    used_topics = set()
    used_keys = set()
    chosen = {}
    for score, topic_name, key in pairs:
        if topic_name in used_topics or key in used_keys:
            continue
        used_topics.add(topic_name)
        used_keys.add(key)
        chosen[topic_name] = key
    for topic in topics:
        topic["extrinsic"] = chosen.get(topic["name"], "")
    return chosen


def attach_frame_ids(mcap: Path, topics):
    from rclpy.serialization import deserialize_message

    names = [item["name"] for item in topics]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(mcap), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=names))
    frames = {name: "" for name in names}
    pending = set(names)
    seen = 0
    while reader.has_next() and pending and seen < 80:
        topic, raw, _stamp = reader.read_next()
        seen += 1
        if topic not in pending:
            continue
        msg = deserialize_message(raw, PointCloud2)
        frames[topic] = msg.header.frame_id or ""
        pending.discard(topic)
    for item in topics:
        item["frame_id"] = frames.get(item["name"], "")
        item["extrinsic"] = ""


def list_topics(mcap: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(mcap), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    found = []
    for info in reader.get_all_topics_and_types():
        if info.type != "sensor_msgs/msg/PointCloud2":
            continue
        if "lidar" not in info.name:
            continue
        label, frame = topic_meta(info.name)
        found.append({"name": info.name, "label": label, "frame": frame, "frame_id": "", "extrinsic": ""})
    if not found:
        raise SystemExit("数据包里没有激光雷达点云话题")
    attach_frame_ids(mcap, found)
    return found


def project_rtk(fixes, mode):
    if mode["kind"] == "utm":
        project = Transformer.from_crs("EPSG:4326", f"EPSG:{mode['epsg']}", always_xy=True)
        east, north = project.transform(fixes[:, 2], fixes[:, 1])
        height = fixes[:, 3]
        return np.column_stack((east, north, height))
    if mode["kind"] == "mercator":
        crs = CRS.from_proj4(
            f"+proj=tmerc +lat_0={mode['latitude']} +lon_0={mode['longitude']} +k={mode['scale']} "
            "+x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
        )
        project = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
        east, north = project.transform(fixes[:, 2], fixes[:, 1])
        height = fixes[:, 3] - mode["altitude"]
        return np.column_stack((east, north, height))
    east_north = None
    project = Transformer.from_crs("EPSG:4326", f"EPSG:{mode['epsg']}", always_xy=True)
    east, north = project.transform(fixes[:, 2], fixes[:, 1])
    xyz = np.column_stack((east, north, fixes[:, 3])) - mode["origin"]
    return xyz if east_north is None else xyz


def height_offset(rtk_xyz, cloud):
    delta = cloud[:, :2] - rtk_xyz[0, :2]
    near = cloud[np.einsum("ij,ij->i", delta, delta) <= 20.0 ** 2]
    if len(near) < 50:
        return 0.0, 0.0
    low, high = np.percentile(near[:, 2], [10, 90])
    if low - 8.0 <= rtk_xyz[0, 2] <= high + 8.0:
        return 0.0, 0.0
    ground = float(np.percentile(near[:, 2], 20))
    gap = float(rtk_xyz[0, 2] - ground)
    # 先验放到地面以上 1.6 m，覆盖车身和车顶雷达。
    return gap - 1.6, gap


def load_scene(mcap: Path, map_path: Path, status):
    status("正在读取话题和 RTK")
    topics = list_topics(mcap)
    fixes, quats = read_gnss(mcap)
    map_path = map_path.resolve()
    if map_path.is_dir():
        info = map_path / "map_projector_info.yaml"
        cloud_path = map_path / "pointcloud_map.pcd"
        if not info.is_file() or not cloud_path.is_file():
            raise SystemExit("地图目录需要 pointcloud_map.pcd 和 map_projector_info.yaml")
        mode = projector_of(info)
        frame = (
            f"横轴墨卡托，起点 ({mode['latitude']:.8f}, {mode['longitude']:.8f})，"
            f"高度零点 {mode['altitude']:.3f} m"
        )
    else:
        cloud_path = map_path
        info = map_path.parent / "map_projector_info.yaml"
        median = sample_median(cloud_path)
        large = float(np.max(median)) > 10000.0
        if info.is_file() and not large:
            mode = projector_of(info)
            frame = (
                f"横轴墨卡托，起点 ({mode['latitude']:.8f}, {mode['longitude']:.8f})，"
                f"高度零点 {mode['altitude']:.3f} m"
            )
        elif large:
            epsg, _traj = utm_epsg(map_path.parent)
            mode = {"kind": "utm", "epsg": epsg}
            frame = f"UTM {epsg}"
        else:
            _epsg, traj = utm_epsg(map_path.parent)
            origin = None if traj is None else traj_grid_origin(traj)
            if origin is None:
                raise SystemExit("局部地图缺少 map_projector_info.yaml 或轨迹起点")
            mode = {"kind": "offset", "epsg": _epsg, "origin": origin}
            frame = f"局部坐标，起点减去 UTM ({origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f})"
    rtk_xyz = project_rtk(fixes, mode)
    bounds = (
        float(rtk_xyz[:, 0].min() - MARGIN),
        float(rtk_xyz[:, 0].max() + MARGIN),
        float(rtk_xyz[:, 1].min() - MARGIN),
        float(rtk_xyz[:, 1].max() + MARGIN),
    )
    cloud = load_points(cloud_path, bounds, MATCH_VOXEL, status)
    offset, gap = height_offset(rtk_xyz, cloud)
    note = "RTK 高度与地图一致"
    if offset != 0.0:
        rtk_xyz[:, 2] -= offset
        note = f"椭球高比地图地面高 {gap:.2f} m，位置先验放在地面以上 1.6 m"
    local_origin = np.array([float(rtk_xyz[:, 0].mean()), float(rtk_xyz[:, 1].mean()), 0.0])
    cloud = cloud - local_origin
    rtk_xyz = rtk_xyz - local_origin
    rtk = {"time": fixes[:, 0], "xyz": rtk_xyz, "pose_time": quats[:, 0], "quat": quats[:, 1:]}
    status("地图和 RTK 已就绪")
    return {
        "map": cloud,
        "rtk": rtk,
        "origin": local_origin,
        "frame": frame,
        "note": note,
        "height_offset": offset,
        "topics": topics,
        "yaw_span": pair.yaw_span(rtk),
        "map_path": str(map_path),
        "mcap": str(mcap),
    }


def pack_cloud(points, intensity):
    count = int(len(points))
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = count
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * count
    msg.is_dense = True
    packed = np.empty((count, 4), dtype=np.float32)
    packed[:, :3] = points[:, :3]
    packed[:, 3] = intensity
    msg.data = packed.tobytes()
    return msg


def hex_rgb(value):
    text = str(value).lstrip("#")
    return tuple(int(text[index:index + 2], 16) for index in (0, 2, 4))


def pack_view():
    with SESSION.lock:
        revision = SESSION.view_revision
        layers = []
        if SESSION.map_cloud is not None and len(SESSION.map_cloud):
            layers.append((display_points(SESSION.map_cloud, 120000), (210, 210, 210), 2, 0))
        if SESSION.rtk_cloud is not None and len(SESSION.rtk_cloud):
            layers.append((np.asarray(SESSION.rtk_cloud), (255, 140, 0), 6, 1))
        ordered = sorted(SESSION.lidars.items(), key=lambda item: item[1].get("slot", 0))
        for topic, item in ordered:
            cloud = SESSION.preview.get(topic)
            if cloud is None or len(cloud) == 0:
                continue
            layers.append((display_points(cloud, 8000), hex_rgb(item["color"]), 3, 0))
    parts = [struct.pack("<4sII", b"WPV1", revision, len(layers))]
    for points, color, size, line in layers:
        xyz = np.ascontiguousarray(points[:, :3], dtype="<f4")
        parts.append(struct.pack("<IBBBBB3x", len(xyz), color[0], color[1], color[2], size, line))
        parts.append(xyz.tobytes())
    return b"".join(parts)


def display_points(points, limit):
    if len(points) <= limit:
        return points
    stride = int(np.ceil(len(points) / limit))
    return points[::stride]


def transformed(scan, pose):
    return (pose[:3, :3] @ scan.T).T + pose[:3, 3]


def shown_pose(topic, start):
    anchor = SESSION.anchors.get(topic)
    view = SESSION.view_bodies.get(topic)
    if anchor is None or view is None or start is None:
        return start
    return view @ np.linalg.inv(anchor) @ start


class Session:
    def __init__(self):
        self.lock = threading.Lock()
        self.scene = None
        self.phase = "sources"
        self.message = "选择地图和标定数据包，然后读取。"
        self.mcap = None
        self.map_path = None
        self.error = ""
        self.log = []
        self.reference = ""
        self.selected = []
        self.lidars = {}
        self.poses = {}
        self.scans = {}
        self.anchors = {}
        self.view_scans = {}
        self.view_bodies = {}
        self.view_stamps = {}
        self.result = None
        self.busy = False
        self.map_cloud = None
        self.rtk_cloud = None
        self.preview = {}
        self.dirty = False
        self.view_revision = 0
        self.pose_ticket = {}
        self.extrinsic_path = ""
        self.extrinsic_poses = {}
        self.extrinsic_keys = []
        self.initial_extrinsics = {}
        self.extrinsic_request = None

    def add_log(self, text):
        print(text, flush=True)
        with self.lock:
            self.log.append(text)
            self.log = self.log[-80:]
            self.message = text

    def _lidar_snapshot(self):
        lidars = {}
        for key, value in self.lidars.items():
            item = dict(value)
            pose = self.poses.get(key)
            if pose is not None:
                item["pose"] = pose_dict(pose)
                item["view_pose"] = pose_dict(shown_pose(key, pose))
            lidars[key] = item
        return lidars

    def snapshot(self):
        with self.lock:
            topics = [] if self.scene is None else self.scene["topics"]
            return {
                "phase": self.phase,
                "message": self.message,
                "mcap": "" if self.mcap is None else str(self.mcap),
                "map_path": "" if self.map_path is None else str(self.map_path),
                "suggest_mcap": os.environ.get("SUGGEST_MCAP", ""),
                "suggest_map": os.environ.get("SUGGEST_MAP", ""),
                "suggest_yaml": os.environ.get("SUGGEST_YAML", ""),
                "extrinsic_path": self.extrinsic_path,
                "extrinsic_keys": list(self.extrinsic_keys),
                "error": self.error,
                "frame": "" if self.scene is None else self.scene["frame"],
                "note": "" if self.scene is None else self.scene["note"],
                "yaw_span": 0.0 if self.scene is None else round(self.scene["yaw_span"], 1),
                "map_points": 0 if self.scene is None else int(len(self.scene["map"])),
                "rtk_count": 0 if self.scene is None else int(len(self.scene["rtk"]["time"])),
                "topics": topics,
                "reference": self.reference,
                "selected": list(self.selected),
                "lidars": self._lidar_snapshot(),
                "log": list(self.log),
                "result": self.result,
                "busy": self.busy,
                "view_revision": self.view_revision,
                "saved_starts": saved_starts_info(self.map_path, self.mcap),
            }


SESSION = Session()


def align_topic(topic, skip):
    scene = SESSION.scene
    scans = pair.read_scans(active_mcap(), topic, 4, skip)
    if not scans:
        return None
    last = None
    for stamp, scan in scans:
        body = pair.body_pose(scene["rtk"], stamp)
        if body is None:
            continue
        z_offsets = (-1.6, -0.4, 0.8, 2.0) if scene["height_offset"] else (-0.5, 0.7, 1.9)
        pose, info = pair.initialize_from_rtk(scan, body, scene["map"], z_offsets)
        locked = info["median"] <= 0.40 and info["inlier_1"] >= 0.85
        last = (stamp, pose, scan, info, locked)
        if locked:
            return last
    return last


def remember_preview(topic, scan, pose):
    cloud = display_points(transformed(scan, pose), 12000)
    with SESSION.lock:
        SESSION.preview[topic] = cloud
        SESSION.dirty = True
        SESSION.view_revision += 1


def ensure_scan(topic):
    with SESSION.lock:
        if topic not in SESSION.lidars:
            raise SystemExit("这颗雷达不在当前标定里")
        cached = SESSION.scans.get(topic)
        stamp = SESSION.lidars[topic].get("stamp")
        skip = int(SESSION.lidars[topic].get("skip", 0))
    if cached is not None:
        return cached, stamp
    scans = pair.read_scans(active_mcap(), topic, 1, skip)
    if not scans:
        return None, None
    stamp, scan = scans[0]
    with SESSION.lock:
        SESSION.scans[topic] = scan
        SESSION.lidars[topic]["stamp"] = float(stamp)
    return scan, float(stamp)


def publish_manual(topic, scan, pose, detail, info=None, locked=False, ticket=None):
    with SESSION.lock:
        shown_scan = SESSION.view_scans.get(topic)
        shown = shown_pose(topic, pose)
    cloud = display_points(transformed(scan if shown_scan is None else shown_scan, shown), 12000)
    with SESSION.lock:
        if topic not in SESSION.lidars:
            return False
        if ticket is not None and SESSION.pose_ticket.get(topic) != ticket:
            return False
        SESSION.preview[topic] = cloud
        SESSION.poses[topic] = pose
        SESSION.dirty = True
        SESSION.view_revision += 1
        item = SESSION.lidars[topic]
        item["status"] = "ready" if locked else "manual"
        item["locked"] = locked
        item["confirmed"] = False
        item["method"] = "gicp" if info else "manual"
        item["detail"] = detail
        for key in ("median", "inlier_0_3", "inlier_1", "overlap", "refined"):
            item.pop(key, None)
        if info is not None:
            item["median"] = round(float(info["median"]), 3)
            item["inlier_0_3"] = round(float(info["inlier_0_3"]), 3)
            item["inlier_1"] = round(float(info["inlier_1"]), 3)
            item["method"] = "gicp"
    return True


def manual_transform(topic, stamp, values):
    with SESSION.lock:
        current = None if topic not in SESSION.poses else SESSION.poses[topic].copy()
        scene = SESSION.scene
    if values.get("keep_rotation") and current is not None:
        transform = current
        transform[:3, 3] = (values["x"], values["y"], values["z"])
        return transform
    if all(key in values for key in ("roll", "pitch", "yaw")):
        return pair.pose_matrix(values)
    body = None if scene is None or stamp is None else pair.body_pose(scene["rtk"], stamp)
    transform = np.eye(4) if body is None else np.array(body, copy=True)
    transform[:3, 3] = (values["x"], values["y"], values["z"])
    return transform


def run_manual(topic, values, ticket):
    try:
        scan, stamp = ensure_scan(topic)
        if scan is None:
            raise SystemExit("没有可用点云")
        pose = manual_transform(topic, stamp, values)
        with SESSION.lock:
            if SESSION.pose_ticket.get(topic) != ticket or SESSION.busy:
                return
        if not publish_manual(topic, scan, pose, "位置已发布，可改数值或用 GICP 精配准", ticket=ticket):
            return
        label, _frame = topic_meta(topic)
        SESSION.add_log(
            f"{label} 位置 ({pose[0, 3]:.2f}, {pose[1, 3]:.2f}, {pose[2, 3]:.2f})"
        )
    except (Exception, SystemExit) as exc:
        traceback.print_exc()
        with SESSION.lock:
            if topic in SESSION.lidars and SESSION.pose_ticket.get(topic) == ticket:
                SESSION.lidars[topic]["detail"] = str(exc)
        SESSION.add_log(f"发布位置失败: {exc}")


def gicp_refine(scan, guess, map_points):
    local = pair.local_cloud(map_points, guess[:2, 3], 45.0)
    if len(local) < 300:
        raise SystemExit("这个位置 45 m 内地图点不足")
    current = np.array(guess, dtype=np.float64, copy=True)
    last = None
    for distance, iterations in ((8.0, 30), (3.0, 25), (1.0, 20), (0.4, 15)):
        last = small_gicp.align(
            local,
            scan,
            current,
            registration_type="GICP",
            voxel_resolution=0.4,
            downsampling_resolution=0.4,
            max_correspondence_distance=distance,
            num_threads=8,
            max_iterations=iterations,
            verbose=False,
        )
        current = np.asarray(last.T_target_source, dtype=np.float64)
    stats = pair.near_residual(cKDTree(local), scan, current)
    stats["converged"] = bool(last.converged)
    stats["shift_xy"] = float(np.linalg.norm(current[:2, 3] - guess[:2, 3]))
    return current, stats


def run_gicp(topic):
    try:
        with SESSION.lock:
            if topic not in SESSION.lidars or topic not in SESSION.poses or SESSION.busy:
                return
            SESSION.busy = True
            SESSION.pose_ticket[topic] = SESSION.pose_ticket.get(topic, 0) + 1
            SESSION.lidars[topic]["status"] = "running"
            guess = SESSION.poses[topic].copy()
            map_points = SESSION.scene["map"]
        scan, _stamp = ensure_scan(topic)
        if scan is None:
            raise SystemExit("没有可用点云")
        label, _frame = topic_meta(topic)
        SESSION.add_log(f"{label} 正在用 GICP 精配准")
        pose, info = gicp_refine(scan, guess, map_points)
        if info["shift_xy"] > 15.0 and info["median"] > 1.0:
            raise SystemExit(f"GICP 偏离 {info['shift_xy']:.1f} m，保留原位置")
        locked = info["median"] <= 0.40 and info["inlier_1"] >= 0.85
        detail = "起点可以确认" if locked else "残差不合格，继续微调后再配准"
        publish_manual(topic, scan, pose, detail, info, locked)
        with SESSION.lock:
            SESSION.busy = False
        SESSION.add_log(
            f"{label} GICP 近处 {info['median']:.3f} m，1 m 内 {info['inlier_1']:.3f}，"
            f"平移 {info['shift_xy']:.2f} m，{'锁定' if locked else '未锁定'}"
        )
    except (Exception, SystemExit) as exc:
        traceback.print_exc()
        with SESSION.lock:
            SESSION.busy = False
            if topic in SESSION.lidars:
                SESSION.lidars[topic]["status"] = "manual"
                SESSION.lidars[topic]["detail"] = str(exc)
        SESSION.add_log(f"GICP 精配准失败: {exc}")


def align_with_extrinsic(topic, values):
    scans = pair.read_scans(active_mcap(), topic, 4, 0)
    if not scans:
        return None
    extrinsic = pair.pose_matrix(values)
    last = None
    for stamp, scan in scans:
        body = pair.body_pose(SESSION.scene["rtk"], stamp)
        if body is None:
            continue
        try:
            pose, info = gicp_refine(scan, body @ extrinsic, SESSION.scene["map"])
        except SystemExit:
            continue
        info = dict(info)
        info["method"] = "初始外参"
        info.setdefault("overlap", info["inlier_1"])
        info.setdefault("refined_overlap", info["inlier_1"])
        locked = info["median"] <= 0.40 and info["inlier_1"] >= 0.85
        last = (stamp, pose, scan, info, locked)
        if locked or info["median"] <= 1.0:
            return last
    return last


def run_align(topics, reference):
    try:
        with SESSION.lock:
            SESSION.busy = True
            SESSION.phase = "aligning"
            SESSION.error = ""
            SESSION.reference = reference
            SESSION.selected = list(topics)
            SESSION.poses = {}
            SESSION.scans = {}
            SESSION.anchors = {}
            SESSION.view_scans = {}
            SESSION.view_bodies = {}
            SESSION.view_stamps = {}
            SESSION.preview = {}
            SESSION.result = None
            SESSION.view_revision += 1
            SESSION.dirty = True
            SESSION.lidars = {}
            for index, topic in enumerate(topics):
                label, _frame = topic_meta(topic)
                SESSION.lidars[topic] = {
                    "label": label,
                    "status": "waiting",
                    "locked": False,
                    "confirmed": False,
                    "color": COLORS[index % len(COLORS)],
                    "slot": index,
                    "skip": 0,
                }
        for topic in topics:
            with SESSION.lock:
                SESSION.lidars[topic]["status"] = "running"
            label, _frame = topic_meta(topic)
            found = None
            with SESSION.lock:
                key = SESSION.initial_extrinsics.get(topic, "")
                values = None if not key else SESSION.extrinsic_poses.get(key)
            if values is not None:
                SESSION.add_log(
                    f"{label} 用初始外参 {key} 配准，外参高度 {values['z']:.2f} m，不改成 1.0"
                )
                found = align_with_extrinsic(topic, values)
            if found is None:
                SESSION.add_log(f"{label} 正在用 RTK 位置搜索航向")
                found = align_topic(topic, 0)
            if found is None:
                with SESSION.lock:
                    SESSION.lidars[topic]["status"] = "failed"
                    SESSION.lidars[topic]["detail"] = "没有可用点云"
                continue
            stamp, pose, scan, info, locked = found
            anchor = pair.body_pose(SESSION.scene["rtk"], stamp)
            with SESSION.lock:
                if anchor is not None:
                    SESSION.anchors[topic] = anchor
                    SESSION.view_bodies[topic] = anchor
                SESSION.view_scans[topic] = scan
                SESSION.view_stamps[topic] = float(stamp)
            remember_preview(topic, scan, pose)
            with SESSION.lock:
                SESSION.poses[topic] = pose
                SESSION.scans[topic] = scan
                item = SESSION.lidars[topic]
                item.update(
                    {
                        "status": "ready" if locked else "failed",
                        "locked": locked,
                        "stamp": stamp,
                        "method": info.get("method", "gicp"),
                        "overlap": round(float(info.get("overlap", info["inlier_1"])), 3),
                        "refined": round(float(info.get("refined_overlap", info["inlier_1"])), 3),
                        "median": round(float(info["median"]), 3),
                        "inlier_0_3": round(float(info["inlier_0_3"]), 3),
                        "inlier_1": round(float(info["inlier_1"]), 3),
                        "detail": "起点可以确认" if locked else "残差不合格，换一帧再搜",
                    }
                )
            SESSION.add_log(
                f"{label} {info.get('method', 'gicp')} 近处 {info['median']:.3f} m，"
                f"1 m 内 {info['inlier_1']:.3f}，雷达高度 {pose[2, 3]:.2f} m，"
                f"{'锁定' if locked else '未锁定'}"
            )
        with SESSION.lock:
            SESSION.phase = "review"
            SESSION.busy = False
            SESSION.message = "确认每一颗雷达的第一帧起点"
    except (Exception, SystemExit) as exc:
        traceback.print_exc()
        with SESSION.lock:
            SESSION.busy = False
            SESSION.phase = "review" if SESSION.lidars else "select"
            SESSION.error = str(exc)
        SESSION.add_log(f"起点搜索失败: {exc}")


def synced_body(rtk, stamp):
    def nearest(times):
        if len(times) == 0:
            return None
        index = int(np.searchsorted(times, stamp))
        gaps = []
        for candidate in (index - 1, index):
            if 0 <= candidate < len(times):
                gaps.append(abs(float(times[candidate]) - float(stamp)))
        return min(gaps) if gaps else None

    if nearest(rtk["time"]) is None or nearest(rtk["time"]) > SYNC_GAP:
        return None
    if nearest(rtk["pose_time"]) is None or nearest(rtk["pose_time"]) > SYNC_GAP:
        return None
    return pair.body_pose(rtk, stamp)


def next_synced_scan(topic, after_stamp, rtk):
    reader = pair.open_reader(active_mcap(), topic)
    seen = 0
    while reader.has_next():
        _topic, raw, _stamp = reader.read_next()
        scan = pair.read_scan(raw)
        if scan is None:
            continue
        seen += 1
        stamp, points = scan
        if stamp <= after_stamp + 1.0e-3:
            continue
        body = synced_body(rtk, stamp)
        if body is None:
            continue
        return seen, float(stamp), pair.voxel_down(points, pair.SCAN_VOXEL), body
    return None


def carry_with_rtk(old_body, new_body, pose):
    return new_body @ np.linalg.inv(old_body) @ pose


def run_retry(topic):
    try:
        with SESSION.lock:
            if topic not in SESSION.lidars or SESSION.busy:
                return
            if topic not in SESSION.poses:
                SESSION.error = "还没有外参，不能换帧"
                return
            SESSION.busy = True
            SESSION.error = ""
            pose = SESSION.poses[topic].copy()
            after_stamp = SESSION.view_stamps.get(topic)
            if after_stamp is None:
                after_stamp = float(SESSION.lidars[topic].get("stamp") or 0.0)
            anchor = None if topic not in SESSION.anchors else SESSION.anchors[topic].copy()
            rtk = SESSION.scene["rtk"]
            if anchor is None:
                anchor = pair.body_pose(rtk, float(SESSION.lidars[topic].get("stamp") or after_stamp))
                if anchor is not None:
                    SESSION.anchors[topic] = anchor
        found = next_synced_scan(topic, after_stamp, rtk)
        if found is None:
            with SESSION.lock:
                SESSION.lidars[topic]["detail"] = "后面没有与 RTK 同步的点云"
                SESSION.busy = False
            return
        _seen, stamp, scan, new_body = found
        shown = pose if anchor is None else new_body @ np.linalg.inv(anchor) @ pose
        with SESSION.lock:
            SESSION.view_scans[topic] = scan
            SESSION.view_bodies[topic] = new_body
            SESSION.view_stamps[topic] = stamp
            SESSION.lidars[topic]["detail"] = "已换点云，起点仍是第一帧"
        remember_preview(topic, scan, shown)
        with SESSION.lock:
            SESSION.busy = False
        label, _frame = topic_meta(topic)
        SESSION.add_log(f"{label} 已换点云，起点数值不变")
    except (Exception, SystemExit) as exc:
        traceback.print_exc()
        with SESSION.lock:
            SESSION.busy = False
            if topic in SESSION.lidars:
                SESSION.lidars[topic]["status"] = "manual"
                SESSION.lidars[topic]["detail"] = str(exc)
        SESSION.add_log(f"换帧失败: {exc}")


def pose_dict(transform):
    roll, pitch, yaw = pair.rotation_to_rpy(transform[:3, :3])
    return {
        "x": float(transform[0, 3]),
        "y": float(transform[1, 3]),
        "z": float(transform[2, 3]),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
    }


def write_result(reference, estimates, rtk_pose, span, baseline):
    ref_frame = topic_meta(reference)[1]
    lines = [
        f"# 父坐标系是基准雷达 {ref_frame}。",
        "# 单位 m / rad，RPY 为固定轴 XYZ。",
        "# RTK 位置是接收机输出的后轴中心，没有再加天线杆臂。",
        "# 写入 baseline 时，基准雷达高度固定为 1.0 m，只影响本文件，不改变标定过程。",
        "# 其余雷达由该基准位姿乘上相对外参，和基准的相对位置保持不变。",
        f"# 地图 {SESSION.map_path}",
        f"# 数据包 {SESSION.mcap}",
        f"# 坐标系 {SESSION.scene['frame']}。{SESSION.scene['note']}。",
        f"# RTK 航向变化 {span:.1f} deg。",
        "",
    ]
    for topic, estimate in estimates.items():
        frame = topic_meta(topic)[1]
        lines.append(
            f"# {frame} 相对 {ref_frame}。使用帧 {estimate['count']}，平移离散度 {estimate['spread']:.3f} m。"
        )
        lines.append(f"{frame}_{ref_frame}:")
        for field in ("x", "y", "z", "roll", "pitch", "yaw"):
            lines.append(f"  {field}: {estimate[field]:.10f}")
        lines.append("")
    shown = []
    if rtk_pose is not None:
        lines.append(
            f"# RTK 后轴中心在基准雷达坐标系下。使用帧 {rtk_pose['count']}，"
            f"平移离散度 {rtk_pose['spread']:.3f} m。"
        )
        lines.append(f"rtk_{ref_frame}:")
        for field in ("x", "y", "z", "roll", "pitch", "yaw"):
            lines.append(f"  {field}: {rtk_pose[field]:.10f}")
        lines.append("")
        inverse = pair.invert_pose(rtk_pose)
        lines.append("# 标定得到的基准雷达在 RTK 后轴坐标系下，高度未改。")
        lines.append(f"{ref_frame}_rtk:")
        for field in ("x", "y", "z", "roll", "pitch", "yaw"):
            lines.append(f"  {field}: {inverse[field]:.10f}")
        lines.append("")
        shown = [rtk_pose, inverse]
    if baseline:
        ref_pose = next(iter(baseline.values()))
        if ref_pose.get("known") is False:
            lines.append("# baseline 水平位置和姿态没有可用的 RTK 结果，只把基准高度写成 1.0。")
        else:
            lines.append("# baseline 外参。基准雷达 z 固定为 1.0，其余雷达相对基准不变。")
        for frame, pose in baseline.items():
            lines.append(f"{frame}_base_link:")
            for field in ("x", "y", "z", "roll", "pitch", "yaw"):
                lines.append(f"  {field}: {pose[field]:.10f}")
            lines.append("")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text("\n".join(lines), encoding="utf-8")
    return shown


def relative_prior(reference, topic):
    with SESSION.lock:
        if reference not in SESSION.poses or topic not in SESSION.poses:
            raise SystemExit("缺少雷达起点，不能计算相对外参")
        ref_pose = SESSION.poses[reference].copy()
        side_pose = SESSION.poses[topic].copy()
        ref_anchor = None if reference not in SESSION.anchors else SESSION.anchors[reference].copy()
        side_anchor = None if topic not in SESSION.anchors else SESSION.anchors[topic].copy()
    if ref_anchor is not None and side_anchor is not None:
        ref_pose = np.linalg.inv(ref_anchor) @ ref_pose
        side_pose = np.linalg.inv(side_anchor) @ side_pose
    return np.linalg.inv(ref_pose) @ side_pose


def estimate_from_matrix(transform, count, spread):
    pose = pose_dict(transform)
    pose["count"] = int(count)
    pose["spread"] = float(spread)
    return pose


def match_lidar_pair(reference, topic, prior, progress=None):
    ref_scans = []
    reader = pair.open_reader(active_mcap(), reference)
    while reader.has_next():
        _topic, raw, _stamp = reader.read_next()
        scan = pair.read_scan(raw)
        if scan is None:
            continue
        stamp, points = scan
        ref_scans.append((float(stamp), pair.voxel_down(points, pair.SCAN_VOXEL)))
    if not ref_scans:
        return []
    ref_times = np.asarray([item[0] for item in ref_scans])
    reader = pair.open_reader(active_mcap(), topic)
    relatives = []
    usable = count = failed = missing = 0
    while reader.has_next():
        _topic, raw, _stamp = reader.read_next()
        scan = pair.read_scan(raw)
        if scan is None:
            continue
        stamp, points = scan
        usable += 1
        if (usable - 1) % pair.SIDE_STRIDE != 0:
            continue
        count += 1
        index = int(np.searchsorted(ref_times, stamp))
        chosen = None
        gap = 0.05
        for candidate in (index - 1, index):
            if 0 <= candidate < len(ref_scans):
                delta = abs(ref_scans[candidate][0] - float(stamp))
                if delta <= gap:
                    gap = delta
                    chosen = ref_scans[candidate][1]
        if chosen is None or len(chosen) < 100:
            missing += 1
            refined = None
        else:
            source = pair.voxel_down(points, pair.SCAN_VOXEL)
            result = small_gicp.align(
                chosen,
                source,
                prior,
                registration_type="GICP",
                voxel_resolution=pair.SCAN_VOXEL,
                downsampling_resolution=pair.SCAN_VOXEL,
                max_correspondence_distance=2.0,
                num_threads=8,
                max_iterations=20,
                verbose=False,
            )
            candidate = np.asarray(result.T_target_source, dtype=np.float64)
            shift = float(np.linalg.norm(candidate[:3, 3] - prior[:3, 3]))
            if bool(result.converged) and shift <= pair.FAIL_SHIFT:
                refined = candidate
                relatives.append(candidate)
            else:
                refined = None
                failed += 1
        if count == 1 or count % 10 == 0:
            if progress is not None:
                progress(count, len(relatives), failed)
    return relatives


def baseline_reference_pose(reference, rtk_pose):
    measured = None
    if rtk_pose is not None:
        measured = pair.invert_pose(rtk_pose)
    else:
        with SESSION.lock:
            anchor = None if reference not in SESSION.anchors else SESSION.anchors[reference].copy()
            pose = None if reference not in SESSION.poses else SESSION.poses[reference].copy()
        if anchor is not None and pose is not None:
            measured = pose_dict(np.linalg.inv(anchor) @ pose)
    known = measured is not None
    if not known:
        measured = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    pose = {key: float(measured[key]) for key in ("x", "y", "z", "roll", "pitch", "yaw")}
    pose["measured_z"] = pose["z"]
    pose["z"] = BASELINE_Z
    pose["known"] = known
    return pose


def baseline_extrinsics(reference, ref_pose, estimates):
    reference_pose = {key: ref_pose[key] for key in ("x", "y", "z", "roll", "pitch", "yaw")}
    reference_pose["known"] = bool(ref_pose.get("known"))
    ordered = {topic_meta(reference)[1]: reference_pose}
    base = pair.pose_matrix(ref_pose)
    for topic, estimate in estimates.items():
        side = pose_dict(base @ pair.pose_matrix(estimate))
        ordered[topic_meta(topic)[1]] = side
    return ordered


def run_optimize():
    try:
        with SESSION.lock:
            if SESSION.busy:
                return
            pending = [topic for topic in SESSION.selected if not SESSION.lidars[topic].get("confirmed")]
            if pending:
                SESSION.error = "还有雷达起点没有确认"
                return
            reference = SESSION.reference
            if reference not in SESSION.poses:
                SESSION.error = "没有基准雷达起点"
                return
            SESSION.busy = True
            SESSION.phase = "optimizing"
            SESSION.error = ""
            start = SESSION.poses[reference].copy()
            reference_locked = bool(SESSION.lidars[reference].get("locked"))
        scene = SESSION.scene
        estimates = {}
        stamps = poses = None
        rtk_pose = None
        if reference_locked:
            def reference_progress(usable, matched, failed):
                SESSION.add_log(f"基准雷达已处理 {usable} 帧，成功 {matched}，失败 {failed}")

            try:
                SESSION.add_log("开始逐帧优化基准雷达")
                stamps, poses = pair.match_reference(
                    active_mcap(), scene["map"], start, reference, reference_progress
                )
            except SystemExit as exc:
                SESSION.add_log(f"地图轨迹不可用：{exc}。改为雷达相对外参")
                stamps = None
        else:
            SESSION.add_log("基准雷达未达到 RTK 条件，先标定雷达之间的相对外参")
        if stamps is not None:
            for topic in SESSION.selected:
                if topic == reference:
                    continue
                with SESSION.lock:
                    side_pose = SESSION.poses[topic]
                    side_stamp = SESSION.lidars[topic]["stamp"]
                reference_pose = pair.nearest_pose(stamps, poses, side_stamp, gap=1.0)
                if reference_pose is None:
                    SESSION.add_log(f"{topic_meta(topic)[0]} 起点时刻没有基准雷达位姿")
                    continue
                prior = np.linalg.inv(reference_pose) @ side_pose

                def side_progress(count, pairs, failed, label=topic_meta(topic)[0]):
                    SESSION.add_log(f"{label} 已匹配 {count} 帧，成功 {pairs}，失败 {failed}")

                relatives = pair.match_side(
                    active_mcap(), scene["map"], stamps, poses, prior, topic, side_progress
                )
                estimate = pair.robust_pose(relatives)
                if estimate is None:
                    SESSION.add_log(f"{topic_meta(topic)[0]} 有效匹配不足")
                    continue
                estimates[topic] = estimate
                SESSION.add_log(
                    f"{topic_meta(topic)[0]} t=({estimate['x']:.3f}, {estimate['y']:.3f}, {estimate['z']:.3f})"
                )
            rtk_pose = pair.calibrate_rtk(stamps, poses, scene["rtk"])
            if rtk_pose is None:
                SESSION.add_log("基准雷达与 RTK 的有效样本不足")
            else:
                SESSION.add_log(
                    f"RTK t=({rtk_pose['x']:.3f}, {rtk_pose['y']:.3f}, {rtk_pose['z']:.3f})"
                )
        else:
            for topic in SESSION.selected:
                if topic == reference:
                    continue
                label = topic_meta(topic)[0]
                prior = relative_prior(reference, topic)

                def pair_progress(count, pairs, failed, name=label):
                    SESSION.add_log(f"{name} 相对配准 {count} 帧，成功 {pairs}，失败 {failed}")

                SESSION.add_log(f"{label} 正在与基准雷达配准")
                relatives = match_lidar_pair(reference, topic, prior, pair_progress)
                estimate = pair.robust_pose(relatives)
                if estimate is None:
                    estimate = estimate_from_matrix(prior, 1, 0.0)
                    SESSION.add_log(f"{label} 多帧不足，使用当前起点的相对位置")
                estimates[topic] = estimate
                SESSION.add_log(
                    f"{label} t=({estimate['x']:.3f}, {estimate['y']:.3f}, {estimate['z']:.3f})"
                )
        ref_baseline = baseline_reference_pose(reference, rtk_pose)
        baseline = baseline_extrinsics(reference, ref_baseline, estimates)
        SESSION.add_log(
            f"写入 baseline 时基准高度 {ref_baseline['measured_z']:.3f} m 改为 {BASELINE_Z:.1f} m"
        )
        write_result(reference, estimates, rtk_pose, scene["yaw_span"], baseline)
        if stamps is not None:
            trajectory = OUTPUT.with_suffix(".trajectory.txt")
            pair.write_trajectory(trajectory, stamps, poses, scene["origin"], scene["frame"])
        result = {
            "path": str(OUTPUT),
            "reference": topic_meta(reference)[1],
            "lidars": {
                topic_meta(topic)[1]: {key: estimate[key] for key in ("x", "y", "z", "roll", "pitch", "yaw", "count", "spread")}
                for topic, estimate in estimates.items()
            },
            "baseline": baseline,
        }
        if rtk_pose is not None:
            inverse = pair.invert_pose(rtk_pose)
            result["rtk"] = {key: rtk_pose[key] for key in ("x", "y", "z", "roll", "pitch", "yaw", "count", "spread")}
            result["lidar_in_rtk"] = {
                key: inverse[key] for key in ("x", "y", "z", "roll", "pitch", "yaw")
            }
        with SESSION.lock:
            SESSION.result = result
            SESSION.phase = "done"
            SESSION.busy = False
            SESSION.message = f"结果已写入 {OUTPUT}"
        SESSION.add_log(f"结果已写入 {OUTPUT}")
    except (Exception, SystemExit) as exc:
        traceback.print_exc()
        with SESSION.lock:
            SESSION.busy = False
            SESSION.phase = "review"
            SESSION.error = str(exc)
        SESSION.add_log(f"逐帧优化失败: {exc}")


def data_roots():
    roots = []
    for item in os.environ.get("DATA_ROOTS", "").split(":"):
        if not item:
            continue
        path = Path(item)
        if path.is_dir():
            roots.append(path.resolve())
    return roots


def path_allowed(path: Path) -> bool:
    try:
        resolved = path.resolve()
    except OSError:
        return False
    for root in data_roots():
        if resolved == root or root in resolved.parents:
            return True
    return False


def active_mcap() -> Path:
    path = SESSION.mcap
    if path is None:
        raise SystemExit("还没有选择数据包")
    return path


def validate_sources(mcap_text, map_text):
    if not str(mcap_text).strip() or not str(map_text).strip():
        raise SystemExit("请选择地图和标定数据包")
    mcap = Path(str(mcap_text)).expanduser()
    map_path = Path(str(map_text)).expanduser()
    if not path_allowed(mcap) or not path_allowed(map_path):
        raise SystemExit("路径不在可读取的数据目录里")
    mcap = mcap.resolve()
    map_path = map_path.resolve()
    if not mcap.is_file() or mcap.suffix.lower() != ".mcap":
        raise SystemExit("标定数据需要是 .mcap 文件")
    if map_path.is_dir():
        if not (map_path / "pointcloud_map.pcd").is_file():
            raise SystemExit("地图目录里没有 pointcloud_map.pcd")
    elif not map_path.is_file() or map_path.suffix.lower() != ".pcd":
        raise SystemExit("地图需要是 .pcd 文件，或含 pointcloud_map.pcd 的目录")
    return mcap, map_path


def clear_calibration():
    SESSION.selected = []
    SESSION.lidars = {}
    SESSION.poses = {}
    SESSION.scans = {}
    SESSION.anchors = {}
    SESSION.view_scans = {}
    SESSION.view_bodies = {}
    SESSION.view_stamps = {}
    SESSION.preview = {}
    SESSION.result = None
    SESSION.pose_ticket = {}
    SESSION.dirty = True
    SESSION.view_revision += 1


def clear_runtime():
    clear_calibration()
    SESSION.scene = None
    SESSION.map_cloud = None
    SESSION.rtk_cloud = None
    SESSION.reference = ""
    SESSION.initial_extrinsics = {}


def validate_extrinsic(text):
    raw = str(text or "").strip()
    if not raw:
        return None
    path = Path(raw).expanduser()
    if not path_allowed(path):
        raise SystemExit("外参路径不在可读取的数据目录里")
    path = path.resolve()
    if not path.is_file() or path.suffix.lower() not in {".yaml", ".yml"}:
        raise SystemExit("初始外参需要是 yaml 文件")
    return path


def apply_extrinsic_request():
    path = SESSION.extrinsic_request
    if path is None:
        with SESSION.lock:
            SESSION.extrinsic_path = ""
            SESSION.extrinsic_poses = {}
            SESSION.extrinsic_keys = []
            if SESSION.scene is not None:
                for topic in SESSION.scene["topics"]:
                    topic["extrinsic"] = ""
        return 0
    poses = load_extrinsic_yaml(path)
    with SESSION.lock:
        SESSION.extrinsic_path = str(path)
        SESSION.extrinsic_poses = poses
        SESSION.extrinsic_keys = list(poses)
        topics = [] if SESSION.scene is None else SESSION.scene["topics"]
        matched = assign_extrinsics(topics, list(poses))
    SESSION.add_log(f"初始外参 {path.name}：{len(poses)} 个名称，匹配 {len(matched)} 个话题")
    return len(matched)


def starts_path(map_path, mcap):
    if not map_path or not mcap:
        return None
    digest = hashlib.sha1(f"{map_path}\n{mcap}".encode()).hexdigest()[:16]
    return OUTPUT.parent / "start_cache" / f"{digest}.yaml"


def saved_starts_info(map_path, mcap):
    path = starts_path(map_path, mcap)
    empty = {"available": False, "count": 0, "saved_at": ""}
    if path is None or not path.is_file():
        return empty
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return empty
    if str(data.get("map")) != str(map_path) or str(data.get("mcap")) != str(mcap):
        return empty
    return {
        "available": True,
        "count": len(data.get("lidars") or {}),
        "saved_at": str(data.get("saved_at") or ""),
    }


def scan_at_stamp(topic, stamp):
    reader = pair.open_reader(active_mcap(), topic)
    seen = 0
    target = float(stamp)
    while reader.has_next():
        _topic, raw, _stamp = reader.read_next()
        scan = pair.read_scan(raw)
        if scan is None:
            continue
        seen += 1
        found, points = scan
        if abs(float(found) - target) <= 2.0e-3:
            return seen, float(found), pair.voxel_down(points, pair.SCAN_VOXEL)
        if float(found) > target + 0.5:
            break
    return None


def run_restore_starts():
    try:
        with SESSION.lock:
            map_path = SESSION.map_path
            mcap = SESSION.mcap
            rtk = None if SESSION.scene is None else SESSION.scene["rtk"]
        path = starts_path(map_path, mcap)
        data = {} if path is None or not path.is_file() else yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if str(data.get("map")) != str(map_path) or str(data.get("mcap")) != str(mcap):
            raise SystemExit("这组地图和数据包没有已保存的起点")
        loaded = []
        for topic, saved in (data.get("lidars") or {}).items():
            pose = saved.get("pose") or {}
            if "x" not in pose:
                continue
            found = scan_at_stamp(topic, saved.get("stamp") or 0.0)
            if found is None:
                SESSION.add_log(f"{saved.get('label') or topic} 找不到保存时的点云帧")
                continue
            seen, stamp, scan = found
            transform = pair.pose_matrix(pose)
            body = None if rtk is None else pair.body_pose(rtk, stamp)
            loaded.append((topic, saved, seen, stamp, scan, transform, body))
        if not loaded:
            raise SystemExit("保存的起点都没有对应点云")
        with SESSION.lock:
            clear_calibration()
            reference = str(data.get("reference") or loaded[0][0])
            topics = [item[0] for item in loaded]
            if reference not in topics:
                reference = topics[0]
            SESSION.reference = reference
            SESSION.selected = topics
            SESSION.phase = "review"
            SESSION.error = ""
            for index, (topic, saved, seen, stamp, scan, transform, body) in enumerate(loaded):
                label, _frame = topic_meta(topic)
                SESSION.lidars[topic] = {
                    "label": saved.get("label") or label,
                    "status": "ready" if saved.get("locked") else "manual",
                    "locked": bool(saved.get("locked")),
                    "confirmed": bool(saved.get("confirmed")),
                    "color": saved.get("color") or COLORS[index % len(COLORS)],
                    "slot": int(saved.get("slot") if saved.get("slot") is not None else index),
                    "skip": seen,
                    "stamp": stamp,
                    "method": "saved",
                    "detail": "已恢复保存的起点",
                }
                SESSION.poses[topic] = transform
                SESSION.scans[topic] = scan
                SESSION.view_scans[topic] = scan
                SESSION.view_stamps[topic] = stamp
                SESSION.preview[topic] = display_points(transformed(scan, transform), 12000)
                if body is not None:
                    SESSION.anchors[topic] = body
                    SESSION.view_bodies[topic] = body
            SESSION.dirty = True
            SESSION.view_revision += 1
            SESSION.busy = False
            SESSION.message = f"已恢复 {len(loaded)} 颗雷达的起点"
        SESSION.add_log(f"已恢复 {len(loaded)} 颗雷达的临时起点")
    except (Exception, SystemExit) as exc:
        traceback.print_exc()
        with SESSION.lock:
            SESSION.busy = False
            SESSION.error = str(exc)
        SESSION.add_log(f"恢复起点失败: {exc}")


def load_worker():
    try:
        scene = load_scene(SESSION.mcap, SESSION.map_path, SESSION.add_log)
        display = pair.voxel_down(scene["map"], 0.80) if len(scene["map"]) > 250000 else scene["map"]
        stride = max(1, len(scene["rtk"]["xyz"]) // 1500)
        with SESSION.lock:
            SESSION.scene = scene
            SESSION.reference = preferred_reference(scene["topics"])
            SESSION.map_cloud = display
            SESSION.rtk_cloud = scene["rtk"]["xyz"][::stride]
            SESSION.dirty = True
            SESSION.view_revision += 1
            SESSION.busy = False
            SESSION.phase = "select"
            SESSION.message = "选择要标定的雷达。可选初始外参，没有外参时位置先验使用这一包的 RTK。"
        SESSION.add_log(
            f"地图 {len(scene['map'])} 点，RTK {len(scene['rtk']['time'])} 个，航向变化 {scene['yaw_span']:.1f}°"
        )
        SESSION.add_log(scene["note"])
        try:
            apply_extrinsic_request()
        except (Exception, SystemExit) as exc:
            SESSION.add_log(f"初始外参未使用: {exc}")
    except (Exception, SystemExit) as exc:
        traceback.print_exc()
        with SESSION.lock:
            SESSION.busy = False
            SESSION.phase = "sources"
            SESSION.error = str(exc)
            SESSION.message = "读取失败，请重新选择地图和数据包。"
        SESSION.add_log(f"读取失败: {exc}")


class Publisher(Node):
    def __init__(self):
        super().__init__("map_calib_view")
        self.map_pub = self.create_publisher(PointCloud2, "/calib/map", QOS)
        self.rtk_pub = self.create_publisher(PointCloud2, "/calib/rtk", QOS)
        self.lidar_pubs = [
            self.create_publisher(PointCloud2, f"/calib/lidar/n{index}/points", QOS) for index in range(len(COLORS))
        ]
        self.create_timer(0.5, self.publish_pending)

    def publish_pending(self):
        with SESSION.lock:
            if not SESSION.dirty:
                return
            map_cloud = None if SESSION.map_cloud is None else SESSION.map_cloud.copy()
            rtk_cloud = None if SESSION.rtk_cloud is None else SESSION.rtk_cloud.copy()
            previews = {topic: cloud.copy() for topic, cloud in SESSION.preview.items()}
            slots = {topic: item.get("slot", 0) for topic, item in SESSION.lidars.items()}
            SESSION.dirty = False
        if map_cloud is not None and len(map_cloud):
            self.map_pub.publish(pack_cloud(map_cloud, 40))
        if rtk_cloud is not None and len(rtk_cloud):
            self.rtk_pub.publish(pack_cloud(rtk_cloud, 255))
        slot_clouds = [None] * len(self.lidar_pubs)
        for topic, cloud in previews.items():
            slot_clouds[slots.get(topic, 0) % len(self.lidar_pubs)] = cloud
        empty = np.zeros((0, 3), dtype=np.float64)
        for slot, cloud in enumerate(slot_clouds):
            self.lidar_pubs[slot].publish(pack_cloud(empty if cloud is None else cloud, 200))


def create_app():
    root = Path(__file__).resolve().parent
    app = Flask(__name__, template_folder=str(root / "templates"), static_folder=str(root / "static"))
    app.config["TEMPLATES_AUTO_RELOAD"] = True
    app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0

    @app.get("/")
    def index():
        return render_template("map_calib.html")

    @app.get("/api/state")
    def state():
        return jsonify(SESSION.snapshot())

    @app.get("/api/view")
    def view():
        payload = pack_view()
        return Response(payload, mimetype="application/octet-stream")

    @app.get("/api/browse")
    def browse():
        kind = request.args.get("kind", "")
        raw = request.args.get("path", "").strip()
        roots = data_roots()
        if not raw:
            return jsonify({
                "path": "",
                "parent": "",
                "map_dir": False,
                "entries": [{"name": f"{root}/", "path": str(root), "kind": "dir"} for root in roots],
            })
        path = Path(raw)
        if not path.exists() or not path_allowed(path):
            return jsonify({"error": "这个路径不在可浏览的数据目录里"}), 400
        path = path.resolve()
        if path.is_file():
            path = path.parent
        try:
            children = sorted(path.iterdir(), key=lambda item: (not item.is_dir(), item.name.lower()))
        except OSError as exc:
            return jsonify({"error": str(exc)}), 400
        entries = []
        for child in children:
            if child.name.startswith("."):
                continue
            try:
                is_dir = child.is_dir()
            except OSError:
                continue
            if is_dir:
                entries.append({"name": f"{child.name}/", "path": str(child), "kind": "dir"})
                continue
            suffix = child.suffix.lower()
            if kind == "mcap" and suffix != ".mcap":
                continue
            if kind == "map" and suffix != ".pcd":
                continue
            if kind == "yaml" and suffix not in {".yaml", ".yml"}:
                continue
            if suffix not in {".mcap", ".pcd", ".yaml", ".yml"}:
                continue
            entries.append({"name": child.name, "path": str(child), "kind": "file"})
        parent = "" if any(path == root for root in roots) else str(path.parent)
        return jsonify({
            "path": str(path),
            "parent": parent,
            "map_dir": (path / "pointcloud_map.pcd").is_file(),
            "entries": entries,
        })

    @app.post("/api/load")
    def load():
        payload = request.get_json(force=True)
        try:
            mcap, map_path = validate_sources(payload.get("mcap", ""), payload.get("map", ""))
            yaml_path = validate_extrinsic(payload.get("yaml", ""))
        except SystemExit as exc:
            return jsonify({"error": str(exc)}), 400
        with SESSION.lock:
            if SESSION.busy:
                return jsonify({"error": "正在读取，请稍候"}), 409
            SESSION.busy = True
            SESSION.phase = "loading"
            SESSION.error = ""
            SESSION.mcap = mcap
            SESSION.map_path = map_path
            SESSION.extrinsic_request = yaml_path
            clear_runtime()
            SESSION.message = f"正在读取 {map_path.name} 和 {mcap.name}"
        threading.Thread(target=load_worker, daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/extrinsics")
    def extrinsics():
        payload = request.get_json(force=True)
        try:
            yaml_path = validate_extrinsic(payload.get("yaml", ""))
        except SystemExit as exc:
            return jsonify({"error": str(exc)}), 400
        with SESSION.lock:
            if SESSION.busy or SESSION.scene is None:
                return jsonify({"error": "请先读取地图和数据包"}), 409
            SESSION.extrinsic_request = yaml_path
        try:
            apply_extrinsic_request()
        except SystemExit as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify({"ok": True})

    @app.post("/api/sources")
    def sources():
        with SESSION.lock:
            if SESSION.busy:
                return jsonify({"error": "正在处理，不能更换数据"}), 409
            clear_runtime()
            SESSION.phase = "sources"
            SESSION.error = ""
            SESSION.message = "选择地图和标定数据包，然后读取。"
        return jsonify({"ok": True})

    @app.post("/api/save-starts")
    def save_starts():
        with SESSION.lock:
            if SESSION.busy or SESSION.mcap is None or SESSION.map_path is None:
                return jsonify({"error": "当前不能保存起点"}), 409
            lidars = {}
            for topic in SESSION.selected:
                pose = SESSION.poses.get(topic)
                item = SESSION.lidars.get(topic)
                if pose is None or item is None:
                    continue
                lidars[topic] = {
                    "label": item.get("label", ""),
                    "stamp": item.get("stamp"),
                    "skip": int(item.get("skip", 0)),
                    "locked": bool(item.get("locked")),
                    "confirmed": bool(item.get("confirmed")),
                    "color": item.get("color"),
                    "slot": int(item.get("slot", 0)),
                    "pose": pose_dict(pose),
                }
            document = {
                "map": str(SESSION.map_path),
                "mcap": str(SESSION.mcap),
                "reference": SESSION.reference,
                "saved_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "lidars": lidars,
            }
            path = starts_path(SESSION.map_path, SESSION.mcap)
        if not document["lidars"] or path is None:
            return jsonify({"error": "还没有可保存的起点"}), 400
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        SESSION.add_log(f"已临时保存 {len(document['lidars'])} 颗雷达的起点")
        return jsonify({"ok": True, "count": len(document["lidars"])})

    @app.post("/api/restore-starts")
    def restore_starts():
        with SESSION.lock:
            if SESSION.busy or SESSION.scene is None:
                return jsonify({"error": "当前不能恢复起点"}), 409
            path = starts_path(SESSION.map_path, SESSION.mcap)
            if path is None or not path.is_file():
                return jsonify({"error": "这组地图和数据包没有已保存的起点"}), 400
            SESSION.busy = True
            SESSION.error = ""
        threading.Thread(target=run_restore_starts, daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/align")
    def align():
        payload = request.get_json(force=True)
        topics = [item for item in payload.get("topics", []) if isinstance(item, str)]
        reference = payload.get("reference", "")
        requested = payload.get("extrinsics") or {}
        with SESSION.lock:
            known = set() if SESSION.scene is None else {item["name"] for item in SESSION.scene["topics"]}
            if SESSION.phase == "loading" or SESSION.busy:
                return jsonify({"error": "当前不能开始"}), 409
            if reference not in topics or any(topic not in known for topic in topics):
                return jsonify({"error": "雷达选择无效"}), 400
            if len(topics) > len(COLORS):
                return jsonify({"error": f"一次最多标定 {len(COLORS)} 颗雷达"}), 400
            mapping = {}
            for topic in topics:
                key = str(requested.get(topic, "") or "").strip()
                if not key:
                    continue
                if key not in SESSION.extrinsic_poses:
                    return jsonify({"error": f"外参里没有 {key}"}), 400
                mapping[topic] = key
            SESSION.initial_extrinsics = mapping
        threading.Thread(target=run_align, args=(topics, reference), daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/pose")
    def pose():
        payload = request.get_json(force=True)
        topic = payload.get("topic", "")
        try:
            values = {key: float(payload[key]) for key in ("x", "y", "z")}
        except (KeyError, TypeError, ValueError):
            return jsonify({"error": "位置数值无效"}), 400
        for key in ("roll", "pitch", "yaw"):
            if key in payload and payload[key] not in (None, ""):
                try:
                    values[key] = float(payload[key])
                except (TypeError, ValueError):
                    return jsonify({"error": "角度数值无效"}), 400
        values["keep_rotation"] = bool(payload.get("keep_rotation"))
        with SESSION.lock:
            if topic not in SESSION.lidars or SESSION.busy:
                return jsonify({"error": "当前不能修改起点"}), 409
            ticket = SESSION.pose_ticket.get(topic, 0) + 1
            SESSION.pose_ticket[topic] = ticket
        threading.Thread(target=run_manual, args=(topic, values, ticket), daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/gicp")
    def gicp():
        topic = request.get_json(force=True).get("topic", "")
        with SESSION.lock:
            if SESSION.busy or topic not in SESSION.lidars:
                return jsonify({"error": "当前不能精配准"}), 409
            if topic not in SESSION.poses:
                return jsonify({"error": "先点选位置或填写数值"}), 400
        threading.Thread(target=run_gicp, args=(topic,), daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/confirm")
    def confirm():
        topic = request.get_json(force=True).get("topic", "")
        with SESSION.lock:
            item = SESSION.lidars.get(topic)
            if item is None or topic not in SESSION.poses:
                return jsonify({"error": "还没有起点"}), 400
            item["confirmed"] = True
        return jsonify({"ok": True})

    @app.post("/api/retry")
    def retry():
        topic = request.get_json(force=True).get("topic", "")
        with SESSION.lock:
            if topic not in SESSION.lidars or SESSION.busy:
                return jsonify({"error": "当前不能重搜"}), 409
        threading.Thread(target=run_retry, args=(topic,), daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/remove")
    def remove():
        topic = request.get_json(force=True).get("topic", "")
        with SESSION.lock:
            if SESSION.busy or topic not in SESSION.lidars:
                return jsonify({"error": "当前不能移出"}), 409
            if topic == SESSION.reference:
                return jsonify({"error": "基准雷达不能移出"}), 400
            SESSION.selected = [item for item in SESSION.selected if item != topic]
            SESSION.lidars.pop(topic, None)
            SESSION.preview.pop(topic, None)
            SESSION.poses.pop(topic, None)
            SESSION.scans.pop(topic, None)
            SESSION.anchors.pop(topic, None)
            SESSION.view_scans.pop(topic, None)
            SESSION.view_bodies.pop(topic, None)
            SESSION.view_stamps.pop(topic, None)
            SESSION.pose_ticket.pop(topic, None)
            SESSION.dirty = True
            SESSION.view_revision += 1
        return jsonify({"ok": True})

    @app.post("/api/optimize")
    def optimize():
        with SESSION.lock:
            if SESSION.busy or SESSION.phase not in {"review", "done"}:
                return jsonify({"error": "当前不能开始逐帧优化"}), 409
        threading.Thread(target=run_optimize, daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/reset")
    def reset():
        with SESSION.lock:
            if SESSION.busy or SESSION.scene is None:
                return jsonify({"error": "当前不能返回"}), 409
            SESSION.phase = "select"
            SESSION.selected = []
            clear_calibration()
            SESSION.error = ""
            SESSION.message = "选择要标定的雷达。位置先验使用这一包的 RTK。"
        return jsonify({"ok": True})

    return app


def main():
    rclpy.init()
    node = Publisher()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    app = create_app()
    try:
        app.run(host="0.0.0.0", port=PORT, threaded=True, use_reloader=False)
    finally:
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
