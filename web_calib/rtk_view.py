#!/usr/bin/env python3
"""按外参和 RTK 轨迹播放点云。可选叠加带经纬度或已投影的地图。

外参相对后轴中心。后轴中心就是 RTK 输出点，播放时不再把高度改成 1.0。
"""

import os
import struct
import threading
import time
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from flask import Flask, Response, jsonify, render_template, request
from geometry_msgs.msg import PoseStamped
from pyproj import CRS, Transformer
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix, PointCloud2

PORT = int(os.environ.get("PORT", "8768"))
FIX = "/sensing/gnss/ublox/nav_sat_fix"
POSE = "/sensing/gnss/pose"
FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
SKIP_KEYS = {"base_link", "gnss_base_link"}
COLORS = (
    (255, 60, 60),
    (60, 200, 80),
    (70, 140, 255),
    (255, 200, 40),
    (80, 220, 220),
    (220, 120, 220),
    (255, 120, 60),
    (180, 180, 255),
)
MARGIN = 80.0
BIN = 0.08
POINT_LIMIT = 5000
MAP_LIMIT = 140000


def data_roots():
    roots = []
    for item in os.environ.get("DATA_ROOTS", "").split(":"):
        if not item:
            continue
        path = Path(item)
        if path.is_dir():
            roots.append(path.resolve())
    return roots


def path_allowed(path: Path):
    resolved = path.resolve()
    return any(resolved == root or root in resolved.parents for root in data_roots())


def open_reader(mcap: Path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(mcap), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def load_yaml_poses(path: Path):
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
        if name in SKIP_KEYS or any(field not in block for field in FIELDS):
            continue
        poses[name] = {field: float(block[field]) for field in FIELDS}
    if not poses:
        raise SystemExit(f"{path} 里没有雷达外参")
    return poses


def rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def pose_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = rpy_matrix(pose["roll"], pose["pitch"], pose["yaw"])
    transform[:3, 3] = (pose["x"], pose["y"], pose["z"])
    return transform


def quat_to_rotation(quat):
    x, y, z, w = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def interp(times, values, stamp):
    if stamp < times[0] - 0.2 or stamp > times[-1] + 0.2:
        return None
    if stamp <= times[0]:
        return values[0]
    if stamp >= times[-1]:
        return values[-1]
    index = int(np.searchsorted(times, stamp))
    index = min(max(index, 1), len(times) - 1)
    span = float(times[index] - times[index - 1])
    ratio = 0.0 if span <= 0.0 else (stamp - float(times[index - 1])) / span
    return (1.0 - ratio) * values[index - 1] + ratio * values[index]


def list_cloud_topics(mcap: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(mcap), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    names = []
    for info in reader.get_all_topics_and_types():
        if info.type != "sensor_msgs/msg/PointCloud2":
            continue
        if "lidar" not in info.name and "points" not in info.name:
            continue
        names.append(info.name)
    if not names:
        raise SystemExit("数据包里没有点云话题")
    frames = {name: "" for name in names}
    reader = open_reader(mcap, names)
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
    return [{"name": name, "frame_id": frames[name]} for name in names]


def read_rtk(mcap: Path):
    reader = open_reader(mcap, [FIX, POSE])
    fixes, quats = [], []
    while reader.has_next():
        topic, raw, _stamp = reader.read_next()
        if topic == FIX:
            msg = deserialize_message(raw, NavSatFix)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            fixes.append((stamp, msg.latitude, msg.longitude, msg.altitude))
        elif topic == POSE:
            msg = deserialize_message(raw, PoseStamped)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            quat = msg.pose.orientation
            quats.append((stamp, quat.x, quat.y, quat.z, quat.w))
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


def utm_epsg(directory: Path):
    for traj in sorted(directory.glob("*.traj")):
        head = traj.read_text(encoding="utf-8", errors="replace")[:4000]
        found = __import__("re").search(r'AUTHORITY\["EPSG","(326\d{2})"\]', head)
        if found:
            return found.group(1)
    return "32650"


def map_mode(map_path: Path):
    if map_path is None:
        return {"kind": "utm", "epsg": "32650"}, None, "UTM 32650，未加载地图"
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
        return mode, cloud_path, frame
    info = map_path.parent / "map_projector_info.yaml"
    median = sample_median(map_path)
    large = float(np.max(median)) > 10000.0
    if info.is_file() and not large:
        mode = projector_of(info)
        frame = (
            f"横轴墨卡托，起点 ({mode['latitude']:.8f}, {mode['longitude']:.8f})，"
            f"高度零点 {mode['altitude']:.3f} m"
        )
        return mode, map_path, frame
    if large:
        epsg = utm_epsg(map_path.parent)
        return {"kind": "utm", "epsg": epsg}, map_path, f"UTM {epsg}，地图已在投影坐标"
    raise SystemExit("局部地图缺少 map_projector_info.yaml，无法和 RTK 对齐")


def project_rtk(fixes, mode):
    if mode["kind"] == "utm":
        project = Transformer.from_crs("EPSG:4326", f"EPSG:{mode['epsg']}", always_xy=True)
        east, north = project.transform(fixes[:, 2], fixes[:, 1])
        return np.column_stack((east, north, fixes[:, 3]))
    crs = CRS.from_proj4(
        f"+proj=tmerc +lat_0={mode['latitude']} +lon_0={mode['longitude']} +k={mode['scale']} "
        "+x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
    )
    project = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    east, north = project.transform(fixes[:, 2], fixes[:, 1])
    height = fixes[:, 3] - mode["altitude"]
    return np.column_stack((east, north, height))


def load_map(path: Path, bounds, status):
    layout = pcd_layout(path)
    kept = []
    step = layout["step"]
    seen = 0
    with path.open("rb") as handle:
        handle.seek(layout["offset"])
        while True:
            raw = handle.read(1_000_000 * step)
            if not raw:
                break
            xyz = chunk_xyz(raw, layout)
            finite = np.isfinite(xyz).all(axis=1)
            xmin, xmax, ymin, ymax = bounds
            finite &= (xyz[:, 0] >= xmin) & (xyz[:, 0] <= xmax)
            finite &= (xyz[:, 1] >= ymin) & (xyz[:, 1] <= ymax)
            if np.any(finite):
                kept.append(thin(xyz[finite], 20000))
                if len(kept) >= 8:
                    kept = [thin(np.vstack(kept), MAP_LIMIT)]
            seen += len(xyz)
            status(f"正在读取地图，已扫描 {seen / 1e6:.0f} 百万点")
    if not kept:
        raise SystemExit("地图范围内没有点")
    return thin(np.vstack(kept), MAP_LIMIT)


def xyz_of(msg):
    fields = {field.name: field for field in msg.fields}
    count = int(msg.width) * int(msg.height)
    step = int(msg.point_step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    count = min(count, raw.size // step if step else 0)
    if count == 0 or any(name not in fields or fields[name].datatype != 7 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)
    packed = raw[: count * step].reshape(count, step)
    columns = []
    for name in ("x", "y", "z"):
        offset = int(fields[name].offset)
        column = np.ascontiguousarray(packed[:, offset : offset + 4]).view(np.float32).reshape(-1)
        columns.append(column)
    return np.column_stack(columns).astype(np.float64)


def keep_range(points):
    finite = np.isfinite(points).all(axis=1)
    radius = np.linalg.norm(points, axis=1)
    return points[finite & (radius >= 0.8) & (radius <= 90.0)]


def thin(points, limit):
    count = len(points)
    if count <= limit:
        return points
    step = int(np.ceil(count / limit))
    return points[::step]


def body_pose(rtk, stamp):
    position = interp(rtk["time"], rtk["xyz"], stamp)
    quat = interp(rtk["pose_time"], rtk["quat"], stamp)
    if position is None or quat is None:
        return None
    quat = quat / np.linalg.norm(quat)
    transform = np.eye(4)
    transform[:3, :3] = quat_to_rotation(quat)
    transform[:3, 3] = position
    return transform


def pack_layers(layers):
    parts = [struct.pack("<4sII", b"WPV1", 1, len(layers))]
    for points, color, size, line in layers:
        xyz = np.ascontiguousarray(points[:, :3], dtype="<f4")
        parts.append(struct.pack("<IBBBBB3x", len(xyz), color[0], color[1], color[2], size, line))
        parts.append(xyz.tobytes())
    return b"".join(parts)


class Player:
    def __init__(self):
        self.lock = threading.Lock()
        self.phase = "setup"
        self.message = "选择 mcap 和外参，读取话题后再播放。"
        self.error = ""
        self.busy = False
        self.mcap = os.environ.get("SUGGEST_MCAP", "")
        self.yaml = os.environ.get("SUGGEST_YAML", "")
        self.map = os.environ.get("SUGGEST_MAP", "")
        self.topics = []
        self.keys = []
        self.poses = {}
        self.assignments = []
        self.frames = []
        self.map_cloud = None
        self.rtk_line = None
        self.frame_text = ""
        self.index = 0
        self.playing = False
        self.rate = 1.0
        self.stop = False

    def set_message(self, text):
        with self.lock:
            self.message = text

    def snapshot(self):
        with self.lock:
            frame = None
            if self.frames:
                frame = self.frames[self.index]
            return {
                "phase": self.phase,
                "message": self.message,
                "error": self.error,
                "busy": self.busy,
                "mcap": self.mcap,
                "yaml": self.yaml,
                "map": self.map,
                "suggest_mcap": os.environ.get("SUGGEST_MCAP", ""),
                "suggest_yaml": os.environ.get("SUGGEST_YAML", ""),
                "suggest_map": os.environ.get("SUGGEST_MAP", ""),
                "control_url": os.environ.get("CONTROL_URL", "http://127.0.0.1:8769"),
                "topics": self.topics,
                "keys": self.keys,
                "assignments": self.assignments,
                "frames": len(self.frames),
                "index": self.index,
                "playing": self.playing,
                "rate": self.rate,
                "frame": self.frame_text,
                "stamp": None if frame is None else frame["stamp"],
                "time": 0.0 if frame is None or not self.frames else frame["stamp"] - self.frames[0]["stamp"],
                "duration": 0.0 if len(self.frames) < 2 else self.frames[-1]["stamp"] - self.frames[0]["stamp"],
            }


PLAYER = Player()


def resolve_file(raw, suffix):
    text = str(raw or "").strip()
    if not text:
        raise SystemExit("路径为空")
    path = Path(text)
    if not path.exists() or not path_allowed(path):
        raise SystemExit(f"路径不可用: {text}")
    path = path.resolve()
    if path.is_dir() or path.suffix.lower() not in suffix:
        raise SystemExit(f"需要 {' 或 '.join(suffix)} 文件: {text}")
    return path


def inspect_worker(mcap: Path, yaml_path: Path):
    try:
        PLAYER.set_message("正在读取话题和外参")
        topics = list_cloud_topics(mcap)
        poses = load_yaml_poses(yaml_path)
        keys = list(poses)
        for topic in topics:
            frame = topic["frame_id"]
            suggested = f"{frame}_base_link" if frame and f"{frame}_base_link" in poses else ""
            topic["suggested"] = suggested
        with PLAYER.lock:
            PLAYER.topics = topics
            PLAYER.keys = keys
            PLAYER.poses = poses
            PLAYER.mcap = str(mcap)
            PLAYER.yaml = str(yaml_path)
            PLAYER.phase = "assign"
            PLAYER.error = ""
            PLAYER.message = f"{len(topics)} 个点云话题，{len(keys)} 个外参。确认对应关系后播放。"
            PLAYER.busy = False
    except (Exception, SystemExit) as exc:
        with PLAYER.lock:
            PLAYER.error = str(exc)
            PLAYER.message = str(exc)
            PLAYER.busy = False
            PLAYER.phase = "setup"


def prepare_worker(mcap, yaml_path, map_path, assignments):
    try:
        PLAYER.set_message("正在读取 RTK")
        poses = load_yaml_poses(yaml_path)
        chosen = []
        for index, item in enumerate(assignments):
            key = item["key"]
            if key not in poses:
                raise SystemExit(f"外参里没有 {key}")
            chosen.append(
                {
                    "topic": item["topic"],
                    "key": key,
                    "pose": pose_matrix(poses[key]),
                    "color": COLORS[index % len(COLORS)],
                }
            )
        fixes, quats = read_rtk(mcap)
        mode, cloud_path, frame_text = map_mode(map_path)
        rtk_xyz = project_rtk(fixes, mode)
        bounds = (
            float(rtk_xyz[:, 0].min() - MARGIN),
            float(rtk_xyz[:, 0].max() + MARGIN),
            float(rtk_xyz[:, 1].min() - MARGIN),
            float(rtk_xyz[:, 1].max() + MARGIN),
        )
        cloud = None
        if cloud_path is not None:
            cloud = load_map(cloud_path, bounds, PLAYER.set_message)
        origin = np.array([float(rtk_xyz[:, 0].mean()), float(rtk_xyz[:, 1].mean()), 0.0])
        rtk_xyz = rtk_xyz - origin
        if cloud is not None:
            cloud = cloud - origin
        rtk = {"time": fixes[:, 0], "xyz": rtk_xyz, "pose_time": quats[:, 0], "quat": quats[:, 1:]}
        step = max(1, len(rtk_xyz) // 1500)
        line = np.ascontiguousarray(rtk_xyz[::step], dtype=np.float32)
        by_topic = {item["topic"]: item for item in chosen}
        reader = open_reader(mcap, list(by_topic))
        frames = []
        current = None
        used = {item["topic"]: 0 for item in chosen}
        PLAYER.set_message("正在按 RTK 变换点云")
        while reader.has_next():
            topic, raw, _stamp = reader.read_next()
            sensor = by_topic.get(topic)
            if sensor is None:
                continue
            msg = deserialize_message(raw, PointCloud2)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            body = body_pose(rtk, stamp)
            if body is None:
                continue
            points = keep_range(xyz_of(msg))
            if len(points) == 0:
                continue
            world = body @ sensor["pose"]
            moved = (world[:3, :3] @ points.T).T + world[:3, 3]
            moved = thin(moved, POINT_LIMIT).astype(np.float32)
            if current is None or topic in current["clouds"] or stamp - current["stamp"] > BIN:
                if current is not None:
                    frames.append(current)
                current = {"stamp": float(stamp), "clouds": {}}
            current["clouds"][topic] = moved
            used[topic] += 1
            if sum(used.values()) % 40 == 0:
                PLAYER.set_message(f"已变换 {sum(used.values())} 帧")
        if current is not None and current["clouds"]:
            frames.append(current)
        if not frames:
            raise SystemExit("没有可播放的点云")
        shown = [
            {"topic": item["topic"], "key": item["key"], "color": "#{:02x}{:02x}{:02x}".format(*item["color"])}
            for item in chosen
        ]
        with PLAYER.lock:
            PLAYER.frames = frames
            PLAYER.map_cloud = None if cloud is None else np.ascontiguousarray(cloud, dtype=np.float32)
            PLAYER.rtk_line = line
            PLAYER.assignments = shown
            PLAYER.frame_text = frame_text
            PLAYER.index = 0
            PLAYER.playing = True
            PLAYER.phase = "play"
            PLAYER.error = ""
            PLAYER.mcap = str(mcap)
            PLAYER.yaml = str(yaml_path)
            PLAYER.map = "" if map_path is None else str(map_path)
            PLAYER.message = f"共 {len(frames)} 帧。白点是地图，橙线是 RTK。基准是 RTK 后轴中心。"
            PLAYER.busy = False
    except (Exception, SystemExit) as exc:
        with PLAYER.lock:
            PLAYER.error = str(exc)
            PLAYER.message = str(exc)
            PLAYER.busy = False
            PLAYER.playing = False


def clock():
    while not PLAYER.stop:
        with PLAYER.lock:
            playing = PLAYER.playing
            frames = PLAYER.frames
            index = PLAYER.index
            rate = PLAYER.rate
        if not playing or len(frames) < 2:
            time.sleep(0.05)
            continue
        current = frames[index]["stamp"]
        nxt = frames[(index + 1) % len(frames)]["stamp"]
        delta = nxt - current
        if delta <= 0.0 or index + 1 >= len(frames):
            delta = 0.1
        time.sleep(max(0.02, min(0.5, delta / max(rate, 0.1))))
        with PLAYER.lock:
            if PLAYER.playing and PLAYER.frames:
                PLAYER.index = (PLAYER.index + 1) % len(PLAYER.frames)


def view_bytes():
    with PLAYER.lock:
        layers = []
        if PLAYER.map_cloud is not None and len(PLAYER.map_cloud):
            layers.append((PLAYER.map_cloud, (210, 210, 210), 2, 0))
        if PLAYER.rtk_line is not None and len(PLAYER.rtk_line):
            layers.append((PLAYER.rtk_line, (255, 140, 0), 4, 1))
        frame = None if not PLAYER.frames else PLAYER.frames[PLAYER.index]
        colors = {item["topic"]: tuple(int(item["color"][index : index + 2], 16) for index in (1, 3, 5)) for item in PLAYER.assignments}
        if frame is not None:
            for topic, points in frame["clouds"].items():
                layers.append((points, colors.get(topic, (255, 255, 255)), 3, 0))
    if not layers:
        layers.append((np.zeros((0, 3), dtype=np.float32), (0, 0, 0), 1, 0))
    return pack_layers(layers)


def create_app():
    root = Path(__file__).resolve().parent
    app = Flask(__name__, template_folder=str(root / "templates"), static_folder=str(root / "static"))
    app.config["TEMPLATES_AUTO_RELOAD"] = True
    app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0

    @app.get("/")
    def index():
        return render_template("rtk_view.html")

    @app.get("/api/state")
    def state():
        return jsonify(PLAYER.snapshot())

    @app.get("/api/view")
    def view():
        return Response(view_bytes(), mimetype="application/octet-stream")

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
                "entries": [{"name": f"{root.name}/", "path": str(root), "kind": "dir"} for root in roots],
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
        suffixes = {".mcap"} if kind == "mcap" else {".yaml", ".yml"} if kind == "yaml" else {".pcd"}
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
            if child.suffix.lower() not in suffixes:
                continue
            entries.append({"name": child.name, "path": str(child), "kind": "file"})
        parent = "" if any(path == root for root in roots) else str(path.parent)
        return jsonify({
            "path": str(path),
            "parent": parent,
            "map_dir": (path / "pointcloud_map.pcd").is_file() and (path / "map_projector_info.yaml").is_file(),
            "entries": entries,
        })

    @app.post("/api/inspect")
    def inspect():
        payload = request.get_json(force=True)
        try:
            mcap = resolve_file(payload.get("mcap", ""), {".mcap"})
            yaml_path = resolve_file(payload.get("yaml", ""), {".yaml", ".yml"})
        except SystemExit as exc:
            return jsonify({"error": str(exc)}), 400
        with PLAYER.lock:
            if PLAYER.busy:
                return jsonify({"error": "正在处理，请稍候"}), 409
            PLAYER.busy = True
            PLAYER.playing = False
            PLAYER.error = ""
            PLAYER.phase = "inspect"
        threading.Thread(target=inspect_worker, args=(mcap, yaml_path), daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/api/play")
    def play():
        payload = request.get_json(force=True)
        assignments = []
        for item in payload.get("assignments", []):
            topic = str(item.get("topic", "")).strip()
            key = str(item.get("key", "")).strip()
            if topic and key:
                assignments.append({"topic": topic, "key": key})
        if not assignments:
            return jsonify({"error": "至少为一颗雷达选择外参"}), 400
        try:
            mcap = resolve_file(payload.get("mcap", ""), {".mcap"})
            yaml_path = resolve_file(payload.get("yaml", ""), {".yaml", ".yml"})
            map_text = str(payload.get("map", "") or "").strip()
            map_path = None
            if map_text:
                map_path = Path(map_text)
                if not map_path.exists() or not path_allowed(map_path):
                    raise SystemExit(f"地图路径不可用: {map_text}")
                map_path = map_path.resolve()
                if map_path.is_file() and map_path.suffix.lower() != ".pcd":
                    raise SystemExit("地图需要是 pcd，或含 pointcloud_map.pcd 的目录")
        except SystemExit as exc:
            return jsonify({"error": str(exc)}), 400
        with PLAYER.lock:
            if PLAYER.busy:
                return jsonify({"error": "正在处理，请稍候"}), 409
            known = {item["name"] for item in PLAYER.topics}
            if any(item["topic"] not in known for item in assignments):
                return jsonify({"error": "话题不在这个数据包里"}), 400
            PLAYER.busy = True
            PLAYER.playing = False
            PLAYER.error = ""
            PLAYER.phase = "prepare"
            PLAYER.frames = []
            PLAYER.message = "正在准备播放"
        threading.Thread(
            target=prepare_worker,
            args=(mcap, yaml_path, map_path, assignments),
            daemon=True,
        ).start()
        return jsonify({"ok": True})

    @app.post("/api/transport")
    def transport():
        payload = request.get_json(force=True)
        with PLAYER.lock:
            if not PLAYER.frames:
                return jsonify({"error": "还没有可播放的数据"}), 409
            if "rate" in payload:
                PLAYER.rate = float(np.clip(float(payload["rate"]), 0.25, 8.0))
            if "index" in payload:
                PLAYER.index = int(np.clip(int(payload["index"]), 0, len(PLAYER.frames) - 1))
            if "playing" in payload:
                PLAYER.playing = bool(payload["playing"])
        return jsonify({"ok": True})

    return app


def main():
    threading.Thread(target=clock, daemon=True).start()
    app = create_app()
    app.run(host="0.0.0.0", port=PORT, threaded=True)


if __name__ == "__main__":
    main()
