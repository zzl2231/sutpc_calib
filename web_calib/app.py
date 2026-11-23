#!/usr/bin/env python3
"""网页手动标定：订阅回放点云，按当前外参变到 base_link，Flask 改外参并暂停播放。"""

import importlib
import os
import threading
from pathlib import Path

import numpy as np
import rclpy
from flask import Flask, jsonify, render_template, request
from builtin_interfaces.msg import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

CALIB_YAML = Path(os.environ.get("CALIB_YAML", "/calib/sensors_calibration_baseline.yaml"))
SAVE_YAML = Path(os.environ.get("CALIB_SAVE", "/calib/sensors_calibration_manual.yaml"))
MIN_RANGE = 0.05

LIDARS = (
    {
        "id": "long_left_front",
        "key": "hesai_lidar_left_front_base_link",
        "topic": "/sutpc/sensor_driver/lidar/long/left_front/lidar_points",
        "output": "/calib/lidar/long/left_front/points",
        "label": "远距左前",
        "color": "#ff3c3c",
    },
    {
        "id": "long_right_end",
        "key": "hesai_lidar_right_end_base_link",
        "topic": "/sutpc/sensor_driver/lidar/long/right_end/lidar_points",
        "output": "/calib/lidar/long/right_end/points",
        "label": "远距右后",
        "color": "#3cc850",
    },
    {
        "id": "blind_left_front",
        "key": "suteng_lidar_left_front_base_link",
        "topic": "/sutpc/sensor_driver/lidar/blind/left_front/lidar_points",
        "output": "/calib/lidar/blind/left_front/points",
        "label": "补盲左前",
        "color": "#468cff",
        "note": "初始值未标定",
    },
    {
        "id": "blind_right_end",
        "key": "suteng_lidar_right_end_base_link",
        "topic": "/sutpc/sensor_driver/lidar/blind/right_end/lidar_points",
        "output": "/calib/lidar/blind/right_end/points",
        "label": "补盲右后",
        "color": "#ffc828",
        "note": "初始值未标定",
    },
)
FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def zero_poses():
    return {lidar["id"]: {name: 0.0 for name in FIELDS} for lidar in LIDARS}


def load_extrinsics(path):
    if not path or not Path(path).is_file():
        return zero_poses()
    values = {}
    current = None
    for raw in Path(path).read_text().splitlines():
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
        values[current][key.strip()] = float(value.strip())
    poses = {}
    for lidar in LIDARS:
        block = values.get(lidar["key"])
        if block is None or any(name not in block for name in FIELDS):
            raise SystemExit(f"{path} 缺少 {lidar['key']}")
        poses[lidar["id"]] = {name: float(block[name]) for name in FIELDS}
    return poses


def rpy_matrix(roll, pitch, yaw):
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    rotation_y = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rotation_z = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rotation_z @ rotation_y @ rotation_x


def unpack_cloud(msg):
    step = msg.point_step
    count = msg.width * msg.height
    if count == 0 or step < 16 or len(msg.data) < count * step:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=count * step).reshape(count, step)
    xyz = np.ascontiguousarray(raw[:, 0:12]).view(np.float32).reshape(count, 3)
    intensity = np.ascontiguousarray(raw[:, 12:16]).view(np.float32).reshape(count)
    finite = np.isfinite(xyz).all(axis=1)
    keep = finite & (np.einsum("ij,ij->i", xyz, xyz) > MIN_RANGE ** 2)
    if not np.any(keep):
        return None
    return xyz[keep], intensity[keep]


def pack_cloud(xyz, intensity, stamp):
    count = int(xyz.shape[0])
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
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
    packed[:, :3] = xyz
    packed[:, 3] = intensity
    msg.data = packed.tobytes()
    return msg


def dump_yaml(poses):
    lines = [
        "# 网页手动标定结果。单位 m / rad，RPY 为固定轴 XYZ。",
        "# 父坐标系 base_link。未提供初始文件时四路从 0 开始。",
        "# X+ 前，Y+ 左，Z+ 上。",
        "",
        "# ---- 远距雷达----",
    ]
    order = [item["id"] for item in LIDARS]
    labels = {item["id"]: item for item in LIDARS}
    for index, lidar_id in enumerate(order):
        if lidar_id.startswith("blind") and not order[index - 1].startswith("blind"):
            lines.append("")
            lines.append("# ---- 补盲雷达----")
        meta = labels[lidar_id]
        if meta.get("note"):
            lines.append(f"# {meta['note']}")
        pose = poses[lidar_id]
        lines.append(f"{meta['key']}:")
        for name in FIELDS:
            lines.append(f"  {name}: {pose[name]:.10f}")
    lines.append("")
    return "\n".join(lines)


class Tuner(Node):
    def __init__(self, initial):
        super().__init__("lidar_extrinsic_tuner")
        self.lock = threading.Lock()
        self.poses = {key: dict(value) for key, value in initial.items()}
        self.initial = {key: dict(value) for key, value in initial.items()}
        self.stats = {item["id"]: {"count": 0, "points": 0} for item in LIDARS}
        self.paused = None
        self._toggle_client = None
        self._paused_client = None
        self._toggle_type = None
        self._paused_type = None
        self._paused_future = None
        self._toggle_future = None
        self.pause_request = threading.Event()
        self.pause_done = threading.Event()
        self.busy = {item["id"]: False for item in LIDARS}
        self.latest = {item["id"]: None for item in LIDARS}
        self.dirty = set()
        self._cloud_group = ReentrantCallbackGroup()
        self._subs = []
        for lidar in LIDARS:
            subscription = self.create_subscription(
                PointCloud2,
                lidar["topic"],
                self._callback(lidar["id"]),
                QOS,
                callback_group=self._cloud_group,
            )
            self._subs.append(subscription)
            publisher = self.create_publisher(PointCloud2, lidar["output"], QOS)
            setattr(self, f"pub_{lidar['id']}", publisher)
        timer_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.2, self.poll_pause, callback_group=timer_group)
        self.create_timer(0.05, self.republish_dirty, callback_group=timer_group)
        if CALIB_YAML.is_file():
            self.get_logger().info(f"初始外参 {CALIB_YAML}")
        else:
            self.get_logger().info("未提供初始外参，四路从 0 开始")

    def _callback(self, lidar_id):
        def callback(msg):
            if self.busy[lidar_id]:
                return
            self.busy[lidar_id] = True
            try:
                unpacked = unpack_cloud(msg)
                if unpacked is None:
                    return
                xyz, intensity = unpacked
                stamp = Time(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec)
                with self.lock:
                    pose = dict(self.poses[lidar_id])
                    self.latest[lidar_id] = (xyz, intensity, stamp)
                self._publish(lidar_id, xyz, intensity, stamp, pose)
                first = self.stats[lidar_id]["count"] == 0
                self.stats[lidar_id]["count"] += 1
                if first:
                    self.get_logger().info(f"{lidar_id} {self.stats[lidar_id]['points']} 点")
            finally:
                self.busy[lidar_id] = False

        return callback

    def _publish(self, lidar_id, xyz, intensity, stamp, pose):
        rotation = rpy_matrix(pose["roll"], pose["pitch"], pose["yaw"])
        translation = np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float64)
        transformed = xyz.astype(np.float64) @ rotation.T + translation
        getattr(self, f"pub_{lidar_id}").publish(pack_cloud(transformed, intensity, stamp))
        self.stats[lidar_id]["points"] = int(transformed.shape[0])

    def republish_dirty(self):
        with self.lock:
            pending = list(self.dirty)
            self.dirty.clear()
            jobs = []
            for lidar_id in pending:
                cached = self.latest[lidar_id]
                if cached is None:
                    continue
                jobs.append((lidar_id, cached, dict(self.poses[lidar_id])))
        for lidar_id, (xyz, intensity, stamp), pose in jobs:
            self._publish(lidar_id, xyz, intensity, stamp, pose)

    def snapshot(self):
        with self.lock:
            poses = {key: dict(value) for key, value in self.poses.items()}
        return {
            "poses": poses,
            "stats": {key: dict(value) for key, value in self.stats.items()},
            "paused": self.paused,
            "save_path": str(SAVE_YAML),
        }

    def update(self, lidar_id, pose):
        cleaned = {name: float(pose[name]) for name in FIELDS}
        with self.lock:
            self.poses[lidar_id] = cleaned
            self.dirty.add(lidar_id)
        return cleaned

    def reset(self):
        with self.lock:
            self.poses = {key: dict(value) for key, value in self.initial.items()}
            poses = {key: dict(value) for key, value in self.poses.items()}
            self.dirty.update(poses)
        return poses

    def save(self):
        with self.lock:
            text = dump_yaml(self.poses)
        SAVE_YAML.write_text(text)
        return str(SAVE_YAML)

    def _import_srv(self, type_name):
        package, kind, name = type_name.split("/")
        if kind != "srv":
            raise RuntimeError(type_name)
        return getattr(importlib.import_module(f"{package}.srv"), name)

    def ensure_player(self):
        if self._toggle_client is not None:
            return self._toggle_client.service_is_ready()
        for name, types in self.get_service_names_and_types():
            if not name.endswith("/toggle_paused") or not types:
                continue
            self._toggle_type = self._import_srv(types[0])
            self._toggle_client = self.create_client(self._toggle_type, name)
            paused_name = name[: -len("toggle_paused")] + "is_paused"
            for other, other_types in self.get_service_names_and_types():
                if other == paused_name and other_types:
                    self._paused_type = self._import_srv(other_types[0])
                    self._paused_client = self.create_client(self._paused_type, other)
            self.get_logger().info(f"播放控制 {name} ({types[0]})")
            return self._toggle_client.service_is_ready()
        return False

    def _apply_paused(self, response):
        if hasattr(response, "paused"):
            self.paused = bool(response.paused)
        elif self.paused is not None:
            self.paused = not self.paused
        else:
            self.paused = True

    def poll_pause(self):
        try:
            self._poll_pause()
        except Exception as exc:
            self.get_logger().warning(f"播放控制失败: {exc}")
            self._toggle_future = None
            self._paused_future = None
            self.pause_done.set()

    def _poll_pause(self):
        self.ensure_player()
        if self._toggle_future is not None and self._toggle_future.done():
            self._apply_paused(self._toggle_future.result())
            self._toggle_future = None
            self.pause_done.set()
        if (
            self.pause_request.is_set()
            and self._toggle_future is None
            and self._toggle_client is not None
            and self._toggle_client.service_is_ready()
        ):
            self.pause_request.clear()
            self._toggle_future = self._toggle_client.call_async(self._toggle_type.Request())
        if self._paused_future is not None and self._paused_future.done():
            self.paused = bool(getattr(self._paused_future.result(), "paused", False))
            self._paused_future = None
        if (
            self._paused_future is None
            and self._toggle_future is None
            and self._paused_client is not None
            and self._paused_client.service_is_ready()
        ):
            self._paused_future = self._paused_client.call_async(self._paused_type.Request())

    def toggle_pause(self):
        self.pause_done.clear()
        self.pause_request.set()
        if not self.pause_done.wait(2.0):
            self.pause_request.clear()
            return None
        return self.paused


def create_app(node):
    app = Flask(__name__, template_folder=str(Path(__file__).resolve().parent / "templates"),
                static_folder=str(Path(__file__).resolve().parent / "static"))

    @app.get("/")
    def index():
        return render_template("index.html", lidars=LIDARS)

    @app.get("/api/state")
    def state():
        return jsonify(node.snapshot())

    @app.post("/api/extrinsics")
    def extrinsics():
        payload = request.get_json(force=True)
        lidar_id = payload.get("id")
        if lidar_id not in node.poses:
            return jsonify({"error": "未知雷达"}), 400
        try:
            pose = node.update(lidar_id, payload)
        except (KeyError, TypeError, ValueError):
            return jsonify({"error": "外参数值无效"}), 400
        return jsonify({"id": lidar_id, "pose": pose})

    @app.post("/api/reset")
    def reset():
        return jsonify({"poses": node.reset()})

    @app.post("/api/save")
    def save():
        return jsonify({"path": node.save()})

    @app.post("/api/toggle_pause")
    def toggle_pause():
        paused = node.toggle_pause()
        if paused is None:
            return jsonify({"error": "播放器还没就绪"}), 503
        return jsonify({"paused": paused})

    return app


def main():
    initial = load_extrinsics(CALIB_YAML)
    rclpy.init()
    node = Tuner(initial)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    app = create_app(node)
    try:
        app.run(host="0.0.0.0", port=8765, threaded=True, use_reloader=False)
    finally:
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
