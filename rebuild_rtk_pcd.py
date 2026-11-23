#!/usr/bin/env python3
"""用外参和 RTK 轨迹把四路激光雷达重建成一张 PCD。

父坐标系是后轴中心。RTK 位置取 NavSatFix，姿态取 /sensing/gnss/pose，
投影到 UTM 32650，不再加天线杆臂，也不把高度改成 1.0。
点云坐标是绝对 UTM，单位米。
"""

import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from pyproj import Transformer
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix, PointCloud2

YAML = Path("/calib/sensors_calibration.yaml")
MCAP = Path("/bag/lidar_gnss_20261116_160451_0.mcap")
OUTPUT = Path("/out/lidar_gnss_20261116_160451_rtk_4lidar.pcd")
VOXEL = 0.10

LIDARS = (
    {
        "key": "hesai_lidar_left_front_base_link",
        "topic": "/sutpc/sensor_driver/lidar/long/left_front/lidar_points",
        "min_range": 2.0,
        "max_range": 80.0,
    },
    {
        "key": "hesai_lidar_right_end_base_link",
        "topic": "/sutpc/sensor_driver/lidar/long/right_end/lidar_points",
        "min_range": 2.0,
        "max_range": 80.0,
    },
    {
        "key": "suteng_lidar_left_front_base_link",
        "topic": "/sutpc/sensor_driver/lidar/blind/left_front/lidar_points",
        "min_range": 0.5,
        "max_range": 30.0,
    },
    {
        "key": "suteng_lidar_right_end_base_link",
        "topic": "/sutpc/sensor_driver/lidar/blind/right_end/lidar_points",
        "min_range": 0.5,
        "max_range": 30.0,
    },
)
FIX = "/sensing/gnss/ublox/nav_sat_fix"
POSE = "/sensing/gnss/pose"


def load_extrinsics(path):
    poses = {}
    name = None
    current = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if line[:1] in (" ", "\t"):
            key, value = line.strip().split(":", 1)
            current[key.strip()] = float(value.strip())
            continue
        if name is not None:
            poses[name] = current
        name = line.split(":", 1)[0].strip()
        current = {}
    if name is not None:
        poses[name] = current
    return poses


def rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
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


def pose_matrix(values):
    transform = rpy_matrix(values["roll"], values["pitch"], values["yaw"])
    transform[:3, 3] = (values["x"], values["y"], values["z"])
    return transform


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


def xyz_of(msg):
    fields = {field.name: field for field in msg.fields}
    count = int(msg.width) * int(msg.height)
    step = int(msg.point_step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    count = min(count, raw.size // step if step else 0)
    if count == 0:
        return np.empty((0, 3), dtype=np.float64)
    if all(name in fields and fields[name].datatype == 7 for name in ("x", "y", "z")):
        packed = raw[: count * step].reshape(count, step)
        columns = []
        for name in ("x", "y", "z"):
            offset = int(fields[name].offset)
            columns.append(np.ascontiguousarray(packed[:, offset : offset + 4]).view(np.float32).reshape(-1))
        return np.column_stack(columns).astype(np.float64)
    from kiss_icp.tools.point_cloud2 import read_point_cloud

    points, _times = read_point_cloud(msg)
    return np.asarray(points, dtype=np.float64)[:, :3]


def keep_range(points, min_range, max_range):
    finite = np.isfinite(points).all(axis=1)
    radius = np.linalg.norm(points[:, :2], axis=1)
    return points[finite & (radius >= min_range) & (radius <= max_range)]


def voxel_down(points, voxel):
    if len(points) == 0:
        return points.reshape(0, 3)
    keys = np.floor(points / voxel).astype(np.int64)
    _unique, index = np.unique(keys, axis=0, return_index=True)
    return points[index]


def read_rtk(mcap):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(mcap), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[FIX, POSE]))
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
    project = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)
    east, north = project.transform(fixes[:, 2], fixes[:, 1])
    xyz = np.column_stack((east, north, fixes[:, 3]))
    print(
        f"rtk={len(fixes)} pose={len(quats)} "
        f"start=({xyz[0, 0]:.3f}, {xyz[0, 1]:.3f}, {xyz[0, 2]:.3f})",
        flush=True,
    )
    return {"time": fixes[:, 0], "xyz": xyz, "pose_time": quats[:, 0], "quat": quats[:, 1:]}


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    count = int(len(points))
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 8 8 8\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    with path.open("wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(np.ascontiguousarray(points, dtype=np.float64).tobytes())


def main():
    extrinsics = load_extrinsics(YAML)
    sensors = []
    for item in LIDARS:
        if item["key"] not in extrinsics:
            raise SystemExit(f"外参里没有 {item['key']}")
        pose = pose_matrix(extrinsics[item["key"]])
        sensors.append({**item, "pose": pose})
        t = pose[:3, 3]
        print(f"{item['key']} t=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})", flush=True)

    rtk = read_rtk(MCAP)
    by_topic = {item["topic"]: item for item in sensors}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(MCAP), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[item["topic"] for item in sensors]))
    chunks = []
    held = 0
    used = {item["topic"]: 0 for item in sensors}
    skipped = {item["topic"]: 0 for item in sensors}

    def absorb(block):
        nonlocal held
        if len(block) == 0:
            return
        reduced = voxel_down(block, VOXEL)
        chunks.append(reduced)
        held += len(reduced)
        if held < 1_500_000:
            return
        merged = voxel_down(np.vstack(chunks), VOXEL)
        chunks.clear()
        chunks.append(merged)
        held = len(merged)
        print(f"voxel {held}", flush=True)

    while reader.has_next():
        topic, raw, _stamp = reader.read_next()
        sensor = by_topic.get(topic)
        if sensor is None:
            continue
        msg = deserialize_message(raw, PointCloud2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        body = body_pose(rtk, stamp)
        if body is None:
            skipped[topic] += 1
            continue
        points = keep_range(xyz_of(msg), sensor["min_range"], sensor["max_range"])
        if len(points) == 0:
            skipped[topic] += 1
            continue
        world = body @ sensor["pose"]
        moved = (world[:3, :3] @ points.T).T + world[:3, 3]
        absorb(moved)
        used[topic] += 1
        if used[topic] % 50 == 0:
            print(f"{sensor['key']} frames={used[topic]}", flush=True)

    if not chunks:
        raise SystemExit("没有可写入的点")
    cloud = voxel_down(np.vstack(chunks), VOXEL)
    write_pcd(OUTPUT, cloud)
    print(f"points={len(cloud)} voxel={VOXEL:.2f} file={OUTPUT}", flush=True)
    for item in sensors:
        print(f"  {item['key']} used={used[item['topic']]} skipped={skipped[item['topic']]}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
