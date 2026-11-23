#!/usr/bin/env python3
"""用补盲雷达第一帧对原点地图做精配准，优化相对后轴的外参。"""

from pathlib import Path

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from kiss_icp.tools.point_cloud2 import read_point_cloud
from pyproj import Transformer
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import NavSatFix, PointCloud2
import small_gicp

MAP_PATH = Path("/map/20261113150315_0_voxel010_origin.pcd")
MCAP = Path("/bag/lidar.mcap")
TRAJ_PATH = Path("/traj/gnss_trajectory_origin.pcd")
OUTPUT = Path("/calib/sensors_calibration_manual.yaml")
ORIGIN = np.array([193393.401, 2503059.966, 129.917], dtype=np.float64)
VOXEL = 0.15
MIN_RANGE = 0.4
MAX_RANGE = 30.0
MATCH_RADIUS = 40.0

LIDARS = {
    "left": {
        "key": "suteng_lidar_left_front_base_link",
        "topic": "/sutpc/sensor_driver/lidar/blind/left_front/lidar_points",
        "prior": np.array([3.8677, 1.343, 0.851, 3.1276300196, -0.0122173048, -2.1764255772]),
    },
    "right": {
        "key": "suteng_lidar_right_end_base_link",
        "topic": "/sutpc/sensor_driver/lidar/blind/right_end/lidar_points",
        "prior": np.array([-0.4169, -0.9643, 0.64, 3.1363566658, 0.0017453293, 0.9177115740]),
    },
}


def wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    return transform


def rotation_to_rpy(rotation):
    pitch = float(np.arctan2(-rotation[2, 0], np.hypot(rotation[2, 1], rotation[2, 2])))
    roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return roll, pitch, yaw


def rpy_near(rotation, reference):
    roll, pitch, yaw = rotation_to_rpy(rotation)
    return (
        reference[0] + wrap(roll - reference[0]),
        reference[1] + wrap(pitch - reference[1]),
        reference[2] + wrap(yaw - reference[2]),
    )


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


def pose_from_vector(values):
    transform = rpy_matrix(values[3], values[4], values[5])
    transform[:3, 3] = values[:3]
    return transform


def read_pcd_xyz(path):
    with path.open("rb") as handle:
        while True:
            line = handle.readline()
            if line.startswith(b"DATA"):
                break
        raw = handle.read()
    return np.frombuffer(raw, dtype=np.float64).reshape(-1, 3).copy()


def voxel_down(points, voxel):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _unique, index = np.unique(keys, axis=0, return_index=True)
    return np.asarray(points[np.sort(index)], dtype=np.float64)


def interp(times, values, stamp):
    if stamp < times[0] - 1.0 or stamp > times[-1] + 1.0:
        print(f"interp miss stamp={stamp:.3f} range={times[0]:.3f}..{times[-1]:.3f}", flush=True)
        return None
    if stamp <= times[0]:
        return values[0]
    if stamp >= times[-1]:
        return values[-1]
    index = int(np.searchsorted(times, stamp))
    index = min(max(index, 1), len(times) - 1)
    span = times[index] - times[index - 1]
    ratio = 0.0 if span <= 0.0 else (stamp - times[index - 1]) / span
    return (1.0 - ratio) * values[index - 1] + ratio * values[index]


def read_first_frames():
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(MCAP), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = ["/sensing/gnss/ublox/nav_sat_fix", "/sensing/gnss/pose", *[item["topic"] for item in LIDARS.values()]]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    fixes, poses = [], []
    scans = {}
    while reader.has_next():
        topic, raw, _stamp = reader.read_next()
        if topic.endswith("nav_sat_fix"):
            msg = deserialize_message(raw, NavSatFix)
            fixes.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.latitude, msg.longitude, msg.altitude))
        elif topic.endswith("/pose"):
            msg = deserialize_message(raw, PoseStamped)
            q = msg.pose.orientation
            poses.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, q.x, q.y, q.z, q.w))
        elif len(scans) < len(LIDARS):
            for name, item in LIDARS.items():
                if topic == item["topic"] and name not in scans:
                    msg = deserialize_message(raw, PointCloud2)
                    points, _times = read_point_cloud(msg)
                    xyz = np.asarray(points, dtype=np.float64)[:, :3]
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    scans[name] = (stamp, xyz)
        if (
            len(scans) == len(LIDARS)
            and fixes
            and poses
            and fixes[-1][0] > max(item[0] for item in scans.values()) + 0.2
            and poses[-1][0] > max(item[0] for item in scans.values()) + 0.2
        ):
            break
    return np.asarray(fixes, dtype=np.float64), np.asarray(poses, dtype=np.float64), scans


def residual(tree, scan_map):
    distances, _index = tree.query(scan_map, k=1, workers=-1)
    return {
        "median": float(np.median(distances)),
        "p90": float(np.percentile(distances, 90)),
        "inlier_0_3": float(np.mean(distances < 0.3)),
        "inlier_1": float(np.mean(distances < 1.0)),
    }


def align(local, scan, guess):
    current = guess
    last = None
    for distance, iterations in ((1.5, 40), (0.6, 30), (0.25, 25)):
        last = small_gicp.align(
            local,
            scan,
            current,
            registration_type="GICP",
            voxel_resolution=VOXEL,
            downsampling_resolution=VOXEL,
            max_correspondence_distance=distance,
            num_threads=8,
            max_iterations=iterations,
            verbose=False,
        )
        current = np.asarray(last.T_target_source, dtype=np.float64)
    return current, bool(last.converged), int(last.iterations)


def main():
    print("loading map", flush=True)
    cloud = read_pcd_xyz(MAP_PATH)
    traj = read_pcd_xyz(TRAJ_PATH)
    print(f"map={len(cloud)} traj_start={traj[0].tolist()}", flush=True)
    fixes, poses, scans = read_first_frames()
    project = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)
    east, north = project.transform(fixes[:, 2], fixes[:, 1])
    fix_xyz = np.column_stack((east, north, fixes[:, 3])) - ORIGIN
    order = np.argsort(poses[:, 0])
    poses = poses[order]
    for index in range(1, len(poses)):
        if np.dot(poses[index, 1:], poses[index - 1, 1:]) < 0.0:
            poses[index, 1:] *= -1.0
    print(
        f"fix0={fix_xyz[0].tolist()} traj_delta={np.linalg.norm(fix_xyz[0] - traj[0]):.3f} m",
        flush=True,
    )
    results = {}
    for name, item in LIDARS.items():
        stamp, xyz = scans[name]
        finite = np.isfinite(xyz).all(axis=1)
        distance = np.linalg.norm(xyz, axis=1)
        keep = finite & (distance >= MIN_RANGE) & (distance <= MAX_RANGE)
        scan = voxel_down(xyz[keep], VOXEL)
        print(
            f"  stamp={stamp:.3f} fix={fixes[0, 0]:.3f}..{fixes[-1, 0]:.3f} "
            f"pose={poses[0, 0]:.3f}..{poses[-1, 0]:.3f}",
            flush=True,
        )
        position = interp(fixes[:, 0], fix_xyz, stamp)
        quat = interp(poses[:, 0], poses[:, 1:], stamp)
        if position is None or quat is None:
            raise SystemExit(f"{name}: 第一帧没有对应的 GNSS 位姿")
        quat = quat / np.linalg.norm(quat)
        body = np.eye(4)
        body[:3, :3] = quat_to_rotation(quat)
        body[:3, 3] = position
        prior = pose_from_vector(item["prior"])
        guess = body @ prior
        center = guess[:2, 3]
        print(f"  guess_t=({guess[0, 3]:.3f}, {guess[1, 3]:.3f}, {guess[2, 3]:.3f})", flush=True)
        delta = cloud[:, :2] - center
        local = cloud[np.einsum("ij,ij->i", delta, delta) <= MATCH_RADIUS ** 2]
        print(
            f"\n===== {item['key']} =====\n"
            f"  raw={len(xyz)} kept={int(keep.sum())} voxel={len(scan)} "
            f"range_p50={np.median(distance[finite]):.2f} map_local={len(local)}",
            flush=True,
        )
        print(f"  body_t=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})", flush=True)
        tree = cKDTree(local)
        before = residual(tree, (guess[:3, :3] @ scan.T).T + guess[:3, 3])
        refined, converged, iterations = align(local, scan, guess)
        shift = refined[:3, 3] - guess[:3, 3]
        after = residual(tree, (refined[:3, :3] @ scan.T).T + refined[:3, 3])
        extrinsic = np.linalg.inv(body) @ refined
        roll, pitch, yaw = rpy_near(extrinsic[:3, :3], item["prior"][3:])
        result = {
            "x": float(extrinsic[0, 3]),
            "y": float(extrinsic[1, 3]),
            "z": float(extrinsic[2, 3]),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "converged": converged,
            "iterations": iterations,
            "shift": float(np.linalg.norm(shift[:2])),
            "before": before,
            "after": after,
        }
        results[name] = result
        print(
            f"  converged={converged} iters={iterations} shift_xy={result['shift']:.3f} m\n"
            f"  before median={before['median']:.3f} p90={before['p90']:.3f} "
            f"inlier<0.3={before['inlier_0_3']:.3f} <1={before['inlier_1']:.3f}\n"
            f"  after  median={after['median']:.3f} p90={after['p90']:.3f} "
            f"inlier<0.3={after['inlier_0_3']:.3f} <1={after['inlier_1']:.3f}\n"
            f"  t=({result['x']:.4f}, {result['y']:.4f}, {result['z']:.4f}) "
            f"rpy_deg=({np.degrees(roll):.3f}, {np.degrees(pitch):.3f}, {np.degrees(yaw):.3f})",
            flush=True,
        )
    text = OUTPUT.read_text(encoding="utf-8")
    for name, item in LIDARS.items():
        result = results[name]
        block = "\n".join(
            [
                f"{item['key']}:",
                f"  x: {result['x']:.10f}",
                f"  y: {result['y']:.10f}",
                f"  z: {result['z']:.10f}",
                f"  roll: {result['roll']:.10f}",
                f"  pitch: {result['pitch']:.10f}",
                f"  yaw: {result['yaw']:.10f}",
            ]
        )
        start = text.index(f"{item['key']}:")
        end = start
        lines = text[start:].splitlines()
        consumed = 0
        for line in lines:
            consumed += 1
            if consumed > 1 and line and not line.startswith(" "):
                consumed -= 1
                break
        end = start + sum(len(line) + 1 for line in lines[:consumed])
        text = text[:start] + block + "\n" + text[end:]
    note = (
        "# 补盲雷达由第一帧对 20261113150315_0_voxel010_origin.pcd 精配准。\n"
        "# 车体位置用 gnss_trajectory_origin 同一套起点坐标。\n"
    )
    if not text.startswith("# 补盲雷达由第一帧"):
        text = note + text
    OUTPUT.write_text(text, encoding="utf-8")
    print(f"wrote {OUTPUT}", flush=True)


if __name__ == "__main__":
    main()
