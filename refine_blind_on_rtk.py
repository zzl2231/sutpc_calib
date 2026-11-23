#!/usr/bin/env python3
"""用 RTK 长距外参摆好第一帧，再把两颗补盲雷达的第一帧配到这个参考上。"""

from pathlib import Path

import numpy as np
import rosbag2_py
import small_gicp
from geometry_msgs.msg import PoseStamped
from kiss_icp.tools.point_cloud2 import read_point_cloud
from pyproj import Transformer
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import NavSatFix, PointCloud2

MAP_PATH = Path("/map/20261113150315_0_voxel010_origin.pcd")
MCAP = Path("/bag/lidar.mcap")
RTK_YAML = Path("/calib/apolong_lidars_rtk.yaml")
OUTPUT = Path("/calib/sensors_calibration_manual.yaml")
ORIGIN = np.array([193393.401, 2503059.966, 129.917], dtype=np.float64)
VOXEL = 0.12
MAP_RADIUS = 35.0

LONG = {
    "left": {
        "rtk_key": "hesai_lidar_long_left_front_rtk",
        "key": "hesai_lidar_left_front_base_link",
        "topic": "/sutpc/sensor_driver/lidar/long/left_front/lidar_points",
        "min_range": 1.0,
        "max_range": 40.0,
    },
    "right": {
        "rtk_key": "hesai_lidar_long_right_end_rtk",
        "key": "hesai_lidar_right_end_base_link",
        "topic": "/sutpc/sensor_driver/lidar/long/right_end/lidar_points",
        "min_range": 1.0,
        "max_range": 40.0,
    },
}
BLIND = {
    "left": {
        "key": "suteng_lidar_left_front_base_link",
        "topic": "/sutpc/sensor_driver/lidar/blind/left_front/lidar_points",
        "prior": np.array([3.8720793857, 1.1340532125, 1.0580473917, 3.1174378304, 0.0062778004, -2.1976163799]),
    },
    "right": {
        "key": "suteng_lidar_right_end_base_link",
        "topic": "/sutpc/sensor_driver/lidar/blind/right_end/lidar_points",
        "prior": np.array([-0.4786507665, -0.8919434319, 1.0681163193, 3.1470655492, 0.0066092620, 0.9273787022]),
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


def parse_yaml(path):
    values = {}
    current = None
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
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
    return values


def vector_of(block):
    return np.array([block[name] for name in ("x", "y", "z", "roll", "pitch", "yaw")], dtype=np.float64)


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


def prepare(xyz, min_range, max_range):
    finite = np.isfinite(xyz).all(axis=1)
    distance = np.linalg.norm(xyz, axis=1)
    keep = finite & (distance >= min_range) & (distance <= max_range)
    return voxel_down(xyz[keep], VOXEL), int(keep.sum()), float(np.median(distance[finite])) if np.any(finite) else 0.0


def interp(times, values, stamp):
    if len(times) == 0 or stamp < times[0] - 1.0 or stamp > times[-1] + 1.0:
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


def body_pose(fix_t, fix_xyz, pose_t, pose_q, stamp):
    position = interp(fix_t, fix_xyz, stamp)
    quat = interp(pose_t, pose_q, stamp)
    if position is None or quat is None:
        return None
    quat = quat / np.linalg.norm(quat)
    transform = np.eye(4)
    transform[:3, :3] = quat_to_rotation(quat)
    transform[:3, 3] = position
    return transform


def read_first_frames():
    topics = [
        "/sensing/gnss/ublox/nav_sat_fix",
        "/sensing/gnss/pose",
        *[item["topic"] for item in LONG.values()],
        *[item["topic"] for item in BLIND.values()],
    ]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(MCAP), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    fixes, poses = [], []
    scans = {}
    wanted = {item["topic"]: name for name, item in {**LONG, **BLIND}.items()}
    # LONG and BLIND both use left/right. Keep them distinct.
    wanted = {}
    for name, item in LONG.items():
        wanted[item["topic"]] = ("long", name)
    for name, item in BLIND.items():
        wanted[item["topic"]] = ("blind", name)
    while reader.has_next():
        topic, raw, _stamp = reader.read_next()
        if topic.endswith("nav_sat_fix"):
            msg = deserialize_message(raw, NavSatFix)
            fixes.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.latitude, msg.longitude, msg.altitude))
        elif topic.endswith("/pose"):
            msg = deserialize_message(raw, PoseStamped)
            q = msg.pose.orientation
            poses.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, q.x, q.y, q.z, q.w))
        elif topic in wanted and wanted[topic] not in scans:
            msg = deserialize_message(raw, PointCloud2)
            points, _times = read_point_cloud(msg)
            xyz = np.asarray(points, dtype=np.float64)[:, :3]
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            scans[wanted[topic]] = (stamp, xyz)
        if len(scans) == 4 and fixes and poses:
            latest = max(item[0] for item in scans.values())
            if fixes[-1][0] > latest + 0.2 and poses[-1][0] > latest + 0.2:
                break
    return np.asarray(fixes, dtype=np.float64), np.asarray(poses, dtype=np.float64), scans


def apply_transform(transform, points):
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def residual(tree, points):
    distances, _index = tree.query(points, k=1, workers=-1)
    return {
        "median": float(np.median(distances)),
        "p90": float(np.percentile(distances, 90)),
        "inlier_0_3": float(np.mean(distances < 0.3)),
        "inlier_1": float(np.mean(distances < 1.0)),
    }


def align(target, source, guess):
    current = guess
    last = None
    for distance, iterations in ((1.5, 40), (0.6, 30), (0.25, 25)):
        last = small_gicp.align(
            target,
            source,
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


def z_only(target_points, source, prior):
    rotation = prior[:3, :3]
    translation = prior[:3, 3].copy()
    tree = cKDTree(target_points)
    for gate in (0.80, 0.40, 0.25):
        moved = (rotation @ source.T).T + translation
        distances, index = tree.query(moved, k=1, workers=-1)
        keep = distances < gate
        if int(keep.sum()) < 300:
            break
        translation[2] += float(np.median(target_points[index[keep], 2] - moved[keep, 2]))
    moved = (rotation @ source.T).T + translation
    distances, _index = tree.query(moved, k=1, workers=-1)
    return float(translation[2]), float(np.median(distances)), float(np.mean(distances < 0.30))


def block(key, values):
    lines = [f"{key}:"]
    for name in ("x", "y", "z", "roll", "pitch", "yaw"):
        lines.append(f"  {name}: {values[name]:.10f}")
    return "\n".join(lines)


def main():
    rtk = parse_yaml(RTK_YAML)
    long_pose = {}
    for name, item in LONG.items():
        long_pose[name] = pose_from_vector(vector_of(rtk[item["rtk_key"]]))
        pose = vector_of(rtk[item["rtk_key"]])
        print(
            f"rtk {item['key']} t=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})",
            flush=True,
        )

    print("loading map", flush=True)
    cloud = read_pcd_xyz(MAP_PATH)
    fixes, poses, scans = read_first_frames()
    if len(scans) != 4:
        raise SystemExit(f"第一帧没齐: {sorted(scans)}")
    project = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)
    east, north = project.transform(fixes[:, 2], fixes[:, 1])
    fix_xyz = np.column_stack((east, north, fixes[:, 3])) - ORIGIN
    order = np.argsort(poses[:, 0])
    poses = poses[order]
    for index in range(1, len(poses)):
        if np.dot(poses[index, 1:], poses[index - 1, 1:]) < 0.0:
            poses[index, 1:] *= -1.0
    fix_t = fixes[:, 0]
    pose_t = poses[:, 0]
    pose_q = poses[:, 1:]

    long_scan = {}
    for name, item in LONG.items():
        stamp, xyz = scans[("long", name)]
        scan, kept, range_p50 = prepare(xyz, item["min_range"], item["max_range"])
        long_scan[name] = (stamp, scan)
        print(
            f"long {name} stamp={stamp:.3f} raw={len(xyz)} kept={kept} voxel={len(scan)} range_p50={range_p50:.2f}",
            flush=True,
        )

    results = {}
    for name, item in BLIND.items():
        stamp, xyz = scans[("blind", name)]
        scan, kept, range_p50 = prepare(xyz, 0.4, 30.0)
        body = body_pose(fix_t, fix_xyz, pose_t, pose_q, stamp)
        if body is None:
            raise SystemExit(f"{item['key']}: 第一帧没有 GNSS 位姿")
        pieces = []
        for long_name, (long_stamp, long_points) in long_scan.items():
            long_body = body_pose(fix_t, fix_xyz, pose_t, pose_q, long_stamp)
            motion = np.eye(4) if long_body is None else np.linalg.inv(body) @ long_body
            pieces.append(apply_transform(motion @ long_pose[long_name], long_points))
        long_target = voxel_down(np.vstack(pieces), VOXEL)
        prior = pose_from_vector(item["prior"])
        z_long, long_median, long_inlier = z_only(long_target, scan, prior)
        guess = body @ prior
        center = guess[:2, 3]
        delta = cloud[:, :2] - center
        local = cloud[np.einsum("ij,ij->i", delta, delta) <= MAP_RADIUS ** 2]
        map_tree = cKDTree(voxel_down(local, 0.25))
        before = residual(map_tree, apply_transform(guess, scan))
        refined, converged, iterations = align(local, scan, guess)
        extrinsic = np.linalg.inv(body) @ refined
        roll, pitch, yaw = rpy_near(extrinsic[:3, :3], item["prior"][3:])
        result = {
            "x": float(extrinsic[0, 3]),
            "y": float(extrinsic[1, 3]),
            "z": float(extrinsic[2, 3]),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }
        pose = pose_from_vector(np.array([result["x"], result["y"], result["z"], roll, pitch, yaw]))
        after = residual(map_tree, apply_transform(body @ pose, scan))
        shift = pose[:3, 3] - prior[:3, 3]
        print(
            f"\n===== {item['key']} =====\n"
            f"  stamp={stamp:.3f} raw={len(xyz)} kept={kept} voxel={len(scan)} "
            f"range_p50={range_p50:.2f} map_local={len(local)} long_target={len(long_target)}\n"
            f"  converged={converged} iters={iterations} "
            f"shift=({shift[0]:+.3f}, {shift[1]:+.3f}, {shift[2]:+.3f}) m\n"
            f"  map  before median={before['median']:.3f} p90={before['p90']:.3f} "
            f"inlier<0.3={before['inlier_0_3']:.3f}\n"
            f"  map  after  median={after['median']:.3f} p90={after['p90']:.3f} "
            f"inlier<0.3={after['inlier_0_3']:.3f}\n"
            f"  long z-only {prior[2, 3]:.3f} -> {z_long:.3f} "
            f"median={long_median:.3f} inlier<0.3={long_inlier:.3f}\n"
            f"  t=({result['x']:.4f}, {result['y']:.4f}, {result['z']:.4f}) "
            f"rpy_deg=({np.degrees(roll):.3f}, {np.degrees(pitch):.3f}, {np.degrees(yaw):.3f})",
            flush=True,
        )
        if after["median"] > 0.15 or float(np.linalg.norm(shift)) > 0.50:
            raise SystemExit(f"{item['key']}: 第一帧没有配上地图，不写文件")
        # 远距点云只用来核对高度。和地图结果差得多时，高度跟 RTK 远距雷达。
        if abs(z_long - result["z"]) > 0.08 and long_inlier >= 0.45 and abs(z_long - prior[2, 3]) < 0.30:
            locked = dict(result)
            locked["z"] = z_long
            locked_pose = pose_from_vector(
                np.array([locked["x"], locked["y"], locked["z"], locked["roll"], locked["pitch"], locked["yaw"]])
            )
            locked_map = residual(map_tree, apply_transform(body @ locked_pose, scan))
            print(
                f"  use long height z={z_long:.4f} map median={locked_map['median']:.3f}",
                flush=True,
            )
            if locked_map["median"] < 0.20:
                result = locked
        results[name] = result

    lines = [
        "# 远距雷达外参来自 apolong_lidars_rtk.yaml。",
        "# 补盲雷达按该外参的坐标系，用第一帧对原点地图重新配准。",
        "# 单位 m / rad，RPY 为固定轴 XYZ。父坐标系 base_link。",
        "# X+ 前，Y+ 左，Z+ 上。",
        "",
        "# ---- 远距雷达----",
    ]
    for name, item in LONG.items():
        pose = vector_of(rtk[item["rtk_key"]])
        values = {field: float(pose[index]) for index, field in enumerate(("x", "y", "z", "roll", "pitch", "yaw"))}
        lines.append(block(item["key"], values))
    lines.append("")
    lines.append("# ---- 补盲雷达----")
    lines.append("# 第一帧对地图配准，高度参照 RTK 远距雷达")
    for name, item in BLIND.items():
        if name == "right":
            lines.append("# 第一帧对地图配准，高度参照 RTK 远距雷达")
        lines.append(block(item["key"], results[name]))
    lines.append("")
    OUTPUT.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {OUTPUT}", flush=True)


if __name__ == "__main__":
    main()
