#!/usr/bin/env python3
"""用原始 RTK 位置作先验，把远距左前雷达配到绝对坐标地图。"""

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

MAP_PATH = Path("/map/park.pcd")
MCAP = Path("/bag/lidar.mcap")
TOPIC = "/sutpc/sensor_driver/lidar/long/left_front/lidar_points"
EXTRINSIC = np.array(
    [3.6424937670, 1.1154305787, 1.2411615606, -0.0044037537, -0.0128260696, 2.2294231652]
)
SEED = 20261118
CROP = 45.0


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


def voxel_down(points, voxel):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return np.asarray(points[index], dtype=np.float64)


def interp(times, values, stamp):
    if stamp < times[0] - 0.2 or stamp > times[-1] + 0.2:
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


def read_frames():
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(MCAP), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(
        rosbag2_py.StorageFilter(
            topics=[TOPIC, "/sensing/gnss/ublox/nav_sat_fix", "/sensing/gnss/pose"]
        )
    )
    fixes, poses = [], []
    kept = []
    last_keep = None
    seen = 0
    pose_xyz = []
    while reader.has_next():
        topic, raw, _stamp = reader.read_next()
        if topic.endswith("nav_sat_fix"):
            msg = deserialize_message(raw, NavSatFix)
            fixes.append(
                (
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    msg.latitude,
                    msg.longitude,
                    msg.altitude,
                )
            )
        elif topic.endswith("/pose"):
            msg = deserialize_message(raw, PoseStamped)
            quat = msg.pose.orientation
            position = msg.pose.position
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            poses.append((stamp, quat.x, quat.y, quat.z, quat.w))
            pose_xyz.append((stamp, position.x, position.y, position.z))
        else:
            seen += 1
            msg = deserialize_message(raw, PointCloud2)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if last_keep is not None and stamp - last_keep < 1.0:
                continue
            points, _times = read_point_cloud(msg)
            xyz = np.asarray(points, dtype=np.float64)[:, :3]
            distance = np.linalg.norm(xyz, axis=1)
            keep = np.isfinite(xyz).all(axis=1) & (distance >= 2.0) & (distance <= 60.0)
            scan = voxel_down(xyz[keep], 0.3)
            if len(scan) < 200:
                continue
            kept.append((stamp, scan))
            last_keep = stamp
    fixes = np.asarray(fixes, dtype=np.float64)
    poses = np.asarray(poses, dtype=np.float64)
    pose_xyz = np.asarray(pose_xyz, dtype=np.float64)
    order = np.argsort(poses[:, 0])
    poses = poses[order]
    for index in range(1, len(poses)):
        if np.dot(poses[index, 1:], poses[index - 1, 1:]) < 0.0:
            poses[index, 1:] *= -1.0
    project = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)
    east, north = project.transform(fixes[:, 2], fixes[:, 1])
    rtk_xyz = np.column_stack((east, north, fixes[:, 3]))
    print(
        f"lidar_frames={seen} kept_1s={len(kept)} rtk={len(fixes)} "
        f"rtk0=({rtk_xyz[0, 0]:.3f}, {rtk_xyz[0, 1]:.3f}, {rtk_xyz[0, 2]:.3f})",
        flush=True,
    )
    if len(pose_xyz):
        print(
            f"pose.position0=({pose_xyz[0, 1]:.3f}, {pose_xyz[0, 2]:.3f}, {pose_xyz[0, 3]:.3f}) "
            f"与 RTK 平移差 {np.linalg.norm(pose_xyz[0, 1:] - rtk_xyz[0]):.1f} m，先验不用 pose.position",
            flush=True,
        )
    rng = np.random.default_rng(SEED)
    chosen = []
    for index in rng.permutation(len(kept)):
        stamp = kept[index][0]
        if all(abs(stamp - item[0]) > 3.0 for item in chosen):
            chosen.append(kept[index])
        if len(chosen) == 5:
            break
    return fixes[:, 0], rtk_xyz, poses[:, 0], poses[:, 1:], chosen


def body_pose(fix_t, rtk_xyz, pose_t, pose_q, stamp):
    position = interp(fix_t, rtk_xyz, stamp)
    quat = interp(pose_t, pose_q, stamp)
    if position is None or quat is None:
        return None
    quat = quat / np.linalg.norm(quat)
    transform = np.eye(4)
    transform[:3, :3] = quat_to_rotation(quat)
    transform[:3, 3] = position
    return transform


def load_crop(bounds):
    xmin, xmax, ymin, ymax = bounds
    kept = []
    with MAP_PATH.open("rb") as handle:
        while not handle.readline().startswith(b"DATA"):
            pass
        chunk_points = 1_000_000
        chunk_bytes = chunk_points * 30
        while True:
            raw = handle.read(chunk_bytes)
            if not raw:
                break
            count = len(raw) // 30
            packed = np.frombuffer(raw, dtype=np.uint8, count=count * 30).reshape(count, 30)
            xyz = np.ascontiguousarray(packed[:, :24]).view(np.float64).reshape(count, 3)
            mask = (
                (xyz[:, 0] >= xmin)
                & (xyz[:, 0] <= xmax)
                & (xyz[:, 1] >= ymin)
                & (xyz[:, 1] <= ymax)
                & np.isfinite(xyz).all(axis=1)
            )
            if np.any(mask):
                kept.append(xyz[mask])
    cloud = np.vstack(kept)
    return voxel_down(cloud, 0.4)


def residual(tree, points):
    distances, index = tree.query(points, k=1, workers=-1)
    near = distances < 2.0
    if np.any(near):
        z_bias = float(np.median(tree.data[index[near], 2] - points[near, 2]))
    else:
        z_bias = float("nan")
    return {
        "median": float(np.median(distances)),
        "p90": float(np.percentile(distances, 90)),
        "inlier_0_3": float(np.mean(distances < 0.3)),
        "inlier_1": float(np.mean(distances < 1.0)),
        "z_bias": z_bias,
    }


def align(local, scan, guess):
    current = guess
    last = None
    for distance, iterations in ((3.0, 30), (1.0, 25), (0.4, 20)):
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
    return current, bool(last.converged), int(last.iterations), float(last.error)


def main():
    fix_t, rtk_xyz, pose_t, pose_q, frames = read_frames()
    extrinsic = rpy_matrix(EXTRINSIC[3], EXTRINSIC[4], EXTRINSIC[5])
    extrinsic[:3, 3] = EXTRINSIC[:3]
    bodies = []
    usable = []
    for stamp, scan in frames:
        body = body_pose(fix_t, rtk_xyz, pose_t, pose_q, stamp)
        if body is None:
            print(f"stamp={stamp:.3f} 没有 RTK", flush=True)
            continue
        bodies.append(body)
        usable.append((stamp, scan))
    centers = np.vstack([body[:2, 3] for body in bodies])
    bounds = (
        float(centers[:, 0].min() - CROP),
        float(centers[:, 0].max() + CROP),
        float(centers[:, 1].min() - CROP),
        float(centers[:, 1].max() + CROP),
    )
    print(
        f"rtk prior bbox x={bounds[0]:.1f}..{bounds[1]:.1f} y={bounds[2]:.1f}..{bounds[3]:.1f}",
        flush=True,
    )
    print("loading map crop", flush=True)
    cloud = load_crop(bounds)
    print(f"map_crop={len(cloud)}", flush=True)
    # 只在求解器内部减去公共平移，RTK 与地图的相对位置不变。
    local_origin = np.array([centers[:, 0].mean(), centers[:, 1].mean(), 0.0])
    cloud_local = cloud - local_origin
    ok = 0
    for (stamp, scan), body in zip(usable, bodies):
        guess = body @ extrinsic
        guess_local = guess.copy()
        guess_local[:3, 3] -= local_origin
        center = guess_local[:2, 3]
        delta = cloud_local[:, :2] - center
        local = cloud_local[np.einsum("ij,ij->i", delta, delta) <= 40.0 ** 2]
        tree = cKDTree(local)
        moved = (guess_local[:3, :3] @ scan.T).T + guess_local[:3, 3]
        near_scan = scan[np.linalg.norm(scan, axis=1) < 30.0]
        near_moved = (guess_local[:3, :3] @ near_scan.T).T + guess_local[:3, 3]
        before = residual(tree, moved)
        before_near = residual(tree, near_moved)
        refined, converged, iterations, gicp_error = align(local, scan, guess_local)
        after_moved = (refined[:3, :3] @ scan.T).T + refined[:3, 3]
        after = residual(tree, after_moved)
        shift = refined[:3, 3] - guess_local[:3, 3]
        refined_utm = refined[:3, 3] + local_origin
        locked = before_near["median"] < 0.25 and before_near["inlier_1"] > 0.7 and np.linalg.norm(shift[:2]) < 2.0
        ok += int(locked)
        print(
            f"\nstamp={stamp:.3f} points={len(scan)} map_local={len(local)} "
            f"small_gicp converged={converged} iters={iterations} error={gicp_error:.3f}\n"
            f"  rtk_prior=({guess[0, 3]:.3f}, {guess[1, 3]:.3f}, {guess[2, 3]:.3f})\n"
            f"  before median={before['median']:.3f} p90={before['p90']:.3f} "
            f"<0.3={before['inlier_0_3']:.3f} <1={before['inlier_1']:.3f} z_bias={before['z_bias']:.3f}\n"
            f"  near30 median={before_near['median']:.3f} <0.3={before_near['inlier_0_3']:.3f} "
            f"<1={before_near['inlier_1']:.3f} z_bias={before_near['z_bias']:.3f}\n"
            f"  after  median={after['median']:.3f} p90={after['p90']:.3f} "
            f"<0.3={after['inlier_0_3']:.3f} <1={after['inlier_1']:.3f}\n"
            f"  shift=({shift[0]:+.3f}, {shift[1]:+.3f}, {shift[2]:+.3f}) m "
            f"matched=({refined_utm[0]:.3f}, {refined_utm[1]:.3f}, {refined_utm[2]:.3f}) "
            f"{'锁定' if locked else '未锁定'}",
            flush=True,
        )
    print(f"\nlocked {ok}/{len(usable)}", flush=True)


if __name__ == "__main__":
    main()
