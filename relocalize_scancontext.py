#!/usr/bin/env python3
"""从地图质心出发，用 Scan Context 和分扇区重叠对远距左前雷达做粗重定位。"""

from pathlib import Path

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from kiss_icp.tools.point_cloud2 import read_point_cloud
from pyproj import Transformer
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import NavSatFix, PointCloud2

MAP_PATH = Path("/map/merged_centroid0.pcd")
MCAP = Path("/bag/lidar.mcap")
TOPIC = "/sutpc/sensor_driver/lidar/long/left_front/lidar_points"
CENTROID = np.array([193512.6777938, 2503137.781991, 112.7042120891], dtype=np.float64)
EXTRINSIC = np.array([3.6424937670, 1.1154305787, 1.2411615606, -0.0044037537, -0.0128260696, 2.2294231652])
RINGS = 20
SECTORS = 60
MAX_RANGE = 60.0
MIN_RANGE = 3.0
GRID = 20.0
SEED = 20261118


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


def wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def yaw_of(rotation):
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def voxel_down(points, voxel):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return np.asarray(points[index], dtype=np.float64)


def read_map():
    with MAP_PATH.open("rb") as handle:
        while not handle.readline().startswith(b"DATA"):
            pass
        raw = handle.read()
    count = len(raw) // 30
    packed = np.frombuffer(raw, dtype=np.uint8, count=count * 30).reshape(count, 30)
    xyz = np.ascontiguousarray(packed[:, :24]).view(np.float64).reshape(count, 3)
    finite = np.isfinite(xyz).all(axis=1)
    return voxel_down(xyz[finite], 1.5)


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
    while reader.has_next():
        topic, raw, _stamp = reader.read_next()
        if topic.endswith("nav_sat_fix"):
            msg = deserialize_message(raw, NavSatFix)
            fixes.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.latitude, msg.longitude, msg.altitude))
        elif topic.endswith("/pose"):
            msg = deserialize_message(raw, PoseStamped)
            quat = msg.pose.orientation
            poses.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, quat.x, quat.y, quat.z, quat.w))
        else:
            seen += 1
            msg = deserialize_message(raw, PointCloud2)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if last_keep is not None and stamp - last_keep < 1.0:
                continue
            points, _times = read_point_cloud(msg)
            xyz = np.asarray(points, dtype=np.float64)[:, :3]
            distance = np.linalg.norm(xyz, axis=1)
            keep = np.isfinite(xyz).all(axis=1) & (distance >= MIN_RANGE) & (distance <= MAX_RANGE)
            scan = voxel_down(xyz[keep], 0.8)
            if len(scan) < 200:
                continue
            kept.append((stamp, scan))
            last_keep = stamp
    fixes = np.asarray(fixes, dtype=np.float64)
    poses = np.asarray(poses, dtype=np.float64)
    order = np.argsort(poses[:, 0])
    poses = poses[order]
    for index in range(1, len(poses)):
        if np.dot(poses[index, 1:], poses[index - 1, 1:]) < 0.0:
            poses[index, 1:] *= -1.0
    project = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)
    east, north = project.transform(fixes[:, 2], fixes[:, 1])
    fix_xyz = np.column_stack((east, north, fixes[:, 3])) - CENTROID
    print(f"lidar_frames={seen} kept_1s={len(kept)} gnss={len(fixes)}", flush=True)
    rng = np.random.default_rng(SEED)
    chosen = []
    for index in rng.permutation(len(kept)):
        stamp = kept[index][0]
        if all(abs(stamp - item[0]) > 3.0 for item in chosen):
            chosen.append(kept[index])
        if len(chosen) == 5:
            break
    return fixes[:, 0], fix_xyz, poses[:, 0], poses[:, 1:], chosen


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


def scan_context(xy, z, sensor_z=0.0):
    relative = z - sensor_z
    radius = np.hypot(xy[:, 0], xy[:, 1])
    keep = (radius >= MIN_RANGE) & (radius <= MAX_RANGE) & np.isfinite(relative)
    context = np.zeros((RINGS, SECTORS), dtype=np.float64)
    if not np.any(keep):
        return context
    ring = np.clip(np.floor(radius[keep] / MAX_RANGE * RINGS).astype(np.int64), 0, RINGS - 1)
    angle = np.arctan2(xy[keep, 1], xy[keep, 0])
    sector = np.clip(np.floor((angle + np.pi) / (2.0 * np.pi) * SECTORS).astype(np.int64), 0, SECTORS - 1)
    height = np.maximum(relative[keep], 0.0)
    # 每个格子保留最大高度。
    order = np.argsort(height)
    context[ring[order], sector[order]] = height[order]
    return context


def context_distance(query, candidate):
    best_distance = None
    best_shift = 0
    for shift in range(SECTORS):
        rolled = np.roll(query, shift, axis=1)
        numerator = np.sum(rolled * candidate, axis=0)
        query_norm = np.linalg.norm(rolled, axis=0)
        candidate_norm = np.linalg.norm(candidate, axis=0)
        both_empty = (query_norm < 1e-6) & (candidate_norm < 1e-6)
        one_empty = ((query_norm < 1e-6) ^ (candidate_norm < 1e-6))
        similarity = np.zeros(SECTORS, dtype=np.float64)
        usable = ~(both_empty | one_empty)
        similarity[usable] = numerator[usable] / (query_norm[usable] * candidate_norm[usable])
        similarity[both_empty] = 1.0
        distance = float(1.0 - np.mean(similarity))
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_shift = shift
    yaw = best_shift * (2.0 * np.pi / SECTORS)
    return best_distance, yaw


def sector_overlap(scan, pose, tree):
    yaw = pose[2]
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    xy = scan[:, :2] @ rotation.T + pose[:2]
    points = np.column_stack((xy, scan[:, 2] + pose[3]))
    distances, _index = tree.query(points, k=1, workers=-1)
    angle = np.arctan2(scan[:, 1], scan[:, 0])
    sectors = np.clip(np.floor((angle + np.pi) / (2.0 * np.pi) * 12).astype(np.int64), 0, 11)
    scores = []
    for sector in range(12):
        chosen = sectors == sector
        if int(chosen.sum()) < 20:
            continue
        scores.append(float(np.mean(distances[chosen] < 2.0)))
    if not scores:
        return 0.0
    return float(np.mean(scores))


def local_cloud(cloud, xy, radius):
    delta = cloud[:, :2] - xy
    return cloud[np.einsum("ij,ij->i", delta, delta) <= radius ** 2]


def sensor_height(local):
    near = local[np.linalg.norm(local[:, :2] - local[:, :2].mean(axis=0), axis=1) < 25.0]
    base = near if len(near) > 50 else local
    return float(np.percentile(base[:, 2], 20) + 1.5)


def refine_overlap(scan, cloud, xy, yaw, sensor_z):
    local = local_cloud(cloud, xy, MAX_RANGE + 15.0)
    if len(local) < 100:
        return (*xy, yaw, sensor_z), 0.0
    tree = cKDTree(local)
    best = None
    best_score = -1.0
    for dx in np.linspace(-16.0, 16.0, 5):
        for dy in np.linspace(-16.0, 16.0, 5):
            for dyaw in np.linspace(-0.20, 0.20, 5):
                pose = np.array([xy[0] + dx, xy[1] + dy, yaw + dyaw, sensor_z])
                score = sector_overlap(scan, pose, tree)
                if score > best_score:
                    best_score = score
                    best = pose
    return best, best_score


def main():
    print("loading map", flush=True)
    cloud = read_map()
    print(f"map_voxel={len(cloud)} min={cloud.min(0)} max={cloud.max(0)}", flush=True)
    fix_t, fix_xyz, pose_t, pose_q, frames = read_frames()
    extrinsic = rpy_matrix(EXTRINSIC[3], EXTRINSIC[4], EXTRINSIC[5])
    extrinsic[:3, 3] = EXTRINSIC[:3]
    print(f"centroid_start=(0,0,0) samples={len(frames)}", flush=True)

    keys = np.floor(cloud[:, :2] / GRID).astype(np.int64)
    cells = np.unique(keys, axis=0)
    candidates = []
    contexts = []
    for cell in cells:
        center = (cell.astype(np.float64) + 0.5) * GRID
        local = local_cloud(cloud, center, MAX_RANGE)
        if len(local) < 400:
            continue
        height = sensor_height(local)
        relative = local[:, :2] - center
        context = scan_context(relative, local[:, 2], height)
        if np.count_nonzero(context) < 30:
            continue
        candidates.append((center, height, local))
        contexts.append(context)
    print(f"place_candidates={len(candidates)}", flush=True)

    for stamp, scan in frames:
        body = body_pose(fix_t, fix_xyz, pose_t, pose_q, stamp)
        if body is None:
            print(f"\nstamp={stamp:.3f} 没有 GNSS", flush=True)
            continue
        truth = body @ extrinsic
        true_xy = truth[:2, 3]
        true_yaw = yaw_of(truth[:3, :3])
        true_z = float(truth[2, 3])
        centroid_distance = float(np.linalg.norm(true_xy))
        query = scan_context(scan[:, :2], scan[:, 2], 0.0)

        oracle_local = local_cloud(cloud, true_xy, MAX_RANGE)
        oracle_context = scan_context(oracle_local[:, :2] - true_xy, oracle_local[:, 2], true_z)
        oracle_distance, oracle_yaw = context_distance(query, oracle_context)
        oracle_tree = cKDTree(oracle_local)
        oracle_overlap = sector_overlap(scan, np.array([true_xy[0], true_xy[1], true_yaw, true_z]), oracle_tree)

        centroid_local = local_cloud(cloud, np.zeros(2), MAX_RANGE)
        centroid_height = sensor_height(centroid_local) if len(centroid_local) else 0.0
        centroid_context = scan_context(centroid_local[:, :2], centroid_local[:, 2], centroid_height)
        centroid_distance_sc, centroid_yaw = context_distance(query, centroid_context)
        centroid_tree = cKDTree(centroid_local) if len(centroid_local) else None
        centroid_overlap = 0.0
        if centroid_tree is not None:
            best_centroid = -1.0
            best_yaw = centroid_yaw
            for dyaw in np.linspace(-np.pi, np.pi, 36, endpoint=False):
                score = sector_overlap(
                    scan, np.array([0.0, 0.0, dyaw, centroid_height]), centroid_tree
                )
                if score > best_centroid:
                    best_centroid = score
                    best_yaw = dyaw
            centroid_overlap = best_centroid
            centroid_yaw = best_yaw

        ranked = []
        nearest_rank = None
        nearest_distance = None
        nearest_gap = None
        for index, context in enumerate(contexts):
            distance, yaw = context_distance(query, context)
            center = candidates[index][0]
            gap = float(np.linalg.norm(center - true_xy))
            ranked.append((distance, index, yaw, gap))
            if nearest_gap is None or gap < nearest_gap:
                nearest_gap = gap
                nearest_rank = index
                nearest_distance = distance
        ranked.sort(key=lambda item: item[0])
        true_rank = next(rank for rank, item in enumerate(ranked, start=1) if item[1] == nearest_rank)
        shortlist = ranked[:40]
        best_pose = None
        best_score = -1.0
        best_descriptor = None
        for distance, index, yaw, _gap in shortlist:
            center, height, local = candidates[index]
            if len(local) < 100:
                continue
            tree = cKDTree(local)
            score = sector_overlap(scan, np.array([center[0], center[1], yaw, height]), tree)
            if score > best_score:
                best_score = score
                best_pose = np.array([center[0], center[1], yaw, height])
                best_descriptor = distance
        refined, refined_score = refine_overlap(scan, cloud, best_pose[:2], best_pose[2], best_pose[3])
        error_xy = float(np.linalg.norm(refined[:2] - true_xy))
        error_yaw = abs(np.degrees(wrap(refined[2] - true_yaw)))
        print(
            f"\nstamp={stamp:.3f} points={len(scan)} gt=({true_xy[0]:.1f}, {true_xy[1]:.1f}, {true_z:.1f}) "
            f"yaw={np.degrees(true_yaw):.1f} deg dist_centroid={centroid_distance:.1f} m\n"
            f"  oracle  sc_yaw_err={abs(np.degrees(wrap(oracle_yaw - true_yaw))):.1f} deg "
            f"sc_dist={oracle_distance:.3f} overlap={oracle_overlap:.3f}\n"
            f"  centroid overlap={centroid_overlap:.3f} yaw_err={abs(np.degrees(wrap(centroid_yaw - true_yaw))):.1f} deg "
            f"xy_err={centroid_distance:.1f} m\n"
            f"  true_cell rank={true_rank}/{len(ranked)} gap={nearest_gap:.1f} m sc_dist={nearest_distance:.3f}\n"
            f"  global  sc_dist={best_descriptor:.3f} overlap={refined_score:.3f} "
            f"xy_err={error_xy:.1f} m yaw_err={error_yaw:.1f} deg "
            f"est=({refined[0]:.1f}, {refined[1]:.1f})",
            flush=True,
        )


if __name__ == "__main__":
    main()
