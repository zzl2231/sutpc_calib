#!/usr/bin/env python3
"""以 long_left_front 的地图匹配轨迹为基准，标定 long_right_end 和 RTK。

没有起点文件时，用 RTK 位置加 Scan Context 航向搜索得到初值，再 small-gicp 精配准。
然后逐帧图匹配，并初始化右雷达相对前向雷达的外参。
RTK 也写在前向雷达坐标系下。
"""

import argparse
import struct
import time
from pathlib import Path

import numpy as np
import rosbag2_py
import small_gicp
from geometry_msgs.msg import PoseStamped
from pyproj import Transformer
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import NavSatFix, PointCloud2
from kiss_icp.tools.point_cloud2 import read_point_cloud


REFERENCE = "long_left_front"
SIDE = "long_right_end"
TOPICS = {
    REFERENCE: "/sutpc/sensor_driver/lidar/long/left_front/lidar_points",
    SIDE: "/sutpc/sensor_driver/lidar/long/right_end/lidar_points",
}
MAP_VOXEL = 0.10
SCAN_VOXEL = 0.30
MIN_RANGE = 2.0
MAX_RANGE = 100.0
SIDE_STRIDE = 5
MATCH_RADIUS = 80.0
FAIL_SHIFT = 3.0
MAX_GAP = 0.30
RINGS = 20
SECTORS = 60
CONTEXT_RANGE = 60.0


def wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def circular_median(angles):
    angles = np.asarray(angles, dtype=np.float64)
    base = float(angles[0])
    return float(wrap(np.median(wrap(angles - base)) + base))


def skew(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def so3_log(rotation):
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    theta = float(np.arccos(cosine))
    if theta < 1.0e-9:
        return np.zeros(3)
    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    return 0.5 * axis * theta / np.sin(theta)


def so3_exp(vector):
    theta = float(np.linalg.norm(vector))
    if theta < 1.0e-9:
        return np.eye(3)
    axis = vector / theta
    skew_axis = skew(axis)
    return np.eye(3) + np.sin(theta) * skew_axis + (1.0 - np.cos(theta)) * (skew_axis @ skew_axis)


def blend_pose(pose_a, pose_b, ratio):
    relative = pose_a[:3, :3].T @ pose_b[:3, :3]
    out = np.eye(4)
    out[:3, :3] = pose_a[:3, :3] @ so3_exp(so3_log(relative) * ratio)
    out[:3, 3] = (1.0 - ratio) * pose_a[:3, 3] + ratio * pose_b[:3, 3]
    return out


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


def stamp_sec(msg):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def voxel_down(points, voxel):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _unique, index = np.unique(keys, axis=0, return_index=True)
    return points[index]


def load_las(path: Path) -> np.ndarray:
    with path.open("rb") as handle:
        header = handle.read(227)
        data_off = struct.unpack_from("<I", header, 96)[0]
        reclen = struct.unpack_from("<H", header, 105)[0]
        count = struct.unpack_from("<I", header, 107)[0]
        scales = np.array(struct.unpack_from("<3d", header, 131))
        offsets = np.array(struct.unpack_from("<3d", header, 155))
        handle.seek(data_off)
        raw = handle.read(count * reclen)
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(count, reclen)
    xyz_i = np.ascontiguousarray(arr[:, :12]).view(np.int32).reshape(count, 3)
    return xyz_i.astype(np.float64) * scales + offsets


def load_start_poses(path: Path) -> dict:
    poses, name, rows = {}, None, []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith("# long_"):
            name = line[2:].split()[0]
            rows = []
            continue
        if name is None or not line or line.startswith("#"):
            continue
        rows.append([float(item) for item in line.split()])
        if len(rows) == 4:
            poses[name] = np.asarray(rows, dtype=np.float64)
            name, rows = None, []
    for key in (REFERENCE, SIDE):
        if key not in poses:
            raise SystemExit(f"起点位姿文件缺少 {key}: {path}")
    return poses


def open_reader(mcap: Path, topics):
    if isinstance(topics, str):
        topics = [topics]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(mcap), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    return reader


def read_scan(raw):
    if len(raw) < 32:
        return None
    try:
        msg = deserialize_message(raw, PointCloud2)
        points, _timestamps = read_point_cloud(msg)
    except Exception:
        return None
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3 or len(xyz) == 0:
        return None
    xyz = xyz[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    range2 = np.einsum("ij,ij->i", xyz, xyz)
    keep = finite & (range2 >= MIN_RANGE * MIN_RANGE) & (range2 <= MAX_RANGE * MAX_RANGE)
    xyz = xyz[keep]
    if len(xyz) < 100:
        return None
    return stamp_sec(msg), xyz


def predict(history, stamp, start):
    if not history:
        return start
    if len(history) == 1:
        return history[-1][1]
    stamp_a, pose_a = history[-2]
    stamp_b, pose_b = history[-1]
    span = stamp_b - stamp_a
    if span <= 1.0e-6:
        return pose_b
    return blend_pose(pose_a, pose_b, (stamp - stamp_a) / span)


def align_scan(scan, guess, map_points):
    local_delta = map_points[:, :2] - guess[:2, 3]
    local = map_points[np.einsum("ij,ij->i", local_delta, local_delta) <= MATCH_RADIUS ** 2]
    if len(local) < 300 or len(scan) < 100:
        return None
    result = small_gicp.align(
        local,
        scan,
        guess,
        registration_type="GICP",
        voxel_resolution=SCAN_VOXEL,
        downsampling_resolution=SCAN_VOXEL,
        max_correspondence_distance=2.0,
        num_threads=8,
        max_iterations=20,
        verbose=False,
    )
    candidate = np.asarray(result.T_target_source, dtype=np.float64)
    shift = float(np.linalg.norm(candidate[:2, 3] - guess[:2, 3]))
    if not bool(result.converged) or shift > FAIL_SHIFT:
        return None
    return candidate


def match_reference(mcap, map_points, start, topic=None, progress=None):
    topic = topic or TOPICS[REFERENCE]
    print(f"\n===== {topic} 逐帧图匹配 =====", flush=True)
    reader = open_reader(mcap, topic)
    history = []
    usable = failed = 0
    started = time.perf_counter()
    while reader.has_next():
        _topic, raw, _stamp = reader.read_next()
        scan = read_scan(raw)
        if scan is None:
            continue
        stamp, points = scan
        usable += 1
        guess = predict(history, stamp, start)
        refined = align_scan(voxel_down(points, SCAN_VOXEL), guess, map_points)
        if refined is None:
            failed += 1
        else:
            history.append((stamp, refined))
        if usable == 1 or usable % 20 == 0:
            pose = refined if refined is not None else guess
            print(
                f"  {usable} xy=({pose[0, 3]:.2f}, {pose[1, 3]:.2f}) "
                f"matched={refined is not None} failed={failed}",
                flush=True,
            )
            if progress is not None:
                progress(usable, len(history), failed)
    elapsed = time.perf_counter() - started
    print(
        f"  frames={usable} matched={len(history)} failed={failed} elapsed={elapsed:.1f}s",
        flush=True,
    )
    if len(history) < 20:
        raise SystemExit(f"{REFERENCE} 地图匹配成功帧不足，不能作为车体轨迹")
    stamps = np.asarray([item[0] for item in history])
    poses = [item[1] for item in history]
    travel = float(np.linalg.norm(poses[-1][:3, 3] - poses[0][:3, 3]))
    print(f"  trajectory={len(poses)} travel={travel:.2f} m", flush=True)
    return stamps, poses


def trajectory_pose(stamps, poses, stamp):
    if stamp < stamps[0] or stamp > stamps[-1]:
        return None
    index = int(np.searchsorted(stamps, stamp))
    if index <= 0:
        return poses[0] if abs(stamps[0] - stamp) <= MAX_GAP else None
    if index >= len(stamps):
        return poses[-1] if abs(stamps[-1] - stamp) <= MAX_GAP else None
    span = float(stamps[index] - stamps[index - 1])
    if span <= 1.0e-6 or span > MAX_GAP:
        return None
    ratio = (stamp - float(stamps[index - 1])) / span
    return blend_pose(poses[index - 1], poses[index], ratio)


def match_side(mcap, map_points, stamps, poses, prior, topic=None, progress=None):
    topic = topic or TOPICS[SIDE]
    print(f"\n===== {topic} 使用基准雷达轨迹做帧图匹配 =====", flush=True)
    reader = open_reader(mcap, topic)
    relatives = []
    usable = count = failed = missing = 0
    started = time.perf_counter()
    while reader.has_next():
        _topic, raw, _stamp = reader.read_next()
        scan = read_scan(raw)
        if scan is None:
            continue
        stamp, points = scan
        usable += 1
        if (usable - 1) % SIDE_STRIDE != 0:
            continue
        count += 1
        body = trajectory_pose(stamps, poses, stamp)
        guess = None if body is None else body @ prior
        refined = None if guess is None else align_scan(voxel_down(points, SCAN_VOXEL), guess, map_points)
        if body is None:
            missing += 1
        elif refined is None:
            failed += 1
        else:
            relatives.append(np.linalg.inv(body) @ refined)
        pose = refined if refined is not None else guess
        if count == 1 or count % 10 == 0:
            if pose is None:
                print(f"  {count} no trajectory failed={failed} missing={missing}", flush=True)
            else:
                print(
                    f"  {count} xy=({pose[0, 3]:.2f}, {pose[1, 3]:.2f}) "
                    f"matched={refined is not None} failed={failed} missing={missing}",
                    flush=True,
                )
            if progress is not None:
                progress(count, len(relatives), failed)
    elapsed = time.perf_counter() - started
    print(
        f"  frames={count}/{usable} pairs={len(relatives)} failed={failed} "
        f"missing={missing} elapsed={elapsed:.1f}s",
        flush=True,
    )
    return relatives


def robust_pose(samples):
    if len(samples) < 20:
        return None
    translations = np.array([item[:3, 3] for item in samples])
    center = np.median(translations, axis=0)
    keep = np.linalg.norm(translations - center, axis=1) < 1.0
    rolls, pitches, yaws = [], [], []
    for transform, used in zip(samples, keep, strict=True):
        if not used:
            continue
        roll, pitch, yaw = rotation_to_rpy(transform[:3, :3])
        rolls.append(roll)
        pitches.append(pitch)
        yaws.append(yaw)
    rolls, pitches, yaws = np.asarray(rolls), np.asarray(pitches), np.asarray(yaws)
    yaw_center = circular_median(yaws)
    angle_keep = np.abs(wrap(yaws - yaw_center)) < np.deg2rad(8.0)
    if int(angle_keep.sum()) < 20:
        return None
    chosen = translations[keep][angle_keep]
    return {
        "x": float(np.median(chosen[:, 0])),
        "y": float(np.median(chosen[:, 1])),
        "z": float(np.median(chosen[:, 2])),
        "roll": float(np.median(rolls[angle_keep])),
        "pitch": float(np.median(pitches[angle_keep])),
        "yaw": circular_median(yaws[angle_keep]),
        "count": int(angle_keep.sum()),
        "spread": float(np.median(np.linalg.norm(chosen - np.median(chosen, axis=0), axis=1))),
    }


GNSS_FIX = "/sensing/gnss/ublox/nav_sat_fix"
GNSS_POSE = "/sensing/gnss/pose"
CONTEXT_MIN_RANGE = 3.0
MAP_CROP_MARGIN = 90.0


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


def read_rtk(mcap: Path):
    reader = open_reader(mcap, [GNSS_FIX, GNSS_POSE])
    fixes, quats = [], []
    while reader.has_next():
        topic, raw, _stamp = reader.read_next()
        if topic.endswith("nav_sat_fix"):
            msg = deserialize_message(raw, NavSatFix)
            fixes.append(
                (
                    stamp_sec(msg),
                    msg.latitude,
                    msg.longitude,
                    msg.altitude,
                )
            )
        elif topic.endswith("/pose"):
            msg = deserialize_message(raw, PoseStamped)
            quat = msg.pose.orientation
            quats.append((stamp_sec(msg), quat.x, quat.y, quat.z, quat.w))
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
        f"rtk0=({xyz[0, 0]:.3f}, {xyz[0, 1]:.3f}, {xyz[0, 2]:.3f})",
        flush=True,
    )
    return {"time": fixes[:, 0], "xyz": xyz, "pose_time": quats[:, 0], "quat": quats[:, 1:]}


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


def yaw_span(rtk):
    yaws = []
    for quat in rtk["quat"][::10]:
        rotation = quat_to_rotation(quat / np.linalg.norm(quat))
        yaws.append(float(np.arctan2(rotation[1, 0], rotation[0, 0])))
    if len(yaws) < 2:
        return 0.0
    unwrapped = np.unwrap(np.asarray(yaws))
    return float(np.degrees(unwrapped.max() - unwrapped.min()))


def load_pcd_crop(path: Path, bounds, voxel):
    xmin, xmax, ymin, ymax = bounds
    kept = []
    with path.open("rb") as handle:
        line = b""
        while True:
            line = handle.readline()
            if not line:
                raise SystemExit(f"PCD 没有 DATA: {path}")
            if line.startswith(b"DATA"):
                if b"binary" not in line.split():
                    raise SystemExit(f"只读取 binary PCD: {line.decode('ascii', 'replace').strip()}")
                break
        chunk_points = 1_000_000
        while True:
            raw = handle.read(chunk_points * 30)
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
                kept.append(voxel_down(xyz[mask], voxel))
    if not kept:
        raise SystemExit("RTK 轨迹周围没有地图点")
    return voxel_down(np.vstack(kept), voxel)


def scan_context(xy, z):
    radius = np.hypot(xy[:, 0], xy[:, 1])
    keep = (radius >= CONTEXT_MIN_RANGE) & (radius <= CONTEXT_RANGE) & np.isfinite(z)
    context = np.zeros((RINGS, SECTORS), dtype=np.float64)
    if not np.any(keep):
        return context
    ring = np.clip(np.floor(radius[keep] / CONTEXT_RANGE * RINGS).astype(np.int64), 0, RINGS - 1)
    angle = np.arctan2(xy[keep, 1], xy[keep, 0])
    sector = np.clip(np.floor((angle + np.pi) / (2.0 * np.pi) * SECTORS).astype(np.int64), 0, SECTORS - 1)
    height = np.maximum(z[keep], 0.0)
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
        one_empty = (query_norm < 1e-6) ^ (candidate_norm < 1e-6)
        similarity = np.zeros(SECTORS, dtype=np.float64)
        usable = ~(both_empty | one_empty)
        similarity[usable] = numerator[usable] / (query_norm[usable] * candidate_norm[usable])
        similarity[both_empty] = 1.0
        distance = float(1.0 - np.mean(similarity))
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_shift = shift
    return best_distance, best_shift * (2.0 * np.pi / SECTORS)


def yaw_rotation(yaw):
    cosine, sine = np.cos(yaw), np.sin(yaw)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def sector_overlap(scan, pose, tree):
    rotation = yaw_rotation(pose[2])[:2, :2]
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


def near_residual(tree, scan, transform):
    points = (transform[:3, :3] @ scan.T).T + transform[:3, 3]
    chosen = np.linalg.norm(scan, axis=1) < 30.0
    if not np.any(chosen):
        chosen = np.ones(len(scan), dtype=bool)
    distances, _index = tree.query(points[chosen], k=1, workers=-1)
    return {
        "median": float(np.median(distances)),
        "inlier_0_3": float(np.mean(distances < 0.3)),
        "inlier_1": float(np.mean(distances < 1.0)),
    }


def radon_occupancy(xy, n_angle, n_bin, extent):
    """θ 取满 [0, 2π)。旋转点云会使正弦图沿角度循环平移。"""
    angles = np.linspace(0.0, 2.0 * np.pi, n_angle, endpoint=False)
    proj = xy[:, :1] * np.cos(angles) + xy[:, 1:2] * np.sin(angles)
    idx = np.floor((proj + extent) * (n_bin / (2.0 * extent))).astype(np.int32)
    valid = (idx >= 0) & (idx < n_bin)
    sino = np.zeros((n_angle, n_bin), dtype=np.float64)
    angle_id = np.broadcast_to(np.arange(n_angle, dtype=np.int32), idx.shape)
    chosen = valid.ravel()
    code = angle_id.ravel()[chosen] * np.int32(n_bin) + idx.ravel()[chosen]
    if code.size:
        sino.ravel()[np.unique(code)] = 1.0
    return sino, angles


def translation_invariant_ring(sino):
    """沿投影距离做傅里叶变换并取幅度，平移在这个描述子里消失。"""
    magnitude = np.abs(np.fft.rfft(sino, axis=1))[:, 1:]
    norm = np.linalg.norm(magnitude, axis=1, keepdims=True)
    np.divide(magnitude, norm, out=magnitude, where=norm > 1e-8)
    return magnitude


def circular_shift(corr, index):
    count = len(corr)
    left = corr[(index - 1) % count]
    right = corr[(index + 1) % count]
    denom = left - 2.0 * corr[index] + right
    if abs(denom) < 1e-9:
        return 0.0
    return float(np.clip(0.5 * (left - right) / denom, -0.5, 0.5))


def ring_candidates(scan_xy, map_xy, extent=45.0, n_angle=180, n_bin=128):
    """RING++ 的航向和平移求解。返回使地图 ≈ R(yaw) @ 扫描 + (dx, dy) 的候选。"""
    scan_xy = np.asarray(scan_xy, dtype=np.float64)[:, :2]
    map_xy = np.asarray(map_xy, dtype=np.float64)[:, :2]
    scan_xy = scan_xy[np.linalg.norm(scan_xy, axis=1) <= extent]
    map_xy = map_xy[np.linalg.norm(map_xy, axis=1) <= extent]
    if len(scan_xy) < 50 or len(map_xy) < 50:
        return [(0.0, 0.0, 0.0)]
    sino_map, angles = radon_occupancy(map_xy, n_angle, n_bin, extent)
    sino_scan, _angles = radon_occupancy(scan_xy, n_angle, n_bin, extent)
    corr = np.fft.ifft(
        np.fft.fft(translation_invariant_ring(sino_scan), axis=0)
        * np.conj(np.fft.fft(translation_invariant_ring(sino_map), axis=0)),
        axis=0,
    ).real.sum(axis=1)
    yaws = []
    for index in np.argsort(corr)[::-1]:
        yaw = wrap(-(int(index) + circular_shift(corr, int(index))) / n_angle * 2.0 * np.pi)
        if all(abs(wrap(yaw - chosen)) > np.deg2rad(20.0) for chosen in yaws):
            yaws.append(yaw)
        if len(yaws) == 3:
            break
    if yaws:
        opposite = wrap(yaws[0] + np.pi)
        if all(abs(wrap(opposite - chosen)) > np.deg2rad(20.0) for chosen in yaws):
            yaws.append(opposite)
    bin_size = 2.0 * extent / n_bin
    found = []
    for yaw in yaws:
        rotated = (yaw_rotation(yaw)[:2, :2] @ scan_xy.T).T
        sino_rot, _angles = radon_occupancy(rotated, n_angle, n_bin, extent)
        shift_corr = np.fft.ifft(
            np.fft.fft(sino_map, axis=1) * np.conj(np.fft.fft(sino_rot, axis=1)),
            axis=1,
        ).real
        shift = np.argmax(shift_corr, axis=1).astype(np.float64)
        shift[shift > n_bin / 2.0] -= n_bin
        support = sino_rot.sum(axis=1)
        weight = np.maximum(shift_corr.max(axis=1), 0.0) * support
        usable = support >= max(float(np.percentile(support, 55.0)), 1.0)
        if int(usable.sum()) < 8:
            usable = support > 0
        if int(usable.sum()) < 8:
            usable = np.ones(n_angle, dtype=bool)
        design = np.column_stack((np.cos(angles[usable]), np.sin(angles[usable])))
        scale = np.sqrt(np.maximum(weight[usable], 0.0))
        if float(scale.sum()) < 1e-6:
            scale = np.ones(int(usable.sum()))
        solved, *_rest = np.linalg.lstsq(design * scale[:, None], shift[usable] * bin_size * scale, rcond=None)
        found.append((float(yaw), float(solved[0]), float(solved[1])))
    return found


def initialize_from_scan_context(scan, body, map_points, z_offsets=(-0.5, 0.7, 1.9)):
    """RTK 只提供位置。用 Scan Context 和分扇区选出航向后再精配准。"""
    position = body[:3, 3].copy()
    local = local_cloud(map_points, position[:2], CONTEXT_RANGE)
    if len(local) < 300:
        raise SystemExit("RTK 位置 60 m 内地图点不足，无法搜索航向")
    tree = cKDTree(local)
    probe = scan[::4]
    relative = (body[:3, :3].T @ (local - position).T).T
    query = scan_context(scan[:, :2], scan[:, 2])
    library = scan_context(relative[:, :2], relative[:, 2])
    sc_distance, yaw_in_body = context_distance(query, library)
    sc_rotation = body[:3, :3] @ yaw_rotation(yaw_in_body)
    sc_yaw = float(np.arctan2(sc_rotation[1, 0], sc_rotation[0, 0]))
    sc_overlap = sector_overlap(probe, np.array([position[0], position[1], sc_yaw, position[2]]), tree)
    best_sector_yaw = 0.0
    best_sector_overlap = -1.0
    for yaw in np.linspace(-np.pi, np.pi, 72, endpoint=False):
        score = sector_overlap(probe, np.array([position[0], position[1], float(yaw), position[2]]), tree)
        if score > best_sector_overlap:
            best_sector_overlap = float(score)
            best_sector_yaw = float(yaw)
    if sc_overlap >= best_sector_overlap:
        chosen_yaw = sc_yaw
        chosen_name = "scan_context"
        chosen_overlap = sc_overlap
    else:
        chosen_yaw = best_sector_yaw
        chosen_name = "sector"
        chosen_overlap = best_sector_overlap
    refined_pose = np.array([position[0], position[1], chosen_yaw, position[2]])
    refined_score = chosen_overlap
    for dx in np.linspace(-6.0, 6.0, 7):
        for dy in np.linspace(-6.0, 6.0, 7):
            for dz in z_offsets:
                for dyaw in (-0.12, 0.0, 0.12):
                    pose = np.array(
                        [position[0] + dx, position[1] + dy, chosen_yaw + dyaw, position[2] + dz]
                    )
                    score = sector_overlap(probe, pose, tree)
                    if score > refined_score:
                        refined_score = float(score)
                        refined_pose = pose
    guess = np.eye(4)
    guess[:3, :3] = yaw_rotation(refined_pose[2])
    guess[:3, 3] = (refined_pose[0], refined_pose[1], refined_pose[3])
    gicp_map = local_cloud(map_points, guess[:2, 3], 45.0)
    current = guess
    last = None
    for distance, iterations in ((8.0, 30), (3.0, 25), (1.0, 20), (0.4, 15)):
        last = small_gicp.align(
            gicp_map,
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
    stats = near_residual(cKDTree(gicp_map), scan, current)
    shift = float(np.linalg.norm(current[:2, 3] - position[:2]))
    info = {
        "method": chosen_name,
        "overlap": chosen_overlap,
        "refined_overlap": refined_score,
        "sc_distance": sc_distance,
        "sector_overlap": best_sector_overlap,
        "shift_xy": shift,
        "converged": bool(last.converged),
        **stats,
    }
    return current, info


def initialize_from_ring(scan, body, map_points, z_offsets=(-0.5, 0.7, 1.9)):
    """RTK 只提供位置。RING++ 求解航向和水平平移，再 small-gicp 精配准。"""
    position = body[:3, 3].copy()
    local = local_cloud(map_points, position[:2], CONTEXT_RANGE)
    if len(local) < 300:
        raise SystemExit("RTK 位置 60 m 内地图点不足，无法搜索航向")
    candidates = ring_candidates(scan[:, :2], local[:, :2] - position[:2])
    tree = cKDTree(local)
    best_rank = None
    guess = np.eye(4)
    guess[:3, 3] = position
    chosen = (0.0, 0.0, 0.0)
    for yaw, dx, dy in candidates:
        translations = [(0.0, 0.0)]
        if float(np.hypot(dx, dy)) <= 8.0:
            translations.append((dx, dy))
        for tx, ty in translations:
            for dz in z_offsets:
                pose = np.eye(4)
                pose[:3, :3] = yaw_rotation(yaw)
                pose[:3, 3] = (position[0] + tx, position[1] + ty, position[2] + dz)
                stats = near_residual(tree, scan, pose)
                rank = (stats["inlier_1"], -stats["median"])
                if best_rank is None or rank > best_rank:
                    best_rank = rank
                    guess = pose
                    chosen = (yaw, tx, ty)
    gicp_map = local_cloud(map_points, position[:2], 45.0)
    current = guess
    last = None
    for distance, iterations in ((8.0, 30), (3.0, 25), (1.0, 20), (0.4, 15)):
        last = small_gicp.align(
            gicp_map,
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
    anchored = np.array(guess, copy=True)
    anchored[0, 3] = position[0]
    anchored[1, 3] = position[1]
    anchored_pose = anchored
    for distance, iterations in ((8.0, 30), (3.0, 25), (1.0, 20), (0.4, 15)):
        aligned = small_gicp.align(
            gicp_map,
            scan,
            anchored_pose,
            registration_type="GICP",
            voxel_resolution=0.4,
            downsampling_resolution=0.4,
            max_correspondence_distance=distance,
            num_threads=8,
            max_iterations=iterations,
            verbose=False,
        )
        anchored_pose = np.asarray(aligned.T_target_source, dtype=np.float64)
    stats = near_residual(cKDTree(gicp_map), scan, current)
    anchored_stats = near_residual(tree, scan, anchored_pose)
    if (anchored_stats["inlier_1"], -anchored_stats["median"]) > (stats["inlier_1"], -stats["median"]):
        current = anchored_pose
        stats = anchored_stats
        chosen = (chosen[0], 0.0, 0.0)
    info = {
        "method": "ring++",
        "overlap": stats["inlier_1"],
        "refined_overlap": stats["inlier_1"],
        "sc_distance": 0.0,
        "sector_overlap": 0.0,
        "ring_yaw": float(chosen[0]),
        "ring_xy": (float(chosen[1]), float(chosen[2])),
        "shift_xy": float(np.linalg.norm(current[:2, 3] - position[:2])),
        "converged": bool(last.converged),
        **stats,
    }
    return current, info


def initialize_from_rtk(scan, body, map_points, z_offsets=(-0.5, 0.7, 1.9)):
    """在线起点仍用 Scan Context 和扇区 overlap。RING++ 见 initialize_from_ring。"""
    return initialize_from_scan_context(scan, body, map_points, z_offsets)


def read_scans(mcap: Path, topic: str, limit: int, skip: int = 0):
    reader = open_reader(mcap, topic)
    scans = []
    skipped = 0
    while reader.has_next() and len(scans) < limit:
        _topic, raw, _stamp = reader.read_next()
        scan = read_scan(raw)
        if scan is None:
            continue
        if skipped < skip:
            skipped += 1
            continue
        scans.append((scan[0], voxel_down(scan[1], SCAN_VOXEL)))
    return scans


def initialize_lidar(name, scans, rtk, map_points, prefer=None):
    print(f"\n===== {name} RTK 位置 + Scan Context 航向搜索 =====", flush=True)
    for stamp, scan in scans:
        body = body_pose(rtk, stamp)
        if body is None:
            continue
        started = time.perf_counter()
        pose, info = initialize_from_rtk(scan, body, map_points)
        print(
            f"  stamp={stamp:.3f} {info['method']} overlap={info['overlap']:.3f} "
            f"refined={info['refined_overlap']:.3f} sc_dist={info['sc_distance']:.3f} "
            f"sector={info['sector_overlap']:.3f}",
            flush=True,
        )
        print(
            f"  small_gicp converged={info['converged']} shift_xy={info['shift_xy']:.2f} m "
            f"near30={info['median']:.3f} <0.3={info['inlier_0_3']:.3f} <1={info['inlier_1']:.3f} "
            f"elapsed={time.perf_counter() - started:.1f}s",
            flush=True,
        )
        if info["median"] > 0.40 or info["inlier_1"] < 0.85:
            print("  这一帧没有锁定，继续尝试下一帧", flush=True)
            continue
        if prefer is not None and prefer(stamp) is None:
            print("  这一帧没有对应的前向雷达位姿", flush=True)
            continue
        return stamp, pose
    raise SystemExit(f"{name} 没能用 RTK 和 Scan Context 得到起点")


def nearest_pose(stamps, poses, stamp, gap=0.50):
    index = int(np.searchsorted(stamps, stamp))
    best = None
    best_gap = gap
    for candidate in (index - 1, index):
        if candidate < 0 or candidate >= len(stamps):
            continue
        delta = abs(float(stamps[candidate]) - stamp)
        if delta <= best_gap:
            best_gap = delta
            best = poses[candidate]
    return best


def calibrate_rtk(stamps, poses, rtk):
    samples = []
    for stamp, pose in zip(stamps, poses, strict=True):
        body = body_pose(rtk, stamp)
        if body is None:
            continue
        samples.append(np.linalg.inv(pose) @ body)
    return robust_pose(samples)


def pose_matrix(pose: dict):
    transform = rpy_matrix(pose["roll"], pose["pitch"], pose["yaw"])
    transform[:3, 3] = (pose["x"], pose["y"], pose["z"])
    return transform


def invert_pose(pose: dict):
    inverse = np.linalg.inv(pose_matrix(pose))
    roll, pitch, yaw = rotation_to_rpy(inverse[:3, :3])
    return {
        "x": float(inverse[0, 3]),
        "y": float(inverse[1, 3]),
        "z": float(inverse[2, 3]),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "count": pose["count"],
        "spread": pose["spread"],
    }


def append_pose(lines, key, pose):
    lines.append(f"{key}:")
    for field in ("x", "y", "z", "roll", "pitch", "yaw"):
        lines.append(f"  {field}: {pose[field]:.10f}")
    lines.append("")


def write_trajectory(path: Path, stamps, poses, origin, frame="与地图相同的投影坐标"):
    lines = [
        "# 基准雷达相对地图的逐帧匹配轨迹。外参以这颗雷达为父坐标系。",
        f"# 坐标：{frame}。单位 m / rad，RPY 为固定轴 XYZ。",
        f"# 匹配时减去的局部原点 {origin[0]:.3f} {origin[1]:.3f} {origin[2]:.3f}",
        "# stamp x y z roll pitch yaw",
    ]
    for stamp, pose in zip(stamps, poses, strict=True):
        roll, pitch, yaw = rotation_to_rpy(pose[:3, :3])
        translation = pose[:3, 3] + origin
        lines.append(
            f"{stamp:.9f} {translation[0]:.6f} {translation[1]:.6f} {translation[2]:.6f} "
            f"{roll:.8f} {pitch:.8f} {yaw:.8f}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {path}", flush=True)


def write_extrinsic(path: Path, side: dict, rtk_pose: dict | None, span: float):
    lines = [
        "# 父坐标系是前向 long 雷达 hesai_lidar_left_front。",
        "# 单位 m / rad，RPY 为固定轴 XYZ。",
        "# RTK 位置是接收机输出的后轴中心，没有再加天线杆臂。",
        f"# RTK 航向变化 {span:.1f} deg。",
        "",
        f"# long_right_end 相对 long_left_front。使用帧 {side['count']}，平移离散度 {side['spread']:.3f} m。",
    ]
    append_pose(lines, "hesai_lidar_right_end_hesai_lidar_left_front", side)
    if rtk_pose is not None:
        lines.append(
            f"# RTK 后轴中心在前向 long 雷达坐标系下。使用帧 {rtk_pose['count']}，"
            f"平移离散度 {rtk_pose['spread']:.3f} m。"
        )
        append_pose(lines, "rtk_hesai_lidar_left_front", rtk_pose)
        inverse = invert_pose(rtk_pose)
        lines.append("# 同一关系的逆：前向 long 雷达在 RTK 后轴坐标系下。")
        append_pose(lines, "hesai_lidar_left_front_rtk", inverse)
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {path}", flush=True)


def prepare_map(args, rtk):
    xyz = rtk["xyz"]
    bounds = (
        float(xyz[:, 0].min() - MAP_CROP_MARGIN),
        float(xyz[:, 0].max() + MAP_CROP_MARGIN),
        float(xyz[:, 1].min() - MAP_CROP_MARGIN),
        float(xyz[:, 1].max() + MAP_CROP_MARGIN),
    )
    print(
        f"loading map crop x=({bounds[0]:.1f}, {bounds[1]:.1f}) y=({bounds[2]:.1f}, {bounds[3]:.1f})",
        flush=True,
    )
    cloud = load_pcd_crop(args.map, bounds, 0.30)
    origin = np.array([float(xyz[:, 0].mean()), float(xyz[:, 1].mean()), 0.0])
    cloud = cloud - origin
    rtk["xyz"] = xyz - origin
    print(f"map points={len(cloud)} local_origin=({origin[0]:.3f}, {origin[1]:.3f}, 0)", flush=True)
    return cloud, origin


def run_without_start(args):
    if args.map.suffix.lower() != ".pcd":
        raise SystemExit("没有起点时地图需要是绝对坐标 PCD")
    rtk = read_rtk(args.mcap)
    span = yaw_span(rtk)
    print(f"rtk yaw span={span:.1f} deg", flush=True)
    map_points, origin = prepare_map(args, rtk)
    reference_scans = read_scans(args.mcap, TOPICS[REFERENCE], 5)
    _stamp, start = initialize_lidar(REFERENCE, reference_scans, rtk, map_points)
    stamps, poses = match_reference(args.mcap, map_points, start)
    args.trajectory.parent.mkdir(parents=True, exist_ok=True)
    write_trajectory(args.trajectory, stamps, poses, origin)
    side_scans = read_scans(args.mcap, TOPICS[SIDE], 6)

    def side_time_gap(stamp):
        index = int(np.searchsorted(stamps, stamp))
        gaps = []
        for candidate in (index - 1, index):
            if 0 <= candidate < len(stamps):
                gaps.append(abs(float(stamps[candidate]) - stamp))
        if not gaps or min(gaps) > 0.20:
            return None
        return min(gaps)

    side_stamp, side_pose = initialize_lidar(SIDE, side_scans, rtk, map_points, side_time_gap)
    reference_pose = nearest_pose(stamps, poses, side_stamp, gap=0.20)
    if reference_pose is None:
        raise SystemExit("右雷达起点时刻没有前向雷达轨迹")
    prior = np.linalg.inv(reference_pose) @ side_pose
    roll, pitch, yaw = rotation_to_rpy(prior[:3, :3])
    print(
        f"initialized extrinsic t=({prior[0, 3]:.3f}, {prior[1, 3]:.3f}, {prior[2, 3]:.3f}) "
        f"rpy_deg=({np.degrees(roll):.2f}, {np.degrees(pitch):.2f}, {np.degrees(yaw):.2f})",
        flush=True,
    )
    relatives = match_side(args.mcap, map_points, stamps, poses, prior)
    estimate = robust_pose(relatives)
    if estimate is None:
        raise SystemExit("右雷达有效匹配不足，外参未写出。前向雷达轨迹已经保存。")
    correction = np.linalg.inv(prior) @ pose_matrix(estimate)
    angle = float(np.degrees(np.arccos(np.clip((np.trace(correction[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))))
    print(
        f"{SIDE}: used={estimate['count']} "
        f"t=({estimate['x']:.3f}, {estimate['y']:.3f}, {estimate['z']:.3f}) "
        f"rpy_deg=({np.degrees(estimate['roll']):.2f}, {np.degrees(estimate['pitch']):.2f}, {np.degrees(estimate['yaw']):.2f}) "
        f"spread={estimate['spread']:.3f} m",
        flush=True,
    )
    print(
        f"correction vs initialized extrinsic: t=({correction[0, 3]:.3f}, {correction[1, 3]:.3f}, {correction[2, 3]:.3f}) m "
        f"angle={angle:.3f} deg",
        flush=True,
    )
    rtk_pose = calibrate_rtk(stamps, poses, rtk)
    if rtk_pose is None:
        print("前向雷达与 RTK 的有效样本不足", flush=True)
    else:
        inverse = invert_pose(rtk_pose)
        print(
            f"rtk in {REFERENCE}: used={rtk_pose['count']} "
            f"t=({rtk_pose['x']:.3f}, {rtk_pose['y']:.3f}, {rtk_pose['z']:.3f}) "
            f"rpy_deg=({np.degrees(rtk_pose['roll']):.2f}, {np.degrees(rtk_pose['pitch']):.2f}, {np.degrees(rtk_pose['yaw']):.2f}) "
            f"spread={rtk_pose['spread']:.3f} m",
            flush=True,
        )
        print(
            f"{REFERENCE} in rtk: t=({inverse['x']:.3f}, {inverse['y']:.3f}, {inverse['z']:.3f}) "
            f"rpy_deg=({np.degrees(inverse['roll']):.2f}, {np.degrees(inverse['pitch']):.2f}, {np.degrees(inverse['yaw']):.2f})",
            flush=True,
        )
    args.extrinsic.parent.mkdir(parents=True, exist_ok=True)
    write_extrinsic(args.extrinsic, estimate, rtk_pose, span)


def run_with_start(args):
    starts = load_start_poses(args.start)
    prior = np.linalg.inv(starts[REFERENCE]) @ starts[SIDE]
    roll, pitch, yaw = rotation_to_rpy(prior[:3, :3])
    print(
        f"start extrinsic t=({prior[0, 3]:.3f}, {prior[1, 3]:.3f}, {prior[2, 3]:.3f}) "
        f"rpy_deg=({np.degrees(roll):.2f}, {np.degrees(pitch):.2f}, {np.degrees(yaw):.2f})",
        flush=True,
    )
    print("loading map", flush=True)
    map_points = voxel_down(load_las(args.map), MAP_VOXEL)
    print(f"map points={len(map_points)}", flush=True)
    stamps, poses = match_reference(args.mcap, map_points, starts[REFERENCE])
    args.trajectory.parent.mkdir(parents=True, exist_ok=True)
    write_trajectory(args.trajectory, stamps, poses, np.zeros(3))
    relatives = match_side(args.mcap, map_points, stamps, poses, prior)
    estimate = robust_pose(relatives)
    if estimate is None:
        raise SystemExit("有效匹配不足，外参未写出。前向雷达轨迹已经保存。")
    correction = np.linalg.inv(prior) @ pose_matrix(estimate)
    angle = float(np.degrees(np.arccos(np.clip((np.trace(correction[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))))
    print(
        f"{SIDE}: used={estimate['count']} "
        f"t=({estimate['x']:.3f}, {estimate['y']:.3f}, {estimate['z']:.3f}) "
        f"rpy_deg=({np.degrees(estimate['roll']):.2f}, {np.degrees(estimate['pitch']):.2f}, {np.degrees(estimate['yaw']):.2f}) "
        f"spread={estimate['spread']:.3f} m",
        flush=True,
    )
    print(
        f"correction vs start extrinsic: t=({correction[0, 3]:.3f}, {correction[1, 3]:.3f}, {correction[2, 3]:.3f}) m "
        f"angle={angle:.3f} deg",
        flush=True,
    )
    args.extrinsic.parent.mkdir(parents=True, exist_ok=True)
    write_extrinsic(args.extrinsic, estimate, None, 0.0)


def main():
    parser = argparse.ArgumentParser(description="以前向 long 雷达为基准标定 long_right_end 和 RTK")
    parser.add_argument("--mcap", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--start", type=Path, help="两颗雷达的 4x4 起点。省略时用 RTK 和 Scan Context")
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--extrinsic", type=Path, required=True)
    args = parser.parse_args()
    if not args.mcap.is_file():
        raise SystemExit(f"找不到 mcap: {args.mcap}")
    if not args.map.is_file():
        raise SystemExit(f"找不到地图: {args.map}")
    if args.start is None:
        run_without_start(args)
    else:
        run_with_start(args)


if __name__ == "__main__":
    main()
