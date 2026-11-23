#!/usr/bin/env python3
"""不用先验外参，比较 RING++ 起点和原来的 Scan Context / 扇区 overlap。

外参只在事后比较相对基准雷达的坐标变换。
"""

import time
from pathlib import Path

import numpy as np

import long_pair_calib as pair
import map_calib

MCAP = Path("/bag/cityloong_calib_data_1/lidar_gnss_20261110_171441_0.mcap")
MAP = Path("/bag/cityloong_calib_data_1/map_20261022_morning_2_traffic_signs")
EXTRINSIC = Path("/data/chengjiaolong/calib/lidar_extrinsic_accurate.yaml")
REFERENCE = "hesai_lidar_front_mid_top"


def rotation_angle(rotation):
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def pose_delta(estimated, truth):
    delta = np.linalg.inv(truth) @ estimated
    roll, pitch, yaw = pair.rotation_to_rpy(delta[:3, :3])
    return {
        "t": float(np.linalg.norm(delta[:3, 3])),
        "angle": rotation_angle(delta[:3, :3]),
        "dx": float(delta[0, 3]),
        "dy": float(delta[1, 3]),
        "dz": float(delta[2, 3]),
        "roll": float(np.degrees(roll)),
        "pitch": float(np.degrees(pitch)),
        "yaw": float(np.degrees(yaw)),
    }


def text_pose(transform):
    roll, pitch, yaw = pair.rotation_to_rpy(transform[:3, :3])
    translation = transform[:3, 3]
    return (
        f"t=({translation[0]: .3f}, {translation[1]: .3f}, {translation[2]: .3f}) "
        f"rpy_deg=({np.degrees(roll): .2f}, {np.degrees(pitch): .2f}, {np.degrees(yaw): .2f})"
    )


def keep_range(points):
    finite = np.isfinite(points).all(axis=1)
    radius = np.linalg.norm(points, axis=1)
    return points[finite & (radius >= 2.0) & (radius <= 50.0)]


def first_scan(mcap, topic, rtk):
    reader = pair.open_reader(mcap, topic)
    while reader.has_next():
        _topic, raw, _stamp = reader.read_next()
        scan = pair.read_scan(raw)
        if scan is None:
            continue
        stamp, points = scan
        body = pair.body_pose(rtk, float(stamp))
        if body is None:
            continue
        cloud = keep_range(np.asarray(points, dtype=np.float64))
        if len(cloud) < 1000:
            continue
        return float(stamp), pair.voxel_down(cloud, 0.30), body
    raise SystemExit(f"{topic} 没有和 RTK 同步的点云")


def check_ring():
    line = np.linspace(-20.0, 20.0, 500)
    walls = np.vstack((
        np.column_stack((line, np.full(line.size, 8.0))),
        np.column_stack((np.full(400, -12.0), np.linspace(-15.0, 18.0, 400))),
        np.column_stack((np.linspace(-5.0, 16.0, 250), np.linspace(-18.0, -4.0, 250))),
    ))
    yaw = 1.15
    translation = np.array([3.5, -2.2])
    mapped = (pair.yaw_rotation(yaw)[:2, :2] @ walls.T).T + translation
    found = pair.ring_candidates(walls, mapped)
    best = min(
        found,
        key=lambda item: abs(pair.wrap(item[0] - yaw)) + 0.2 * np.hypot(item[1] - translation[0], item[2] - translation[1]),
    )
    yaw_error = abs(np.degrees(pair.wrap(best[0] - yaw)))
    xy_error = float(np.hypot(best[1] - translation[0], best[2] - translation[1]))
    print(
        f"RING 自检 yaw {np.degrees(best[0]):.2f} deg（真值 {np.degrees(yaw):.2f}）"
        f" 平移 ({best[1]:.2f}, {best[2]:.2f}) 误差 {yaw_error:.2f} deg / {xy_error:.2f} m",
        flush=True,
    )
    if yaw_error > 2.0 or xy_error > 0.5:
        raise SystemExit("RING++ 没有恢复已知平面刚体变换")


def main():
    check_ring()
    poses = map_calib.load_extrinsic_yaml(EXTRINSIC)
    scene = map_calib.load_scene(MCAP, MAP, print)
    wanted = []
    for topic in scene["topics"]:
        frame = topic.get("frame_id") or topic.get("frame")
        key = f"{frame}_base_link"
        if key in poses:
            wanted.append((topic["name"], frame, key))
    if not any(frame == REFERENCE for _topic, frame, _key in wanted):
        raise SystemExit(f"数据包里没有基准 {REFERENCE}")
    print(f"比较 {len(wanted)} 颗有外参的雷达，基准 {REFERENCE}", flush=True)
    matched = {}
    for topic, frame, key in wanted:
        stamp, scan, body = first_scan(MCAP, topic, scene["rtk"])
        print(f"\n{frame} stamp={stamp:.3f} scan={len(scan)}", flush=True)
        started = time.perf_counter()
        old_pose, old_info = pair.initialize_from_scan_context(scan, body, scene["map"])
        old_seconds = time.perf_counter() - started
        started = time.perf_counter()
        ring_pose, ring_info = pair.initialize_from_ring(scan, body, scene["map"])
        ring_seconds = time.perf_counter() - started
        truth = pair.pose_matrix(poses[key])
        old_body = np.linalg.inv(body) @ old_pose
        ring_body = np.linalg.inv(body) @ ring_pose
        print(
            f"  Scan Context {old_seconds:.1f}s {old_info['method']} "
            f"median={old_info['median']:.3f} inlier<1={old_info['inlier_1']:.3f}",
            flush=True,
        )
        print(
            f"  RING++ {ring_seconds:.1f}s median={ring_info['median']:.3f} "
            f"inlier<1={ring_info['inlier_1']:.3f} "
            f"yaw={np.degrees(ring_info['ring_yaw']):.1f} "
            f"xy=({ring_info['ring_xy'][0]:.2f}, {ring_info['ring_xy'][1]:.2f})",
            flush=True,
        )
        for name, estimate in (("Scan Context", old_body), ("RING++", ring_body)):
            error = pose_delta(estimate, truth)
            print(f"  {name} 相对 base_link {text_pose(estimate)}", flush=True)
            print(f"    与外参差 {error['t']:.3f} m，{error['angle']:.2f} deg", flush=True)
        matched[frame] = {"truth": truth, "old": old_body, "ring": ring_body}
    truth_ref = matched[REFERENCE]["truth"]
    print("\n===== 相对基准雷达，与外参对比 =====", flush=True)
    for frame, item in matched.items():
        if frame == REFERENCE:
            continue
        truth = np.linalg.inv(truth_ref) @ item["truth"]
        print(f"\n{frame} 相对 {REFERENCE}", flush=True)
        print(f"  外参 {text_pose(truth)}", flush=True)
        for name, key in (("Scan Context", "old"), ("RING++", "ring")):
            estimate = np.linalg.inv(matched[REFERENCE][key]) @ item[key]
            error = pose_delta(estimate, truth)
            print(f"  {name} {text_pose(estimate)}", flush=True)
            print(
                f"    差 {error['t']:.3f} m / {error['angle']:.2f} deg  "
                f"dt=({error['dx']:+.3f}, {error['dy']:+.3f}, {error['dz']:+.3f}) "
                f"drpy=({error['roll']:+.2f}, {error['pitch']:+.2f}, {error['yaw']:+.2f}) deg",
                flush=True,
            )


if __name__ == "__main__":
    main()
