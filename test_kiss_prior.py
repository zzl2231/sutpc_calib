#!/usr/bin/env python3
"""RTK 可靠、不用先验外参时，比较 KISS-Matcher 和现有起点搜索。

两套起点都只使用地图和 RTK 位置。外参只在事后比较相对基准雷达的坐标变换。
"""

import sys
import time
from pathlib import Path

import numpy as np

import long_pair_calib as pair
import map_calib

MCAP = Path("/bag/cityloong_calib_data_1/lidar_gnss_20261110_171441_0.mcap")
MAP = Path("/bag/cityloong_calib_data_1/map_20261022_morning_2_traffic_signs")
EXTRINSIC = Path("/data/chengjiaolong/calib/lidar_extrinsic_accurate.yaml")
REFERENCE = "hesai_lidar_front_mid_top"
FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


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
    t = transform[:3, 3]
    return (
        f"t=({t[0]: .3f}, {t[1]: .3f}, {t[2]: .3f}) "
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


def local_map(cloud, xy, radius):
    delta = cloud[:, :2] - xy
    return cloud[np.einsum("ij,ij->i", delta, delta) <= radius ** 2]


def kiss_solution(scan, target, voxel):
    import kiss_matcher as km

    matcher = km.KISSMatcher(km.KISSMatcherConfig(voxel))
    solution = matcher.estimate(
        np.ascontiguousarray(scan, dtype=np.float32),
        np.ascontiguousarray(target, dtype=np.float32),
    )
    pose = np.eye(4)
    pose[:3, :3] = np.asarray(solution.rotation, dtype=np.float64)
    pose[:3, 3] = np.asarray(solution.translation, dtype=np.float64)
    return pose, bool(solution.valid)


def kiss_match(scan, target):
    return kiss_solution(scan, target, 0.40)


def gicp_refine(scan, guess, cloud):
    current = guess
    last = None
    for distance, iterations in ((8.0, 30), (3.0, 25), (1.0, 20), (0.4, 15)):
        last = pair.small_gicp.align(
            cloud,
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
    return current, bool(last.converged)


def kiss_around_rtk(scan, body, cloud):
    """RTK 位置可靠。航向采样后用 KISS-Matcher 修正，只保留落在 RTK 附近的结果，再 GICP。"""
    position = body[:3, 3]
    target = local_map(cloud, position[:2], 45.0)
    radius = np.linalg.norm(scan, axis=1)
    near = scan[radius <= 40.0]
    if len(near) < 800:
        near = scan
    tree = pair.cKDTree(target)
    best = None
    for yaw in np.linspace(-np.pi, np.pi, 12, endpoint=False):
        guess = np.eye(4)
        guess[:3, :3] = pair.yaw_rotation(float(yaw))
        guess[:3, 3] = position
        candidates = [("yaw", guess)]
        placed = (guess[:3, :3] @ near.T).T + guess[:3, 3]
        delta, valid = kiss_match(placed, target)
        correction = rotation_angle(delta[:3, :3])
        moved = float(np.linalg.norm(delta[:3, 3]))
        if valid and correction <= 75.0 and moved <= 8.0:
            candidates.append(("kiss", delta @ guess))
        for source, pose in candidates:
            shift = float(np.linalg.norm(pose[:2, 3] - position[:2]))
            if shift > 8.0:
                continue
            fit = pair.near_residual(tree, near, pose)
            rank = (fit["inlier_1"], -fit["median"])
            if best is None or rank > best[0]:
                best = (rank, pose, valid, source, shift, float(yaw))
    if best is None:
        guess = np.eye(4)
        guess[:3, 3] = position
        pose, _converged = gicp_refine(near, guess, target)
        fit = pair.near_residual(tree, near, pose)
        return pose, False, fit, float(np.linalg.norm(pose[:2, 3] - position[:2])), 0.0, len(target), "none"
    _rank, pose, valid, source, shift, yaw = best
    refined, _converged = gicp_refine(near, pose, target)
    refined_shift = float(np.linalg.norm(refined[:2, 3] - position[:2]))
    if refined_shift <= 8.0:
        refined_fit = pair.near_residual(tree, near, refined)
        if refined_fit["inlier_1"] >= best[0][0] - 0.02:
            pose, fit, shift = refined, refined_fit, refined_shift
            source = f"{source}+gicp"
        else:
            fit = pair.near_residual(tree, near, pose)
    else:
        fit = pair.near_residual(tree, near, pose)
    return pose, valid, fit, shift, yaw, len(target), source


def check_kiss_pose():
    grid = np.linspace(-8.0, 8.0, 40)
    xx, yy = np.meshgrid(grid, grid)
    ground = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size)))
    wall = np.column_stack((np.full(xx.size, 6.0), xx.ravel(), yy.ravel() * 0.4 + 2.0))
    source = np.vstack((ground, wall))
    yaw = 1.2
    rotation = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    )
    translation = np.array([12.0, -4.0, 1.5])
    truth = np.eye(4)
    truth[:3, :3] = rotation
    truth[:3, 3] = translation
    target = (rotation @ source.T).T + translation
    pose, valid = kiss_solution(source, target, 0.30)
    error = pose_delta(pose, truth)
    print(
        f"KISS 自检 valid={valid} 平移 {error['t']:.3f} m 旋转 {error['angle']:.2f} deg",
        flush=True,
    )
    if error["t"] > 0.5 or error["angle"] > 2.0:
        print("平面合成数据有对称歧义，改用实车点云判断", flush=True)


def existing_match(scan, body, cloud):
    pose, info = pair.initialize_from_rtk(scan, body, cloud, (-0.5, 0.7, 1.9))
    return pose, info


def main():
    import kiss_matcher as km

    print(f"kiss_matcher {getattr(km, '__version__', '')}", flush=True)
    check_kiss_pose()
    poses = map_calib.load_extrinsic_yaml(EXTRINSIC)
    if f"{REFERENCE}_base_link" not in poses:
        raise SystemExit(f"外参里没有基准 {REFERENCE}_base_link")

    def status(text):
        print(text, flush=True)

    scene = map_calib.load_scene(MCAP, MAP, status)
    rtk = scene["rtk"]
    wanted = []
    for topic in scene["topics"]:
        frame = topic.get("frame_id") or topic.get("frame")
        key = f"{frame}_base_link"
        if key in poses:
            wanted.append((topic["name"], frame, key))
    if not any(frame == REFERENCE for _topic, frame, _key in wanted):
        raise SystemExit("数据包里没有基准雷达，或外参名没有配上")
    print(f"比较 {len(wanted)} 颗有外参的雷达，基准 {REFERENCE}", flush=True)

    matched = {}
    for topic, frame, key in wanted:
        stamp, scan, body = first_scan(MCAP, topic, rtk)
        print(f"\n{frame} stamp={stamp:.3f} scan={len(scan)}", flush=True)
        started = time.perf_counter()
        old_pose, old_info = existing_match(scan, body, scene["map"])
        old_seconds = time.perf_counter() - started
        started = time.perf_counter()
        kiss_pose, valid, kiss_fit, shift, yaw, target_count, source = kiss_around_rtk(
            scan, body, scene["map"]
        )
        kiss_seconds = time.perf_counter() - started
        truth = pair.pose_matrix(poses[key])
        old_body = np.linalg.inv(body) @ old_pose
        kiss_body = np.linalg.inv(body) @ kiss_pose
        print(
            f"  现有方法 {old_seconds:.1f}s median={old_info['median']:.3f} "
            f"inlier<1={old_info['inlier_1']:.3f} yaw={old_info['method']}",
            flush=True,
        )
        print(
            f"  KISS {kiss_seconds:.1f}s {source} valid={valid} map={target_count} "
            f"shift={shift:.2f} m seed_yaw={np.degrees(yaw):.0f} deg "
            f"median={kiss_fit['median']:.3f} inlier<1={kiss_fit['inlier_1']:.3f}",
            flush=True,
        )
        for name, estimate in (("现有方法", old_body), ("KISS", kiss_body)):
            error = pose_delta(estimate, truth)
            print(
                f"  {name} 相对 base_link {text_pose(estimate)}",
                flush=True,
            )
            print(
                f"    与外参差 {error['t']:.3f} m，{error['angle']:.2f} deg",
                flush=True,
            )
        matched[frame] = {
            "truth": truth,
            "old": old_body,
            "kiss": kiss_body,
        }

    truth_ref = matched[REFERENCE]["truth"]
    print("\n===== 相对基准雷达，与外参对比 =====", flush=True)
    print(f"外参 {EXTRINSIC}", flush=True)
    for frame, item in matched.items():
        if frame == REFERENCE:
            continue
        truth = np.linalg.inv(truth_ref) @ item["truth"]
        print(f"\n{frame} 相对 {REFERENCE}", flush=True)
        print(f"  外参 {text_pose(truth)}", flush=True)
        for name, key in (("现有方法", "old"), ("KISS", "kiss")):
            estimate = np.linalg.inv(matched[REFERENCE][key]) @ item[key]
            error = pose_delta(estimate, truth)
            print(f"  {name} {text_pose(estimate)}", flush=True)
            print(
                f"    差 {error['t']:.3f} m / {error['angle']:.2f} deg  "
                f"dt=({error['dx']:+.3f}, {error['dy']:+.3f}, {error['dz']:+.3f}) "
                f"drpy=({error['roll']:+.2f}, {error['pitch']:+.2f}, {error['yaw']:+.2f}) deg",
                flush=True,
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
