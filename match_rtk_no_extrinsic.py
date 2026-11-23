#!/usr/bin/env python3
"""初值只用 RTK 位姿，不乘外参。Scan Context 或分扇区确定航向后再 small-gicp。"""

import numpy as np
import small_gicp
from scipy.spatial import cKDTree

import match_rtk_prior as base
import relocalize_scancontext as sc

EXTRINSIC = base.EXTRINSIC.copy()


def rz(yaw):
    transform = np.eye(4)
    transform[0, 0] = transform[1, 1] = np.cos(yaw)
    transform[0, 1] = -np.sin(yaw)
    transform[1, 0] = np.sin(yaw)
    return transform


def yaw_of(rotation):
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def pose_from_yaw(position, yaw):
    transform = rz(yaw)
    transform[:3, 3] = position
    return transform


def near_stats(tree, points, scan):
    stats = base.residual(tree, points)
    chosen = np.linalg.norm(scan, axis=1) < 30.0
    near = base.residual(tree, points[chosen]) if np.any(chosen) else stats
    return stats, near


def gicp(local, scan, guess):
    current = guess
    last = None
    for distance, iterations in ((8.0, 30), (3.0, 25), (1.0, 20), (0.4, 15)):
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
    return current, bool(last.converged), int(last.iterations)


def main():
    fix_t, rtk_xyz, pose_t, pose_q, frames = base.read_frames()
    extrinsic = base.rpy_matrix(EXTRINSIC[3], EXTRINSIC[4], EXTRINSIC[5])
    extrinsic[:3, 3] = EXTRINSIC[:3]
    usable = []
    bodies = []
    for stamp, scan in frames:
        body = base.body_pose(fix_t, rtk_xyz, pose_t, pose_q, stamp)
        if body is None:
            continue
        usable.append((stamp, scan))
        bodies.append(body)
    centers = np.vstack([body[:2, 3] for body in bodies])
    bounds = (
        float(centers[:, 0].min() - 50.0),
        float(centers[:, 0].max() + 50.0),
        float(centers[:, 1].min() - 50.0),
        float(centers[:, 1].max() + 50.0),
    )
    print("loading map crop", flush=True)
    cloud = base.load_crop(bounds)
    print(f"map_crop={len(cloud)}", flush=True)
    local_origin = np.array([centers[:, 0].mean(), centers[:, 1].mean(), 0.0])
    cloud_local = cloud - local_origin
    ok = 0
    for (stamp, scan), body in zip(usable, bodies):
        truth = body @ extrinsic
        true_yaw = yaw_of(truth[:3, :3])
        position = body[:3, 3].copy()
        local = sc.local_cloud(cloud, position[:2], 60.0)
        tree = cKDTree(local)
        probe = scan[::4]
        relative = (body[:3, :3].T @ (local - position).T).T
        query = sc.scan_context(scan[:, :2], scan[:, 2], 0.0)
        library = sc.scan_context(relative[:, :2], relative[:, 2], 0.0)
        sc_distance, yaw_in_body = sc.context_distance(query, library)
        sc_rotation = body[:3, :3] @ rz(yaw_in_body)[:3, :3]
        sc_yaw = yaw_of(sc_rotation)
        sc_overlap = sc.sector_overlap(probe, np.array([position[0], position[1], sc_yaw, position[2]]), tree)

        best_sector_yaw = 0.0
        best_sector_overlap = -1.0
        for yaw in np.linspace(-np.pi, np.pi, 72, endpoint=False):
            score = sc.sector_overlap(probe, np.array([position[0], position[1], yaw, position[2]]), tree)
            if score > best_sector_overlap:
                best_sector_overlap = score
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
                for dz in (-0.5, 0.7, 1.9):
                    for dyaw in (-0.12, 0.0, 0.12):
                        pose = np.array(
                            [position[0] + dx, position[1] + dy, chosen_yaw + dyaw, position[2] + dz]
                        )
                        score = sc.sector_overlap(probe, pose, tree)
                        if score > refined_score:
                            refined_score = score
                            refined_pose = pose
        guess = pose_from_yaw(np.array([refined_pose[0], refined_pose[1], refined_pose[3]]), refined_pose[2])
        guess_local = guess.copy()
        guess_local[:3, 3] -= local_origin
        center = guess_local[:2, 3]
        delta = cloud_local[:, :2] - center
        gicp_map = cloud_local[np.einsum("ij,ij->i", delta, delta) <= 45.0 ** 2]
        refined, converged, iterations = gicp(gicp_map, scan, guess_local)
        matched = refined[:3, 3] + local_origin
        matched_yaw = yaw_of(refined[:3, :3])
        eval_tree = cKDTree(gicp_map + local_origin)
        before_points = (guess[:3, :3] @ scan.T).T + guess[:3, 3]
        after_points = (refined[:3, :3] @ scan.T).T + matched
        raw_points = (body[:3, :3] @ scan.T).T + body[:3, 3]
        raw_tree = cKDTree(local)
        raw_stats, raw_near = near_stats(raw_tree, raw_points, scan)
        before_stats, before_near = near_stats(eval_tree, before_points, scan)
        after_stats, after_near = near_stats(eval_tree, after_points, scan)
        xy_error = float(np.linalg.norm(matched[:2] - truth[:2, 3]))
        yaw_error = abs(np.degrees(wrap(matched_yaw - true_yaw)))
        locked = after_near["median"] < 0.30 and after_near["inlier_1"] > 0.9 and xy_error < 1.5 and yaw_error < 10.0
        ok += int(locked)
        print(
            f"\nstamp={stamp:.3f}\n"
            f"  rtk_only near30={raw_near['median']:.3f} <0.3={raw_near['inlier_0_3']:.3f}\n"
            f"  {chosen_name} overlap={chosen_overlap:.3f} sc_dist={sc_distance:.3f} "
            f"sector_overlap={best_sector_overlap:.3f} refined_overlap={refined_score:.3f}\n"
            f"  small_gicp converged={converged} iters={iterations} "
            f"near30={after_near['median']:.3f} <0.3={after_near['inlier_0_3']:.3f} <1={after_near['inlier_1']:.3f}\n"
            f"  before_near30={before_near['median']:.3f} xy_err={xy_error:.2f} m yaw_err={yaw_error:.1f} deg "
            f"shift_xy={np.linalg.norm(matched[:2] - guess[:2, 3]):.2f} m "
            f"{'锁定' if locked else '未锁定'}",
            flush=True,
        )
        del before_stats, after_stats
    print(f"\nlocked {ok}/{len(usable)}", flush=True)


if __name__ == "__main__":
    main()
