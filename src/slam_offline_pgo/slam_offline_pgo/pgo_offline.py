"""
pgo_offline.py — Master orchestration script for the offline PGO pipeline.

Stages
------
1. Load or extract keyframes        (io_bag)
2. Build Scan Context database      (scan_context)
3. Detect + verify loop closures    (loop_detector)
4. Build Open3D pose graph          (odom edges + loop edges)
5. Optimise with Levenberg-Marquardt
6. Write TUM trajectories + loop report
7. (Optional) Re-render corrected map

Usage (CLI)
-----------
    ros2 run slam_offline_pgo pgo_offline \
        --bag    /root/ros2_ws/bags/Lake_Circuit_FULL_ANALYSIS/recorded_bag \
        --config /root/ros2_ws/install/slam_offline_pgo/share/slam_offline_pgo/config/pgo_params.yaml \
        --output /root/ros2_ws/bags/Lake_Circuit_FULL_ANALYSIS/results/
"""

import argparse
import csv
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml

from slam_offline_pgo.io_bag         import extract_keyframes, save_keyframes, load_keyframes
from slam_offline_pgo.scan_context   import SCManager
from slam_offline_pgo.loop_detector  import detect_all_loops
from slam_offline_pgo.map_renderer   import build_map, save_map


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _quat_to_rot(q_xyzw):
    qx, qy, qz, qw = q_xyzw
    return np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)


def _rot_to_quat(R):
    """3x3 rotation matrix → [qx, qy, qz, qw] (Shepperd's method, numerically stable)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S  = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S  = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S  = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S  = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return np.array([qx, qy, qz, qw], dtype=np.float64)


def _pose_to_matrix(position, quat_xyzw):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _quat_to_rot(quat_xyzw)
    T[:3, 3]  = position
    return T


def _matrix_to_pose(T):
    """4x4 homogeneous → (position, quat_xyzw)."""
    return T[:3, 3].copy(), _rot_to_quat(T[:3, :3])


def _make_info_matrix(trans_sigma, rot_sigma):
    """
    Open3D 6x6 information matrix with ordering [rot_x, rot_y, rot_z, t_x, t_y, t_z].
    Information = inverse covariance; smaller sigma → larger information → stiffer edge.
    """
    info = np.zeros((6, 6), dtype=np.float64)
    info[0:3, 0:3] = np.eye(3) / (rot_sigma ** 2)
    info[3:6, 3:6] = np.eye(3) / (trans_sigma ** 2)
    return info


# ---------------------------------------------------------------------------
# Pose graph
# ---------------------------------------------------------------------------

def build_pose_graph(keyframes, loop_constraints, pg_params):
    """Assemble an Open3D PoseGraph from keyframes + loop constraints."""
    odom_info = _make_info_matrix(
        float(pg_params['odom_trans_sigma_m']),
        float(pg_params['odom_rot_sigma_rad']),
    )
    loop_info = _make_info_matrix(
        float(pg_params['loop_trans_sigma_m']),
        float(pg_params['loop_rot_sigma_rad']),
    )

    graph = o3d.pipelines.registration.PoseGraph()

    # Nodes — one per keyframe, initial pose = raw FAST-LIO world pose
    world_poses = []
    for kf in keyframes:
        T = _pose_to_matrix(kf['position'], kf['rotation'])
        world_poses.append(T)
        graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(T))

    # Open3D edge convention (verified against the multiway-registration tutorial):
    # the edge's transformation_ is the coords change source→target, i.e.
    #     T_edge @ p_source_frame = p_target_frame,
    # equivalently T_edge = T_world_target^-1 @ T_world_source.

    # Odometry edges — consecutive keyframes
    for i in range(len(keyframes) - 1):
        T_rel = np.linalg.inv(world_poses[i + 1]) @ world_poses[i]
        graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            i, i + 1, T_rel, odom_info, uncertain=False,
        ))

    # Loop edges — ICP already returns T such that T @ p_from = p_to, which is
    # exactly T_world_to^-1 @ T_world_from — the same convention as above.
    for c in loop_constraints:
        graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            c.from_index, c.to_index, c.T_from_to, loop_info, uncertain=True,
        ))

    return graph


def optimise_pose_graph(graph, pg_params):
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance = float(pg_params.get('max_correspondence_distance', 0.1)),
        edge_prune_threshold        = float(pg_params.get('edge_prune_threshold',        0.25)),
        preference_loop_closure     = float(pg_params.get('preference_loop_closure',     2.0)),
        reference_node              = int  (pg_params.get('reference_node',              0)),
    )
    o3d.pipelines.registration.global_optimization(
        graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option,
    )


# ---------------------------------------------------------------------------
# Output writers
# ---------------------------------------------------------------------------

def write_tum_trajectory(keyframes, poses, output_path):
    """
    Write TUM-format trajectory: `timestamp tx ty tz qx qy qz qw`.

    If `poses` is None, uses the raw FAST-LIO poses from each keyframe.
    Otherwise `poses` is a list of (position(3,), quat_xyzw(4,)) aligned with keyframes.
    """
    with open(output_path, 'w') as f:
        for i, kf in enumerate(keyframes):
            if poses is None:
                pos, quat = kf['position'], kf['rotation']
            else:
                pos, quat = poses[i]
            t = kf['timestamp']
            f.write(
                f"{t:.9f} "
                f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
                f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n"
            )


def write_loop_report(constraints, output_path):
    with open(output_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'from_index', 'to_index',
            'sc_distance', 'icp_fitness', 'icp_rmse',
            'translation_m', 'yaw_deg',
        ])
        for c in constraints:
            t_mag = float(np.linalg.norm(c.T_from_to[:3, 3]))
            yaw   = float(np.rad2deg(np.arctan2(c.T_from_to[1, 0], c.T_from_to[0, 0])))
            w.writerow([
                c.from_index, c.to_index,
                f"{c.sc_distance:.4f}",
                f"{c.icp_fitness:.4f}",
                f"{c.icp_rmse:.4f}",
                f"{t_mag:.4f}",
                f"{yaw:.2f}",
            ])


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

def run_pipeline(bag_path, config_path, output_dir, force_extract=False, no_map=False):
    with open(config_path) as f:
        cfg = yaml.safe_load(f)['slam_offline_pgo']

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # ---------- Stage 1: keyframes ----------
    kf_path = output_dir / 'keyframes.npz'
    if kf_path.exists() and not force_extract:
        print(f"\n[Stage 1] Loading existing keyframes from {kf_path}")
        keyframes = load_keyframes(kf_path)
    else:
        print(f"\n[Stage 1] Extracting keyframes from bag {bag_path}")
        keyframes = extract_keyframes(bag_path, cfg)
        save_keyframes(keyframes, kf_path)
    print(f"  {len(keyframes)} keyframes loaded")

    # ---------- Stage 2: Scan Context ----------
    print(f"\n[Stage 2] Building Scan Context database")
    sc = SCManager(cfg['scan_context'])
    for kf in keyframes:
        sc.add_keyframe(kf['index'], kf['cloud'], kf['path_length_m'])
    sc.build_index()

    # ---------- Stage 3: Loop detection ----------
    print(f"\n[Stage 3] Detecting and verifying loop closures")
    t = time.time()
    constraints = detect_all_loops(keyframes, sc, cfg)
    print(f"  Elapsed: {time.time() - t:.1f}s")

    # ---------- Stage 4: Pose graph ----------
    print(f"\n[Stage 4] Building pose graph")
    graph = build_pose_graph(keyframes, constraints, cfg['pose_graph'])
    print(f"  {len(graph.nodes)} nodes, {len(graph.edges)} edges "
          f"({len(keyframes) - 1} odometry, {len(constraints)} loops)")

    # ---------- Stage 5: Optimise ----------
    print(f"\n[Stage 5] Optimising pose graph")
    t = time.time()
    optimise_pose_graph(graph, cfg['pose_graph'])
    print(f"  Elapsed: {time.time() - t:.1f}s")

    # ---------- Stage 6: Extract corrected poses ----------
    corrected_matrices = [np.asarray(n.pose, dtype=np.float64) for n in graph.nodes]
    corrected_poses    = [_matrix_to_pose(T) for T in corrected_matrices]

    # Loop closure error metric (headline number)
    start_pos     = keyframes[0]['position']
    orig_end_pos  = keyframes[-1]['position']
    corr_end_pos  = corrected_poses[-1][0]
    err_before    = float(np.linalg.norm(orig_end_pos - start_pos))
    err_after     = float(np.linalg.norm(corr_end_pos - start_pos))
    print(f"\n  Loop closure error (end-to-start distance):")
    print(f"    Before PGO : {err_before:6.2f} m")
    print(f"    After  PGO : {err_after:6.2f} m")

    # ---------- Stage 7: Write trajectories + report ----------
    print(f"\n[Stage 7] Writing outputs to {output_dir}")
    tum_before = output_dir / 'original_trajectory.tum'
    tum_after  = output_dir / 'corrected_trajectory.tum'
    report     = output_dir / 'loop_closure_report.csv'

    write_tum_trajectory(keyframes, None,             tum_before)
    write_tum_trajectory(keyframes, corrected_poses,  tum_after)
    write_loop_report(constraints, report)

    for p in (tum_before, tum_after, report):
        print(f"  {p.name}")

    # ---------- Stage 8: Render maps ----------
    if no_map:
        print(f"\n[Stage 8] --no-map: skipping map rendering")
    else:
        print(f"\n[Stage 8] Rendering maps")
        voxel = float(cfg['map'].get('voxel_size_m', 0.05))

        print(f"  Building 'before' map ...")
        pcd_before = build_map(keyframes, voxel_size=voxel)
        save_map(pcd_before, output_dir / 'merged_map_before.pcd')

        print(f"  Building 'after' map ...")
        pcd_after = build_map(keyframes, voxel_size=voxel, corrected_poses=corrected_poses)
        save_map(pcd_after, output_dir / 'merged_map_after.pcd')

    print(f"\nDone.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Offline loop closure + pose graph optimisation for FAST-LIO2"
    )
    parser.add_argument('--bag',    required=True,
                        help='Path to recorded ROS 2 bag directory')
    parser.add_argument('--config', required=True,
                        help='Path to pgo_params.yaml')
    parser.add_argument('--output', required=True,
                        help='Output directory for all results')
    parser.add_argument('--force-extract', action='store_true',
                        help='Re-extract keyframes even if keyframes.npz already exists')
    parser.add_argument('--no-map', action='store_true',
                        help='Skip before/after map rendering')
    args = parser.parse_args()

    run_pipeline(
        bag_path     = args.bag,
        config_path  = args.config,
        output_dir   = args.output,
        force_extract= args.force_extract,
        no_map       = args.no_map,
    )
