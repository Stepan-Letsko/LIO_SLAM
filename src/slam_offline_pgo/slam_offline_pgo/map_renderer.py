"""
map_renderer.py — Build a global point cloud map from keyframe poses + clouds.

Transforms each keyframe cloud into the world frame using its pose,
concatenates everything, optionally voxel-downsamples, and saves as .pcd.

For the "before PGO" map the clouds are already in world frame (from
/cloud_registered), so no transformation is needed.  For the "after PGO"
map, pass corrected_poses to re-project each cloud through the corrected pose.

Usage (CLI)
-----------
    ros2 run slam_offline_pgo map_renderer \
        --keyframes /path/to/keyframes.npz \
        --output    /path/to/merged_map.pcd \
        --voxel     0.05
"""

import argparse
import os
import time
from pathlib import Path

import numpy as np
import open3d as o3d

from slam_offline_pgo.io_bag import load_keyframes


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _quat_to_rot(q_xyzw):
    """
    Convert a unit quaternion [qx, qy, qz, qw] to a 3×3 rotation matrix.
    """
    qx, qy, qz, qw = q_xyzw
    return np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)


# ---------------------------------------------------------------------------
# Core functions
# ---------------------------------------------------------------------------

def build_map(keyframes, voxel_size=0.05, corrected_poses=None):
    """
    Merge keyframe clouds into a single world-frame point cloud.

    Keyframe clouds are stored in SENSOR-LOCAL frame (see io_bag._world_to_local),
    so we always apply a pose to project them back into the world.

    Parameters
    ----------
    keyframes : list of dict
        Each dict must contain:
          'position'  : np.ndarray(3,)  — original FAST-LIO world-frame translation
          'rotation'  : np.ndarray(4,)  — original pose as qx qy qz qw
          'cloud'     : np.ndarray(N,3) — points in SENSOR frame
    voxel_size : float
        Voxel grid leaf size in metres.  Set to 0 to skip downsampling.
    corrected_poses : list of (position, rotation_qxyzw) or None
        If provided, must have the same length as keyframes.  Uses the corrected
        pose instead of the original for each keyframe.

    Returns
    -------
    pcd : open3d.geometry.PointCloud
        Merged (and optionally downsampled) map.
    """
    if corrected_poses is not None and len(corrected_poses) != len(keyframes):
        raise ValueError(
            f"corrected_poses length ({len(corrected_poses)}) != "
            f"keyframes length ({len(keyframes)})"
        )

    all_pts = []

    for i, kf in enumerate(keyframes):
        cloud_local = kf['cloud'].astype(np.float64)  # (N, 3) sensor-frame

        if corrected_poses is not None:
            t, q = corrected_poses[i]
        else:
            t, q = kf['position'], kf['rotation']

        R = _quat_to_rot(q)
        cloud_world = cloud_local @ R.T + t    # row-vector form of R @ v + t

        all_pts.append(cloud_world.astype(np.float32))

        if (i + 1) % 50 == 0:
            print(f"  Merged {i + 1}/{len(keyframes)} keyframes ...", flush=True)

    merged = np.vstack(all_pts)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(merged)

    if voxel_size > 0:
        before = len(pcd.points)
        pcd = pcd.voxel_down_sample(voxel_size)
        print(f"  Voxel filter {voxel_size}m: {before:,} → {len(pcd.points):,} points")

    return pcd


def save_map(pcd, output_path):
    """Save an Open3D PointCloud to a .pcd file."""
    output_path = str(output_path)
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(output_path, pcd)
    size_mb = os.path.getsize(output_path) / 1e6
    print(f"  Saved → {output_path}  ({size_mb:.1f} MB)")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Render merged point cloud map from keyframes"
    )
    parser.add_argument('--keyframes', required=True,
                        help='Path to keyframes .npz file')
    parser.add_argument('--output',    required=True,
                        help='Output .pcd file path')
    parser.add_argument('--voxel',     type=float, default=0.05,
                        help='Voxel grid leaf size in metres (0 = no downsampling)')
    args = parser.parse_args()

    print(f"\n=== map_renderer: Building Point Cloud Map ===")
    print(f"  Keyframes : {args.keyframes}")
    print(f"  Output    : {args.output}")
    print(f"  Voxel     : {args.voxel} m\n")

    t_start = time.time()

    keyframes = load_keyframes(args.keyframes)
    print(f"  Loaded {len(keyframes)} keyframes")

    pcd = build_map(keyframes, voxel_size=args.voxel)
    save_map(pcd, args.output)

    print(f"\n  Total points in map : {len(pcd.points):,}")
    print(f"  Wall time           : {time.time() - t_start:.1f} s")
    print("\nDone.")
