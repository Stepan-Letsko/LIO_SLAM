"""
loop_detector.py — Loop closure detection via Scan Context + ICP verification.

For each keyframe, queries the SC database for candidates then runs
Open3D Generalized ICP to verify the geometric match.

Classes
-------
LoopConstraint
    Data class holding a verified loop: frame indices, relative transform,
    SC distance, ICP fitness.

Functions
---------
verify_with_icp(cloud_from, cloud_to, yaw_init_rad, params)
    Run a single GICP verification and return (T_from_to, fitness, rmse) or
    (None, fitness, rmse) if the match fails the fitness threshold.

detect_all_loops(keyframes, sc_manager, params)
    Run the full detection pass over all keyframes and return the list of
    verified LoopConstraints.
"""

from dataclasses import dataclass
import numpy as np
import open3d as o3d


# ---------------------------------------------------------------------------
# Data class
# ---------------------------------------------------------------------------

@dataclass
class LoopConstraint:
    """A verified loop closure between two keyframes."""
    from_index:  int
    to_index:    int
    T_from_to:   np.ndarray    # 4×4 transform such that p_to = T @ p_from (homogeneous)
    sc_distance: float
    icp_fitness: float
    icp_rmse:    float


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _initial_transform_from_yaw(yaw_rad):
    """
    Build a 4×4 initial guess with a pure rotation about Z by yaw_rad and
    zero translation.  Scan Context provides yaw but no translation — ICP
    is responsible for finding the translation.
    """
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    T = np.eye(4, dtype=np.float64)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T


def _prepare_cloud(xyz, voxel_size):
    """
    Make an Open3D PointCloud, voxel-downsample, and estimate per-point
    covariances (required by the Generalized ICP transformation estimator).
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
    search = o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 3, max_nn=30)
    pcd.estimate_covariances(search_param=search)
    return pcd


# ---------------------------------------------------------------------------
# Single-pair verification
# ---------------------------------------------------------------------------

def verify_with_icp(cloud_from, cloud_to, yaw_init_rad, params):
    """
    Run Generalized ICP between two sensor-frame clouds.

    Parameters
    ----------
    cloud_from : np.ndarray (N, 3) — source cloud in its own sensor frame
    cloud_to   : np.ndarray (M, 3) — target cloud in its own sensor frame
    yaw_init_rad : float — initial yaw about Z from Scan Context
    params : dict with keys
        voxel_size_m, max_correspondence_m, max_iterations, fitness_threshold

    Returns
    -------
    T_from_to : np.ndarray (4, 4) or None
        Transform mapping cloud_from → cloud_to frame if fitness >= threshold,
        otherwise None.
    fitness : float
        Open3D inlier ratio (fraction of source points with a match).
    rmse : float
        Mean squared distance of inlier correspondences.
    """
    voxel          = float(params.get('voxel_size_m',         0.3))
    max_corr       = float(params.get('max_correspondence_m', 1.0))
    max_iter       = int  (params.get('max_iterations',       50))
    fit_thresh     = float(params.get('fitness_threshold',    0.2))
    max_translation = float(params.get('max_translation_m',   10.0))

    if cloud_from.shape[0] < 50 or cloud_to.shape[0] < 50:
        return None, 0.0, 0.0

    src = _prepare_cloud(cloud_from, voxel)
    tgt = _prepare_cloud(cloud_to,   voxel)

    T_init = _initial_transform_from_yaw(yaw_init_rad)

    # Single-pass GICP from the SC-derived yaw guess.  Tried a two-pass coarse/
    # fine scheme; the coarse pass pulls some pairs into spurious distant
    # optima that the fine pass cannot recover from.  Single-pass at max_corr
    # is tight enough to stay local, and the fitness threshold filters bad
    # matches.
    result = o3d.pipelines.registration.registration_generalized_icp(
        src, tgt,
        max_corr,
        T_init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter),
    )

    T = np.asarray(result.transformation, dtype=np.float64)
    fitness = float(result.fitness)
    rmse    = float(result.inlier_rmse)

    # Sanity check: true loop-closure transforms on sensor-frame clouds should
    # be small (clouds see the same physical place).  A huge translation means
    # ICP slid the scan to a spurious match.
    if np.linalg.norm(T[:3, 3]) > max_translation:
        return None, fitness, rmse

    if fitness < fit_thresh:
        return None, fitness, rmse

    return T, fitness, rmse


# ---------------------------------------------------------------------------
# Full-pass detection
# ---------------------------------------------------------------------------

def detect_all_loops(keyframes, sc_manager, params):
    """
    Run loop detection over every keyframe and return verified constraints.

    Parameters
    ----------
    keyframes : list of dict — from io_bag.extract_keyframes / load_keyframes
    sc_manager : SCManager — with all descriptors added and index built
    params : dict — top-level 'slam_offline_pgo' block from pgo_params.yaml

    Returns
    -------
    constraints : list of LoopConstraint — verified loops ready for the pose graph
    """
    icp_params = params['icp']

    constraints = []
    tried_pairs = set()
    n_sc_candidates = 0
    n_icp_fail      = 0

    for i in range(len(sc_manager)):
        match, sc_dist, yaw = sc_manager.detect_loop(i)
        if match is None:
            continue

        # Canonicalise the pair so we only verify each unique loop once
        pair = (min(i, match), max(i, match))
        if pair in tried_pairs:
            continue
        tried_pairs.add(pair)
        n_sc_candidates += 1

        # Direction: from lower to higher index (matches the canonical pair)
        from_idx, to_idx = pair
        # If we're running the pair in the opposite direction from the SC query,
        # the yaw needs to flip sign (SC gives yaw from query → match).
        yaw_used = yaw if from_idx == i else -yaw

        T, fitness, rmse = verify_with_icp(
            keyframes[from_idx]['cloud'],
            keyframes[to_idx]['cloud'],
            yaw_used,
            icp_params,
        )

        if T is None:
            n_icp_fail += 1
            print(f"  [ICP FAIL] kf {from_idx:3d} → kf {to_idx:3d}  "
                  f"sc={sc_dist:.3f}  fit={fitness:.3f}  rmse={rmse:.3f}")
            continue

        constraints.append(LoopConstraint(
            from_index  = from_idx,
            to_index    = to_idx,
            T_from_to   = T,
            sc_distance = sc_dist,
            icp_fitness = fitness,
            icp_rmse    = rmse,
        ))
        # Report translation implied by the constraint for a quick sanity read
        dt = np.linalg.norm(T[:3, 3])
        print(f"  [ICP PASS] kf {from_idx:3d} → kf {to_idx:3d}  "
              f"sc={sc_dist:.3f}  fit={fitness:.3f}  rmse={rmse:.3f}  "
              f"|Δt|={dt:.2f}m")

    print(
        f"\n  SC candidates : {n_sc_candidates}"
        f"\n  ICP accepted  : {len(constraints)}"
        f"\n  ICP rejected  : {n_icp_fail}"
    )
    return constraints
