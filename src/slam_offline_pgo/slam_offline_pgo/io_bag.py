"""
io_bag.py — Stage 1: Keyframe extraction from a ROS 2 bag.

Reads /Odometry and /cloud_registered from a recorded bag,
matches them by timestamp, samples keyframes by motion threshold,
and saves the result to a .npz file for all downstream stages.

Usage (CLI)
-----------
    ros2 run slam_offline_pgo io_bag \
        --bag    /root/ros2_ws/bags/Lake_Circuit_FULL_ANALYSIS/recorded_bag \
        --config /root/ros2_ws/install/slam_offline_pgo/share/slam_offline_pgo/config/pgo_params.yaml \
        --output /root/ros2_ws/bags/Lake_Circuit_FULL_ANALYSIS/results/keyframes.npz
"""

import argparse
import bisect
import time
from pathlib import Path

import numpy as np
import yaml

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ---------------------------------------------------------------------------
# PointCloud2 helpers
# ---------------------------------------------------------------------------

# sensor_msgs/msg/PointField datatype constants → numpy dtype
_FIELD_DTYPE = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


def _pointcloud2_to_xyz(msg):
    """
    Extract XYZ points from a sensor_msgs/msg/PointCloud2 message.

    Works with any field layout (XYZI, XYZIRT, etc.) as long as fields
    named 'x', 'y', 'z' are present.

    Returns
    -------
    points : np.ndarray, shape (N, 3), dtype float32
        Valid (non-NaN, non-zero-distance) points only.
    """
    n_points = msg.width * msg.height
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # Build field offset map
    field_offset = {}
    for f in msg.fields:
        field_offset[f.name] = f.offset

    for required in ('x', 'y', 'z'):
        if required not in field_offset:
            raise ValueError(
                f"PointCloud2 message is missing field '{required}'. "
                f"Available fields: {list(field_offset.keys())}"
            )

    point_step = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n_points, point_step)

    # Extract x, y, z columns (each is 4 bytes / float32)
    points = np.zeros((n_points, 3), dtype=np.float32)
    for col_idx, name in enumerate(('x', 'y', 'z')):
        offset = field_offset[name]
        # Slice the 4 bytes for this field from every row, make contiguous, reinterpret
        col_bytes = np.ascontiguousarray(raw[:, offset:offset + 4])
        points[:, col_idx] = col_bytes.view(np.float32).reshape(-1)

    # Remove NaN / inf / zero-distance points (scan origin artefacts)
    valid = (
        np.isfinite(points).all(axis=1) &
        (np.linalg.norm(points, axis=1) > 0.01)
    )
    return points[valid]


# ---------------------------------------------------------------------------
# Bag reading helpers
# ---------------------------------------------------------------------------

def _open_reader(bag_path, topics):
    """Open a rosbag2 sequential reader filtered to the requested topics."""
    bag_path = Path(bag_path)
    if bag_path.is_file():
        bag_path = bag_path.parent  # rosbag2_py needs the folder, not the .db3

    storage_opts = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id='sqlite3'
    )
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, converter_opts)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    for topic in topics:
        if topic not in type_map:
            available = list(type_map.keys())
            raise RuntimeError(
                f"Topic '{topic}' not found in bag.\n"
                f"Available topics: {available}"
            )

    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader, type_map


def _read_odometry(bag_path, topic):
    """
    Load all odometry messages from the bag.

    Returns
    -------
    timestamps : np.ndarray, shape (M,), float64   — seconds
    positions  : np.ndarray, shape (M, 3), float64 — x y z
    rotations  : np.ndarray, shape (M, 4), float64 — qx qy qz qw
    """
    reader, type_map = _open_reader(bag_path, [topic])
    msg_class = get_message(type_map[topic])

    timestamps, positions, rotations = [], [], []

    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, msg_class)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        timestamps.append(ts)
        positions.append([p.x, p.y, p.z])
        rotations.append([q.x, q.y, q.z, q.w])

    return (
        np.array(timestamps, dtype=np.float64),
        np.array(positions,  dtype=np.float64),
        np.array(rotations,  dtype=np.float64),
    )


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _quat_to_rot(q_xyzw):
    """Convert a unit quaternion [qx, qy, qz, qw] to a 3×3 rotation matrix."""
    qx, qy, qz, qw = q_xyzw
    return np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)


def _world_to_local(cloud_world, position, rotation_qxyzw):
    """
    Transform points from world frame back to sensor (local) frame:
        cloud_local = R.T @ (cloud_world - t)

    Works on row-vector arrays of shape (N, 3).
    """
    R = _quat_to_rot(rotation_qxyzw)
    return ((cloud_world.astype(np.float64) - position) @ R).astype(np.float32)


def _quat_angle_diff(q1_xyzw, q2_xyzw):
    """
    Rotation angle (radians) between two unit quaternions.

    Uses the formula: angle = 2 * arccos(|q_rel.w|)
    where q_rel = q2 * q1^{-1}.
    """
    # Unpack as (x, y, z, w)
    x1, y1, z1, w1 = q1_xyzw
    x2, y2, z2, w2 = q2_xyzw

    # q_rel = q2 * conjugate(q1)  (Hamilton product)
    w_rel = w2 * w1 + x2 * x1 + y2 * y1 + z2 * z1
    # Clamp for numerical safety before arccos
    w_rel = float(np.clip(abs(w_rel), 0.0, 1.0))
    return 2.0 * np.arccos(w_rel)


# ---------------------------------------------------------------------------
# Main extraction function
# ---------------------------------------------------------------------------

def extract_keyframes(bag_path, params):
    """
    Read a ROS 2 bag and return keyframes sampled by motion threshold.

    Parameters
    ----------
    bag_path : str | Path
        Path to the ROS 2 bag directory (or .db3 file).
    params : dict
        Pipeline parameters from pgo_params.yaml
        (top-level key 'slam_offline_pgo').

    Returns
    -------
    keyframes : list of dict, each containing:
        {
          'index':     int,
          'timestamp': float,           # seconds
          'position':  np.ndarray(3,),  # x y z  (float64)
          'rotation':  np.ndarray(4,),  # qx qy qz qw  (float64)
          'cloud':     np.ndarray(N,3), # xyz points  (float32)
        }
    """
    odom_topic  = params.get('odometry_topic', '/Odometry')
    cloud_topic = params.get('cloud_topic',    '/cloud_registered')
    min_dist_m  = float(params.get('keyframe_min_dist_m',    1.0))
    min_angle_r = float(params.get('keyframe_min_angle_rad', 0.2))
    carrier_r   = float(params.get('carrier_filter_radius_m', 0.0))

    bag_path = Path(bag_path)
    if bag_path.is_file():
        bag_path = bag_path.parent

    # ------------------------------------------------------------------
    # Pass 1: load all odometry into sorted arrays for fast timestamp lookup
    # ------------------------------------------------------------------
    print("  Reading odometry ... ", end='', flush=True)
    t0 = time.time()
    odom_ts, odom_pos, odom_rot = _read_odometry(bag_path, odom_topic)
    print(f"{len(odom_ts)} poses in {time.time()-t0:.1f}s")

    if len(odom_ts) == 0:
        raise RuntimeError(f"No messages found on '{odom_topic}'")

    # ------------------------------------------------------------------
    # Pass 2: stream cloud messages, look up nearest odometry, threshold
    # ------------------------------------------------------------------
    print("  Streaming clouds and sampling keyframes ...")
    reader, type_map = _open_reader(bag_path, [cloud_topic])
    cloud_msg_class = get_message(type_map[cloud_topic])

    keyframes = []
    last_pos = None
    last_rot = None
    n_clouds_read = 0
    path_length_m = 0.0  # accumulated path length for SC gating later

    while reader.has_next():
        _, data, ros_ts_ns = reader.read_next()
        msg = deserialize_message(data, cloud_msg_class)

        # Timestamp from the message header (seconds)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        n_clouds_read += 1

        # Find the closest odometry message by timestamp (binary search)
        idx = bisect.bisect_left(odom_ts, ts)
        idx = int(np.clip(idx, 0, len(odom_ts) - 1))
        # Check neighbour in case the right index is closer
        if idx > 0 and abs(odom_ts[idx - 1] - ts) < abs(odom_ts[idx] - ts):
            idx -= 1

        pos = odom_pos[idx]  # (3,)
        rot = odom_rot[idx]  # (4,) qxyzw

        # ------------------------------------------------------------------
        # Motion threshold check
        # ------------------------------------------------------------------
        if last_pos is None:
            # Always save the very first cloud as keyframe 0
            is_keyframe = True
        else:
            dist_moved  = float(np.linalg.norm(pos - last_pos))
            angle_moved = _quat_angle_diff(last_rot, rot)
            is_keyframe = (dist_moved > min_dist_m) or (angle_moved > min_angle_r)

        if not is_keyframe:
            continue

        # ------------------------------------------------------------------
        # Extract point cloud
        # ------------------------------------------------------------------
        try:
            cloud_xyz = _pointcloud2_to_xyz(msg)
        except Exception as e:
            print(f"  [warn] Skipping cloud at t={ts:.3f}: {e}")
            continue

        if cloud_xyz.shape[0] == 0:
            print(f"  [warn] Empty cloud at t={ts:.3f}, skipping")
            continue

        # /cloud_registered is in WORLD frame (FAST-LIO projects it via the pose).
        # All downstream modules (Scan Context, ICP, map re-rendering) need the
        # SENSOR-frame cloud, so we undo the pose here.
        cloud_xyz = _world_to_local(cloud_xyz, pos, rot)

        # Radial filter (sensor frame) — remove the handheld carrier's body
        if carrier_r > 0.0:
            horiz_r = np.linalg.norm(cloud_xyz[:, :2], axis=1)
            cloud_xyz = cloud_xyz[horiz_r > carrier_r]

        # Update accumulated path length
        if last_pos is not None:
            path_length_m += float(np.linalg.norm(pos - last_pos))

        kf = {
            'index':           len(keyframes),
            'timestamp':       float(ts),
            'position':        pos.copy(),
            'rotation':        rot.copy(),
            'cloud':           cloud_xyz,
            'path_length_m':   path_length_m,
        }
        keyframes.append(kf)
        last_pos = pos.copy()
        last_rot = rot.copy()

        if len(keyframes) % 50 == 0:
            print(f"    keyframe {len(keyframes):4d} | "
                  f"t={ts:.1f}s | "
                  f"pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) | "
                  f"pts={cloud_xyz.shape[0]}")

    print(f"  Scanned {n_clouds_read} clouds → {len(keyframes)} keyframes saved")
    return keyframes


# ---------------------------------------------------------------------------
# Save / load helpers  (used by all downstream modules)
# ---------------------------------------------------------------------------

def save_keyframes(keyframes, output_path):
    """
    Save keyframe list to a compressed .npz file.

    Layout
    ------
    timestamps   : (N,)    float64
    positions    : (N, 3)  float64
    rotations    : (N, 4)  float64  (qx qy qz qw)
    path_lengths : (N,)    float64  accumulated path in metres
    cloud_sizes  : (N,)    int64    number of points per keyframe
    clouds_flat  : (M, 3)  float32  all clouds concatenated row-wise
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    timestamps   = np.array([kf['timestamp']     for kf in keyframes], dtype=np.float64)
    positions    = np.array([kf['position']      for kf in keyframes], dtype=np.float64)
    rotations    = np.array([kf['rotation']      for kf in keyframes], dtype=np.float64)
    path_lengths = np.array([kf['path_length_m'] for kf in keyframes], dtype=np.float64)
    cloud_sizes  = np.array([kf['cloud'].shape[0] for kf in keyframes], dtype=np.int64)
    clouds_flat  = np.vstack([kf['cloud'] for kf in keyframes]).astype(np.float32)

    np.savez_compressed(
        output_path,
        timestamps=timestamps,
        positions=positions,
        rotations=rotations,
        path_lengths=path_lengths,
        cloud_sizes=cloud_sizes,
        clouds_flat=clouds_flat,
    )
    size_mb = output_path.stat().st_size / 1e6
    print(f"  Saved {len(keyframes)} keyframes → {output_path}  ({size_mb:.1f} MB)")


def load_keyframes(npz_path):
    """
    Load keyframes saved by save_keyframes().

    Returns
    -------
    keyframes : list of dict  (same format as extract_keyframes output)
    """
    data = np.load(npz_path)
    timestamps   = data['timestamps']
    positions    = data['positions']
    rotations    = data['rotations']
    path_lengths = data['path_lengths']
    cloud_sizes  = data['cloud_sizes']
    clouds_flat  = data['clouds_flat']

    keyframes = []
    offset = 0
    for i in range(len(timestamps)):
        n = int(cloud_sizes[i])
        keyframes.append({
            'index':         i,
            'timestamp':     float(timestamps[i]),
            'position':      positions[i],
            'rotation':      rotations[i],
            'cloud':         clouds_flat[offset:offset + n],
            'path_length_m': float(path_lengths[i]),
        })
        offset += n

    return keyframes


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Extract keyframes from a ROS 2 bag for offline loop closure"
    )
    parser.add_argument('--bag',    required=True,
                        help='Path to recorded ROS 2 bag directory')
    parser.add_argument('--config', required=True,
                        help='Path to pgo_params.yaml')
    parser.add_argument('--output', required=True,
                        help='Output .npz file path (e.g. results/keyframes.npz)')
    args = parser.parse_args()

    # Load config
    with open(args.config, 'r') as f:
        full_cfg = yaml.safe_load(f)
    params = full_cfg['slam_offline_pgo']

    print(f"\n=== io_bag: Keyframe Extraction ===")
    print(f"  Bag:    {args.bag}")
    print(f"  Output: {args.output}")
    print(f"  Thresholds: dist>{params['keyframe_min_dist_m']}m  "
          f"angle>{params['keyframe_min_angle_rad']}rad\n")

    t_start = time.time()
    keyframes = extract_keyframes(args.bag, params)
    save_keyframes(keyframes, args.output)

    total_path = keyframes[-1]['path_length_m'] if keyframes else 0.0
    print(f"\n  Total keyframes : {len(keyframes)}")
    print(f"  Path length     : {total_path:.1f} m")
    print(f"  Time span       : {keyframes[-1]['timestamp'] - keyframes[0]['timestamp']:.1f} s")
    print(f"  Wall time       : {time.time() - t_start:.1f} s")
    print(f"\nDone.")
