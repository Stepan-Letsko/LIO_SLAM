"""
scan_context.py — Scan Context descriptor (Python port of irapkaist/scancontext).

Encodes a point cloud into a compact 2-D polar descriptor (rings × sectors)
where each cell stores the maximum Z-height of points that fall in it.
A rotation-invariant ring key is derived for fast KD-tree candidate retrieval.

Mirrors the reference C++ implementation in FAST_LIO_ROS2/src/Scancontext.cpp.

Classes
-------
SCManager
    Build descriptors, maintain a searchable database, detect loop candidates.
"""

import numpy as np
from scipy.spatial import cKDTree


class SCManager:
    """
    Scan Context descriptor manager.

    Typical usage:

        sc = SCManager(params['scan_context'])
        for kf in keyframes:
            sc.add_keyframe(kf['index'], kf['cloud'], kf['path_length_m'])
        sc.build_index()
        for i in range(len(keyframes)):
            match, dist, yaw = sc.detect_loop(i)
    """

    def __init__(self, params):
        # Geometry
        self.num_rings      = int(params['num_rings'])
        self.num_sectors    = int(params['num_sectors'])
        self.max_radius     = float(params['max_radius_m'])
        self.lidar_height   = float(params['lidar_height_m'])

        # Acceptance and search
        self.dist_threshold   = float(params['dist_threshold'])
        self.num_candidates   = int(params['num_candidates'])
        self.min_index_gap    = int(params['min_index_gap'])
        self.min_path_length  = float(params['min_path_length_m'])

        # Search ratio for sector-key alignment (C++ SEARCH_RATIO = 0.1)
        self.search_ratio = 0.1

        # Storage (aligned by keyframe index)
        self.descriptors  = []   # list of np.ndarray (num_rings, num_sectors)
        self.ring_keys    = []   # list of np.ndarray (num_rings,)
        self.path_lengths = []   # list of float

        self._kdtree = None
        self._index_size = 0

    # ------------------------------------------------------------------
    # Descriptor construction
    # ------------------------------------------------------------------

    def make_descriptor(self, cloud_xyz):
        """
        Build a Scan Context descriptor from a point cloud.

        Parameters
        ----------
        cloud_xyz : np.ndarray, shape (N, 3)

        Returns
        -------
        desc : np.ndarray, shape (num_rings, num_sectors), float64
            Each bin stores the maximum (z + lidar_height) of points that fall
            in it.  Empty bins are 0.0.
        """
        if cloud_xyz.shape[0] == 0:
            return np.zeros((self.num_rings, self.num_sectors), dtype=np.float64)

        pts = cloud_xyz.astype(np.float64)
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

        # Lift so all z values tend to be positive
        z_lifted = z + self.lidar_height

        # Polar coordinates (range, angle in [0, 360))
        azim_range = np.sqrt(x * x + y * y)
        azim_angle = np.mod(np.rad2deg(np.arctan2(y, x)), 360.0)

        # Keep only points within the max radius
        valid = azim_range <= self.max_radius
        z_lifted   = z_lifted[valid]
        azim_range = azim_range[valid]
        azim_angle = azim_angle[valid]

        if z_lifted.size == 0:
            return np.zeros((self.num_rings, self.num_sectors), dtype=np.float64)

        # Bin indices
        ring_idx = np.floor(azim_range / self.max_radius * self.num_rings).astype(np.int32)
        ring_idx = np.clip(ring_idx, 0, self.num_rings - 1)
        sector_idx = np.floor(azim_angle / 360.0 * self.num_sectors).astype(np.int32)
        sector_idx = np.clip(sector_idx, 0, self.num_sectors - 1)

        # Max-accumulate z per bin.  Use a large-negative sentinel so the first
        # point in each bin wins regardless of sign (matches the C++ NO_POINT logic).
        sentinel = -1e9
        desc = np.full((self.num_rings, self.num_sectors), sentinel, dtype=np.float64)
        np.maximum.at(desc, (ring_idx, sector_idx), z_lifted)

        # Empty bins back to zero (so cosine similarity treats them as "no point")
        desc[desc == sentinel] = 0.0
        return desc

    def make_ring_key(self, desc):
        """Rotation-invariant ring key: row-wise mean of the SC matrix."""
        return desc.mean(axis=1)

    def make_sector_key(self, desc):
        """Sector key: column-wise mean of the SC matrix (used for fast alignment)."""
        return desc.mean(axis=0)

    # ------------------------------------------------------------------
    # Database management
    # ------------------------------------------------------------------

    def add_keyframe(self, index, cloud_xyz, path_length_m):
        """
        Compute and store descriptor + ring key for a new keyframe.

        Keyframes must be added in order (index == current length).
        """
        if index != len(self.descriptors):
            raise ValueError(
                f"Keyframes must be added in order (expected index "
                f"{len(self.descriptors)}, got {index})"
            )
        desc = self.make_descriptor(cloud_xyz)
        self.descriptors.append(desc)
        self.ring_keys.append(self.make_ring_key(desc))
        self.path_lengths.append(float(path_length_m))
        # Invalidate cached index
        self._kdtree = None

    def build_index(self):
        """Build a KD-tree over all stored ring keys (call after adding all keyframes)."""
        if len(self.ring_keys) == 0:
            raise RuntimeError("Cannot build index: no keyframes added yet")
        keys = np.vstack(self.ring_keys)
        self._kdtree = cKDTree(keys)
        self._index_size = len(self.ring_keys)

    def __len__(self):
        return len(self.descriptors)

    # ------------------------------------------------------------------
    # SC distance + yaw alignment
    # ------------------------------------------------------------------

    def _distance_direct(self, sc1, sc2_shifted):
        """
        Column-wise cosine distance between two SC matrices.
        Columns where either side has zero norm are skipped (no info).
        Returns 1 - mean(cosine_similarity).
        """
        n1 = np.linalg.norm(sc1,         axis=0)
        n2 = np.linalg.norm(sc2_shifted, axis=0)
        valid = (n1 > 0) & (n2 > 0)
        if not valid.any():
            return 1.0
        num = (sc1[:, valid] * sc2_shifted[:, valid]).sum(axis=0)
        den = n1[valid] * n2[valid]
        sim = (num / den).mean()
        return 1.0 - float(sim)

    def sc_distance(self, sc1, sc2):
        """
        Return (min_distance, best_shift) between two SC matrices.

        Two-stage alignment:
          1. Fast coarse alignment via cyclic sector-key difference
          2. Fine search in ±SEARCH_RADIUS columns around the coarse optimum
        """
        # Stage 1 — sector-key alignment
        vkey1 = self.make_sector_key(sc1)
        vkey2 = self.make_sector_key(sc2)

        coarse_shift = 0
        coarse_diff  = np.inf
        for s in range(self.num_sectors):
            diff = np.linalg.norm(vkey1 - np.roll(vkey2, s))
            if diff < coarse_diff:
                coarse_diff = diff
                coarse_shift = s

        # Stage 2 — fine search around coarse_shift
        search_radius = max(1, round(0.5 * self.search_ratio * self.num_sectors))
        shifts = {coarse_shift}
        for i in range(1, search_radius + 1):
            shifts.add((coarse_shift + i) % self.num_sectors)
            shifts.add((coarse_shift - i) % self.num_sectors)

        min_dist = np.inf
        argmin_shift = 0
        for s in shifts:
            d = self._distance_direct(sc1, np.roll(sc2, s, axis=1))
            if d < min_dist:
                min_dist = d
                argmin_shift = s

        return float(min_dist), int(argmin_shift)

    # ------------------------------------------------------------------
    # Loop detection
    # ------------------------------------------------------------------

    def detect_loop(self, query_index):
        """
        Search for the best loop closure candidate for the given keyframe.

        Returns
        -------
        match_index : int or None
            Index of the best matching keyframe, or None if no candidate
            passes the distance threshold.
        sc_distance : float
            Scan Context distance of the best candidate (0 = identical,
            1 = orthogonal).  np.inf if no valid candidates found.
        yaw_rad : float
            Estimated yaw offset between query and candidate (radians).
        """
        if self._kdtree is None:
            self.build_index()

        if query_index < 0 or query_index >= self._index_size:
            raise IndexError(f"query_index {query_index} out of range [0, {self._index_size})")

        # Candidate retrieval from ring-key KD-tree
        k = min(self.num_candidates + 1, self._index_size)   # +1 in case the query itself is top-1
        query_key = self.ring_keys[query_index].reshape(1, -1)
        _, cand_idxs = self._kdtree.query(query_key, k=k)
        cand_idxs = np.atleast_1d(cand_idxs.ravel())

        # Gate candidates by index gap and path-length gap
        query_path = self.path_lengths[query_index]
        min_dist = np.inf
        best_idx = -1
        best_shift = 0

        for cand in cand_idxs:
            cand = int(cand)
            if cand == query_index:
                continue
            if abs(query_index - cand) < self.min_index_gap:
                continue
            if abs(query_path - self.path_lengths[cand]) < self.min_path_length:
                continue

            d, shift = self.sc_distance(
                self.descriptors[query_index],
                self.descriptors[cand],
            )
            if d < min_dist:
                min_dist = d
                best_idx = cand
                best_shift = shift

        if best_idx < 0:
            return None, float('inf'), 0.0

        yaw_rad = best_shift * (2.0 * np.pi / self.num_sectors)

        if min_dist >= self.dist_threshold:
            return None, min_dist, yaw_rad

        return best_idx, min_dist, yaw_rad
