"""
visualise_sc.py — Render Scan Context descriptors to PNG for thesis figures.

Modes
-----
  single : one descriptor heatmap + matching top-down point cloud
  pair   : query descriptor + candidate descriptor + column-aligned candidate,
           useful for showing how Scan Context recognises revisited places.

Usage
-----
    ros2 run slam_offline_pgo visualise_sc --mode single \
        --keyframes /path/to/keyframes.npz --config /path/to/pgo_params.yaml \
        --index 268 --output /tmp/sc_268.png

    ros2 run slam_offline_pgo visualise_sc --mode pair \
        --keyframes /path/to/keyframes.npz --config /path/to/pgo_params.yaml \
        --query 110 --match 231 --output /tmp/sc_pair_110_231.png
"""

import argparse
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')   # no GUI in Docker
import matplotlib.pyplot as plt

from slam_offline_pgo.io_bag import load_keyframes
from slam_offline_pgo.scan_context import SCManager


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _plot_descriptor(ax, desc, title):
    """Plot an SC matrix as a heatmap: sectors on X (0-360°), rings on Y."""
    num_rings, num_sectors = desc.shape
    im = ax.imshow(
        desc,
        aspect='auto',
        origin='lower',
        cmap='viridis',
        extent=[0, 360, 0, num_rings],
    )
    ax.set_xlabel('Azimuth sector (deg)')
    ax.set_ylabel('Range ring (0 = close, N = far)')
    ax.set_title(title)
    ax.set_xticks(np.arange(0, 361, 60))
    ax.set_yticks(np.arange(0, num_rings + 1, 4))
    return im


def _plot_topdown(ax, cloud, max_radius, num_rings, num_sectors, title):
    """Top-down scatter of the point cloud with polar grid overlay."""
    inside = np.linalg.norm(cloud[:, :2], axis=1) <= max_radius
    xy = cloud[inside, :2]
    z  = cloud[inside, 2]

    ax.scatter(xy[:, 0], xy[:, 1], c=z, cmap='viridis', s=0.3, alpha=0.6)

    # Polar grid
    theta = np.linspace(0, 2 * np.pi, 360)
    for r in np.linspace(max_radius / num_rings, max_radius, num_rings):
        ax.plot(r * np.cos(theta), r * np.sin(theta), color='white', lw=0.2, alpha=0.5)
    for s in range(num_sectors):
        a = s * 2 * np.pi / num_sectors
        ax.plot([0, max_radius * np.cos(a)], [0, max_radius * np.sin(a)],
                color='white', lw=0.2, alpha=0.5)

    ax.set_xlim(-max_radius, max_radius)
    ax.set_ylim(-max_radius, max_radius)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.set_facecolor('black')


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def render_single(keyframes, sc_params, index, output):
    sc = SCManager(sc_params)
    cloud = keyframes[index]['cloud']
    desc = sc.make_descriptor(cloud)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    _plot_topdown(
        axes[0], cloud,
        sc.max_radius, sc.num_rings, sc.num_sectors,
        f'Keyframe {index} — top-down point cloud',
    )
    im = _plot_descriptor(
        axes[1], desc,
        f'Scan Context descriptor (rings={sc.num_rings}, sectors={sc.num_sectors})',
    )
    fig.colorbar(im, ax=axes[1], label='Max Z + lidar_height (m)', shrink=0.85)
    fig.suptitle(f"Scan Context — keyframe {index}", fontsize=13, y=1.00)
    fig.tight_layout()
    fig.savefig(output, dpi=150, bbox_inches='tight')
    print(f"  Saved → {output}")


def render_pair(keyframes, sc_params, query_idx, match_idx, output):
    sc = SCManager(sc_params)
    desc_q = sc.make_descriptor(keyframes[query_idx]['cloud'])
    desc_m = sc.make_descriptor(keyframes[match_idx]['cloud'])

    # Compute alignment (column shift) and SC distance
    dist, shift = sc.sc_distance(desc_q, desc_m)
    yaw_deg = shift * (360.0 / sc.num_sectors)
    desc_m_aligned = np.roll(desc_m, shift, axis=1)

    vmax = max(desc_q.max(), desc_m.max(), desc_m_aligned.max())

    fig, axes = plt.subplots(3, 1, figsize=(11, 9))
    for ax, mat, title in zip(
        axes,
        [desc_q, desc_m, desc_m_aligned],
        [
            f'Query — keyframe {query_idx}',
            f'Candidate — keyframe {match_idx} (raw)',
            f'Candidate — keyframe {match_idx} (aligned, shift={shift} sectors / {yaw_deg:.0f}°)',
        ],
    ):
        im = ax.imshow(mat, aspect='auto', origin='lower', cmap='viridis',
                       vmin=0, vmax=vmax,
                       extent=[0, 360, 0, sc.num_rings])
        ax.set_xlabel('Azimuth sector (deg)')
        ax.set_ylabel('Ring')
        ax.set_title(title)
        ax.set_xticks(np.arange(0, 361, 60))

    fig.suptitle(
        f"Scan Context loop match — dist={dist:.3f} (threshold={sc.dist_threshold})",
        fontsize=13,
    )
    fig.colorbar(im, ax=axes, label='Max Z + lidar_height (m)', shrink=0.8, location='right')
    fig.savefig(output, dpi=150, bbox_inches='tight')
    print(f"  Saved → {output}")
    print(f"  SC distance    : {dist:.3f}")
    print(f"  Sector shift   : {shift} / {sc.num_sectors}  ({yaw_deg:.1f}° yaw)")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Visualise Scan Context descriptors")
    parser.add_argument('--mode',      required=True, choices=['single', 'pair'])
    parser.add_argument('--keyframes', required=True, help='Path to keyframes .npz')
    parser.add_argument('--config',    required=True, help='Path to pgo_params.yaml')
    parser.add_argument('--output',    required=True, help='Output PNG path')
    parser.add_argument('--index',     type=int, help='Keyframe index (single mode)')
    parser.add_argument('--query',     type=int, help='Query keyframe index (pair mode)')
    parser.add_argument('--match',     type=int, help='Match keyframe index (pair mode)')
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)['slam_offline_pgo']
    sc_params = cfg['scan_context']

    keyframes = load_keyframes(args.keyframes)
    print(f"  Loaded {len(keyframes)} keyframes")
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)

    if args.mode == 'single':
        if args.index is None:
            parser.error('--index is required for single mode')
        render_single(keyframes, sc_params, args.index, args.output)
    elif args.mode == 'pair':
        if args.query is None or args.match is None:
            parser.error('--query and --match are required for pair mode')
        render_pair(keyframes, sc_params, args.query, args.match, args.output)
