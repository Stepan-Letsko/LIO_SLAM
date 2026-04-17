"""
evaluate.py — Trajectory evaluation and visualisation for the offline PGO pipeline.

Compares the raw FAST-LIO trajectory against the PGO-corrected trajectory and
emits:

    * a top-down (XY) overlay plot with start/end markers and loop edges,
    * a side (XZ) overlay plot for vertical drift,
    * a plain-text summary report with closing error + path length,
    * (optional) ground-truth APE numbers when a reference trajectory is given.

Primary metric (no ground truth): **closing error** — the Euclidean distance
between the first and last keyframe pose.  For a loop dataset this number
quantifies accumulated drift.

Usage (CLI)
-----------
    ros2 run slam_offline_pgo evaluate \\
        --raw       /path/to/original_trajectory.tum \\
        --corrected /path/to/corrected_trajectory.tum \\
        --output    /path/to/results/ \\
        [--loops    /path/to/loop_closure_report.csv] \\
        [--gt       /path/to/ground_truth.tum]
"""

import argparse
import csv
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use('Agg')            # headless (no GUI in Docker)
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# TUM I/O
# ---------------------------------------------------------------------------

def load_tum(path):
    """
    Load a TUM-format trajectory: `timestamp tx ty tz qx qy qz qw` per line.

    Returns
    -------
    timestamps : np.ndarray (N,)
    positions  : np.ndarray (N, 3)
    """
    data = np.loadtxt(str(path))
    if data.ndim == 1:
        data = data[None, :]
    return data[:, 0], data[:, 1:4]


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def compute_closing_error(positions):
    """Euclidean distance between the first and last pose (m)."""
    return float(np.linalg.norm(positions[-1] - positions[0]))


def compute_path_length(positions):
    """Total travelled distance along the trajectory (m)."""
    diffs = np.diff(positions, axis=0)
    return float(np.linalg.norm(diffs, axis=1).sum())


def compute_ape(positions_est, positions_gt):
    """
    Absolute Position Error against a ground-truth trajectory of matching length.
    Assumes 1-to-1 index alignment (same keyframes/timestamps on both sides).

    Returns
    -------
    dict with rmse, mean, median, max (all metres)
    """
    if positions_est.shape != positions_gt.shape:
        raise ValueError(
            f"Estimate and GT shape mismatch: {positions_est.shape} vs {positions_gt.shape}"
        )
    err = np.linalg.norm(positions_est - positions_gt, axis=1)
    return {
        'rmse':   float(np.sqrt(np.mean(err ** 2))),
        'mean':   float(np.mean(err)),
        'median': float(np.median(err)),
        'max':    float(np.max(err)),
    }


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _read_loops_csv(path):
    """Parse a loop_closure_report.csv → list of (from_index, to_index)."""
    pairs = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            pairs.append((int(row['from_index']), int(row['to_index'])))
    return pairs


def _plot_overlay(raw, corrected, axes_pair, output_path, title,
                  loop_pairs=None, gt=None):
    """
    Draw a 2D overlay of raw vs corrected (and optional GT) for the given
    axis indices (e.g. (0, 1) → XY top-down, (0, 2) → XZ side view).
    """
    ax_i, ax_j = axes_pair
    labels = ['X (m)', 'Y (m)', 'Z (m)']

    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(raw[:, ax_i], raw[:, ax_j],
            color='#d62728', lw=1.4, alpha=0.8,
            label=f'Raw FAST-LIO (closing err {compute_closing_error(raw):.2f} m)')
    ax.plot(corrected[:, ax_i], corrected[:, ax_j],
            color='#2ca02c', lw=1.4, alpha=0.9,
            label=f'After PGO    (closing err {compute_closing_error(corrected):.2f} m)')

    if gt is not None:
        ax.plot(gt[:, ax_i], gt[:, ax_j],
                color='#1f77b4', lw=1.0, alpha=0.7, ls='--', label='Ground truth')

    # Start / end markers — drawn on the corrected trajectory
    ax.plot(corrected[0, ax_i], corrected[0, ax_j],
            marker='o', ms=10, mfc='white', mec='green', mew=2, ls='', label='Start')
    ax.plot(corrected[-1, ax_i], corrected[-1, ax_j],
            marker='X', ms=10, color='green', ls='', label='End')

    # Loop closure edges — drawn on the corrected trajectory
    if loop_pairs:
        for (a, b) in loop_pairs:
            if a < len(corrected) and b < len(corrected):
                ax.plot(
                    [corrected[a, ax_i], corrected[b, ax_i]],
                    [corrected[a, ax_j], corrected[b, ax_j]],
                    color='#9467bd', lw=1.0, alpha=0.6, ls=':',
                )
        # Legend entry only — one representative line
        ax.plot([], [], color='#9467bd', lw=1.0, ls=':',
                label=f'Loop edges (n={len(loop_pairs)})')

    ax.set_xlabel(labels[ax_i])
    ax.set_ylabel(labels[ax_j])
    ax.set_title(title)
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_trajectories(raw, corrected, output_dir, loop_pairs=None, gt=None):
    """Emit XY and XZ overlay plots."""
    output_dir = Path(output_dir)
    _plot_overlay(raw, corrected, (0, 1),
                  output_dir / 'trajectory_xy.png',
                  title='Trajectory — top-down (XY)',
                  loop_pairs=loop_pairs, gt=gt)
    _plot_overlay(raw, corrected, (0, 2),
                  output_dir / 'trajectory_xz.png',
                  title='Trajectory — side view (XZ)',
                  loop_pairs=loop_pairs, gt=gt)


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def write_report(raw, corrected, output_path, loop_count=None, ape=None):
    """Write a human-readable text report of the evaluation metrics."""
    ce_raw  = compute_closing_error(raw)
    ce_corr = compute_closing_error(corrected)
    pl_raw  = compute_path_length(raw)
    pl_corr = compute_path_length(corrected)
    reduction_pct = 100.0 * (1.0 - ce_corr / ce_raw) if ce_raw > 0 else 0.0

    lines = [
        "=" * 60,
        "  SLAM Offline PGO — Evaluation Report",
        "=" * 60,
        "",
        f"  Keyframes           : {len(raw)}",
        f"  Loop closures used  : {loop_count if loop_count is not None else 'n/a'}",
        "",
        "  Closing error (end-to-start distance):",
        f"    Raw FAST-LIO      : {ce_raw:8.3f} m",
        f"    After PGO         : {ce_corr:8.3f} m",
        f"    Reduction         : {reduction_pct:7.1f} %",
        "",
        "  Path length:",
        f"    Raw FAST-LIO      : {pl_raw:8.2f} m",
        f"    After PGO         : {pl_corr:8.2f} m",
        "",
        "  Drift rate (closing err / path length):",
        f"    Raw FAST-LIO      : {100.0 * ce_raw / pl_raw:7.2f} %  of path",
        f"    After PGO         : {100.0 * ce_corr / pl_corr:7.2f} %  of path",
        "",
    ]
    if ape is not None:
        lines += [
            "  Absolute Position Error vs ground truth (after PGO):",
            f"    RMSE              : {ape['rmse']:8.3f} m",
            f"    Mean              : {ape['mean']:8.3f} m",
            f"    Median            : {ape['median']:8.3f} m",
            f"    Max               : {ape['max']:8.3f} m",
            "",
        ]
    lines.append("=" * 60)

    text = "\n".join(lines)
    with open(output_path, 'w') as f:
        f.write(text + "\n")
    print(text)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Evaluate raw vs corrected trajectory")
    parser.add_argument('--raw',       required=True, help='Raw FAST-LIO trajectory (.tum)')
    parser.add_argument('--corrected', required=True, help='PGO-corrected trajectory (.tum)')
    parser.add_argument('--output',    required=True, help='Output directory for plots + report')
    parser.add_argument('--loops',     default=None,  help='Optional loop_closure_report.csv to overlay')
    parser.add_argument('--gt',        default=None,  help='Ground truth .tum (optional)')
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    _, raw       = load_tum(args.raw)
    _, corrected = load_tum(args.corrected)
    if len(raw) != len(corrected):
        raise ValueError(
            f"Raw ({len(raw)}) and corrected ({len(corrected)}) trajectories "
            f"have different lengths"
        )

    loop_pairs = _read_loops_csv(args.loops) if args.loops else None

    gt = None
    ape = None
    if args.gt:
        _, gt = load_tum(args.gt)
        if len(gt) == len(corrected):
            ape = compute_ape(corrected, gt)
        else:
            print(f"  [warn] GT has {len(gt)} poses vs corrected {len(corrected)} — skipping APE.")

    plot_trajectories(raw, corrected, output_dir, loop_pairs=loop_pairs, gt=gt)
    write_report(
        raw, corrected,
        output_dir / 'evaluation_report.txt',
        loop_count=len(loop_pairs) if loop_pairs else None,
        ape=ape,
    )
    print(f"\n  Plots + report saved to {output_dir}\n")
