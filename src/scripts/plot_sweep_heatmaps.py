#!/usr/bin/env python3
import pandas as pd
import matplotlib
matplotlib.use('Agg') # Use headless backend to prevent X11 display errors
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.colors import ListedColormap, BoundaryNorm, LinearSegmentedColormap
import argparse
from pathlib import Path

def generate_sweep_heatmaps(csv_path):
    csv_file = Path(csv_path)
    
    # 1. Load Data
    if not csv_file.exists():
        print(f"Error: Data file not found at {csv_path}")
        return
        
    df = pd.read_csv(csv_file)

    # 2. Calculate Frame Drop Rate
    baseline_frames = df['total_frames'].max()
    df['frame_drop_percent'] = (1 - (df['total_frames'] / baseline_frames)) * 100
    df['frame_drop_percent'] = df['frame_drop_percent'].clip(lower=0)

    # 3. Pivot Data into 2D Matrices
    drop_rate_matrix = df.pivot(index="k_stride", columns="v_size", values="frame_drop_percent")
    latency_matrix = df.pivot(index="k_stride", columns="v_size", values="avg_latency_ms")

    # Sort Y-axis so k=1 is at the top
    drop_rate_matrix = drop_rate_matrix.sort_index(ascending=True)
    latency_matrix = latency_matrix.sort_index(ascending=True)

    # 4. Setup color maps and drawing functions
    sns.set_theme(style="white")

    # ---------------------------------------------------------
    # Heatmap Config: Frame Drop Rate (%) - SMOOTH GRADIENT
    # ---------------------------------------------------------
    max_drop_val = max(df['frame_drop_percent'].max(), 15.0)
    
    node_1 = 1.0 / max_drop_val
    node_3 = 3.0 / max_drop_val
    node_10 = 10.0 / max_drop_val
    
    nodes_drop = [0.0, node_1, node_3, node_10, 1.0]
    colors_drop = ['#2ca02c', '#ffcf0e', '#ff7f0e', '#d62728', '#8b0000'] # Green, Yellow, Orange, Red, Dark Red
    
    cmap_drops_gradient = LinearSegmentedColormap.from_list("DropGradient", list(zip(nodes_drop, colors_drop)))
    norm_drops = plt.Normalize(vmin=0, vmax=max_drop_val)

    def draw_drops(ax):
        sns.heatmap(drop_rate_matrix, ax=ax, annot=True, fmt=".1f", cmap=cmap_drops_gradient, norm=norm_drops,
                    linewidths=1, linecolor='black', cbar_kws={'label': 'Frame Drop Rate (%)'})
        ax.set_title('Frame Drop Rate (%)', fontsize=14, pad=10)
        ax.set_xlabel('Voxel Grid Size: $v$ (meters)', fontsize=12)
        ax.set_ylabel('Point Filter: $k$', fontsize=12)

    # ---------------------------------------------------------
    # Heatmap Config: Average Latency (ms) - SMOOTH GRADIENT
    # ---------------------------------------------------------
    latency_max_val = max(df['avg_latency_ms'].max(), 150.0)
    
    node_70 = 70.0 / latency_max_val
    node_100 = 100.0 / latency_max_val
    
    nodes = [0.0, node_70, node_100, 1.0]
    colors = ['#2ca02c', '#ffcf0e', '#d62728', '#8b0000'] # Green, Yellow, Red, Dark Red
    
    cmap_latency_gradient = LinearSegmentedColormap.from_list("CustomGradient", list(zip(nodes, colors)))
    norm_latency = plt.Normalize(vmin=0, vmax=latency_max_val)

    def draw_latency(ax):
        sns.heatmap(latency_matrix, ax=ax, annot=True, fmt=".1f", cmap=cmap_latency_gradient, norm=norm_latency,
                    linewidths=1, linecolor='black', cbar_kws={'label': 'Avg Latency (ms)'})
        ax.set_title('Average Processing Latency (ms)', fontsize=14, pad=10)
        ax.set_xlabel('Voxel Grid Size: $v$ (meters)', fontsize=12)
        ax.set_ylabel('Point Filter: $k$', fontsize=12)

    # 5. Generate and Save All Three Plots
    # PLOT 1: Combined Side-by-Side
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('FAST-LIO2 Compute Trade-off on Raspberry Pi 5', fontsize=16, fontweight='bold')
    draw_drops(axes[0])
    draw_latency(axes[1])
    plt.tight_layout()
    plt.subplots_adjust(top=0.88)
    out_combined = csv_file.parent / 'compute_tradeoff_combined.png'
    plt.savefig(str(out_combined), dpi=300, bbox_inches='tight')
    print(f"Combined heatmap saved to: {out_combined}")
    plt.close()

    # PLOT 2: Only Frame Drop Rate
    fig, ax = plt.subplots(figsize=(8, 6))
    draw_drops(ax)
    plt.tight_layout()
    out_drops = csv_file.parent / 'compute_tradeoff_drops.png'
    plt.savefig(str(out_drops), dpi=300, bbox_inches='tight')
    print(f"Frame Drop Rate heatmap saved to: {out_drops}")
    plt.close()

    # PLOT 3: Only Average Latency
    fig, ax = plt.subplots(figsize=(8, 6))
    draw_latency(ax)
    plt.tight_layout()
    out_latency = csv_file.parent / 'compute_tradeoff_latency.png'
    plt.savefig(str(out_latency), dpi=300, bbox_inches='tight')
    print(f"Average Latency heatmap saved to: {out_latency}")
    plt.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate Heatmaps from Sweep Data")
    parser.add_argument("csv_path", 
                        help="Path to the sweep_results_matrix.csv file")
    args = parser.parse_args()
    
    generate_sweep_heatmaps(args.csv_path)