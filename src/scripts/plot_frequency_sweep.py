#!/usr/bin/env python3
import pandas as pd
import matplotlib
matplotlib.use('Agg') # Use headless backend
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
from pathlib import Path

def plot_frequency_sweep(csv_path):
    csv_file = Path(csv_path)
    if not csv_file.exists():
        print(f"Error: {csv_path} not found.")
        return

    df = pd.read_csv(csv_file)
    
    # 1. Filter and prepare datasets
    df_baseline = df[df['Experiment'] == 'Baseline'].copy()
    
    df_imu = df[df['Experiment'] == 'IMU_Starvation'].copy()
    if not df_baseline.empty:
        bl_imu = df_baseline.iloc[0].copy()
        bl_imu['Experiment'] = 'IMU_Starvation'
        bl_imu['Target_Hz'] = 100.0
        df_imu = pd.concat([pd.DataFrame([bl_imu]), df_imu], ignore_index=True)
        
    df_lidar = df[df['Experiment'] == 'LiDAR_Starvation'].copy()
    if not df_baseline.empty:
        bl_lidar = df_baseline.iloc[0].copy()
        bl_lidar['Experiment'] = 'LiDAR_Starvation'
        bl_lidar['Target_Hz'] = 10.0
        df_lidar = pd.concat([pd.DataFrame([bl_lidar]), df_lidar], ignore_index=True)
    
    # Convert Target_Hz to float
    df_imu['Target_Hz'] = df_imu['Target_Hz'].astype(float)
    df_lidar['Target_Hz'] = df_lidar['Target_Hz'].astype(float)
    
    # Sort descending
    df_imu = df_imu.sort_values('Target_Hz', ascending=False)
    df_lidar = df_lidar.sort_values('Target_Hz', ascending=False)

    # 2. Calculate % Change for APE 
    # (Replacing 0.0 with a 1mm EPSILON to avoid divide-by-zero math errors)
    EPSILON = 0.001  

    if not df_imu.empty:
        base_imu_ape = df_imu['APE_RMSE_m'].iloc[0]
        if base_imu_ape <= 0.0: base_imu_ape = EPSILON
        df_imu['APE_Pct_Change'] = ((df_imu['APE_RMSE_m'].clip(lower=EPSILON) - base_imu_ape) / base_imu_ape) * 100
        
    if not df_lidar.empty:
        base_lid_ape = df_lidar['APE_RMSE_m'].iloc[0]
        if base_lid_ape <= 0.0: base_lid_ape = EPSILON
        df_lidar['APE_Pct_Change'] = ((df_lidar['APE_RMSE_m'].clip(lower=EPSILON) - base_lid_ape) / base_lid_ape) * 100

    # Set aesthetic theme
    sns.set_theme(style="whitegrid")
    
    # ---------------------------------------------------------
    # PLOT 1: Absolute Pose Error (m)
    # ---------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle('FAST-LIO2 Sensor Starvation: Absolute Pose Error', fontsize=16, fontweight='bold')
    
    if not df_imu.empty:
        axes[0].plot(df_imu['Target_Hz'].astype(str) + " Hz", df_imu['APE_RMSE_m'], 
                     marker='o', color='#d62728', linewidth=2.5, markersize=8)
        axes[0].set_title('IMU Degradation', fontsize=14, pad=10)
        axes[0].set_xlabel('IMU Frequency', fontsize=12)
        axes[0].set_ylabel('Absolute Pose Error RMSE (m)', fontsize=12)
        axes[0].set_ylim(bottom=0)

    if not df_lidar.empty:
        axes[1].plot(df_lidar['Target_Hz'].astype(str) + " Hz", df_lidar['APE_RMSE_m'], 
                     marker='s', color='#1f77b4', linewidth=2.5, markersize=8)
        axes[1].set_title('LiDAR Degradation', fontsize=14, pad=10)
        axes[1].set_xlabel('LiDAR Frequency', fontsize=12)
        axes[1].set_ylabel('Absolute Pose Error RMSE (m)', fontsize=12)
        axes[1].set_ylim(bottom=0)

    plt.tight_layout()
    out_abs = csv_file.parent / 'degradation_absolute_ape.png'
    plt.savefig(str(out_abs), dpi=300, bbox_inches='tight')
    plt.close()

    # ---------------------------------------------------------
    # PLOT 2: Relative (%) Change
    # ---------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle('FAST-LIO2 Sensor Starvation: APE Relative Growth (%)', fontsize=16, fontweight='bold')
    
    if not df_imu.empty:
        axes[0].plot(df_imu['Target_Hz'].astype(str) + " Hz", df_imu['APE_Pct_Change'], 
                     marker='o', color='#ff7f0e', linewidth=2.5, markersize=8)
        axes[0].set_title('IMU Degradation', fontsize=14, pad=10)
        axes[0].set_xlabel('IMU Frequency', fontsize=12)
        axes[0].set_ylabel('APE Increase (%)', fontsize=12)
        axes[0].set_ylim(bottom=0)

    if not df_lidar.empty:
        axes[1].plot(df_lidar['Target_Hz'].astype(str) + " Hz", df_lidar['APE_Pct_Change'], 
                     marker='s', color='#2ca02c', linewidth=2.5, markersize=8)
        axes[1].set_title('LiDAR Degradation', fontsize=14, pad=10)
        axes[1].set_xlabel('LiDAR Frequency', fontsize=12)
        axes[1].set_ylabel('APE Increase (%)', fontsize=12)
        axes[1].set_ylim(bottom=0)

    plt.tight_layout()
    out_rel = csv_file.parent / 'degradation_relative_ape.png'
    plt.savefig(str(out_rel), dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"✅ Absolute plot saved to: {out_abs}")
    print(f"✅ Relative plot saved to: {out_rel}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Plot Frequency Sweep Results")
    parser.add_argument("csv_path", help="Path to frequency_results_matrix.csv")
    args = parser.parse_args()
    plot_frequency_sweep(args.csv_path)