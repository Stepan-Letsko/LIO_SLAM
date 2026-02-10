# Author: Stepan Letsko
# Date: February 10, 2026
# Purpose: This script visualizes ScanContext descriptors saved as CSV files.
#          It reads the flattened descriptor matrices, reshapes them into 2D images,
#          and provides an interactive Matplotlib interface to navigate through
#          the sequence of keyframes. This is useful for debugging loop closure
#          detection and verifying the quality of the descriptors.

import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import os

# --- Configuration (Must match C++ Scancontext.h) ---
PC_NUM_RING = 20
PC_NUM_SECTOR = 60

def main():
    # 1. Get File Path
    # Check command line arguments for input file, default to "scan_contexts.csv"
    default_path = "scan_contexts.csv"
    file_path = sys.argv[1] if len(sys.argv) > 1 else default_path

    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        print(f"Usage: python3 visualize_scan_context.py /path/to/scan_contexts.csv")
        return

    print(f"Loading {file_path}...")

    # 2. Load Data
    # Initialize lists to store timestamps and the 2D descriptor images
    timestamps = []
    images = []

    try:
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                if not row: continue
                try:
                    # Column 0: Timestamp
                    ts = float(row[0])
                    
                    # Columns 1 to End: Pixel Data
                    # Expected size: 20 * 60 = 1200 floats
                    data = np.array([float(x) for x in row[1:]])
                    
                    if len(data) != PC_NUM_RING * PC_NUM_SECTOR:
                        print(f"Skipping malformed row {i}: Found {len(data)} pixels, expected {PC_NUM_RING * PC_NUM_SECTOR}")
                        continue

                    # Reshape 1D array back to 2D Matrix (20 rows, 60 cols)
                    # The C++ implementation saves the matrix row-by-row.
                    # We reshape it back to (Ring, Sector) format.
                    img = data.reshape(PC_NUM_RING, PC_NUM_SECTOR)
                    
                    timestamps.append(ts)
                    images.append(img)
                except ValueError:
                    continue # Skip header or bad lines

    except Exception as e:
        print(f"Error reading file: {e}")
        return

    num_frames = len(images)
    print(f"Successfully loaded {num_frames} frames.")

    if num_frames == 0:
        print("No valid frames found.")
        return

    # 3. Interactive Visualization
    # Create a figure and axis for plotting
    fig, ax = plt.subplots(figsize=(10, 6))
    current_idx = 0

    # Render the first frame
    # cmap='jet': Uses the Jet colormap (Blue=Low, Red=High) which is standard for depth/height data.
    # aspect='auto': Allows the pixels to be non-square to fill the plot area.
    # origin='lower': Ensures Ring 0 (closest to robot) is at the bottom of the Y-axis.
    img_display = ax.imshow(images[current_idx], cmap='jet', aspect='auto', origin='lower')
    
    ax.set_xlabel('Sector (Azimuth 0-360°)')
    ax.set_ylabel('Ring (Distance Near->Far)')
    cbar = plt.colorbar(img_display, ax=ax)
    cbar.set_label('Max Height (Z)')

    title_text = ax.set_title(f"Frame {current_idx+1}/{num_frames} | Time: {timestamps[current_idx]:.2f}s")

    print("\n--- Controls ---")
    print(" [Right Arrow] : Next Frame")
    print(" [Left Arrow]  : Previous Frame")
    print(" [Q]           : Quit")

    def update_plot():
        """Updates the image data and title when the frame index changes."""
        # Update image data
        img_display.set_data(images[current_idx])
        # Auto-scale colors for this specific frame (optional, helps contrast)
        img_display.set_clim(vmin=np.min(images[current_idx]), vmax=np.max(images[current_idx]))
        # Update Title
        title_text.set_text(f"Frame {current_idx+1}/{num_frames} | Time: {timestamps[current_idx]:.2f}s")
        fig.canvas.draw_idle()

    def on_key(event):
        """Handles keyboard events for navigation."""
        nonlocal current_idx
        if event.key == 'right':
            current_idx = (current_idx + 1) % num_frames
            update_plot()
        elif event.key == 'left':
            current_idx = (current_idx - 1 + num_frames) % num_frames
            update_plot()
        elif event.key == 'q':
            plt.close()

    # Connect the key press event to the handler
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()

if __name__ == "__main__":
    main()
