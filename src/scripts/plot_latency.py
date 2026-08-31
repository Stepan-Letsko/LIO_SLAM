import pandas as pd
import matplotlib
matplotlib.use('Agg') # Use headless backend to prevent X11 display errors
import matplotlib.pyplot as plt
import sys
import os

def plot_latency(csv_file, output_image):
    print(f"Generating latency plot from: {csv_file}")
    
    if not os.path.exists(csv_file):
        print(f"Error: CSV file not found at {csv_file}")
        return

    try:
        # Read CSV, handling potential spaces after commas
        df = pd.read_csv(csv_file, skipinitialspace=True)
        
        if df.empty:
            print("Warning: CSV is empty. Skipping plot.")
            return

        # Check for required columns
        if 'math_time' not in df.columns:
            print(f"Error: 'math_time' column not found. Columns: {df.columns}")
            # Fallback to simple plotting if headers are wrong
            return
            
        # Convert to milliseconds
        math_time = df['math_time'] * 1000.0
        
        # Check if io_time exists (it should with the new C++ code)
        if 'io_time' in df.columns:
            io_time = df['io_time'] * 1000.0
            has_io = True
        else:
            io_time = pd.Series([0] * len(df))
            has_io = False

        total_time = math_time + io_time
        
        # Create Plot
        plt.figure(figsize=(12, 6))
        
        # Standard Line Plot 
        plt.plot(total_time, label='Total Processing Time', color='#007acc', linewidth=1)
        plt.fill_between(df.index, total_time, color='#007acc', alpha=0.1) # Very light fill for aesthetics
        plt.title('FAST-LIO2 Processing Latency')

        # Real-time Threshold
        plt.axhline(y=100, color='k', linestyle='--', linewidth=2, label='10Hz Limit (100ms)')
        
        # Labels
        plt.xlabel('Frame Index')
        plt.ylabel('Time (ms)')
        plt.legend(loc='upper left')
        plt.grid(True, linestyle=':', alpha=0.6)
        
        # Statistics Box
        avg_val = total_time.mean()
        max_val = total_time.max()
        
        stats_text = (
            f"Avg Total: {avg_val:.2f} ms\n"
            f"Max Total: {max_val:.2f} ms"
        )
        
        if has_io:
            stats_text += f"\nAvg Math:  {math_time.mean():.2f} ms"
            stats_text += f"\nAvg I/O:   {io_time.mean():.2f} ms"

        # Statistics Box - Updated for Top Right
        plt.text(0.98, 0.95, stats_text, 
                 transform=plt.gca().transAxes, 
                 verticalalignment='top',
                 horizontalalignment='right', # Ensures text stays inside the frame
                 bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

        # Save
        plt.savefig(output_image, dpi=150)
        print(f"Latency plot saved to: {output_image}")
        plt.close()
        
    except Exception as e:
        print(f"Error plotting latency: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 plot_latency.py [CSV_FILE] [OUTPUT_IMAGE_PATH]")
    else:
        plot_latency(sys.argv[1], sys.argv[2])
