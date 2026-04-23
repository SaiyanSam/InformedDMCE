#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os
import argparse

def plot_metrics(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: Could not find {csv_file}")
        print("Please provide the correct path to view_overlap.csv")
        return

    # Read the CSV data
    df = pd.read_csv(csv_file)

    # Create a figure with 3 stacked subplots sharing the X-axis
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Plot 1: Single Area (Total field of view size at any instant)
    ax1.plot(df['time_s'], df['single_area_m2'], label='Total FOV Area', color='dodgerblue', linewidth=2)
    ax1.set_ylabel('Area ($m^2$)')
    ax1.set_title('Instantaneous Total Field of View Area')
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend()

    # Plot 2: Instantaneous Overlap (Spikes when robots cross paths)
    ax2.plot(df['time_s'], df['multi_area_m2'], label='Overlap Area', color='darkorange', linewidth=2)
    ax2.set_ylabel('Area ($m^2$)')
    ax2.set_title('Instantaneous Overlapping FOV Area')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend()

    # Plot 3: Cumulative Overlap (The "Staircase")
    ax3.plot(df['time_s'], df['cum_multi_area_m2'], label='Cumulative Overlap', color='crimson', linewidth=2)
    ax3.set_xlabel('Time (seconds)')
    ax3.set_ylabel('Cumulative Area ($m^2$)')
    ax3.set_title('Cumulative Overlapping Area Over Time')
    ax3.grid(True, linestyle='--', alpha=0.7)
    ax3.legend()

    # Format the layout to look clean
    plt.tight_layout()
    
    # NEW: Define the plot directory relative to the workspace root
    # This assumes the script is run from the project root or dmce_sim exists nearby
    plot_dir = 'dmce_sim/plots'
    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)
        print(f"Created directory: {plot_dir}")
    
    # Create a filename based on the input CSV but save it in the plots folder
    # Example: dmce_sim/plots/view_overlap_7robots_plot.png
    base_name = os.path.basename(csv_file).replace('.csv', '')
    # Optional: include robot count in name if found in path
    if 'robots' in csv_file:
        robot_count = csv_file.split('/')[-3] # Extracts '7robots' from path
        save_path = os.path.join(plot_dir, f"{base_name}_{robot_count}_plot.png")
    else:
        save_path = os.path.join(plot_dir, f"{base_name}_plot.png")
    
    # Save the figure
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Plot successfully saved to: {save_path}")
    
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot overlap metrics from view_overlap.csv')
    parser.add_argument('csv_path', nargs='?', help='Path to view_overlap.csv')
    args = parser.parse_args()

    # If no argument is passed, try to guess the default path relative to the script location
    if args.csv_path:
        target_csv = args.csv_path
    else:
        # Assuming the script is run from inside dmce/swarm_metrics/scripts/
        # and logs are in dmce/dmce_sim/logs/5robots/dmcts/view_overlap.csv
        script_dir = os.path.dirname(os.path.abspath(__file__))
        target_csv = os.path.join(script_dir, '../../dmce_sim/logs/5robots/dmcts/view_overlap.csv')
        
    print(f"Loading data from: {target_csv}")
    plot_metrics(target_csv)
