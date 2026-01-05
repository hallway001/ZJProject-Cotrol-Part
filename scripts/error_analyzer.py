#!/usr/bin/env python3
"""
Error analysis tool for steering error logs.
Analyzes CSV log files and generates statistics and plots.
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import os


def load_error_log(csv_file):
    """Load steering error log from CSV file."""
    try:
        df = pd.read_csv(csv_file)
        return df
    except Exception as e:
        print(f"Error loading log file {csv_file}: {e}")
        return None


def calculate_statistics(df):
    """Calculate error statistics."""
    stats = {}
    
    if 'error' in df.columns:
        errors = df['error'].abs()
        stats['rmse'] = np.sqrt(np.mean(df['error'] ** 2))
        stats['mean_error'] = np.mean(df['error'])
        stats['std_error'] = np.std(df['error'])
        stats['max_error'] = errors.max()
        stats['min_error'] = errors.min()
        stats['median_error'] = errors.median()
        stats['p95_error'] = errors.quantile(0.95)
        stats['p99_error'] = errors.quantile(0.99)
    
    return stats


def plot_error_analysis(df, output_dir):
    """Generate error analysis plots."""
    if df is None or df.empty:
        print("No data to plot")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Steering Error Analysis', fontsize=16)
    
    # Time series plot
    if 'timestamp' in df.columns and 'error' in df.columns:
        ax = axes[0, 0]
        ax.plot(df['timestamp'], df['error'], label='Error', alpha=0.7)
        ax.axhline(y=0, color='r', linestyle='--', label='Zero')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Steering Error (rad)')
        ax.set_title('Error Time Series')
        ax.legend()
        ax.grid(True)
    
    # Error distribution histogram
    if 'error' in df.columns:
        ax = axes[0, 1]
        ax.hist(df['error'], bins=50, alpha=0.7, edgecolor='black')
        ax.set_xlabel('Steering Error (rad)')
        ax.set_ylabel('Frequency')
        ax.set_title('Error Distribution')
        ax.grid(True, alpha=0.3)
    
    # Error vs Velocity
    if 'velocity_x' in df.columns and 'error' in df.columns:
        ax = axes[1, 0]
        ax.scatter(df['velocity_x'], df['error'].abs(), alpha=0.5, s=10)
        ax.set_xlabel('Velocity (m/s)')
        ax.set_ylabel('Absolute Error (rad)')
        ax.set_title('Error vs Velocity')
        ax.grid(True, alpha=0.3)
    
    # Error vs Curvature
    if 'curvature' in df.columns and 'error' in df.columns:
        ax = axes[1, 1]
        ax.scatter(df['curvature'], df['error'].abs(), alpha=0.5, s=10)
        ax.set_xlabel('Path Curvature (1/m)')
        ax.set_ylabel('Absolute Error (rad)')
        ax.set_title('Error vs Curvature')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot
    output_file = os.path.join(output_dir, 'error_analysis.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    plt.close()


def generate_report(df, stats, output_file):
    """Generate text report."""
    with open(output_file, 'w') as f:
        f.write("=" * 60 + "\n")
        f.write("Steering Error Analysis Report\n")
        f.write("=" * 60 + "\n\n")
        
        f.write("Statistics:\n")
        f.write("-" * 60 + "\n")
        for key, value in stats.items():
            f.write(f"{key:20s}: {value:.6f}\n")
        
        f.write("\n" + "=" * 60 + "\n")
        f.write(f"Total samples: {len(df)}\n")
        
        if 'timestamp' in df.columns:
            duration = df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]
            f.write(f"Duration: {duration:.2f} s\n")
    
    print(f"Report saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze steering error logs',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        'input_file',
        type=str,
        help='Input CSV log file'
    )
    
    parser.add_argument(
        '-o', '--output-dir',
        type=str,
        default='.',
        help='Output directory for plots and reports'
    )
    
    parser.add_argument(
        '-p', '--plot',
        action='store_true',
        help='Generate plots'
    )
    
    parser.add_argument(
        '-r', '--report',
        action='store_true',
        default=True,
        help='Generate text report (default: True)'
    )
    
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Load data
    print(f"Loading log file: {args.input_file}")
    df = load_error_log(args.input_file)
    
    if df is None or df.empty:
        print("Failed to load data or data is empty")
        return
    
    print(f"Loaded {len(df)} data points")
    
    # Calculate statistics
    stats = calculate_statistics(df)
    
    print("\nError Statistics:")
    print("-" * 40)
    for key, value in stats.items():
        print(f"{key:20s}: {value:.6f}")
    
    # Generate report
    if args.report:
        report_file = os.path.join(args.output_dir, 'error_report.txt')
        generate_report(df, stats, report_file)
    
    # Generate plots
    if args.plot:
        plot_error_analysis(df, args.output_dir)


if __name__ == '__main__':
    main()

