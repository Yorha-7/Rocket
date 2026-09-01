#!/usr/bin/env python3
"""Plot rocket trajectory from CSV using matplotlib."""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def plot_trajectory(csv_path, output_path):
    """Generate 5-subplot trajectory figure from CSV data."""
    df = pd.read_csv(csv_path)
    t = df['time'].values
    h = df['height'].values
    v = df['velocity'].values
    p = df['pitch'].values

    dt = t[1] - t[0]
    accel = np.gradient(v, dt)

    t_burnout = 1.86
    t_apogee = t[np.argmax(h)]
    h_apogee = np.max(h)
    t_impact = t[-1]

    fig, axes = plt.subplots(3, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle('Rocket Trajectory (3DOF Simulation)', fontsize=14)

    # 1. Height vs Time
    ax = axes[0, 0]
    ax.plot(t, h, 'b-', lw=1, label='Height', markevery=100)
    ax.axvline(t_burnout, color='magenta', ls='--', label='Burnout')
    ax.plot(t_apogee, h_apogee, 'r^', ms=8, label='Apogee')
    ax.plot(t_impact, 0, 'ko', ms=6, label='Impact')
    ax.set(xlabel='Time (s)', ylabel='Height (m)', title='Height vs Time')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 2. Velocity vs Time
    ax = axes[0, 1]
    ax.plot(t, v, 'g-', lw=1, label='Speed', markevery=100)
    ax.set(xlabel='Time (s)', ylabel='Velocity (m/s)', title='Velocity vs Time')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 3. Acceleration vs Time
    ax = axes[1, 0]
    ax.plot(t, accel, 'r-', lw=1, label='Accel', markevery=100)
    ax.axvline(t_burnout, color='magenta', ls='--')
    ax.set(xlabel='Time (s)', ylabel='Accel (m/s^2)', title='Acceleration vs Time')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 4. Pitch Angle vs Time
    ax = axes[1, 1]
    ax.plot(t, p, 'm-', lw=1, label='Pitch', markevery=100)
    ax.set(xlabel='Time (s)', ylabel='Pitch (deg)', title='Pitch Angle vs Time')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 5. Combined Overview (normalized)
    ax = axes[2, 0]
    for data, label, color in [
        ((h - h.min()) / (h.max() - h.min()), 'Height', 'b'),
        ((v - v.min()) / (v.max() - v.min()), 'Velocity', 'g'),
        ((p - p.min()) / (p.max() - p.min()), 'Pitch', 'm')
    ]:
        ax.plot(t, data, f'{color}-', lw=1, label=label, markevery=100)
    ax.set(xlabel='Time (s)', ylabel='Normalized', title='Overview (Normalized)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # Hide empty subplot
    axes[2, 1].set_visible(False)

    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved plot to {output_path}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory.py <input.csv> [output.png]")
        sys.exit(1)
    csv_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else 'rocket_trajectory.png'
    plot_trajectory(csv_path, output_path)