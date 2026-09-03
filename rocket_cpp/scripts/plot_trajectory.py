#!/usr/bin/env python3
"""Plot rocket trajectory from CSV using matplotlib."""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def plot_trajectory(csv_path, output_path):
    """Generate 7-subplot trajectory figure from CSV data."""
    df = pd.read_csv(csv_path)
    t = df['time'].values
    h = df['height'].values
    v = df['velocity'].values
    p = df['pitch'].values
    
    # Load angular data if available
    has_angular = 'ang_vel' in df.columns and 'ang_accel' in df.columns
    if has_angular:
        ang_vel = df['ang_vel'].values
        ang_accel = df['ang_accel'].values
    
    # Trim data at impact: find first zero-height after apogee
    apogee_idx = np.argmax(h)
    impact_candidates = np.where(h[apogee_idx:] <= 1e-6)[0]
    if len(impact_candidates) > 0:
        impact_idx = apogee_idx + impact_candidates[0]
        t = t[:impact_idx + 1]
        h = h[:impact_idx + 1]
        v = v[:impact_idx + 1]
        p = p[:impact_idx + 1]
        if has_angular:
            ang_vel = ang_vel[:impact_idx + 1]
            ang_accel = ang_accel[:impact_idx + 1]
    
    dt = t[1] - t[0]
    accel = np.gradient(v, dt)
    
    t_burnout = 1.86
    t_apogee = t[np.argmax(h)]
    h_apogee = np.max(h)
    t_impact = t[-1]
    
    # 4x2 grid for 7 plots
    fig, axes = plt.subplots(4, 2, figsize=(12, 10), constrained_layout=True)
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
    
    # 6. Angular Velocity vs Time
    ax = axes[2, 0]
    if has_angular:
        ax.plot(t, ang_vel, 'c-', lw=1, label='Ang Vel', markevery=100)
        ax.set(xlabel='Time (s)', ylabel='Ang Vel (deg/s)', title='Angular Velocity vs Time')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, 'No angular data', ha='center', va='center', transform=ax.transAxes)
        ax.set(title='Angular Velocity vs Time')
    
    # 7. Angular Acceleration vs Time
    ax = axes[2, 1]
    if has_angular:
        ax.plot(t, ang_accel, 'y-', lw=1, label='Ang Accel', markevery=100)
        ax.set(xlabel='Time (s)', ylabel='Ang Accel (deg/s^2)', title='Angular Acceleration vs Time')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, 'No angular data', ha='center', va='center', transform=ax.transAxes)
        ax.set(title='Angular Acceleration vs Time')
    
    # 5. Combined Overview (normalized)
    ax = axes[3, 0]
    for data, label, color in [
        ((h - h.min()) / (h.max() - h.min()), 'Height', 'b'),
        ((v - v.min()) / (v.max() - v.min()), 'Velocity', 'g'),
        ((p - p.min()) / (p.max() - p.min()), 'Pitch', 'm'),
        ((ang_vel - ang_vel.min()) / (ang_vel.max() - ang_vel.min()) if has_angular else np.zeros_like(h), 'Ang Vel', 'c'),
    ]:
        ax.plot(t, data, f'{color}-', lw=1, label=label, markevery=100)
    ax.set(xlabel='Time (s)', ylabel='Normalized', title='Overview (Normalized)')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    
    # 7. Torque components (if we can compute)
    ax = axes[3, 1]
    ax.text(0.5, 0.5, 'Reserved for torque analysis', ha='center', va='center', transform=ax.transAxes)
    ax.set(title='Torque Analysis')
    
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved plot to {output_path}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory.py <input.csv> [output.png]")
        sys.exit(1)
    csv_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else 'rocket_trajectory.png'
    plot_trajectory(csv_path, output_path)