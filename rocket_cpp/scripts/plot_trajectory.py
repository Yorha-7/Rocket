#!/usr/bin/env python3
"""Plot rocket trajectory from CSV using matplotlib."""

import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def plot_trajectory(csv_path, output_path):
    """Generate the 7-subplot flight-analysis figure (output_path, e.g.
    rocket_analysis.png) plus the X/Y/Z-vs-time trajectory figure, saved
    as rocket_trajectory.png next to it."""
    df = pd.read_csv(csv_path)
    t = df['time'].values
    h = df['height'].values
    v = df['velocity'].values
    p = df['pitch'].values

    has_xy = {'x', 'y'}.issubset(df.columns)
    if has_xy:
        x = df['x'].values
        y = df['y'].values

    # Load angular data if available
    has_angular = 'ang_vel' in df.columns and 'ang_accel' in df.columns
    if has_angular:
        ang_vel = df['ang_vel'].values
        ang_accel = df['ang_accel'].values

    # Force (world-frame) and pitch-torque breakdown, if this CSV has them
    has_forces = {'fx', 'fy', 'fz'}.issubset(df.columns)
    if has_forces:
        fx, fy, fz = df['fx'].values, df['fy'].values, df['fz'].values
    has_torques = {'torque_gravity', 'torque_aero', 'torque_damping'}.issubset(df.columns)
    if has_torques:
        torque_gravity = df['torque_gravity'].values
        torque_aero = df['torque_aero'].values
        torque_damping = df['torque_damping'].values

    # Trim data at impact: find first zero-height after apogee
    apogee_idx = np.argmax(h)
    impact_candidates = np.where(h[apogee_idx:] <= 1e-6)[0]
    if len(impact_candidates) > 0:
        impact_idx = apogee_idx + impact_candidates[0]
        t = t[:impact_idx + 1]
        h = h[:impact_idx + 1]
        v = v[:impact_idx + 1]
        p = p[:impact_idx + 1]
        if has_xy:
            x = x[:impact_idx + 1]
            y = y[:impact_idx + 1]
        if has_angular:
            ang_vel = ang_vel[:impact_idx + 1]
            ang_accel = ang_accel[:impact_idx + 1]
        if has_forces:
            fx, fy, fz = fx[:impact_idx + 1], fy[:impact_idx + 1], fz[:impact_idx + 1]
        if has_torques:
            torque_gravity = torque_gravity[:impact_idx + 1]
            torque_aero = torque_aero[:impact_idx + 1]
            torque_damping = torque_damping[:impact_idx + 1]
    
    dt = t[1] - t[0]

    # The final sample is the impact frame: ground termination snaps
    # velocity/angular velocity to zero there instantaneously. That's a
    # real "the rocket stopped" state, but differentiating across it
    # produces a fake, huge acceleration spike -- drop that one sample
    # before computing/plotting either acceleration series.
    accel_t = t[:-1]
    accel = np.gradient(v[:-1], dt)
    if has_angular:
        ang_accel_t = t[:-1]
        ang_accel = ang_accel[:-1]

    t_burnout = 1.86
    t_apogee = t[np.argmax(h)]
    h_apogee = np.max(h)
    t_impact = t[-1]
    
    # 4x2 grid for 7 plots
    fig, axes = plt.subplots(4, 2, figsize=(12, 10), constrained_layout=True)
    fig.suptitle('Rocket Flight Analysis (3DOF Simulation)', fontsize=14)
    
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
    ax.plot(accel_t, accel, 'r-', lw=1, label='Accel', markevery=100)
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
        ax.plot(ang_accel_t, ang_accel, 'y-', lw=1, label='Ang Accel', markevery=100)
        ax.set(xlabel='Time (s)', ylabel='Ang Accel (deg/s^2)', title='Angular Acceleration vs Time')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, 'No angular data', ha='center', va='center', transform=ax.transAxes)
        ax.set(title='Angular Acceleration vs Time')
    
    # 5. Forces vs Time (net world-frame force, per axis)
    ax = axes[3, 0]
    if has_forces:
        ax.plot(t, fx, 'r-', lw=1, label='Fx (North)')
        ax.plot(t, fy, 'g-', lw=1, label='Fy (East)')
        ax.plot(t, fz, 'b-', lw=1, label='Fz (Up)')
        ax.axvline(t_burnout, color='magenta', ls='--')
        ax.set(xlabel='Time (s)', ylabel='Force (N)', title='Forces vs Time')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, 'No force data', ha='center', va='center', transform=ax.transAxes)
        ax.set(title='Forces vs Time')

    # 6. Torque Analysis: the three components that add up to pitch torque
    ax = axes[3, 1]
    if has_torques:
        ax.plot(t, torque_gravity, 'r-', lw=1, label='Gravity')
        ax.plot(t, torque_aero, 'g-', lw=1, label='Aerodynamic')
        ax.plot(t, torque_damping, 'b-', lw=1, label='Damping')
        ax.axvline(t_burnout, color='magenta', ls='--')
        ax.set(xlabel='Time (s)', ylabel='Torque (N*m)', title='Torque Analysis')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    else:
        ax.text(0.5, 0.5, 'No torque data', ha='center', va='center', transform=ax.transAxes)
        ax.set(title='Torque Analysis')
    
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved plot to {output_path}")

    # Second figure: x, y, z (height) position components vs time, one
    # subplot each -- this is the "where is it pointing horizontally"
    # counterpart to the height-only plot above.
    if has_xy:
        out_dir = os.path.dirname(output_path)
        trajectory_path = os.path.join(out_dir, 'rocket_trajectory.png') if out_dir else 'rocket_trajectory.png'
        fig2, axes2 = plt.subplots(3, 1, figsize=(8, 9), constrained_layout=True, sharex=True)
        fig2.suptitle('Rocket Position Components vs Time', fontsize=14)

        # Each subplot auto-scales its own y-axis to fill the same panel
        # height, independent of the others -- so a few meters of X drift
        # gets drawn just as tall as a 300+ m altitude swing. Put the real
        # range in the title so the true scale is obvious without having
        # to compare tick labels across panels.
        x_range, y_range, z_range = np.ptp(x), np.ptp(y), np.ptp(h)

        axes2[0].plot(t, x, 'r-', lw=1)
        axes2[0].set(ylabel='X (m)', title=f'X (North) vs Time  [range: {x_range:.2f} m]')
        axes2[0].grid(alpha=0.3)

        axes2[1].plot(t, y, 'g-', lw=1)
        axes2[1].set(ylabel='Y (East, m)', title=f'Y (East) vs Time  [range: {y_range:.2f} m]')
        axes2[1].grid(alpha=0.3)

        axes2[2].plot(t, h, 'b-', lw=1)
        axes2[2].set(xlabel='Time (s)', ylabel='Z / Height (m)', title=f'Z (Height) vs Time  [range: {z_range:.2f} m]')
        axes2[2].grid(alpha=0.3)

        plt.savefig(trajectory_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {trajectory_path}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory.py <input.csv> [output.png]")
        sys.exit(1)
    csv_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else 'rocket_analysis.png'
    plot_trajectory(csv_path, output_path)