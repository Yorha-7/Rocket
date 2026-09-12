#!/usr/bin/env python3
"""Interactive plot of the commanded thrust-vector angle over time.

Reads gimbal_pitch_deg/gimbal_yaw_deg from rocket_trajectory.csv -- the
TVC nozzle's actual (lagged) deflection each step, same value baked into
RocketState.gimbal_pitch_rad/gimbal_yaw_rad by RocketKinematics::step().
Two separate panels, one per axis, since the two are independent
actuators (see README "Thrust Vector Control"). Opens a matplotlib window
(same pattern as trajectory.py) and blocks until it's closed.
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


def plot_tvc_command(csv_path):
    df = pd.read_csv(csv_path)
    t = df['time'].values
    gimbal_pitch = df['gimbal_pitch_deg'].values
    gimbal_yaw = df['gimbal_yaw_deg'].values

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True, constrained_layout=True)
    fig.suptitle('Commanded Thrust Vector Angle vs Time', fontsize=13, fontweight='bold')

    axes[0].plot(t, gimbal_pitch, color='#c0392b', lw=1.3)
    axes[0].axhline(0, color='0.6', lw=0.8, ls=':')
    axes[0].set_ylabel('Gimbal pitch (deg)\n(deflects nozzle toward +X)')
    axes[0].set_title('Pitch-axis actuator (body X-Z plane)')
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, gimbal_yaw, color='#1f5fa8', lw=1.3)
    axes[1].axhline(0, color='0.6', lw=0.8, ls=':')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Gimbal yaw (deg)\n(deflects nozzle toward +Y)')
    axes[1].set_title('Yaw-axis actuator (body Y-Z plane)')
    axes[1].grid(alpha=0.3)

    plt.show()  # blocks here until the window is closed


if __name__ == '__main__':
    csv_path = sys.argv[1] if len(sys.argv) > 1 else 'rocket_trajectory.csv'
    plot_tvc_command(csv_path)
