#!/usr/bin/env python3
"""Interactive plot of the net world-frame X-force over time.

Reads fx from rocket_trajectory.csv -- RocketKinematics::computeNetForce()'s
world-frame X-component (thrust_ned.x() + drag_ned.x(); gravity never
contributes to X). Opens a matplotlib window (same pattern as
trajectory.py/tvc.py) and blocks until it's closed.
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt


def plot_x_force(csv_path):
    df = pd.read_csv(csv_path)
    t = df['time'].values
    fx = df['fx'].values

    fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
    fig.suptitle('Net World-Frame X-Force vs Time', fontsize=13, fontweight='bold')

    ax.plot(t, fx, color='#c0392b', lw=1.3)
    ax.axhline(0, color='0.6', lw=0.8, ls=':')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Fx (N)')
    ax.set_title('fx = thrust_ned.x() + drag_ned.x()  (gravity has no X-component)')
    ax.grid(alpha=0.3)

    plt.show()  # blocks here until the window is closed


if __name__ == '__main__':
    csv_path = sys.argv[1] if len(sys.argv) > 1 else 'rocket_trajectory.csv'
    plot_x_force(csv_path)
