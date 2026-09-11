#!/usr/bin/env python3
"""Interactive 3D trajectory viewer.

Reads the rocket's x, y, z position history from the simulation CSV and
opens one interactive matplotlib 3D figure (rotate/zoom with the mouse).
The window stays open until the user closes it; the program then exits.
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 -- registers the '3d' projection


def plot_trajectory_3d(csv_path):
    df = pd.read_csv(csv_path)
    t = df['time'].values
    x = df['x'].values
    y = df['y'].values
    z = df['height'].values

    # Trim at ground impact, same rule as plot_trajectory.py: first
    # near-zero height sample after apogee.
    apogee_idx = np.argmax(z)
    impact_candidates = np.where(z[apogee_idx:] <= 1e-6)[0]
    if len(impact_candidates) > 0:
        impact_idx = apogee_idx + impact_candidates[0]
        t, x, y, z = t[:impact_idx + 1], x[:impact_idx + 1], y[:impact_idx + 1], z[:impact_idx + 1]

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Color the path by time so direction of travel is visible at a glance.
    sc = ax.scatter(x, y, z, c=t, cmap='viridis', s=4)
    ax.plot(x, y, z, color='gray', lw=0.5, alpha=0.6)

    ax.plot([x[0]], [y[0]], [z[0]], 'go', ms=8, label='Launch')
    ax.plot([x[apogee_idx]], [y[apogee_idx]], [z[apogee_idx]], 'r^', ms=8, label='Apogee')
    ax.plot([x[-1]], [y[-1]], [z[-1]], 'ko', ms=8, label='Impact')

    # Navigation target + success/fail: only present in CSVs from a run
    # that actually passed a Navigation object to simulate() -- older/
    # non-guided CSVs just don't have these columns, so this whole block
    # is skipped rather than erroring.
    extent = [x.max(), y.max(), z.max()]
    has_target = {'target_x', 'target_y', 'target_z'}.issubset(df.columns)
    if has_target:
        # Constant for the whole flight (Navigation's target doesn't move
        # mid-flight yet), so any row's value is the target.
        tx, ty, tz = df['target_x'].values[0], df['target_y'].values[0], df['target_z'].values[0]
        extent += [tx, ty, tz]
        ax.plot([tx], [ty], [tz], marker='*', color='#9b59b6', ms=16,
                 linestyle='None', label='Target', zorder=10)

        # Closest approach over the whole (trimmed, pre-impact) flight --
        # not just the final position -- since this is open-loop "point
        # the nose at it" guidance with no terminal intercept phase, the
        # most meaningful measure of whether it "reached" the target is
        # how close the flight path ever got, not where it ended up after
        # coasting/falling past it.
        dist = np.sqrt((x - tx)**2 + (y - ty)**2 + (z - tz)**2)
        closest_idx = np.argmin(dist)
        closest_dist = dist[closest_idx]

        # 20m against a flight covering hundreds of meters, with no
        # terminal-phase correction and pure open-loop "aim the nose"
        # guidance, is a reasonable "close enough" bar -- not a
        # millimeter-precision intercept requirement.
        TOLERANCE_M = 20.0
        if closest_dist <= TOLERANCE_M:
            label = f'SUCCESS -- reached target (closest approach: {closest_dist:.1f} m)'
            color = '#1e8449'
        else:
            label = f'FAIL -- missed target by {closest_dist:.1f} m (closest approach)'
            color = '#c0392b'
        ax.text2D(0.02, 0.02, label, transform=ax.transAxes, fontsize=11,
                   fontweight='bold', color='white',
                   bbox=dict(facecolor=color, edgecolor='none', boxstyle='round,pad=0.4'))

    ax.set_xlabel('X (m, North)')
    ax.set_ylabel('Y (m, East)')
    ax.set_zlabel('Z (m, Height)')
    ax.set_title('Rocket Trajectory (3D)')
    ax.legend()
    fig.colorbar(sc, ax=ax, shrink=0.6, label='Time (s)')

    # A cube box alone isn't enough for a true equal-unit view -- Matplotlib
    # would still stretch each axis to fill it using its OWN data range
    # (x:~40m, y:~8m, z:~360m), so 1 box-unit means a different number of
    # real meters on each axis and angles still read distorted. Pin all
    # three axis limits to the same fixed span too, so 1 meter of x really
    # does look identical to 1 meter of z -- angles become trustworthy
    # again, at the cost of the flight only filling a corner of the cube.
    # Includes the target (when present) so it's never plotted off-frame.
    cube_side = 50.0 * np.ceil(1.1 * max(extent) / 50.0)
    ax.set_xlim(0, cube_side)
    ax.set_ylim(0, cube_side)
    ax.set_zlim(0, cube_side)
    ax.set_box_aspect((1, 1, 1))

    plt.show()  # blocks here until the window is closed


if __name__ == '__main__':
    csv_path = sys.argv[1] if len(sys.argv) > 1 else 'rocket_trajectory.csv'
    plot_trajectory_3d(csv_path)
