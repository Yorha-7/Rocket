#!/usr/bin/env python3
"""Interactive 3D trajectory viewer AND control panel.

Two modes:
  python3 scripts/trajectory.py                 -- interactive: type a
    target and initial angles, click Run, and this launches a fresh
    (fast/--preview) simulation and redraws the same window with the
    result. This is the normal way to use this script now.
  python3 scripts/trajectory.py <csv_path>       -- plain viewer, no
    controls: shows one existing CSV and exits when the window closes.
    Kept for scripting/backward compatibility.
"""

import sys
import subprocess
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 -- registers the '3d' projection

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
BINARY_PATH = PROJECT_ROOT / "build" / "rocket_cpp"
CSV_PATH = PROJECT_ROOT / "rocket_trajectory.csv"

# Defaults mirror main.cpp's own CliArgs defaults (cli_args.hpp) -- so the
# very first view, before any click, matches what a plain `./build/
# rocket_cpp` with no flags would already produce.
DEFAULTS = {"target_x": "0.0", "target_y": "350.0", "target_z": "1500.0",
            "init_tilt": "0.0", "init_yaw": "10.0"}


def load_trajectory(csv_path):
    """Goal: read one CSV and work out everything draw_trajectory needs
    (trimmed arrays, apogee/burnout/target markers) -- pure data work,
    no plotting, so it's reusable for both the one-shot viewer and the
    interactive redraw path."""
    df = pd.read_csv(csv_path)
    t = df['time'].values
    x = df['x'].values
    y = df['y'].values
    z = df['height'].values
    has_thrust = 'thrust' in df.columns
    thrust = df['thrust'].values if has_thrust else None

    # Trim at ground impact: first near-zero height sample after apogee.
    apogee_idx = np.argmax(z)
    impact_candidates = np.where(z[apogee_idx:] <= 1e-6)[0]
    if len(impact_candidates) > 0:
        impact_idx = apogee_idx + impact_candidates[0]
        t, x, y, z = t[:impact_idx + 1], x[:impact_idx + 1], y[:impact_idx + 1], z[:impact_idx + 1]
        if has_thrust:
            thrust = thrust[:impact_idx + 1]

    # Burnout: last sample with meaningful thrust (0.01 N threshold).
    burnout_idx = None
    if has_thrust:
        powered = np.where(thrust > 0.01)[0]
        if len(powered) > 0:
            burnout_idx = powered[-1]

    target = None
    if {'target_x', 'target_y', 'target_z'}.issubset(df.columns):
        target = (df['target_x'].values[0], df['target_y'].values[0], df['target_z'].values[0])

    return {"t": t, "x": x, "y": y, "z": z, "apogee_idx": apogee_idx,
            "burnout_idx": burnout_idx, "target": target}


def draw_trajectory(ax, data, colorbar_holder):
    """Goal: draw one trajectory onto an EXISTING 3D axes, clearing it
    first -- the piece that makes "Run" update the same window instead
    of opening a new one each click. colorbar_holder is a one-element
    list acting as a mutable box, so the caller's colorbar reference can
    be replaced here without a global."""
    ax.clear()
    t, x, y, z = data["t"], data["x"], data["y"], data["z"]
    apogee_idx, burnout_idx, target = data["apogee_idx"], data["burnout_idx"], data["target"]

    sc = ax.scatter(x, y, z, c=t, cmap='viridis', s=4)
    ax.plot(x, y, z, color='gray', lw=0.5, alpha=0.6)

    ax.plot([x[0]], [y[0]], [z[0]], 'go', ms=8, label='Launch')
    if burnout_idx is not None:
        ax.plot([x[burnout_idx]], [y[burnout_idx]], [z[burnout_idx]], marker='X', color='#e67e22',
                 ms=10, linestyle='None', label='Burnout', zorder=10)
    ax.plot([x[apogee_idx]], [y[apogee_idx]], [z[apogee_idx]], 'r^', ms=8, label='Apogee')
    ax.plot([x[-1]], [y[-1]], [z[-1]], 'ko', ms=8, label='Impact')

    extent = [x.max(), y.max(), z.max()]
    if target is not None:
        tx, ty, tz = target
        extent += [tx, ty, tz]
        ax.plot([tx], [ty], [tz], marker='*', color='#9b59b6', ms=16,
                 linestyle='None', label='Target', zorder=10)

        # Closest approach over the whole flight, not just the final
        # position -- open-loop "point the nose at it" guidance has no
        # terminal intercept phase, so "did it ever get close" is the
        # meaningful measure, not "where did it end up."
        dist = np.sqrt((x - tx)**2 + (y - ty)**2 + (z - tz)**2)
        closest_dist = dist[np.argmin(dist)]

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
    ax.legend(loc='upper left')

    # Goal: true equal-unit axes, not just an equal-looking box -- pin
    # all three limits to the same span (including the target, so it's
    # never off-frame) so 1 meter of x really does look like 1 meter of z.
    cube_side = 50.0 * np.ceil(1.1 * max(extent) / 50.0)
    ax.set_xlim(0, cube_side)
    ax.set_ylim(0, cube_side)
    ax.set_zlim(0, cube_side)
    ax.set_box_aspect((1, 1, 1))

    # Goal: replace the colorbar instead of stacking a new one on every
    # redraw -- fig.colorbar would otherwise add another bar each click.
    if colorbar_holder[0] is not None:
        colorbar_holder[0].remove()
    colorbar_holder[0] = ax.figure.colorbar(sc, ax=ax, shrink=0.6, label='Time (s)')


def plot_trajectory_3d(csv_path):
    """Goal: the old one-shot viewer -- load one CSV, draw it once, block
    until the window closes. No controls, no re-running. Kept for
    backward compatibility (`trajectory.py <csv_path>`)."""
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection='3d')
    draw_trajectory(ax, load_trajectory(csv_path), [None])
    plt.show()


def run_interactive():
    """Goal: the new default entry point -- a control panel (5 text
    boxes + a Run button) next to the 3D plot. Clicking Run launches a
    fresh --preview simulation with the entered parameters and redraws
    the SAME window with the result, instead of opening a new one."""
    fig = plt.figure(figsize=(10, 9))
    AXES_RECT = [0.05, 0.32, 0.9, 0.64]
    plot_state = {"ax": None, "colorbar_holder": [None]}

    def fresh_axes():
        # Goal: recreate the 3D axes (and its colorbar) from scratch on
        # every run, instead of clearing and reusing the same one.
        # fig.colorbar(ax=ax) resizes whatever axes it's attached to, and
        # removing it afterward does NOT fully restore the original size
        # -- a real, confirmed matplotlib quirk: reusing one axes across
        # repeated colorbar create/remove cycles shrinks it a little more
        # each time (measured: axes width 0.56 -> 0.46 -> 0.37 of figure
        # width over 3 redraws). A brand-new axes always starts at exactly
        # AXES_RECT, sidestepping the compounding shrink entirely.
        if plot_state["colorbar_holder"][0] is not None:
            plot_state["colorbar_holder"][0].remove()
            plot_state["colorbar_holder"][0] = None
        if plot_state["ax"] is not None:
            plot_state["ax"].remove()
        plot_state["ax"] = fig.add_axes(AXES_RECT, projection='3d')
        return plot_state["ax"]

    status_ax = fig.add_axes([0.05, 0.24, 0.9, 0.04])
    status_ax.axis('off')
    status_text = status_ax.text(0, 0.5, "", fontsize=10, va='center')

    # Goal: five text boxes, laid out in one row of three (target) and
    # one row of two (initial angles), each prefilled with main.cpp's
    # own defaults.
    box_w, box_h = 0.14, 0.05
    labels_row1 = [("target_x", 0.08), ("target_y", 0.28), ("target_z", 0.48)]
    labels_row2 = [("init_tilt", 0.08), ("init_yaw", 0.28)]
    boxes = {}
    for name, left in labels_row1:
        tb_ax = fig.add_axes([left, 0.15, box_w, box_h])
        boxes[name] = TextBox(tb_ax, name.replace("_", " ") + "  ", initial=DEFAULTS[name])
    for name, left in labels_row2:
        tb_ax = fig.add_axes([left, 0.08, box_w, box_h])
        boxes[name] = TextBox(tb_ax, name.replace("_", " ") + "  ", initial=DEFAULTS[name])

    run_button_ax = fig.add_axes([0.70, 0.08, 0.2, 0.12])
    run_button = Button(run_button_ax, "Run Simulation")

    def run_simulation(_event=None):
        # Goal: parse the 5 fields; on a bad number, tell the user
        # inline instead of crashing.
        try:
            tx = float(boxes["target_x"].text)
            ty = float(boxes["target_y"].text)
            tz = float(boxes["target_z"].text)
            tilt = float(boxes["init_tilt"].text)
            yaw = float(boxes["init_yaw"].text)
        except ValueError:
            status_text.set_text("Error: all five fields must be numbers.")
            status_text.set_color('#c0392b')
            fig.canvas.draw_idle()
            return

        if not BINARY_PATH.exists():
            status_text.set_text(f"Error: {BINARY_PATH} not found -- build it first (see README).")
            status_text.set_color('#c0392b')
            fig.canvas.draw_idle()
            return

        # Goal: show "Running..." and force it onto screen NOW -- the
        # subprocess call below blocks, and matplotlib won't repaint
        # mid-callback on its own.
        status_text.set_text("Running simulation (preview mode)...")
        status_text.set_color('black')
        fig.canvas.draw()
        fig.canvas.flush_events()

        result = subprocess.run(
            [str(BINARY_PATH), "--target", str(tx), str(ty), str(tz),
             "--init-tilt", str(tilt), "--init-yaw", str(yaw), "--preview", "--no-plot"],
            cwd=str(PROJECT_ROOT), capture_output=True, text=True)

        if result.returncode != 0:
            message = result.stderr.strip() or "Simulation failed (no error message captured)."
            status_text.set_text(message[:200])
            status_text.set_color('#c0392b')
            fig.canvas.draw_idle()
            return

        ax = fresh_axes()
        draw_trajectory(ax, load_trajectory(CSV_PATH), plot_state["colorbar_holder"])
        status_text.set_text("Done.")
        status_text.set_color('#1e8449')
        fig.canvas.draw_idle()

    run_button.on_clicked(run_simulation)

    # Goal: show something on first open instead of a blank plot -- run
    # once immediately with the default values already in the text boxes.
    run_simulation()

    plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        plot_trajectory_3d(sys.argv[1])
    else:
        run_interactive()
