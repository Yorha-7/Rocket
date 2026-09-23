#!/usr/bin/env python3
"""Plot the managed interception sweep CSV and exit when its window closes."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# Project-relative locations are deliberate: running this script from any
# working directory still reads/writes the rocket_cpp results tree.
PROJECT_DIR = Path(__file__).resolve().parents[1]
CSV_DIR = PROJECT_DIR / "results" / "csv"
PLOT_DIR = PROJECT_DIR / "results" / "plots"
DEFAULT_CSV = CSV_DIR / "interception_results_terminal_dense_full.csv"
DEFAULT_PLOT = PLOT_DIR / "interception_results_terminal_dense_full.png"


def project_path(path: Path) -> Path:
    """Resolve relative overrides against rocket_cpp, not the caller's cwd."""
    return path if path.is_absolute() else PROJECT_DIR / path


def read_results(path: Path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no result rows in {path}")

    def column(name):
        return np.asarray([float(row[name]) for row in rows], dtype=float)

    return {
        "x": column("target_x"),
        "y": column("target_y"),
        "z": column("target_z"),
        "radius": column("target_radius"),
        "error": column("miss_distance"),
    }


def draw_sphere(ax, radius):
    u = np.linspace(0.0, 2.0 * np.pi, 40)
    v = np.linspace(0.0, np.pi, 20)
    x = radius * np.outer(np.cos(u), np.sin(v))
    y = radius * np.outer(np.sin(u), np.sin(v))
    z = radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(x, y, z, color="gray", alpha=0.12, linewidth=0.5)


def set_equal_axes(ax, radius):
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-radius, radius)
    ax.set_zlim(0.0, radius)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z altitude (m)")


def make_plot(data, output: Path):
    radius = float(np.max(data["radius"]))
    positive_errors = np.maximum(data["error"], 1e-3)
    logarithmic = np.max(positive_errors) / max(np.min(positive_errors), 1e-3) > 100.0
    norm = plt.Normalize(vmin=0.0, vmax=max(np.percentile(data["error"], 98), 1e-3))
    if logarithmic:
        from matplotlib.colors import LogNorm
        norm = LogNorm(vmin=max(np.min(positive_errors), 1e-3), vmax=np.max(positive_errors))

    figure = plt.figure(figsize=(16, 11))
    grid = figure.add_gridspec(2, 2, height_ratios=(1.35, 1.0))
    ax3d = figure.add_subplot(grid[0, :], projection="3d")
    scatter = ax3d.scatter(
        data["x"], data["y"], data["z"], c=positive_errors, cmap="viridis",
        norm=norm, s=10, alpha=0.9, edgecolors="none"
    )
    draw_sphere(ax3d, radius)
    ax3d.scatter([0], [0], [0], color="black", marker="+", s=90, label="launch point")
    set_equal_axes(ax3d, radius)
    ax3d.set_title("Target interception sweep — closest-approach miss distance")
    ax3d.legend(loc="upper left")
    figure.colorbar(scatter, ax=ax3d, pad=0.08, label="miss distance (m)")

    ax_xy = figure.add_subplot(grid[1, 0])
    ax_xy.scatter(data["x"], data["y"], c=positive_errors, cmap="viridis", norm=norm, s=8)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("Horizontal target projection")
    ax_xy.grid(alpha=0.25)

    ax_xz = figure.add_subplot(grid[1, 1])
    ax_xz.scatter(data["x"], data["z"], c=positive_errors, cmap="viridis", norm=norm, s=8)
    ax_xz.set_xlabel("X (m)")
    ax_xz.set_ylabel("Z altitude (m)")
    ax_xz.set_title("Vertical target projection")
    ax_xz.grid(alpha=0.25)

    figure.suptitle(
        f"{len(data['error'])} targets within {radius:.0f} m; "
        f"median miss {np.median(data['error']):.2f} m",
        fontsize=14,
    )
    figure.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv", type=Path, default=DEFAULT_CSV,
        help=f"CSV override (default: {DEFAULT_CSV})",
    )
    parser.add_argument(
        "--output", type=Path, default=DEFAULT_PLOT,
        help=f"PNG override (default: {DEFAULT_PLOT})",
    )
    args = parser.parse_args()

    csv_path = project_path(args.csv)
    plot_path = project_path(args.output)
    data = read_results(csv_path)
    make_plot(data, plot_path)
    print(f"Saved {plot_path}")
    print("Close the plot window to terminate.")
    plt.show(block=True)


if __name__ == "__main__":
    main()
