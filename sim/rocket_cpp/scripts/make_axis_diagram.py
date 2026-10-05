#!/usr/bin/env python3
"""Generate the code-accurate axis and TVC reference diagram."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, Circle, FancyBboxPatch, Polygon, Rectangle


BG = "#F4F7FB"
PANEL = "#FFFFFF"
INK = "#142337"
MUTED = "#617184"
GRID = "#D9E2EC"
WORLD = "#7D8996"
BLUE = "#1769AA"
RED = "#D14B5A"
GREEN = "#239B83"
GOLD = "#E4A33A"
PURPLE = "#7446A6"


def panel(fig, bounds):
    """Draw a rounded panel behind axes expressed in figure coordinates."""
    fig.patches.append(
        FancyBboxPatch(
            (bounds[0], bounds[1]),
            bounds[2],
            bounds[3],
            boxstyle="round,pad=0.008,rounding_size=0.018",
            transform=fig.transFigure,
            facecolor=PANEL,
            edgecolor=GRID,
            linewidth=1.2,
            zorder=-10,
        )
    )


def clean_axis(ax, xlim, ylim):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.axis("off")


def arrow(ax, start, end, color, label=None, width=2.4, ls="-", zorder=5):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops=dict(
            arrowstyle="-|>",
            color=color,
            lw=width,
            linestyle=ls,
            mutation_scale=15,
            shrinkA=0,
            shrinkB=0,
        ),
        zorder=zorder,
    )
    if label:
        dx, dy = np.asarray(end) - np.asarray(start)
        ax.text(
            end[0] + 0.05 * np.sign(dx or 1),
            end[1] + 0.05 * np.sign(dy or 1),
            label,
            color=color,
            fontsize=10,
            fontweight="bold",
            ha="left",
            va="bottom",
            zorder=zorder + 1,
        )


def transform(points, origin, body_angle_from_x):
    """Transform local (right, forward) points into plot coordinates."""
    forward = np.array([np.cos(body_angle_from_x), np.sin(body_angle_from_x)])
    right = np.array([forward[1], -forward[0]])
    return np.array(origin) + points[:, :1] * right + points[:, 1:] * forward


def draw_rocket(ax, origin, body_angle_from_x, scale=1.0):
    body = np.array(
        [
            [-0.16, 0.00],
            [-0.16, 1.15],
            [0.00, 1.62],
            [0.16, 1.15],
            [0.16, 0.00],
        ]
    ) * scale
    left_fin = np.array([[-0.16, 0.08], [-0.34, -0.12], [-0.14, 0.38]]) * scale
    right_fin = left_fin.copy()
    right_fin[:, 0] *= -1
    for shape, fill in ((body, "#E8EEF5"), (left_fin, "#DCE5EF"), (right_fin, "#DCE5EF")):
        ax.add_patch(
            Polygon(
                transform(shape, origin, body_angle_from_x),
                closed=True,
                facecolor=fill,
                edgecolor=INK,
                linewidth=1.5,
                zorder=3,
            )
        )


def heading(fig, x, y, kicker, title):
    fig.text(x, y, kicker, color=BLUE, fontsize=9, fontweight="bold")
    fig.text(x, y - 0.026, title, color=INK, fontsize=15, fontweight="bold")


fig = plt.figure(figsize=(16, 9), dpi=150, facecolor=BG)
fig.text(0.05, 0.945, "ROCKET_CPP REFERENCE", color=BLUE, fontsize=9, fontweight="bold")
fig.text(
    0.05,
    0.902,
    "Coordinate frames and thrust-vector control",
    color=INK,
    fontsize=24,
    fontweight="bold",
)
fig.text(
    0.05,
    0.872,
    "Code-accurate geometry for RocketState::orientation and ThrustVectorControl.",
    color=MUTED,
    fontsize=10.5,
)

left_panel = (0.045, 0.075, 0.43, 0.755)
right_panel = (0.495, 0.075, 0.46, 0.755)
panel(fig, left_panel)
panel(fig, right_panel)
heading(fig, 0.07, 0.795, "01  VEHICLE ATTITUDE", "World frame to body frame")
heading(fig, 0.52, 0.795, "02  ACTUATOR GEOMETRY", "Two-axis thrust-vector control")


# Pitch: side view.
pitch_ax = fig.add_axes([0.075, 0.48, 0.36, 0.25], facecolor="none")
clean_axis(pitch_ax, (-1.25, 2.35), (-0.35, 2.25))
origin = np.array([0.0, 0.0])
theta = np.radians(28.0)
body_angle = np.pi / 2.0 - theta

pitch_ax.plot([-1.1, 2.15], [0, 0], color=GRID, lw=1.2)
arrow(pitch_ax, origin, (0, 1.82), WORLD, width=1.5, ls="--")
pitch_ax.text(0.05, 1.78, "+z", color=WORLD, fontsize=9.2, fontweight="bold")
arrow(pitch_ax, origin, (1.9, 0), WORLD, "world +x", width=1.5, ls="--")
draw_rocket(pitch_ax, origin, body_angle, scale=1.05)
body_z = 1.95 * np.array([np.cos(body_angle), np.sin(body_angle)])
arrow(pitch_ax, origin, body_z, BLUE, "body +Z", width=2.8)
pitch_ax.add_patch(
    Arc(origin, 1.15, 1.15, theta1=90 - 28, theta2=90, color=BLUE, lw=2.2)
)
pitch_ax.text(0.17, 0.57, r"$\theta$", color=BLUE, fontsize=16, fontweight="bold")
pitch_ax.text(-1.1, 2.14, "PITCH  side view", color=INK, fontsize=10, fontweight="bold")
pitch_ax.text(
    -1.1,
    1.62,
    r"$\theta$ = orientation(1)",
    color=MUTED,
    fontsize=8.5,
    linespacing=1.5,
)


# Yaw: plan view.
yaw_ax = fig.add_axes([0.075, 0.19, 0.36, 0.225], facecolor="none")
clean_axis(yaw_ax, (-1.35, 2.25), (-1.1, 1.55))
psi = np.radians(35.0)
arrow(yaw_ax, origin, (1.9, 0), WORLD, "world +x", width=1.5, ls="--")
arrow(yaw_ax, origin, (0, 1.35), WORLD, width=1.5, ls="--")
yaw_ax.text(0.06, 1.33, "+y", color=WORLD, fontsize=9.2, fontweight="bold")
projection = 1.7 * np.array([np.cos(psi), np.sin(psi)])
arrow(yaw_ax, origin, projection, PURPLE, "projection of body +Z", width=2.6)
yaw_ax.add_patch(Arc(origin, 1.12, 1.12, theta1=0, theta2=35, color=PURPLE, lw=2.2))
yaw_ax.text(0.47, 0.13, r"$\psi$", color=PURPLE, fontsize=16, fontweight="bold")
yaw_ax.add_patch(Circle(origin, 0.10, facecolor=PANEL, edgecolor=INK, lw=1.3, zorder=6))
yaw_ax.text(-1.25, 1.46, "YAW  plan view", color=INK, fontsize=10, fontweight="bold")
yaw_ax.text(
    -1.25,
    1.02,
    r"$\psi$ = orientation(2)",
    color=MUTED,
    fontsize=8.5,
    linespacing=1.5,
)
yaw_ax.text(
    -1.25,
    -0.83,
    r"$\phi$ = orientation(0): roll about body +Z (not actively driven)",
    color=GREEN,
    fontsize=8.8,
    fontweight="bold",
)


# TVC: body frame and force/nozzle relationship.
tvc_ax = fig.add_axes([0.52, 0.27, 0.275, 0.455], facecolor="none")
clean_axis(tvc_ax, (-1.9, 2.05), (-2.35, 2.5))
pivot = np.array([0.0, 0.0])

# Rocket tail above the pivot.
tvc_ax.add_patch(Rectangle((-0.35, 0.0), 0.70, 1.55, facecolor="#E8EEF5", edgecolor=INK, lw=1.5))
tvc_ax.add_patch(Polygon([(-0.35, 0.18), (-0.72, -0.12), (-0.35, 0.48)], facecolor="#DCE5EF", edgecolor=INK, lw=1.4))
tvc_ax.add_patch(Polygon([(0.35, 0.18), (0.72, -0.12), (0.35, 0.48)], facecolor="#DCE5EF", edgecolor=INK, lw=1.4))
tvc_ax.add_patch(Circle(pivot, 0.11, facecolor=PANEL, edgecolor=INK, lw=1.5, zorder=8))
tvc_ax.text(0.0, 1.72, "rocket body", color=MUTED, fontsize=8.8, ha="center")

arrow(tvc_ax, pivot, (0, 2.15), BLUE, "body +Z", width=2.5)
arrow(tvc_ax, pivot, (1.25, 0), GREEN, "body +X", width=1.8)
arrow(tvc_ax, pivot, (-0.78, 0.48), GOLD, "body +Y", width=1.8)
arrow(tvc_ax, pivot, (0, -2.05), WORLD, "neutral -Z", width=1.6, ls="--")

# Per-axis limit guides shown in the pitch projection.
for sign in (-1, 1):
    a = np.radians(sign * 30.0)
    edge = 1.90 * np.array([np.sin(a), -np.cos(a)])
    tvc_ax.plot([0, edge[0]], [0, edge[1]], color=GOLD, lw=1.3, ls=(0, (4, 3)), zorder=1)
tvc_ax.text(1.00, -1.66, r"per-axis limit $30^\circ$", color=GOLD, fontsize=8.8, ha="left")

gx = np.radians(18.0)
gy = np.radians(12.0)
nozzle_3d = np.array([np.sin(gx), np.cos(gx) * np.sin(gy), -np.cos(gx) * np.cos(gy)])
# The main view projects body Y diagonally up-left to keep all three axes visible.
nozzle_2d = np.array([nozzle_3d[0] - 0.45 * nozzle_3d[1], nozzle_3d[2] + 0.28 * nozzle_3d[1]])
nozzle_2d /= np.linalg.norm(nozzle_2d)
nozzle_end = 1.85 * nozzle_2d
force_end = -1.72 * nozzle_2d

arrow(tvc_ax, pivot, nozzle_end, PURPLE, width=3.0)
tvc_ax.text(nozzle_end[0] + 0.10, nozzle_end[1] - 0.05, "nozzle direction  n", color=PURPLE, fontsize=9.2, fontweight="bold")
arrow(tvc_ax, pivot, force_end, RED, width=3.0)
tvc_ax.text(force_end[0] - 0.12, force_end[1] + 0.14, "vehicle force  F", color=RED, fontsize=9.2, fontweight="bold", ha="right")

delta_deg = np.degrees(np.arccos(np.clip(-nozzle_3d[2], -1.0, 1.0)))
nozzle_angle = np.degrees(np.arctan2(nozzle_2d[1], nozzle_2d[0]))
tvc_ax.add_patch(
    Arc(pivot, 0.92, 0.92, theta1=-90, theta2=nozzle_angle, color=PURPLE, lw=2.2)
)
tvc_ax.text(0.12, -0.52, rf"$\delta={delta_deg:.1f}^\circ$", color=PURPLE, fontsize=11, fontweight="bold")
tvc_ax.text(-1.82, 2.28, "BODY-FRAME VIEW", color=INK, fontsize=9.5, fontweight="bold")
tvc_ax.text(-1.82, 2.02, "Force is opposite the exhaust direction.", color=MUTED, fontsize=8.8)


# Command-space inset: limits are independent per axis, not a circular cone.
limit_ax = fig.add_axes([0.815, 0.42, 0.105, 0.22], facecolor="none")
limit_ax.set_xlim(-36, 36)
limit_ax.set_ylim(-36, 36)
limit_ax.set_aspect("equal")
limit_ax.spines[["top", "right"]].set_visible(False)
limit_ax.spines[["left", "bottom"]].set_color(WORLD)
limit_ax.tick_params(colors=MUTED, labelsize=7)
limit_ax.set_xticks([-30, 0, 30])
limit_ax.set_yticks([-30, 0, 30])
limit_ax.grid(color=GRID, linewidth=0.7)
limit_ax.add_patch(Rectangle((-30, -30), 60, 60, facecolor=GOLD, alpha=0.12, edgecolor=GOLD, lw=1.8))
limit_ax.scatter([18], [12], s=38, color=PURPLE, zorder=5)
limit_ax.scatter([30], [30], s=28, color=RED, zorder=5)
limit_ax.annotate(
    r"corner: $\delta=41.4^\circ$",
    xy=(30, 30),
    xytext=(-32, 38),
    fontsize=7.5,
    color=RED,
    arrowprops=dict(arrowstyle="->", color=RED, lw=1.0),
)
limit_ax.set_xlabel(r"$g_x$ pitch (deg)", color=MUTED, fontsize=7.5, labelpad=1)
limit_ax.set_ylabel(r"$g_y$ yaw (deg)", color=MUTED, fontsize=7.5, labelpad=1)
fig.text(0.808, 0.665, "COMMAND LIMITS", color=INK, fontsize=9.5, fontweight="bold")
fig.text(0.808, 0.645, "Each servo clamps independently.", color=MUTED, fontsize=8)


# Equations and actuator dynamics.
eq_ax = fig.add_axes([0.805, 0.205, 0.125, 0.16], facecolor="none")
eq_ax.axis("off")
eq_ax.text(0, 1.00, "CODE MAPPING", color=INK, fontsize=9.5, fontweight="bold", va="top")
eq_ax.text(0, 0.80, r"$g_x$ = gimbal_pitch_rad", color=BLUE, fontsize=8.4)
eq_ax.text(0, 0.64, r"$g_y$ = gimbal_yaw_rad", color=GREEN, fontsize=8.4)
eq_ax.text(0, 0.43, r"$n=(\sin g_x,$", color=INK, fontsize=8.2)
eq_ax.text(0.13, 0.29, r"$\cos g_x\sin g_y,$", color=INK, fontsize=8.2)
eq_ax.text(0.13, 0.15, r"$-\cos g_x\cos g_y)$", color=INK, fontsize=8.2)
eq_ax.text(0, -0.07, r"$F=-Tn$", color=RED, fontsize=10, fontweight="bold")

fig.text(
    0.52,
    0.135,
    "ACTUATOR RESPONSE",
    color=INK,
    fontsize=9.5,
    fontweight="bold",
)
fig.text(
    0.52,
    0.104,
    r"$\dot{g}=(g_{target}-g)/\tau$     $\tau_{pitch}=\tau_{yaw}=0.05\,s$     clamp: $|g_x|,|g_y|\leq30^\circ$",
    color=MUTED,
    fontsize=9.2,
)

fig.text(
    0.05,
    0.025,
    "Source of truth: rocket_kinematics.cpp and thrust_vector_control.cpp",
    color=MUTED,
    fontsize=8,
)

out_path = Path(__file__).resolve().parents[1] / "docs" / "axis_and_tvc_reference.png"
out_path.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out_path, dpi=150, facecolor=BG, bbox_inches=None)
plt.close(fig)
print(f"saved {out_path}")
