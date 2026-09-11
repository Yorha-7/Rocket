#!/usr/bin/env python3
"""Generates rocket_cpp/docs/axis_and_tvc_reference.png -- a two-panel 3D
diagram of (1) the existing body/world axis + pitch/yaw/roll convention and
(2) the planned thrust-vector-control gimbal cone, with every angle symbol
mapped to its actual code variable name."""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ---- Shared helpers -------------------------------------------------

def rot_matrix(roll, pitch, yaw):
    """Exact copy of RocketKinematics::rocketToNedFrame()'s formula."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])

def slerp_arc(v1, v2, radius, n=40):
    v1n, v2n = v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)
    dot = np.clip(np.dot(v1n, v2n), -1, 1)
    omega = np.arccos(dot)
    if omega < 1e-6:
        pts = np.tile(v1n*radius, (n, 1))
    else:
        t = np.linspace(0, 1, n)
        s = np.sin(omega)
        pts = (np.sin((1-t)[:, None]*omega)/s)*v1n + (np.sin(t[:, None]*omega)/s)*v2n
        pts = pts*radius
    return pts[:, 0], pts[:, 1], pts[:, 2]

def draw_axis_triad(ax, origin, R, length, labels, colors, lw=2.2, ls='-', alpha=1.0,
                     zorder=5, fontsize=12):
    for i in range(3):
        vec = R[:, i]*length
        ax.quiver(*origin, *vec, color=colors[i], linewidth=lw, linestyle=ls,
                   arrow_length_ratio=0.12, alpha=alpha, zorder=zorder)
        end = np.array(origin) + vec*1.2
        ax.text(*end, labels[i], color=colors[i], fontsize=fontsize, fontweight='bold',
                ha='center', va='center', zorder=zorder+1)

def rocket_silhouette(R, nose_len=1.0, body_len=2.0, radius=0.28, n=24):
    """Body-local outline (nose apex + two body lines + a fin cross), then
    rotated into world coords by R. Body's own +Z runs nose->tail here to
    match the sim (body Z = thrust axis, +Z = nose)."""
    lines = []
    theta = np.linspace(0, 2*np.pi, n)
    # nose apex at z=nose_len+body_len down to shoulder circle at z=body_len
    apex = np.array([0, 0, nose_len + body_len])
    shoulder = np.stack([radius*np.cos(theta), radius*np.sin(theta),
                          np.full(n, body_len)])
    for k in range(0, n, 6):
        lines.append(np.stack([apex, shoulder[:, k]], axis=1))
    # body cylinder: shoulder circle down to tail circle at z=0
    tail = np.stack([radius*np.cos(theta), radius*np.sin(theta), np.zeros(n)])
    lines.append(shoulder)
    lines.append(tail)
    for k in range(0, n, 6):
        lines.append(np.stack([shoulder[:, k], tail[:, k]], axis=1))
    # simple fin crosses at the tail
    for ang in [0, np.pi/2, np.pi, 3*np.pi/2]:
        base = np.array([radius*np.cos(ang), radius*np.sin(ang), 0.15])
        tip = np.array([1.7*radius*np.cos(ang), 1.7*radius*np.sin(ang), -0.05])
        lines.append(np.stack([base, tip], axis=1))
    return [R @ seg for seg in lines]

def style_3d(ax, lim):
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-0.15*lim, lim*1.05)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.pane.set_alpha(0.0)
        pane.line.set_alpha(0.0)
    ax.grid(False)

# ---- Figure -----------------------------------------------------------

fig = plt.figure(figsize=(15, 9.4))
fig.suptitle('rocket_cpp -- axis, orientation-angle & TVC symbol reference',
             fontsize=15, fontweight='bold', y=0.995)

# ===================== PANEL 1: body/world + pitch/yaw/roll ===========
ax1 = fig.add_subplot(1, 2, 1, projection='3d')
pitch, yaw, roll = np.radians(28), np.radians(35), 0.0
R = rot_matrix(roll, pitch, yaw)
L = 2.4

# ground frame: a literal ground plane at world z=0, grounding the world
# frame the same way the rocket silhouette grounds the body frame. Drawn
# first / lowest zorder so the triads and rocket sit visibly on top of it.
ground_r = L*1.15
for v in np.linspace(-ground_r, ground_r, 7):
    ax1.plot([-ground_r, ground_r], [v, v], [0, 0], color='#aaaaaa',
              linewidth=0.6, alpha=0.55, zorder=0)
    ax1.plot([v, v], [-ground_r, ground_r], [0, 0], color='#aaaaaa',
              linewidth=0.6, alpha=0.55, zorder=0)
# launch pad marker at the origin -- this is where position=(0,0,0) and
# body Z's tail/nozzle end sit at t=0 (launch_height aside)
pad_t = np.linspace(0, 2*np.pi, 40)
pad_r = 0.22
ax1.plot(pad_r*np.cos(pad_t), pad_r*np.sin(pad_t), np.zeros_like(pad_t),
          color='#555555', linewidth=1.3, zorder=1)
ax1.text(ground_r*0.95, -ground_r*0.95, 0.02, 'ground frame (world z=0)',
          color='#7f8c8d', fontsize=8.6, ha='right', zorder=1)

# world (reference) triad -- dashed gray, short symbol-only labels
draw_axis_triad(ax1, (0, 0, 0), np.eye(3), L,
                 ['x', 'y', 'z'],
                 ['0.45', '0.45', '0.45'], lw=1.4, ls=(0, (4, 3)), alpha=0.9, fontsize=11)

# body triad -- solid RGB, rotated by (roll,pitch,yaw)
draw_axis_triad(ax1, (0, 0, 0), R, L,
                 ['X', 'Y', 'Z'],
                 ['#c0392b', '#1e8449', '#1f5fa8'], lw=2.8, fontsize=14)

# rocket silhouette along body Z
for seg in rocket_silhouette(R, nose_len=0.9, body_len=1.7, radius=0.24):
    ax1.plot(seg[0], seg[1], seg[2], color='#333333', linewidth=1.1, zorder=3)

# pitch arc: world Z -> body Z
ax1_r = 0.85
ax_, ay_, az_ = slerp_arc(np.array([0, 0, 1]), R[:, 2], ax1_r)
ax1.plot(ax_, ay_, az_, color='#1f5fa8', linewidth=2.6, zorder=6)
mid = len(ax_)//2
ax1.text(ax_[mid]*1.35, ay_[mid]*1.35, az_[mid]*1.35,
          r'$\theta$', color='#1f5fa8', fontsize=15, fontweight='bold',
          ha='center', zorder=7)

# yaw arc: world X -> horizontal projection of body Z, flat in ground plane
horiz = np.array([R[0, 2], R[1, 2], 0.0])
bx_, by_, bz_ = slerp_arc(np.array([1, 0, 0]), horiz, 0.55)
ax1.plot(bx_, by_, bz_, color='#7d3c98', linewidth=2.6, zorder=6)
mid = len(bx_)//2
ax1.text(bx_[mid]*1.6, by_[mid]*1.6, bz_[mid]-0.05,
          r'$\psi$', color='#7d3c98', fontsize=15, fontweight='bold',
          ha='center', zorder=7)
# horizontal projection guide line (dotted) from origin to horiz direction
hn = horiz/np.linalg.norm(horiz)
ax1.plot([0, hn[0]*L*0.9], [0, hn[1]*L*0.9], [0, 0], color='#7d3c98',
          linewidth=1.0, linestyle=':', alpha=0.8)

# roll (phi) has no visible geometry worth drawing here (always 0, no torque
# model) -- just called out in the legend below, to avoid crowding the nose.

style_3d(ax1, 2.5)
ax1.view_init(elev=20, azim=-58)
try:
    ax1.dist = 7.2
except Exception:
    pass
ax1.set_title('Body vs. world frame + orientation angles\n'
               '(illustrative pose: pitch=28deg, yaw=35deg, roll=0)', fontsize=11, pad=14)

legend1 = (
    "Symbol (grey, dashed)   Code             Meaning\n"
    "--------------------------------------------------------------------\n"
    "x, y, z                 position(0..2)   world position (z=altitude,+up)\n"
    "ground plane / pad      z=0, world x-y   ground frame -- position(2)=0\n"
    "\n"
    "Symbol (colored, solid) Code             Meaning\n"
    "--------------------------------------------------------------------\n"
    "X, Y, Z                 body frame axes  Z = nose/thrust axis (NOT X)\n"
    "theta                   orientation(1)   pitch: tilt of Z off vertical z\n"
    "psi                     orientation(2)   yaw: azimuth (compass dir) of tilt\n"
    "phi (not drawn)         orientation(0)   roll about Z -- always 0, no torque"
)
fig.text(0.045, 0.03, legend1, family='monospace', fontsize=8.4, va='bottom')

# ===================== PANEL 2: TVC gimbal cone =========================
ax2 = fig.add_subplot(1, 2, 2, projection='3d')

# short tail segment of the rocket, body-aligned with world (pitch=yaw=0)
for seg in rocket_silhouette(np.eye(3), nose_len=0.0, body_len=1.1, radius=0.26):
    if seg[2].max() <= 1.15:
        ax2.plot(seg[0], seg[1], seg[2], color='#333333', linewidth=1.1, zorder=3)

pivot = np.array([0, 0, 0.0])  # gimbal pivot at the nozzle base
gamma_max = np.radians(20)     # exaggerated for legibility; real hardware is smaller
cone_h = 1.5

# translucent cone surface, opening along -Z (nozzle neutral/exhaust direction)
phi_ = np.linspace(0, 2*np.pi, 60)
t_ = np.linspace(0, 1, 2)
PHI, T = np.meshgrid(phi_, t_)
radius_ = T*cone_h*np.tan(gamma_max)
Xc = radius_*np.cos(PHI)
Yc = radius_*np.sin(PHI)
Zc = -T*cone_h
ax2.plot_surface(Xc, Yc, Zc, color='#f39c12', alpha=0.18, linewidth=0, zorder=1)
# cone rim + a few generator lines
rim_t = np.linspace(0, 2*np.pi, 60)
rim_r = cone_h*np.tan(gamma_max)
ax2.plot(rim_r*np.cos(rim_t), rim_r*np.sin(rim_t), -np.full_like(rim_t, cone_h),
          color='#f39c12', linewidth=1.3, alpha=0.7)
for ang in np.linspace(0, 2*np.pi, 8, endpoint=False):
    ax2.plot([0, rim_r*np.cos(ang)], [0, rim_r*np.sin(ang)], [0, -cone_h],
              color='#f39c12', linewidth=0.8, alpha=0.5)

# body Z (nose reference, up) -- short label, offset sideways from the
# force vector it's parallel to, so the two don't collide
ax2.quiver(*pivot, 0, 0, 1.55, color='#1f5fa8', linewidth=2.0, arrow_length_ratio=0.1, zorder=9)
ax2.text(0.25, 0.15, 1.6, 'Z', color='#1f5fa8', fontsize=13, ha='left', fontweight='bold', zorder=10)
ax2.quiver(*pivot, 0, 0, -cone_h*1.05, color='#888888', linewidth=1.3,
            arrow_length_ratio=0.08, linestyle=(0, (4, 3)), zorder=2)
ax2.text(0.05, 0.05, -cone_h*1.15, '-Z (cone center)', color='#666666',
          fontsize=8.4, ha='left', zorder=9)

# one example deflected nozzle (exhaust) direction inside the cone -- purple,
# deliberately far from both the orange cone and the red force vector so it
# reads as its own thing
delta = np.radians(13)
az_cmd = np.radians(35)
nozzle_dir = np.array([np.sin(delta)*np.cos(az_cmd), np.sin(delta)*np.sin(az_cmd), -np.cos(delta)])
ax2.quiver(*pivot, *(nozzle_dir*cone_h*0.95), color='#6c3483', linewidth=2.6, arrow_length_ratio=0.12, zorder=9)
ax2.text(*(nozzle_dir*cone_h*0.95 + np.array([0.4, 0.05, 0.05])), 'T (nozzle)',
          color='#6c3483', fontsize=10.5, ha='left', fontweight='bold', zorder=10)

# resulting FORCE vector on the vehicle = -T * nozzle_dir  (Newton's 3rd law)
force_dir = -nozzle_dir
ax2.quiver(*pivot, *(force_dir*1.5), color='#c0392b', linewidth=2.4, arrow_length_ratio=0.1, zorder=9)
ax2.text(*(force_dir*1.6 + np.array([-0.75, 0, 0.05])), 'F (force on vehicle)\n= -T x nozzle_dir',
          color='#c0392b', fontsize=9.2, ha='left', fontweight='bold', zorder=10)

# delta arc (deflection off cone-center axis) -- close to the tip, small
dx_, dy_, dz_ = slerp_arc(np.array([0, 0, -1]), nozzle_dir, 0.32)
ax2.plot(dx_, dy_, dz_, color='#6c3483', linewidth=2.2, zorder=8)
mid = int(len(dx_)*0.6)
ax2.text(dx_[mid]*1.7, dy_[mid]*1.7, dz_[mid]*1.7 - 0.05, r'$\delta$',
          color='#6c3483', fontsize=13, fontweight='bold', zorder=10)

# gamma_max arc -- on the OPPOSITE side of the cone from delta/T, and further
# out along -Z so its label has clear air below it before the legend text
edge_dir = np.array([np.sin(gamma_max)*np.cos(az_cmd+np.pi), np.sin(gamma_max)*np.sin(az_cmd+np.pi), -np.cos(gamma_max)])
gx_, gy_, gz_ = slerp_arc(np.array([0, 0, -1]), edge_dir, 0.85)
ax2.plot(gx_, gy_, gz_, color='#f39c12', linewidth=2.2, linestyle='--', zorder=8)
mid = int(len(gx_)*0.55)
ax2.text(gx_[mid]*1.35, gy_[mid]*1.35, gz_[mid]*1.35, r'$\gamma_{max}$',
          color='#b9770e', fontsize=12, fontweight='bold', zorder=10)

style_3d(ax2, 1.9)
ax2.set_zlim(-2.0, 1.9)
ax2.view_init(elev=16, azim=-50)
try:
    ax2.dist = 7.2
except Exception:
    pass
ax2.set_title('Planned TVC gimbal cone (ThrustVectorControl)\n'
               r'($\gamma_{max}$ exaggerated to 20deg here for legibility)', fontsize=11, pad=14)

legend2 = (
    "Symbol      Code                        Meaning\n"
    "-----------------------------------------------------------------------\n"
    "Z / -Z      body Z axis                 nose ref. / nozzle-neutral (cone center)\n"
    "T           deflectedThrust() input     nozzle/exhaust direction (cmd_pitch,cmd_yaw)\n"
    "F           deflectedThrust() return    force on vehicle = -T x nozzle_dir\n"
    "                                        (this is what feeds computeNetForce)\n"
    "delta       (derived from cmd angles)   actual deflection off -Z, <= gamma_max\n"
    "gamma_max   max_gimbal_rad_ (ctor arg)  cone half-angle limit (circular)\n"
    "a,b,g       directionAngles() return    F's angles vs body X,Y,Z\n"
    "                                        (cos^2 a + cos^2 b + cos^2 g = 1)"
)
fig.text(0.545, 0.03, legend2, family='monospace', fontsize=8.4, va='bottom')

plt.subplots_adjust(left=0.02, right=0.98, top=0.80, bottom=0.19, wspace=0.03)
out_path = '/media/jayesh/Acer/Users/scien/Rocket/rocket_cpp/docs/axis_and_tvc_reference.png'
import os
os.makedirs(os.path.dirname(out_path), exist_ok=True)
plt.savefig(out_path, dpi=155)
print('saved', out_path)
