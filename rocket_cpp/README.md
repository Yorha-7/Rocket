# rocket_cpp

C++ 3DOF (translation, all of x/y/z) + 1DOF (pitch) rocket trajectory
simulator. It loads the rocket's geometry from an OpenRocket `.ork` design
file and computes its own drag/normal-force coefficients and mass properties
(Barrowman method) from that geometry — the only things still read from the
`.ork` are thrust and total mass over time (motor performance data, not
something OpenRocket "computed" from the design). Replays the flight through
an Eigen-based Euler integrator, produces `rocket_trajectory.csv`, and calls
a Python/matplotlib script to render `rocket_trajectory.png`. A second,
interactive 3D viewer (`scripts/trajectory.py`) is also available on demand.

**Current physics feature set:**
- Real x/y/z translation: thrust and drag both rotate into the world frame
  through the full roll-pitch-yaw DCM, so a launch-azimuth (`init_yaw`)
  genuinely deflects the flight path sideways, not just forward.
- Pitch dynamics driven by real angle of attack (body axis vs. actual
  velocity vector, not just absolute tilt from vertical) plus aerodynamic
  damping — see [Coordinate frames](#coordinate-frames--axis-conventions)
  for the two correctness bugs this replaced.
- Time-varying CP/CG/inertia and Barrowman aerodynamic coefficients,
  computed from the vehicle's own geometry every step (not flight-averaged
  constants, not read from OpenRocket's own solved simulation).
- Ground-contact termination with exact linear-interpolated impact point.

> **This directory is a staging area.** Not everything in `include/`/`src/`
> is wired into the build. See [Staging Notes](#staging-notes) before
> trusting a header at face value.

## Kinematics library tree

One class, split across a few `.cpp` files by responsibility (each under
~300 lines), usable as a standalone physics library independent of the
`.ork` loader below.

```
rocket_cpp/
├── include/
│   ├── rocket_types.hpp        Shared plain-data structs: RocketState,
│   │                           RocketParams, SimulationConfig, FlightData,
│   │                           MassProperties, FlightConditions,
│   │                           AerodynamicCoefficients.
│   └── rocket_kinematics.hpp   The RocketKinematics class declaration —
│                               the only header a caller needs.
└── src/
    ├── rocket_kinematics.cpp   Translational motion + top-level loop.
    └── pitch_dynamics.cpp      Pitch-axis torque model.
```

To reuse just the physics elsewhere: take those files + `aerodynamics.*`/
`atmosphere.cpp`/`mass_properties_model.*` below + Eigen, construct
`RocketKinematics(params, config, mass_components)`, and call `simulate()`
or `step()`.

### `RocketKinematics` method reference

| Method | File | Purpose |
|---|---|---|
| `RocketKinematics(params, config, mass_components)` | rocket_kinematics.cpp | ctor — builds the `AerodynamicsModel`/`VehicleMassModel`, caches Barrowman CP |
| `step(state, thrust)` | rocket_kinematics.cpp | **public** — advance one `dt`; `state.mass` is this instant's mass |
| `simulate(time, flight_data)` | rocket_kinematics.cpp | **public** — run full flight, one `step()` per sample |
| `getParams()` / `getConfig()` | rocket_kinematics.hpp | **public** — accessors |
| `computeNetForce(state, thrust)` | rocket_kinematics.cpp | **public** — thrust + drag (our own Cd) + gravity, world frame, undivided by mass |
| `computePitchTorques(state)` | pitch_dynamics.cpp | **public** — gravity(=0)/aero/damping torque breakdown, for logging (not used by `step()` itself) |
| `computeAcceleration()` | rocket_kinematics.cpp | `computeNetForce() / mass` |
| `rocketToNedFrame()` | rocket_kinematics.cpp | body → world rotation matrix |
| `buildFlightConditions()` | rocket_kinematics.cpp | state → `FlightConditions` — real angle of attack (pitch minus flight-path angle, not raw pitch), mach, altitude... for the aero model |
| `computeGravityTorque()` | pitch_dynamics.cpp | always `0.0` — gravity has no net torque about a body's own CG (deliberate stub, see below) |
| `computeAerodynamicMoment()` | pitch_dynamics.cpp | aero restoring moment (reads `Cn_alpha` from `AerodynamicsModel`) |
| `computeDampingTorque()` | pitch_dynamics.cpp | resists rotation |
| `computeTotalPitchTorque()` | pitch_dynamics.cpp | sums the 3 torques above |
| `computePitchAcceleration()` | pitch_dynamics.cpp | torque ÷ I_yy, clamped |
| `updatePitchDynamics()` | pitch_dynamics.cpp | builds `MassProperties` from `VehicleMassModel` + cached CP, then integrates |

## Aerodynamics & mass properties (Barrowman method)

Computes Cd/Cn_alpha/CP and CG/inertia from the vehicle's own geometry —
nothing here is read from OpenRocket's solved simulation.

```
rocket_cpp/
├── include/
│   ├── aerodynamics.hpp            AerodynamicsModel
│   └── mass_properties_model.hpp   VehicleMassModel
└── src/
    ├── aerodynamics.cpp        computeCoefficients() dispatcher, body/fin
    │                           Cd/Cn_alpha/Cm_alpha, base/boat-tail drag,
    │                           body-fin interference, Barrowman CP.
    │                           Transonic/supersonic/wave-drag are one-line
    │                           placeholders — never exercised below Mach 0.8.
    ├── atmosphere.cpp          ISA atmosphere (troposphere + isothermal
    │                           stratosphere), Sutherland viscosity, Mach/
    │                           Reynolds, skin-friction Cf, friction drag.
    └── mass_properties_model.cpp   Dry mass/CG (exact, mass-weighted) +
                                    dry I_yy (parallel-axis + body-tube rod
                                    correction) from the parsed component
                                    list; combines with total mass(t) for
                                    CG(t)/I_yy(t).
```

| Method | File | Purpose |
|---|---|---|
| `computeCoefficients(fc)` | aerodynamics.cpp | **public** — Cd/Cn/Ca/Cm/Cn_alpha/Cm_alpha at one flight condition |
| `computeCenterOfPressure()` | aerodynamics.cpp | **public** — Barrowman CP, cm from nose tip (geometry-only, Mach-independent) |
| `getDensity()` / `getSpeedOfSound()` | atmosphere.cpp | **public** — shared with `RocketKinematics`, one atmosphere model for the whole sim |
| `computeBodyCnAlpha()` / `computeFinCnAlpha()` | aerodynamics.cpp | Barrowman normal-force slopes (body = 2/rad; fins via span/chord/sweep + Kfb interference) |
| `computeNoseCp()` / `computeFinCp()` | aerodynamics.cpp | geometric CP of each contributor |
| `computeFrictionDrag()` / `computeSkinFrictionCf()` | atmosphere.cpp | wetted-area friction drag (Blasius/Schlichting + roughness limit) |
| `computeBaseDrag()` / `computeBoatTailDrag()` | aerodynamics.cpp | blunt-base and boat-tail pressure drag |
| `VehicleMassModel(components, ...)` | mass_properties_model.cpp | ctor — dry mass/CG/I_yy from the component list |
| `computeAt(total_mass_kg)` | mass_properties_model.cpp | **public** — splits total mass into (dry structure, fixed) + (motor, = total − dry) → CG(t)/I_yy(t) |

## OpenRocket ingestion (`.ork` → simulation inputs)

```
rocket_cpp/
├── include/
│   ├── ork_archive.hpp           readOrkXml(path) -> string
│   ├── ork_geometry.hpp          parseOrkGeometry(xml) -> RocketParams
│   ├── ork_flightdata.hpp        parseOrkFlightData(xml, dt, motor?) -> FlightData
│   │                             parseOrkLaunchConditions(xml, motor?) -> SimulationConfig
│   ├── ork_mass_components.hpp   parseOrkMassComponents(xml) -> vector<MassComponent>
│   └── ork_loader.hpp            loadOrkRocket(path, dt, motor?) -> OrkRocket
└── src/
    ├── ork_archive.cpp         Unzips the .ork (it's a zip). (libzip)
    ├── ork_geometry.cpp        Walks <nosecone>/<bodytube>/<trapezoidfinset>
    │                           for RocketParams geometry + reference_area. (tinyxml2)
    ├── ork_flightdata.cpp      Picks the embedded <simulation> for the target
    │                           motor (default: config marked default="true"),
    │                           reads thrust/mass + launch conditions,
    │                           resamples to uniform dt. (tinyxml2)
    ├── ork_mass_components.cpp Walks every structural part (nosecone, body
    │                           tube, fins, parachute, shock cord, wadding,
    │                           launch lug, centering rings, motor mount,
    │                           engine block), computes each one's mass from
    │                           its own material density + geometry (or an
    │                           explicit <mass>) and CG position. (tinyxml2)
    └── ork_loader.cpp          Facade tying all of the above together —
                                the one call main.cpp makes.
```

| Function | Purpose |
|---|---|
| `loadOrkRocket(path, dt, motor?)` | **entry point** — `main.cpp` calls only this one |
| `readOrkXml(path)` | unzip `.ork` → design XML string |
| `parseOrkGeometry(xml)` | XML → `RocketParams` (nose/body/fin geometry) |
| `parseOrkFlightData(xml, dt, motor?)` | XML → `FlightData` (thrust, mass) |
| `parseOrkLaunchConditions(xml, motor?)` | XML → `SimulationConfig` (pad height, rod angle) |
| `parseOrkMassComponents(xml)` | XML → structural component list, for `VehicleMassModel` |

## Build & run

### Prerequisites

- A C++17 compiler + CMake
- **Eigen3** — the integrator's vector/matrix math
- **libzip** and **tinyxml2** (both located via pkg-config — the system's
  libzip CMake package is broken, see the comment in `CMakeLists.txt`) — for
  unzipping and parsing the `.ork` design file
- **Python 3** with `pandas`, `numpy`, `matplotlib` — only needed for the
  plots; the simulation itself and the CSV it writes don't depend on Python

### Build

```bash
mkdir -p build && cd build
cmake .. && make
```

### Run a simulation

```bash
./rocket_cpp        # from build/, or wherever the binary ends up
```

This loads `artifacts/rocket.ork`, runs the full flight (boost through
ground impact), and — from the project root, which it `chdir`s into itself
— writes:
- `rocket_trajectory.csv` — the full time-series state
- `rocket_analysis.png` — the 8-panel flight-analysis figure (below)
- `rocket_trajectory.png` — X/Y/Z-vs-time position panels

Console output along the way reports the vehicle's own computed dry
mass/CG/CP/I_yy and drag/normal-force coefficients at a sample flight
condition, so you can sanity-check the geometry parse before trusting the
trajectory.

`main.cpp` hardcodes an absolute path to `artifacts/rocket.ork` and to the
project root for the plot call — update those if you move the tree.

### Customize a launch

There's no CLI yet — launch conditions are edited directly in `main.cpp`:

| What | Where | Notes |
|---|---|---|
| Launch tilt (pitch) | `INIT_TILT_OVERRIDE_DEG` in `main.cpp` | Degrees from vertical. Every simulation embedded in the `.ork` uses a dead-vertical rod (0°), so this override is what actually gives the pitch dynamics something to act on. |
| Launch azimuth (yaw) | `INIT_YAW_OVERRIDE_DEG` in `main.cpp` | Degrees. Fixed for the whole flight (no yaw torque model) — see [Sign conventions](#sign-conventions) for how it combines with tilt. |
| Integrator step size | `config.dt` | Smaller = more accurate but slower and a bigger CSV. |
| Which motor config | `loadOrkRocket(path, dt, motor?)` (`ork_loader.hpp`) | Defaults to the `.ork`'s `default="true"` simulation; pass a motor name to pick another of the 5 embedded configs. |

Re-run `cmake --build build` after editing, then `./build/rocket_cpp` again.

### Visualize a trajectory

The 2D panel figures above are generated automatically. For an interactive,
rotatable 3D view of the same flight:

```bash
python3 scripts/trajectory.py rocket_trajectory.csv   # defaults to this path if omitted
```

Opens a matplotlib window (rotate/zoom with the mouse) with the path colored
by time and launch/apogee/impact markers. The box is a fixed equal-unit cube
(same meter range on all three axes, auto-sized to the flight's extent) so
angles read honestly — see [Coordinate frames](#coordinate-frames--axis-conventions)
for why that matters and what the alternative (a metrically "true to scale"
box) actually looks like for a mostly-vertical flight.

## Coordinate frames & axis conventions

### World frame — labeled "NED", actually Z-up

Commented `x, y, z in NED frame`, but it isn't North-East-Down: gravity is
`(0,0,-g·m)`, ground contact is `z < 0`, and the CSV calls `position(2)`
"height" — only consistent if `+Z` is up:

- `x` — North-ish horizontal (m)
- `y` — East-ish horizontal (m)
- `z` — **altitude above the launch pad, positive up** (m) — a Z-up world frame, not true NED (where +Z would be down).

### Body frame

- **Body Z** — the rocket's longitudinal axis (nose direction). Thrust is applied as `(0, 0, T)` in body frame.
- **Body X / Y** — lateral axes.

Not the traditional aerospace convention (body X-forward, Z-down) — here
the thrust axis is Z.

### Orientation & the body→world rotation

`orientation = (roll, pitch, yaw)` rad, `angular_vel = (p, q, r)` rad/s. Only
pitch (index 1) is dynamic — roll/yaw just wrap into `[0, 2π)` every step, no
torque drives them (see [Staging Notes](#staging-notes)). Yaw does get a real
nonzero **initial** value from `init_yaw` (unlike roll, which never does) —
it just stays fixed at that value for the entire flight, acting like a
constant launch-azimuth offset rather than something that evolves.

`rocketToNedFrame()` builds the body→world rotation matrix — the standard
Z-Y-X aerospace DCM, $R = R_z(\psi)R_y(\theta)R_x(\phi)$ (φ=roll, θ=pitch, ψ=yaw):

$$
R =
\begin{bmatrix}
\cos\psi\cos\theta & \cos\psi\sin\theta\sin\phi - \sin\psi\cos\phi & \cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi \\
\sin\psi\cos\theta & \sin\psi\sin\theta\sin\phi + \cos\psi\cos\phi & \sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi \\
-\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi
\end{bmatrix}
$$

An earlier version of this matrix was missing a `cosθ` factor on `R[1][0]`,
which meant that with roll=yaw=0 (always, in this sim — see below) thrust
and drag only ever landed in the Z axis: `thrust_ned = R·(0,0,T)ᵀ` came out
to `(0, 0, T·cosθ)` regardless of pitch angle, so a pitched-over rocket's
thrust never actually gained a horizontal component. The whole trajectory
was a pure 1-D vertical bounce with a pitch angle riding along on top that
never fed back into translation — invisible until the Forces panel (below)
made `Fx ≡ 0` for the entire flight obvious. Fixed; `Fx` now correctly
tracks `T·sinθ`.

### Yaw's y-component is suppressed relative to x, not equal to it

With roll=0, thrust's world-frame components (from the DCM above) reduce to
`Fx = T·sinθ` but `Fy = T·sinψ·sinθ` — a *product* of two sines, not a
single one. Equal-looking `init_tilt`/`init_yaw` values do **not** produce
comparably-sized x and y drift: for small angles `Fy/Fx ≈ sinψ`, so a 3°
yaw next to a 3° pitch gives y-motion roughly 19× smaller than x-motion, not
the same size. This is a real property of the Z-Y-X Euler order (yaw can't
rotate a vector that's still pointing along the axis it's yawing about,
until pitch has already tilted it off that axis), not a bug — but it's easy
to expect otherwise from the config names alone.

### Two pitch-dynamics correctness bugs (fixed)

Both were only obvious at large tilt angles — near-vertical flight (the
`.ork`'s own dead-vertical launch rod) never exercised the broken cases:

1. **Invalid gravity torque.** An earlier `computeGravityTorque()` modeled
   gravity as a pendulum restoring torque (`-mg·d·sinθ`, `d` = CP-CG
   offset) — physically impossible: a uniform gravitational field exerts
   **zero** net torque about a rigid body's own center of gravity, by
   definition of the CG. At small angles this term was small enough not to
   matter much; at a 70° test tilt it dominated the (also wrong) pitch
   behavior and single-handedly snapped the rocket back through vertical
   within half a second, regardless of how far over it actually started.
   Fixed: the function is now a deliberate always-`0.0` stub (kept, not
   deleted, so the CSV/plots keep the same 3-column torque breakdown and
   show the zero explicitly).
2. **Angle of attack was just raw pitch.** `fc.alpha = state.orientation(1)`
   only approximates real angle of attack — the angle between the body axis
   and the *actual velocity vector* — when the rocket is already flying
   close to vertical. At large tilt it was badly wrong (e.g. a rocket
   flying straight along its own 70°-tilted axis would report a fictitious
   70° angle of attack, generating aerodynamic torque that fights reality
   instead of responding to it). Fixed in `buildFlightConditions()`:
   `alpha = pitch − flight_path_angle`, where the flight-path angle comes
   from the real velocity vector (projected onto the fixed launch-azimuth
   plane, since yaw is static — see above). This reduces to the old
   approximation exactly when velocity is ~0 (pad, or a momentary stall),
   and correctly relaxes toward 0 once the body trims out along its actual
   direction of travel (real weathercocking), instead of always fighting to
   point at vertical.

Combined effect on a 70°/70° tilt test case: apogee went from 364m
(near-vertical baseline) → a buggy 311m (85% of baseline — the invalid
torque was erasing almost the entire tilt penalty) → a physically-argued
268m after both fixes (still recovers real altitude via legitimate
weathercocking, since `cos(70°)·T` is still positive thrust the whole
burn — not a leftover bug).

### Sign conventions

- **Pitch** — 0 = vertical (nose up, +Z); positive pitch tips the nose over. `init_tilt` (from the `.ork`'s launch rod angle) is pitch-from-vertical, applied at `t=0`. Every embedded `.ork` simulation launches from a dead-vertical rod, so `main.cpp` overrides it (`INIT_TILT_OVERRIDE_DEG`) to a small nonzero angle — otherwise the pitch model has nothing to restore from.
- **Yaw** — 0 = launch azimuth aligned with `+X`; positive yaw rotates the launch azimuth toward `+Y`. The `.ork` format has no yaw/azimuth concept at all, so `main.cpp`'s `INIT_YAW_OVERRIDE_DEG` is a pure sim-side override, not derived from the design file. Combines with pitch to produce real x/y motion — see [the sine-product note above](#yaws-y-component-is-suppressed-relative-to-x-not-equal-to-it) before assuming equal tilt/yaw angles give equal-sized drift.
- **CP/CG locations** — cm from the **nose tip**. `d = cp - cg` is the static margin; `d > 0` (CP aft of CG) is stable and produces a restoring torque.

## State & parameter reference

`RocketState` (integrated every step):

| Field | Meaning | Units |
|---|---|---|
| `position` | (x, y, z) — z = altitude above pad | m |
| `velocity` | (vx, vy, vz) in world frame | m/s |
| `orientation` | (roll, pitch, yaw); only pitch evolves after `t=0` — yaw holds whatever `init_yaw` set it to, roll stays 0 | rad |
| `angular_vel` | (p, q, r); only q (index 1) is dynamic | rad/s |
| `mass` | current vehicle mass, overwritten each step from `FlightData` | kg |

`RocketParams` — what the rocket physically **is**, fixed for the whole
flight (parsed by `ork_geometry.cpp`):

| Field | Meaning | Units |
|---|---|---|
| `nose_length`, `nose_shape`, `nose_radius` | nose cone geometry (shape: 0=conical, 1=ogive, 2=hemisphere/ellipsoid, 3=parabolic) | m, —, m |
| `body_length`, `body_diameter` | body tube | m |
| `fin_count`, `fin_span`, `fin_root_chord`, `fin_tip_chord`, `fin_sweep`, `fin_thickness`, `fin_cant`, `fin_root_le_position` | fin set | —, m, m, m, m, m, rad, m |
| `base_diameter`, `boat_tail_length` | tail geometry (defaults to body diameter / 0 if no boat-tail component) | m |
| `surface_roughness` | approximate RMS roughness for the design's named finish | m |
| `reference_area` | `π·(body_diameter/2)²` — normalizes every drag/lift/moment coefficient in the sim | m² |
| `thrust_duration`, `max_thrust` | informational only — actual thrust comes from `FlightData` | s, N |

`FlightData` — motor performance over time, resampled to a uniform `dt`
(parsed by `ork_flightdata.cpp`; the one thing still read from the `.ork`,
since there's no local motor database to derive a thrust curve from):

| Field | Meaning | Units |
|---|---|---|
| `time` | uniform timestep grid | s |
| `thrust` | motor thrust | N |
| `mass` | vehicle mass (OpenRocket reports kg; converted to g here) | g |
| `dry_mass` | mass at burnout | g |

`MassProperties` — a snapshot passed into the pitch-dynamics methods,
`{cp_location_cm, cg_location_cm, I_xx, I_yy, I_zz}`, assembled each step
from `AerodynamicsModel::computeCenterOfPressure()` (cached, geometry-only)
and `VehicleMassModel::computeAt(state.mass)`. `I_zz` = `I_yy` (axisymmetric
body assumption).

`SimulationConfig` — how *we* choose to run the integrator, separate from
anything the rocket's design implies: `sim_duration`, `dt` are our own
choice; `launch_height`, `init_tilt` are pulled from the `.ork`'s launch
conditions for the selected simulation (see the override above); `init_yaw`
has no `.ork` equivalent at all and is purely a `main.cpp` override.

## Mathematical model

Driven each step by real `thrust[i]`/`mass[i]` from `FlightData`; every
coefficient (Cd, Cn_alpha, CP, CG, inertia) is computed from the vehicle's
own geometry by `AerodynamicsModel`/`VehicleMassModel`, not read from a
pre-solved simulation.

### 1. Atmosphere (ISA)

Troposphere (≤11 km): $T=T_0+L\cdot z$, $P=P_0(T/T_0)^{-g_0/(LR)}$; isothermal stratosphere above. $\rho=P/(RT)$. Viscosity via Sutherland's law; speed of sound $=\sqrt{\gamma R T}$, $\gamma=1.4$.

### 2. Forces (translational, world frame)

**Thrust** (body Z, rotated to world by $R$): $\vec F_T = R(0,0,T)^T$

**Drag**: $\vec F_D = -\hat v \cdot \tfrac12\rho v^2 C_d A$, $C_d$ from `computeCoefficients()` below.
*(Note: $\vec F_D$ is already world-aligned, but the code multiplies it by $R$ again anyway, alongside thrust.)*

**Gravity**: $\vec F_g=(0,0,-mg_0)$. **Net accel**: $\vec a=(\vec F_T+\vec F_D+\vec F_g)/m$.

### 3. Aerodynamic coefficients (Barrowman method, `aerodynamics.cpp`)

**Body normal-force slope** (a nose that fully transitions to the body diameter contributes exactly 2/rad, independent of shape/length — the classic Barrowman result), with Prandtl-Glauert compressibility:
$$
C_{N\alpha,body} = \frac{2}{\sqrt{1-M^2}}
$$

**Fin normal-force slope** (N fins, span $s$, body diameter $d$, root/tip chord $C_r,C_t$, leading-edge length $L_m=\sqrt{sweep^2+s^2}$), with body-fin interference $K_{fb}=1+\frac{r}{s+r}$:
$$
C_{N\alpha,fin} = \frac{K_{fb}\cdot 4N(s/d)^2}{1+\sqrt{1+(2L_m/(C_r+C_t))^2}} \cdot \frac{1}{\sqrt{1-M^2}}
$$

**Center of pressure** (Mach-independent — the compressibility factor cancels in this ratio): nose CP is a fixed fraction of nose length by shape (0.666 conical, 0.466 ogive, 0.5 hemisphere/parabolic); fin CP is the standard trapezoid centroid from the root leading edge. Combined:
$$
X_{cp} = \frac{C_{N\alpha,body}\,X_{cp,body} + C_{N\alpha,fin}\,X_{cp,fin}}{C_{N\alpha,body}+C_{N\alpha,fin}}
$$

**Drag** $C_d$ = body pressure (≈0 subsonic, pointed nose) + fin thickness drag + friction (Cf × wetted-area ratio, form-factor corrected) + base drag ($0.12+0.13M^2$, scaled by base/reference area) + boat-tail (0 here — no boat-tail) + wave drag (supersonic-only placeholder, unreachable below Mach 0.8).

### 4. Mass properties (`mass_properties_model.cpp`)

CG is exact — mass-weighted average of every structural component's own CG (bulk: density×volume, surface: density×area, line: density×length; see the `ork_mass_components.cpp` tree above). Dry pitch inertia is parallel-axis (point mass per component) plus a rod correction for the body tube ($mL^2/12$, since it spans a large fraction of the vehicle's length):
$$
I_{yy,dry} = \sum_i m_i(x_i-x_{cg})^2 + \frac{m_{tube}L_{tube}^2}{12}
$$
At each instant, motor mass = `total_mass(t) − dry_mass` (fixed axial position = the motor mount tube's center); combined CG/I_yy follow from the same parallel-axis approach.

### 5. Pitch dynamics (1DOF rotational, `pitch_dynamics.cpp`)

$d=(cp-cg)/100$, $\theta$=pitch, $q$=pitch rate. Angle of attack $\alpha$ is
**not** raw pitch — it's pitch minus the actual flight-path angle $\gamma$
(the direction the velocity vector points, projected onto the fixed
launch-azimuth plane, since yaw is static):

$$
\gamma = \operatorname{atan2}(v_x\cos\psi + v_y\sin\psi,\ v_z), \qquad \alpha = \theta - \gamma
$$

Only two torques act — gravity contributes none (zero net torque about a
body's own CG, by definition; see
[Coordinate frames](#coordinate-frames--axis-conventions) for the bug this
replaced):

$$
\tau_g = 0, \qquad
\tau_a = -\tfrac12\rho v^2 C_{N\alpha}\alpha A d\ (\alpha\text{ clamped }\pm0.5\text{ rad}), \qquad
\tau_d = -\left(0.6\cdot\tfrac12\rho v d^2 A\right)q
$$

$$
\dot q = \frac{\tau_g+\tau_a+\tau_d}{I_{yy}(t)}, \qquad q_{k+1}=q_k+\dot q\,\Delta t, \qquad \theta_{k+1}=\theta_k+q_{k+1}\Delta t
$$

(accel clamped ±100 rad/s², rate ±10 rad/s, angle ±1.5 rad)

### 6. Integration (explicit Euler) & ground termination

$$
\vec v_{k+1}=\vec v_k+\vec a\,\Delta t, \qquad \vec x_{k+1}=\vec x_k+\vec v_{k+1}\Delta t
$$

Roll/yaw just wrap into $[0,2\pi)$, no torque. When $z$ crosses below 0, linear interpolation finds the exact impact point; velocity/orientation/angular velocity zero out there and the state array truncates.

## Output & graphs

Running `./rocket_cpp` writes `rocket_trajectory.csv`
(`time, x, y, height, velocity, pitch, ang_vel, ang_accel, fx, fy, fz,
torque_gravity, torque_aero, torque_damping` — the last 6 are diagnostics
recomputed via `RocketKinematics::computeNetForce()`/`computePitchTorques()`,
not part of the integration itself) and invokes `scripts/plot_trajectory.py`,
producing this 8-panel figure, plus a second `rocket_trajectory.png` with
X/Y/Z position each plotted against time in its own panel:

![Rocket flight analysis](rocket_analysis.png)

For a spatial (not vs-time) view, see `scripts/trajectory.py` — an
interactive 3D viewer covered in [Visualize a trajectory](#visualize-a-trajectory).

| Panel | Data | What to look for |
|---|---|---|
| Height vs Time | `position(2)` | Burnout, apogee (auto-detected), impact |
| Velocity vs Time | $\lVert\vec v\rVert$ | Speed magnitude, not signed vertical velocity |
| Acceleration vs Time | `np.gradient(v, dt)` (recomputed in Python) | Sharp burnout spike is the thrust cutoff. The impact sample is dropped before differentiating — velocity snaps to 0 there, which would otherwise fake a huge spike |
| Pitch Angle vs Time | `orientation(1)` in degrees | Damps fast under thrust (high airspeed → strong damping), swells again near apogee (airspeed → 0, damping vanishes), re-damps during descent |
| Angular Velocity vs Time | `angular_vel(1)` in deg/s | Pitch rate $q$ |
| Angular Acceleration vs Time | finite-difference of angular velocity (impact sample dropped, same reason) | $\dot q$ |
| Forces vs Time | `fx, fy, fz` — net world-frame force | `fz` (thrust − drag − gravity) dominates; `fx` tracks `T·sinθ`; `fy` tracks `T·sinψ·sinθ` — real but noticeably smaller than `fx` for equal tilt/yaw angles (see [the sine-product note](#yaws-y-component-is-suppressed-relative-to-x-not-equal-to-it)), and exactly 0 if `init_yaw` is 0 |
| Torque Analysis | `torque_gravity, torque_aero, torque_damping` | `torque_gravity` is now identically 0 (see [pitch-dynamics bug fixes](#two-pitch-dynamics-correctness-bugs-fixed)) — aerodynamic torque and damping are the only real contributors, with aero typically dominating |

**No parachute is modeled — descent is a fast ballistic fall (~58 m/s
impact), not a slow chute-assisted one.** Earlier versions of this sim
borrowed OpenRocket's own `Drag coefficient` column, which encoded the
parachute as a ~180× Cd spike at deployment; now that Cd is computed by our
own `AerodynamicsModel` (bare-airframe geometry only, no parachute
component), that effect is gone. This is a direct, expected consequence of
no longer reading OpenRocket's solved values — not a bug, but a real
capability gap (see Staging Notes).

## Staging notes

1. **No parachute/recovery-device model.** The `.ork` design has one (see
   above) but nothing in this codebase reads its deploy altitude/CD or
   changes drag after apogee — descent is currently a bare-body freefall.
2. **`rocket_kinematics.h` + `kinematics.c`** — parallel legacy C-ish path
   (separate structs, adds a `normal_coeff` term). Not built, won't compile
   (`Eigen::Matrixd` isn't a real type).
3. **"NED" labeling is inaccurate** — see [Coordinate frames](#coordinate-frames--axis-conventions).
4. **Roll/yaw are purely kinematic** — no torque model, no inertia coupling
   (fin cant is parsed but unused). Yaw gets a real, static initial value
   (`init_yaw`) that genuinely deflects the trajectory (see
   [Coordinate frames](#coordinate-frames--axis-conventions)), but nothing
   ever restores or evolves it in flight — no weathercocking in the yaw
   plane the way pitch has. That, plus real air-relative β (sideslip; only
   α is real now) and Euler-rate kinematics, is what's missing for true
   6DOF.
5. **`I_xx` (roll inertia) is a coarse thin-shell/point estimate** in
   `VehicleMassModel` — fine for now since nothing reads it (no roll torque
   model yet), but should be revisited before any roll dynamics are added.
