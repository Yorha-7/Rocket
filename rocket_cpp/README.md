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
- **Thrust vector control**: a two-axis gimbal actuator (first-order lag,
  travel-limited) that genuinely deflects thrust direction each step — see
  [Thrust Vector Control](#thrust-vector-control).
- **Closed-loop guidance**: `Navigation` reads simulated `Gps`/`Gyro`
  sensors and steers TVC to point the nose at a fixed target — see
  [Sensors & Navigation](#sensors--navigation).
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
├── include/sim/
│   ├── rocket_types.hpp        Shared plain-data structs: RocketState,
│   │                           RocketParams, SimulationConfig, FlightData,
│   │                           MassProperties, FlightConditions,
│   │                           AerodynamicCoefficients.
│   └── rocket_kinematics.hpp   The RocketKinematics class declaration —
│                               the only header a caller needs.
└── src/sim/
    ├── rocket_kinematics.cpp   Translational motion + top-level loop.
    ├── pitch_dynamics.cpp      Pitch-axis torque model.
    └── yaw_dynamics.cpp        Yaw-axis torque model (exact mirror of pitch).
```

To reuse just the physics elsewhere: take everything under `sim/` +
`ork/ork_mass_components.*` (needed by `VehicleMassModel`) + Eigen,
construct `RocketKinematics(params, config, mass_components)`, and call
`simulate()` or `step()`. (`gnc/` is only needed if you also want TVC/
closed-loop guidance driving thrust direction — see
[Thrust Vector Control](#thrust-vector-control) and
[Sensors & Navigation](#sensors--navigation).)

### `RocketKinematics` method reference

| Method | File | Purpose |
|---|---|---|
| `RocketKinematics(params, config, mass_components)` | sim/rocket_kinematics.cpp | ctor — builds the `AerodynamicsModel`/`VehicleMassModel`, caches Barrowman CP |
| `step(state, thrust, tvc_target_dir=(0,0,1))` | sim/rocket_kinematics.cpp | **public** — advance one `dt`; `state.mass` is this instant's mass; commands TVC toward `tvc_target_dir` and bakes the actuator's new position into the returned state. Not `const` — advancing TVC is real state change; must be called in sequence. |
| `simulate(time, flight_data, tvc_targets={}, navigation=nullptr)` | sim/rocket_kinematics.cpp | **public** — run full flight, one `step()` per sample. `navigation`, if given, overrides `tvc_targets` and drives TVC closed-loop each step (see [Sensors & Navigation](#sensors--navigation)) |
| `getParams()` / `getConfig()` | sim/rocket_kinematics.hpp | **public** — accessors |
| `computeNetForce(state, thrust)` | sim/rocket_kinematics.cpp | **public** — thrust (along the TVC-deflected nozzle direction stored in `state`) + drag (our own Cd) + gravity, world frame, undivided by mass |
| `computePitchTorques(state)` | sim/pitch_dynamics.cpp | **public** — gravity(=0)/aero/damping torque breakdown, for logging (not used by `step()` itself) |
| `computeYawTorques(state)` | sim/yaw_dynamics.cpp | **public** — same breakdown, yaw axis |
| `rocketToNedFrame(state)` | sim/rocket_kinematics.cpp | **public, static** — body → world rotation matrix; a pure function of orientation, reused as-is by `Sensors`/`Navigation` instead of a second hand-copied DCM |
| `computeAcceleration()` | sim/rocket_kinematics.cpp | `computeNetForce() / mass` |
| `buildFlightConditions()` | sim/rocket_kinematics.cpp | state → `FlightConditions` — real angle of attack (α) and sideslip (β), both from velocity rotated into body frame, not a raw-angle approximation; mach, altitude... for the aero model |
| `computeGravityTorque()` / `computeYawGravityTorque()` | sim/pitch_dynamics.cpp / sim/yaw_dynamics.cpp | always `0.0` — gravity has no net torque about a body's own CG (deliberate stub, see below) |
| `computeAerodynamicMoment()` / `computeYawAeroMoment()` | sim/pitch_dynamics.cpp / sim/yaw_dynamics.cpp | aero restoring moment (reads `Cn_alpha` from `AerodynamicsModel`) |
| `computeDampingTorque()` / `computeYawDampingTorque()` | sim/pitch_dynamics.cpp / sim/yaw_dynamics.cpp | resists rotation |
| `computeTotalPitchTorque()` / `computeTotalYawTorque()` | sim/pitch_dynamics.cpp / sim/yaw_dynamics.cpp | sums the 3 torques above |
| `computePitchAcceleration()` / `computeYawAcceleration()` | sim/pitch_dynamics.cpp / sim/yaw_dynamics.cpp | torque ÷ inertia, clamped |
| `updatePitchDynamics()` / `updateYawDynamics()` | sim/pitch_dynamics.cpp / sim/yaw_dynamics.cpp | builds `MassProperties` from `VehicleMassModel` + cached CP, then integrates |

## Aerodynamics & mass properties (Barrowman method)

Computes Cd/Cn_alpha/CP and CG/inertia from the vehicle's own geometry —
nothing here is read from OpenRocket's solved simulation.

```
rocket_cpp/
├── include/sim/
│   ├── aerodynamics.hpp            AerodynamicsModel
│   └── mass_properties_model.hpp   VehicleMassModel
└── src/sim/
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
| `computeCoefficients(fc)` | sim/aerodynamics.cpp | **public** — Cd/Cn/Ca/Cm/Cn_alpha/Cm_alpha at one flight condition |
| `computeCenterOfPressure()` | sim/aerodynamics.cpp | **public** — Barrowman CP, cm from nose tip (geometry-only, Mach-independent) |
| `getDensity()` / `getSpeedOfSound()` | sim/atmosphere.cpp | **public** — shared with `RocketKinematics`, one atmosphere model for the whole sim |
| `computeBodyCnAlpha()` / `computeFinCnAlpha()` | sim/aerodynamics.cpp | Barrowman normal-force slopes (body = 2/rad; fins via span/chord/sweep + Kfb interference) |
| `computeNoseCp()` / `computeFinCp()` | sim/aerodynamics.cpp | geometric CP of each contributor |
| `computeFrictionDrag()` / `computeSkinFrictionCf()` | sim/atmosphere.cpp | wetted-area friction drag (Blasius/Schlichting + roughness limit) |
| `computeBaseDrag()` / `computeBoatTailDrag()` | sim/aerodynamics.cpp | blunt-base and boat-tail pressure drag |
| `VehicleMassModel(components, ...)` | sim/mass_properties_model.cpp | ctor — dry mass/CG/I_yy from the component list |
| `computeAt(total_mass_kg)` | sim/mass_properties_model.cpp | **public** — splits total mass into (dry structure, fixed) + (motor, = total − dry) → CG(t)/I_yy(t) |

## Thrust Vector Control

```
rocket_cpp/
├── include/gnc/
│   └── thrust_vector_control.hpp   ThrustVectorControl
└── src/gnc/
    └── thrust_vector_control.cpp   Target decomposition, first-order lag,
                                     nozzle/force geometry.
```

Simulates the physical gimbal hardware, not a perfect instant aim command:
two independent servos tip the nozzle off its neutral (straight-back, body
**−Z**) position — one in the body X-Z plane (`gimbal_pitch`), one in the
body Y-Z plane (`gimbal_yaw`) — each a first-order lag (`τ=0.05s`, settles
~99% in ~0.25s) clamped to a `±12°` physical travel limit, both hardcoded
constants in the class (no config file yet). `RocketState` carries the
actuator's actual position (`gimbal_pitch_rad`/`gimbal_yaw_rad`) so
`computeNetForce()` stays a pure function of `(state, thrust)` — it rebuilds
the nozzle direction from `state`'s own stored angles, never from the live
`ThrustVectorControl` object, so recomputing it later for logging reads the
same historical deflection the integrator actually used at that instant.

| Method | Purpose |
|---|---|
| `commandForceDirection(dir)` | **public** — aim so the FORCE ends up along `dir` (body frame): inverts to a nozzle target (Newton's 3rd law), decomposes into `gimbal_pitch`/`gimbal_yaw`, clamps each to `±12°`. Only sets the target — doesn't move anything |
| `step(dt)` | **public** — advance both actuators one `dt` toward their targets (first-order lag) |
| `currentNozzleDirection()` | **public** — unit vector, body frame, where the nozzle actually is right now |
| `currentForce(thrust)` | **public** — `-thrust * currentNozzleDirection()`; what `computeNetForce()` uses |
| `currentGimbalPitchRad()` / `currentGimbalYawRad()` | **public** — actuator angles, radians (what gets baked into `RocketState`) |
| `currentGimbalPitchDeg()` / `currentGimbalYawDeg()`, `targetGimbalPitchDeg()` / `targetGimbalYawDeg()` | **public** — same, degrees, plus the (unlagged) targets, for logging/plotting |
| `nozzleDirectionFromAngles(gx, gy)` | **public, static** — the nozzle-geometry formula itself, shared by `currentNozzleDirection()` and by `computeNetForce()` rebuilding it from a `RocketState` |

## Sensors & Navigation

```
rocket_cpp/
├── include/gnc/
│   ├── sensors.hpp      namespace sensors { Gyro, Baro, Gps }
│   └── navigation.hpp   Navigation
└── src/gnc/
    ├── sensors.cpp
    └── navigation.cpp
```

Simulated flight-computer sensors, each reading straight from the true
`RocketState` — no noise/bias/drift modeled yet, just the `update()` then
`read...()` shape real sensor drivers use. One class per physical sensor
(mirrors a real flight computer's separate parts), namespaced under
`sensors` rather than one do-everything class:

| Class | Method | Purpose |
|---|---|---|
| `sensors::Gyro` | `update(state, prev_state, dt)` | computes proper (specific) acceleration — total minus gravity, since an accelerometer's proof mass doesn't feel gravity — and angular acceleration, both body frame |
| | `readAccel()` / `readAngularAccel()` | m/s², rad/s² |
| | `readOrientation()` | true `(roll,pitch,yaw)` passthrough — a stand-in for real attitude estimation (gyro/accel fusion), not built yet |
| `sensors::Baro` | `update(state)`, `readPressure()`, `readAltitude()` | ISA pressure at true altitude; derived altitude matches true altitude exactly since no sensor error is modeled yet |
| `sensors::Gps` | `update(state)`, `readPosition()` | world-frame position fix |

`Navigation` is the classic, deliberately un-smart guidance law: **point
the nose at a fixed target**, no PID, no trajectory optimization.
`ThrustVectorControl`'s own actuator lag is the only closed-loop dynamics
involved — stacking a second controller on top would compound lag on lag
and slow the response down, not help it.

| Method | Purpose |
|---|---|
| `Navigation(target_position)` | ctor — world-frame aim point, fixed for the flight (hardcoded by the caller, e.g. `main.cpp`'s `NAV_TARGET`; no in-flight retargeting yet). **Throws `std::invalid_argument`** if `target_position.z() <= 10.0` — not an "out of bounds" check, aiming the nose (and thrust) at/into the ground is unsurvivable regardless of how "in range" the coordinates look |
| `computeTvcTarget(gps, gyro)` | **public** — `target − gps.readPosition()` (world frame), rotated into body frame via `RocketKinematics::rocketToNedFrame(gyro.readOrientation())`. Stateless — no integral term, no memory between calls |

Plugs into `RocketKinematics::simulate(time, flight_data, {}, &navigation)`
— see the method reference above. `main.cpp` writes the target into
`rocket_trajectory.csv` (`target_x/y/z` columns) so `scripts/trajectory.py`
can show whether it was actually reached (see
[Visualize a trajectory](#visualize-a-trajectory)).

## OpenRocket ingestion (`.ork` → simulation inputs)

```
rocket_cpp/
├── include/ork/
│   ├── ork_archive.hpp           readOrkXml(path) -> string
│   ├── ork_geometry.hpp          parseOrkGeometry(xml) -> RocketParams
│   ├── ork_flightdata.hpp        parseOrkFlightData(xml, dt, motor?) -> FlightData
│   │                             parseOrkLaunchConditions(xml, motor?) -> SimulationConfig
│   ├── ork_mass_components.hpp   parseOrkMassComponents(xml) -> vector<MassComponent>
│   └── ork_loader.hpp            loadOrkRocket(path, dt, motor?) -> OrkRocket
└── src/ork/
    ├── ork_archive.cpp         Unzips the .ork (it's a zip). (libzip)
    ├── ork_geometry.cpp        Walks <nosecone>/<bodytube>/<trapezoidfinset>
    │                           for RocketParams geometry + reference_area. (tinyxml2)
    ├── ork_flightdata.cpp      Picks the embedded <simulation> for the target
    │                           motor (default: config marked default="true"),
    │                           reads thrust/mass + launch conditions,
    │                           resamples to uniform dt. Prefers a match that
    │                           actually has flight data (a design can have
    │                           multiple <simulation> entries for the same
    │                           motor config -- a stale, never-run one and a
    │                           real one -- picking blindly grabs whichever
    │                           comes first in the file). (tinyxml2)
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
| `parseOrkFlightData(xml, dt, motor?)` | XML → `FlightData` (thrust, mass). Throws `std::runtime_error` if the matched `<simulation>` has no `<databranch>` at all — meaning that design was never actually run inside OpenRocket before saving; open it there, run the simulation, and re-save before pointing this loader at it. |
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
| Which motor config | `loadOrkRocket(path, dt, motor?)` (`ork/ork_loader.hpp`) | Defaults to the `.ork`'s `default="true"` simulation; pass a motor name to pick another of the 5 embedded configs. |
| Navigation target | `NAV_TARGET` in `main.cpp` | World-frame point `Navigation` steers TVC toward (see [Sensors & Navigation](#sensors--navigation)). Must be `z > 10m` — anything on/near the ground throws at construction. |
| TVC actuator limits/speed | `MAX_GIMBAL_DEG`, `TAU_PITCH_S`, `TAU_YAW_S` in `gnc/thrust_vector_control.hpp` | Hardcoded private constants, no config file yet. |

Re-run `cmake --build build` after editing, then `./build/rocket_cpp` again
— **a source edit alone changes nothing on disk**: `rocket_trajectory.csv`
(and anything reading it, like `trajectory.py`) still reflects the
*previous* build until you rebuild and rerun.

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

If the CSV has `target_x/y/z` columns (i.e. the run passed a `Navigation`
to `simulate()`), the target is plotted as a purple star and a banner shows
**green "SUCCESS"** (closest approach ≤ 20m) or **red "FAIL"** (closest
approach over the whole flight, not just the final position — this is
open-loop point-and-shoot guidance with no terminal intercept phase, so
"did it ever get close" is the meaningful question, not "where did it end
up after coasting past"). CSVs without those columns just skip this —
older/non-guided runs don't error.

### Plot the TVC command history

```bash
python3 scripts/tvc.py rocket_trajectory.csv   # defaults to this path if omitted
```

Opens a matplotlib window (same pattern as `trajectory.py`) with two
stacked panels — `gimbal_pitch_deg` and `gimbal_yaw_deg` vs. time, one per
actuator (see [Thrust Vector Control](#thrust-vector-control)) — so you can
see each axis's command history independently: the first-order rise,
whether/when it hit the `±12°` travel limit, and how long it held there.

## Coordinate frames & axis conventions

![Axis, orientation-angle and planned-TVC symbol reference](docs/axis_and_tvc_reference.png)

Left: body vs. world frame and the pitch (θ)/yaw (ψ)/roll (φ) angles this
sim actually uses, at an illustrative non-zero pose — every symbol maps to
the `orientation(0..2)` index used in code, spelled out below. Right: the
thrust-vector-control gimbal cone (see [Thrust Vector Control](#thrust-vector-control)
for the real, implemented `ThrustVectorControl` class this diagrams) —
nozzle deflection is measured off the body **−Z** axis (nozzle-neutral,
opposite the nose), and the resulting force on the vehicle (what feeds
`computeNetForce()`) is the negation of that direction, per Newton's third
law. The diagram's `γ_max` is exaggerated to 20° for legibility — the
actual `MAX_GIMBAL_DEG` is 12°. Regenerate this image with
`scripts/make_axis_diagram.py` if the axis convention ever changes.

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
| `gimbal_pitch_rad`, `gimbal_yaw_rad` | TVC nozzle's actual (lagged) deflection this instant — see [Thrust Vector Control](#thrust-vector-control) | rad |

`RocketParams` — what the rocket physically **is**, fixed for the whole
flight (parsed by `ork/ork_geometry.cpp`):

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
(parsed by `ork/ork_flightdata.cpp`; the one thing still read from the `.ork`,
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

### 3. Aerodynamic coefficients (Barrowman method, `sim/aerodynamics.cpp`)

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

### 4. Mass properties (`sim/mass_properties_model.cpp`)

CG is exact — mass-weighted average of every structural component's own CG (bulk: density×volume, surface: density×area, line: density×length; see the `ork/ork_mass_components.cpp` tree above). Dry pitch inertia is parallel-axis (point mass per component) plus a rod correction for the body tube ($mL^2/12$, since it spans a large fraction of the vehicle's length):
$$
I_{yy,dry} = \sum_i m_i(x_i-x_{cg})^2 + \frac{m_{tube}L_{tube}^2}{12}
$$
At each instant, motor mass = `total_mass(t) − dry_mass` (fixed axial position = the motor mount tube's center); combined CG/I_yy follow from the same parallel-axis approach.

### 5. Pitch dynamics (1DOF rotational, `sim/pitch_dynamics.cpp`)

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
not part of the integration itself — plus `gimbal_pitch_deg, gimbal_yaw_deg`
straight from each state's stored TVC actuator position, and `target_x,
target_y, target_z` — constant every row, `NAV_TARGET` repeated, for
`trajectory.py`'s success/fail check) and invokes `scripts/plot_trajectory.py`,
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
2. **"NED" labeling is inaccurate** — see [Coordinate frames](#coordinate-frames--axis-conventions).
3. **Roll is purely kinematic** — no torque model, no inertia coupling (fin
   cant is parsed but unused). Yaw *does* now have a real torque model
   (`sim/yaw_dynamics.cpp`, exact mirror of pitch's), so it's no longer purely
   kinematic — but its aerodynamic damping is severely underdamped
   (measured ζ≈0.003 against real flight data, vs. ζ=1 for critical
   damping — the damping torque scales linearly with velocity while the
   restoring moment scales with velocity², so their ratio stays this low
   regardless of speed or altitude; not something that improves at some
   flight phase), so once something excites real sideslip (e.g. TVC actively
   steering in Y) yaw oscillates for most of the flight instead of
   settling. Real air-relative α *and* β are both implemented now
   (rotating velocity into body frame — see `buildFlightConditions()`
   above); Euler-rate kinematics and roll dynamics are still what's
   missing for true 6DOF.
4. **`I_xx` (roll inertia) is a coarse thin-shell/point estimate** in
   `VehicleMassModel` — fine for now since nothing reads it (no roll torque
   model yet), but should be revisited before any roll dynamics are added.
5. **Sensors now have a real noise model, but update-rate limiting is
   still missing.** `Gyro`/`Baro`/`Gps` add white noise plus a slowly
   drifting bias (`GaussMarkovNoise`) grounded in real numbers — the
   actual MPU-9250 datasheet for the gyro/accel, general literature for
   GPS/baro (see `gnc/sensors.hpp`) — but every sensor still reads every
   physics step with no rate limit. `Gyro::readOrientation()` is still a
   bare pass-through of true attitude, deliberately: a real IMU doesn't
   give absolute attitude for free, that needs integrating/fusing the
   rates (or a magnetometer, deliberately left out), which isn't built.
6. **`Navigation` now runs a real per-axis PID** (`Kp=1, Ki=0.6, Kd=0` —
   see its class comment for how these were tuned, and why `Kd` measurably
   hurts here). Still one fixed target for the whole flight, no in-flight
   retargeting, no terminal guidance phase. TVC's actuator limit is
   `±30°` (`ThrustVectorControl::MAX_GIMBAL_DEG`).
7. **GPS noise's correlation time (60s) is comparable to the whole flight
   (~25s)**, so for any single flight it behaves like a near-constant
   several-meter miscalibration rather than noise that averages out — and
   because the guidance law's angular error is a normalized ratio, that
   fixed offset produces a large, *persistent* angular error whenever the
   vehicle happens to be close to the target, not brief jitter a filter
   could remove. Measured: gimbal saturation at the `±30°` limit rose from
   ~40% to ~84-89% of the flight once GPS noise was added, while accuracy
   stayed statistically unchanged (closest approach ~48m either way,
   averaged over repeated noise draws) — a real, known limitation of
   steering off one noisy absolute-position fix with nothing else backing
   it up, not a gain-tuning bug.
8. **`Navigation` now fuses GPS with the accelerometer** (an
   "acceleration-aided" alpha-beta/g-h filter, `estimatePosition()` in
   `gnc/navigation.cpp`, gains derived from real sensor numbers, not
   hand-picked) rather than reading raw GPS straight into the PID. Tested
   head-to-head against raw GPS on the identical scenario (8 runs each):
   mean closest-approach was statistically unchanged (61.9m raw vs. 62.4m
   fused) — confirming #7 above, no filter can reject a persistent bias
   in the only absolute-position sensor available — but run-to-run
   variance dropped ~10x (std 2.0m → 0.2m). That's the real, honest
   benefit: a far more repeatable guided trajectory, not better average
   accuracy. A genuine bias-rejecting fix would need a second independent
   *absolute* reference, which this vehicle doesn't carry.
9. **`simulate()` used to silently cut a flight short if it outlived the
   `.ork`'s own flight-data array** (`n_steps` was clamped to
   `flight_data.time.size()`, not just to the requested `sim_duration`)
   — found via a wide/high off-axis target (`(400, 150, 900)`) whose
   flight genuinely outlasted the `.ork`'s ~45s of recorded thrust/mass
   data, so the loop just stopped mid-air at ~530m, which plotting
   scripts then mislabeled "Impact". Fixed: `n_steps` now comes from
   `sim_duration` alone, and thrust/mass read as 0/dry-mass past the
   recorded data's end (real motors don't restart or un-burn propellant).
   That exposed a second, real finding underneath it: even with the array
   clamp gone, that same scenario didn't reach the ground until t=124s —
   post-burnout, the vehicle settled into an extended near-horizontal
   glide (pitch pinned at the `MAX_PITCH` ~86° safety clamp for tens of
   seconds, over 10 km of horizontal travel) rather than a quick ballistic
   fall, because its nose weathercocked to align with a mostly-horizontal
   velocity vector built up from sustained `±30°` TVC saturation before
   burnout. `main.cpp`'s `sim_duration` raised from 120s to 300s to give
   flights like this room to actually finish. Not obviously a bug in the
   pitch model itself — a legitimate (if extreme) outcome of this sim's
   decoupled pitch/yaw dynamics reacting to a very aggressive target — but
   worth knowing before trusting a "did it reach the ground yet" check on
   an unusual trajectory.
10. **That same extended-glide trajectory (#9) turned out to be a genuine,
    multi-cycle phugoid oscillation**, not a single dip: height after
    apogee goes 2170m -> min 537m (t=47.5) -> max 1051m (t=64.5) -> min
    182m (t=90.8) -> max 428m (t=107.4) -> ground, each swing smaller than
    the last. Verified this isn't an energy-conservation bug two ways:
    specific mechanical energy (`0.5*v^2 + g*h`) drops monotonically in
    aggregate across the whole ~103,680-step post-apogee flight (from
    ~21,700 J/kg to ~30 J/kg); the individual steps that technically
    ticked energy *up* (8202 of them, explicit Euler doesn't exactly
    conserve energy for a nonlinear system, unlike a symplectic
    integrator — see "Integration" above) sum to only 120.6 J/kg total
    against 21,790.5 J/kg of real loss, i.e. <1% and nowhere near enough
    to explain 500m+ swings. The swings themselves are a real, named
    flight-dynamics phenomenon (phugoid: pitch-stable body trading speed
    for altitude and back, weakly damped so it rings instead of settling)
    — and the reason it's THIS dramatic here, rather than a minor wobble,
    traces straight back to #3's already-measured weak pitch/yaw
    aerodynamic damping (same `c_damp` formula, previously quantified at
    ζ≈0.003 for yaw). A better-damped vehicle wouldn't ring like this.
11. **Traced #10's phugoid to its actual root cause: pitch used to
    hard-clamp at `MAX_PITCH` (~85°) for the ENTIRE flight**, not just the
    ignition transient its own comment claimed it guarded. Yaw has never
    clamped, only wrapped (`fmod`) — confirmed that asymmetry, not any
    real physical difference between the axes, was why yaw could settle a
    wild excursion (its own early swings hit 45-158° before damping back
    under 2° by t=40s) while pitch got trapped at its wall instead of
    letting `computeAerodynamicMoment`'s restoring torque keep tracking
    the vehicle's real (extreme but legitimate) velocity vector past 90°.
    Fixed in two steps: pitch now wraps like yaw does, and the angle
    clamp is scoped to only the true ignition window (`elapsed_time_s_ <=
    0.1s`, tracked on `RocketKinematics`) instead of the whole flight —
    matching what the original comment always said it was for.
    **But fully unclamping past ignition exposed a second, real problem**:
    for this aggressive a target, pitch doesn't settle once free — it
    tumbles (measured: 182-193°, up to 475°/s, never reaching the ground
    in 300s). The restoring torque saturates past ~28.6° angle of attack
    (`alpha_limited` in `computeAerodynamicMoment`, a physically-motivated
    stall cap) and, combined with the already-known weak damping, isn't
    strong enough to arrest a large enough excursion once unclamped. Yaw
    never hit this because its swings stayed inside the unsaturated
    range. Left open deliberately — fixing it for real means retuning the
    aerodynamic damping/stall model, not another clamp; scoping the
    ignition clamp correctly was the honest fix for what it actually was,
    not a fix for this deeper, separate finding.
12. **#11's "tumbling" turned out not to be a damping/stall problem at
    all — it was a real, serious, pre-existing bug in `computeNetForce()`**
    that #11's fix only happened to expose. `drag_ned = R * drag_neg` was
    rotating an ALREADY world-frame vector (`drag_neg`, built straight
    from `state.velocity`, which has always been world-frame — see
    `RocketState`'s own doc comment) through the body->world rotation
    matrix a second time. Mostly harmless whenever the vehicle's attitude
    stays close to its velocity direction (weathercocking usually keeps
    it there), which is why every earlier scenario this session looked
    physically clean. But during a fast, wide pitch swing -- exactly what
    #11 unlocked -- attitude and velocity direction can diverge sharply,
    and double-rotating drag through the wrong matrix can flip it from
    opposing velocity to reinforcing it. Measured before the fix: post-
    burnout trough velocities of 922-968 m/s (Mach 2.7+, growing every
    cycle) and 125,046/247,486 post-apogee steps with energy *increasing*
    — not Euler noise, a real net energy source, up to 2313 N of
    unphysical force on a 552g vehicle at one recorded instant. Fixed by
    using `drag_neg` directly (no rotation -- it was never in body frame
    to begin with); only `thrust_ned = R * thrust_body` genuinely needed
    the rotation, since gimbal-commanded thrust really is body-frame.
    Verified after the fix: zero energy-increasing steps across the
    entire post-apogee flight (was 125,046), max post-apogee velocity
    down to 107 m/s, clean ground contact at t=48.9s instead of running
    out the full 300s budget still airborne. #10's earlier "verified, not
    a bug" phugoid finding was real *for that specific scenario*, where
    attitude happened to stay close enough to velocity direction that
    this bug's effect stayed small -- not wrong, just working with data
    quietly corrupted by a latent bug that hadn't been triggered hard
    enough yet to be visible.
