# Simulation

This directory contains the existing software-in-the-loop environment and
everything used exclusively by it. Keeping it under one boundary leaves the
repository root available for future flight-software, avionics, telemetry,
ground-station, and hardware modules.

## Layout

- `rocket_cpp/` — active C++ physics, sensor, navigation, and TVC simulator
- `data/` — OpenRocket design and exported simulation inputs
- `artifacts/` — simulator images and legacy visual references
- `matlab/` and `simulink/` — superseded prototype models
- `docs/` — generated simulator manuals
- `PLAN.md` — original simulator plan

Build the active simulator from the repository root with:

```bash
cmake -S sim/rocket_cpp -B sim/rocket_cpp/build
cmake --build sim/rocket_cpp/build
```

The executable resolves its default vehicle input from `sim/data/rocket.ork`,
so it does not depend on the repository's hardware or future flight modules.
