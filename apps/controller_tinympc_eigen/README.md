# TinyMPC Eigen Controller

This is the supported out-of-tree TinyMPC controller on `main`.

## Build

Run the build from this directory:

```bash
make clean
make
```

The generated firmware artifacts are written to `build/`.

## Generate TinyMPC firmware parameters

The controller consumes a fixed-size specialization generated from the current
upstream TinyMPC checkout. `TinyMPC` at the repository root must point to that
checkout, and Python must have `autograd` and `numpy` installed.

From `apps/controller_tinympc_eigen`, run:

```bash
python3 tools/tinympc_to_crazyflie_adapter.py
make
```

The adapter prompts for the Crazyflie type, attached decks, propeller guards,
horizon, model timestep, solve rate, ADMM iteration cap, half-space capacity,
and constrained portion of the horizon. Press Enter to accept any displayed
default. It then:

1. builds and linearizes the selected Crazyflie dynamics with Autograd;
2. calls current TinyMPC through `tools/tinympc_cpp_bridge.cpp` to precompute
   the Riccati and affine caches; and
3. writes `src/tinympc_generated_params.h` for the STM32 build.

The generated header is the source of truth for the runtime horizon, prediction
timestep, solve rate, iteration cap, A/B/f model, Q/R costs, bounds, hover input,
and solver caches. Do not edit it manually. Regenerate it whenever the vehicle
configuration or solver parameters change, then rebuild and check the reported
flash/RAM use before flying. The embedded half-space store supports at most 25
horizon knots; the adapter rejects larger configurations.

## Flash

```bash
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
```

## Controller Behavior

- The controller uses the Crazyflie out-of-tree app/controller build flow.
- `en_traj` in `src/controller_tinympc.cpp` is a compile-time flag, not a runtime Crazyflie parameter.
- On `main`, `en_traj` defaults to `false`, so the controller follows commander/setpoints unless you explicitly turn the baked trajectory back on in source.
