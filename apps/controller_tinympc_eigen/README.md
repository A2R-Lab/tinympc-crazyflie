# TinyMPC Eigen Controller

This is the supported out-of-tree TinyMPC controller on `main`.

## Codegen Pipeline

Regenerate the deployable controller cache from the `TinyMPC` submodule first:

```bash
cd ../../TinyMPC
cmake -S . -B build
cmake --build build --target codegen_quadrotor_100hz
./build/examples/codegen_quadrotor_100hz
```

Run that from the `TinyMPC/` repo root. Do not create a separate build directory inside `examples/` for this workflow.

When the repo is laid out as this superproject, that command refreshes:

- `src/params_100hz.h`
- `src/generated/quadrotor_100hz_generated.hpp`

`src/params_100hz.h` is the generated deployable cache used by the controller.
The deployed firmware still runs the legacy embedded `TinyMPC-ADMM` solver path, which is the known-good runtime for this controller.

## Build

After codegen has refreshed `src/params_100hz.h`, run the build from this directory:

```bash
make -j8
```

The generated firmware artifacts are written to `build/`.

## Flash

```bash
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
```

## Controller Behavior

- The controller uses the Crazyflie out-of-tree app/controller build flow.
- The params come from top-level `TinyMPC` codegen, but the on-drone runtime remains `TinyMPC-ADMM`.
- `en_traj` in `src/controller_tinympc.cpp` is a compile-time flag, not a runtime Crazyflie parameter.
- On this branch, `en_traj` defaults to `false`, so the controller follows commander/setpoints unless you explicitly turn the baked trajectory on in source.
