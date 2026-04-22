# TinyMPC Eigen Controller

This is the supported out-of-tree TinyMPC controller on `main`.

## Build

Run the build from this directory:

```bash
make clean
make
```

The generated firmware artifacts are written to `build/`.

## Flash

```bash
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
```

## Controller Behavior

- The controller uses the Crazyflie out-of-tree app/controller build flow.
- `en_traj` in `src/controller_tinympc.cpp` is a compile-time flag, not a runtime Crazyflie parameter.
- On `main`, `en_traj` defaults to `false`, so the controller follows commander/setpoints unless you explicitly turn the baked trajectory back on in source.
