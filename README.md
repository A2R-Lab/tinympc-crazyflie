# TinyMPC Crazyflie

`main` ships a single supported out-of-tree controller: `apps/controller_tinympc_eigen`.

The experimental task-based and PSD obstacle-avoidance controller has been split out of the default path. Preserve or continue that work on the `research/eigen-task-psd` branch instead of `main`.

## Build The Supported Controller

1. Clone the repository with submodules:

```bash
git clone --recursive git@github.com:A2R-Lab/tinympc-crazyflie.git
cd tinympc-crazyflie
```

2. Build from the controller app directory, not from the repository root:

```bash
cd apps/controller_tinympc_eigen
make clean
make
```

3. Flash from the same directory:

```bash
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
```

## Notes

- `controller_tinympc_eigen` is the supported controller on `main`.
- The controller build wraps the Crazyflie firmware build system automatically.
- On macOS, the app `Makefile` auto-detects GNU `coreutils` and `gnu-sed` shims when Homebrew provides them.
- Trajectory playback in `controller_tinympc_eigen` is controlled by the compile-time `en_traj` flag in `src/controller_tinympc.cpp`.
