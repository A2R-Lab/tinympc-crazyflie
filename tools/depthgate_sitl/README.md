# Active-controller SITL build

This directory supplies a synthetic receiver for the `old-mpc-hover` controller,
without modifying its hardware receiver. The synthetic source is off by default.
Set `DEPTHGATE_SITL_DEPTHS=3,0.8,1` to publish fixed optical-axis sector depths in
meters at nominal 10 Hz, with 83 ms reported inference duration. These inputs are
fixtures, not simulated camera observations or an obstacle geometry oracle.

An isolated snapshot was built on the flashing VM under
`/home/vboxuser/depthgate-sitl-20260907`, using the existing SDK Docker image's
native Linux C++ compiler. No USB devices or privileges were needed. The local
Docker daemon was unavailable. Snapshot layout:

- `crazyflie-firmware`: copy of primary repository's patched CrazySim firmware,
  excluding build directories and object files.
- `apps/controller_tinympc_eigen`: active app source/core/Eigen snapshot.
- `tools/depthgate_sitl`: these support files.
- `build/cf2`, `configure.log`, `build.log`: native SITL executable and evidence.

Run `prepare.py <isolated-firmware-directory>` only on a copy. It removes the
primary app's racing/debug sources from CMake and replaces the perception source
with this receiver; controller and solver sources remain unchanged.

Inside a Linux container with the snapshot mounted at `/work`:

```sh
python3 /work/tools/depthgate_sitl/prepare.py /work/crazyflie-firmware
cmake -S /work/crazyflie-firmware/sitl_make -B /work/build \
  -DTINYMPC_APP_DIR=/work/apps/controller_tinympc_eigen \
  -DTINYMPC_SITL_SUPPORT_DIR=/work/tools/depthgate_sitl \
  -DCMAKE_CXX_FLAGS=-DTINYMPC_ESPNET_STRAIGHT_TEST=1
cmake --build /work/build --target cf2 -j2
```

Full build and link passed on 2026-09-07. This establishes compilation only;
it is not a closed-loop flight validation. The initial five-second executable
probe produced no output before timeout without a simulator attached. Resync the
active source and rebuild before testing any later controller revision.
