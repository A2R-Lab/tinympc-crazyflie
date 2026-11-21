# TinyMPC Crazyflie

`main` ships a single supported out-of-tree controller: `apps/controller_tinympc_eigen`.

The `sdp-ct` project adds [TinySDP-CT](apps/tinysdp_ct/README.md), an on-board planner with desktop tests and host flight/bench scripts.

The experimental task-based and PSD obstacle-avoidance controller has been split out of the default path. Preserve or continue that work on the `research/eigen-task-psd` branch instead of `main`.

## Container workflow (Linux)

Requires Docker accessible as your normal user (`docker info`), `sudo`, and `udev`.
The Ubuntu 24.04 image provides the ARM toolchain, host build tools, `cfloader`, and the drone picker.
The repo is mounted at `/workspace`; container processes run with your UID/GID, so build outputs stay in the host checkout.

From the repository root, on each new computer:

```bash
git submodule update --init --recursive
./cf-container setup
```

`setup` builds `cf-container:24.04`, configures `plugdev`, installs [USB rules](tools/99-cf-container.rules) at `/etc/udev/rules.d/99-cf-container.rules`, and reloads them. It explains and prints each privileged command before running it. Log out/in if requested.

For TinySDP-CT, apply its required linker fix **once per fresh firmware checkout**, before building. Skip this step if already applied:

```bash
git -C crazyflie-firmware apply ../apps/tinysdp_ct/firmware-ldflags.patch
```

| Command | Purpose |
|---|---|
| `./cf-container build APP` | Build firmware; prints Flash, RAM, and CCM allocation summaries. |
| `./cf-container tests APP` | Run the app's desktop tests. |
| `./cf-container pick` | Select a radio URI using the terminal menu. |
| `./cf-container pick --demo` | Try the menu with simulated drones and no USB access. |
| `./cf-container flash APP URI` | Flash an existing build; does not rebuild it. |
| `./cf-container interactive [--usb]` | Open Bash in `/workspace`; add `--usb` for hardware access. |
| `./cf-container image` | Rebuild the image after Dockerfile changes. |

`APP`: `tinysdp_ct`, `controller_tinympc_eigen`, or `eigen_test`. `eigen_test` has no desktop tests.
New builds default to Brushless (`cf21bl`) for TinySDP-CT and `cf2` for the other apps; subsequent builds reuse `build/.config`.

Brushless example (replace the URI with the one selected in the menu):

```bash
./cf-container build tinysdp_ct
./cf-container tests tinysdp_ct
./cf-container pick
./cf-container flash tinysdp_ct radio://0/80/2M/E7E7E7E7E7
```

## Drone picker

Built with Textual and Typer. Scan, select a drone, or enter a URI manually. Scanning covers the default radio address and saved addresses; enter a custom-address URI once to include it in future scans.

Crazyradio must have its radio firmware installed; see [Bitcraze's initial setup](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyradio-2-0/).

Select a row and click **Rename** or press **F2** to save a local nickname. Recent selections and names persist in `.cf-container/robots.json` (Git-ignored). Demo history is separate. On exit, the picker prints a timestamped action summary; a successful selection also prints its URI. Selecting a drone does not flash it.

## Deployment reports

Reports are created **only when explicitly requested**. Use the [JSON template](tools/reports/deployments/TEMPLATE.json) and save reports, small logs, and source patches under `tools/reports/deployments/<report-id>/`. [AGENTS.md](AGENTS.md) defines naming, source/binary traceability, memory accounting, and commit-message conventions.

## Native build (main controller)

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
