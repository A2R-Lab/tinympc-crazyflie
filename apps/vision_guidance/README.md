# vision_guidance

App-layer **visual standoff / centering** for the Crazyflie, running on top of the
**stock PID controller** (not TinyMPC). It pairs with the GAP8 CV module in
`esp_color_object/comms/comms_deck.c`.

## What it does

Perception → guidance → stock controller. This app does **not** replace the flight
controller — it turns camera detections into setpoints and lets PID fly them.

1. Triggers the GAP8 over CPX every `trigMs` (~50 Hz) with an 8-byte `TriggerRequest`.
2. Receives the GAP8's 26-byte `DetectionResponse` asynchronously
   (`cpxRegisterAppMessageHandler`), validates it, stores it under a mutex.
3. Converts the latest fresh detection into a body-frame velocity `setpoint_t` and
   pushes it via `commanderSetSetpoint(&sp, 3)`.

Control law when armed (`visGuid.enable = 1`):
- Hold altitude `visGuid.targetZ` (needs a Flow / Z-ranging deck).
- Forward/back to hold blob pixel-area `visGuid.targetArea` → standoff distance.
- Yaw (and/or strafe) to keep the blob horizontally centered.
- Target lost / stale (> `detTimeoutMs`) → hold position, zero horizontal velocity.

> **Distance caveat:** the GAP8 does not measure true depth (`real_z_mm` is
> defaulted). Standoff uses **blob pixel-area** as an inverse-distance proxy, valid
> only if the bright target's real-world size is roughly constant. For metric
> distance, add a range sensor and switch the forward law to use it.

## Relationship to the MPC AI-deck bridge (cv branch)

`controller_tinympc.cpp` on `cv` has its own AI-deck CPX bridge (same protocol),
feeding detections into TinyMPC. That is a **separate strategy**. This app builds
the stock controllers only (its `app-config` does not set `CONFIG_CONTROLLER_OOT`),
so the TinyMPC bridge is not compiled in and the two never contend for the GAP8
trigger or the CPX app handler. Select PID at runtime: `stabilizer.controller = 1`.

## Params (`visGuid`)

| param | meaning |
|---|---|
| `enable` | 0 = passthrough (commander in charge), 1 = visual servo |
| `targetZ` | altitude to hold while servoing (m) |
| `targetArea` | desired blob area in pixels (standoff setpoint) |
| `kpDist` | (m/s) per pixel of area error → forward/back |
| `kpYaw` | (deg/s) per pixel of x error → yaw to center |
| `kpStrafe` | (m/s) per pixel of x error → body-y (off by default) |
| `vMaxXY` | clamp on body x/y velocity (m/s) |
| `yawRateMax` | clamp on yaw rate (deg/s) |
| `trigMs` | trigger/guidance loop period (ms) |
| `detTimeoutMs` | detection considered stale after this (ms) |

## Logs (`visGuid`)

`found`, `cx`, `cy`, `area`, `vx`, `vy`, `yawRate`, `rxCount`.

---

## Toolchain setup (one-time, Ubuntu/aarch64)

### Compilers
```bash
sudo apt install -y build-essential               # host gcc/make (Kbuild host tools)
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
```

### Flashing tool (cflib in a venv)
Modern `cflib` (>=0.1.3x) dropped the `cfloader` module the firmware Makefile calls
via `python -m cfloader`; the standalone script only ever shipped with the heavy
`cfclient`/Qt package. `tools/cfloader.py` here is a minimal drop-in replacement.

```bash
sudo apt install -y python3-venv           # provides ensurepip (e.g. python3.14-venv)
python3 -m venv ~/cf-venv
~/cf-venv/bin/python -m pip install --upgrade pip cflib
```

### Crazyradio USB permissions (udev)
```bash
sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
# ensure your user is in plugdev (then re-login if you had to add it):
id -nG | grep -q plugdev || sudo usermod -aG plugdev $USER
```
Verify access (should print a list, no traceback):
```bash
~/cf-venv/bin/python3 -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"
```

## Build & flash

```bash
cd apps/vision_guidance
make

# Put the Crazyflie in bootloader mode: power off, hold power button ~3 s
# until the blue LEDs blink. Then (within the scan window):
CFLOADER="$HOME/cf-venv/bin/python3 $PWD/tools/cfloader.py"
make cload CLOAD_SCRIPT="$CFLOADER"
```

Warm boot (CF powered on, known radio URI — no button needed):
```bash
make cload CLOAD_SCRIPT="$CFLOADER" CLOAD_CMDS="radio://0/80/2M"
```

## First bring-up (do this before any flight)

1. **Props OFF.** Flash, leave `visGuid.enable = 0`, power the AI-deck.
2. In cfclient, watch the `visGuid` log group. Wave the bright target and confirm
   `rxCount` climbs and `found` / `cx` / `area` respond sensibly.
3. Check the **signs**: target to the right (`cx > 80`) should yield negative
   `yawRate`; target too far (small `area`) should yield positive `vx`.
4. Only once the signs look right: take off and hover with a Flow deck, set
   `visGuid.targetZ` to your hover height, then set `visGuid.enable = 1`.
   Set `enable = 0` to hand control back, then land.
