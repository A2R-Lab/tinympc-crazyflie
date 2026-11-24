# TinySDP-CT on the Crazyflie

This folder holds the on-board continuous-time SDP planner of the VOTUM paper, with its commitment rule, as a Crazyflie 2.x out-of-tree app. It builds against this repository's `crazyflie-firmware` submodule.

**Source of truth.** This is a copy of `embedded/tinysdp-ct-crazyflie/` and `include/tinysdp_ct_embedded/` from the paper repository (`ishaanamahajan/sdp-diffusion`, commit `3d21c627`; host scripts and SIL from `e4b74751`), laid out for this repository: the app at the top of this folder, the planner headers in `include/`. Make changes there first and copy them here.

**Firmware patch (required).** At the pinned firmware commit (`de14ff9f`), the firmware's recursive make appends `--specs=nosys.specs` twice when the app uses C++, and the link fails with "attempt to rename spec 'link_gcc_c_sequence'". `firmware-ldflags.patch` is the one-line fix (it de-duplicates `LDFLAGS`), the same change as in the lab's working firmware checkout. Apply it once before building:

```bash
git -C crazyflie-firmware apply ../apps/tinysdp_ct/firmware-ldflags.patch   # from the repository root
```

> Not to be confused with the older TinySDP code on the `psd-ubuntu` and `research/eigen-task-psd` branches of `A2R-Lab/tinympc-crazyflie`. That code is a discrete-time 3×3 lifted-disk constraint inside TinyMPC, with no certificate. This app is the TinySDP-CT method.

| Path | What |
|---|---|
| `Makefile`, `Kbuild`, `app-config*`, `src/` | Firmware app: `src/tsdp_app.c` (tasks, setpoints, params, logs) and `src/tsdp_planner.cpp` (the planner from `include/tinysdp_ct_embedded/`) |
| `include/tinysdp_ct_embedded/` | The embedded planner, checker and route encoder (float, no heap) |
| `firmware-ldflags.patch` | The one-line firmware build fix above |
| `host/fly_tinysdp_ct.py` | Host flight script: takeoff, obstacle streaming, logging, landing, geofence (cflib) |
| `host/scenarios/` | Flight scenarios; the same four cases as the desktop test |
| `test/` | Software-in-the-loop test: the exact app source with stub firmware headers |

## What runs on board

**Planner task** (25 Hz control loop, 40 ms period, planning budget `tsdp.budgetMs`, default 30 ms).
Frame *k* plans from the reference state at the next period boundary. From that boundary it executes the first option that works, the same rule as the simulator:

| Decision | Option |
|---|---|
| 0 | New TinySDP-CT plan: two C¹ cubic Bernstein segments, checked on board in continuous time against every streamed obstacle (radius plus `tsdp.robotR`), with `tsdp.margin` of clearance, and accepted by the commitment rule below |
| 1 | Retained tail of the active trajectory, if it re-certifies against the current obstacles |
| 2 | Braking trajectory from the reference state, if it certifies |
| 3 | Failure: logged; the active reference continues uncertified |

If the state estimate leaves the reference by more than `tsdp.resetD`, the frame plans from the estimate and skips the tail. The check is skipped once in the frame right after such a plan: that plan starts one period late, so during that period the drone still follows the older reference, and comparing against the new plan would trigger another reset (two branches leapfrogging; the SIL tracking-kick scenario had 90 consecutive reset frames before this).

**Commitment rule** (`tsdp.commit = 1`, the default; the simulator's `--acceptance arrival --strict-decrease --commit-executed`, the paper's rule). The executed goal-reaching plan is the commitment, with absolute arrival A = start + duration. While its retained tail still certifies:
- the new plan's horizon is capped at A − t_{k+1} (if fewer than two periods remain, no new plan is solved and the tail runs out);
- a certified new plan replaces the tail only if it arrives no later than A and its effort, the integral of |acceleration|² (exact per cubic piece), is at most the tail's.

Under exact tracking the drone then reaches the goal by the first promise and spends no more than the first plan's effort. Braking, a reset, or a tail that no longer certifies ends the commitment, and the next certified plan makes a new promise. `tsdp.commit = 0` restores first-certified execution: any certified plan replaces the tail, with a fresh horizon every frame (the behaviour of this app before 2026-11-11). The on-board solver is ADMM only (no IPM, no ADMM–IPM switch), so this is the commitment rule with the ADMM engine, not the full VOTUM solver.

**Setpoint task** (100 Hz) samples the active trajectory into world-frame position, velocity and acceleration setpoints, sent at commander priority 3.

**Tracker:** the stock PID controller (`stabilizer.controller = 1`), whose gains `platform_defaults_cf21bl.h` tunes for the Crazyflie 2.1 Brushless (39.3 g). It tracks position with velocity feed-forward and ignores the acceleration feed-forward, which is fine at 0.4–0.5 m/s cruise. Use Mellinger (`--controller 2`) only after tuning its gains and `massThrust` for the Brushless: its defaults are for the brushed Crazyflie.

**Obstacles** arrive over the app channel, one moving sphere per packet:
- format: `uint8 index; float center[3], velocity[3], radius`; index `0xFF` clears all;
- up to 8 obstacles: the checker covers all 8, and the SDP holds the 4 most relevant;
- centers are extrapolated from the receive time;
- obstacles silent for `tsdp.obsTimeout` (0.5 s) are dropped.

**Parameters** (`tsdp.*`): `run`, `gx`, `gy`, `gz`, `budgetMs`, `vmax`, `amax`, `cruise`, `margin`, `robotR`, `resetD`, `obsTimeout`, `commit` (1 commitment rule, 0 first certified).

**Logs** (`tsdp.*`, all on-board wall-clock):

| Group | Variables | Rate |
|---|---|---|
| Timing | `planUs` (solver), `frameUs` (whole frame: solver, checks, fallbacks), `late` (frames past the period) | per frame |
| Decision | `decision` (0 new, 1 tail, 2 brake, 3 fail), `iters`, `attempts`, `base`, `reset`, `failures`, `frames` (counter, for deduplication) | per frame |
| Certificate | `rho`: certified clearance lower bound (m) of the trajectory executed from this frame; −1 if none | per frame |
| Tracking | `spx`, `spy`, `spz` (current setpoint), `trackErr` (\|estimate − setpoint\|) | 100 Hz |
| Obstacles | `obs` (count used), `obsAgeMs` (oldest obstacle's age) | per frame |
| Health | `stackPl`, `stackSp` (free stack words, high-water mark); read-only parameter `tsdp.footprint` (planner bytes) | continuous |
| Reference | `refx`, `refy`, `refz` (state the frame planned from) | per frame |
| Commitment | `promiseMs` (committed arrival minus the switch time, ms; −1 none), `reject` (certified plan rejected: 1 later than promised, 2 costlier than the tail), `recommits` (promises made) | per frame |

## Build and flash (Crazyflie 2.1 Brushless)

```bash
cd apps/tinysdp_ct
make cf21bl_defconfig < /dev/null    # seed the Brushless config first; a plain `make` after switching platforms can hang at an interactive config prompt
make -j4 < /dev/null                 # -> build/cf21bl.bin (needs arm-none-eabi-gcc; on macOS: brew coreutils gnu-sed)
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
```

`app-config` matches the lab's working Brushless build: `CONFIG_PLATFORM_CF21BL`, DShot at 300 kHz, motor arming required. For a brushed Crazyflie 2.x:

```bash
make cf2_defconfig < /dev/null && make -j4 OOT_CONFIG=$PWD/app-config-cf2 < /dev/null    # -> cf2.bin
```

Build checked on 2026-11-11 for the Brushless with the commitment rule (macOS, arm-none-eabi-gcc 13.3), with the planner at 4 obstacles in the SDP, 8 covered by the checker, Anderson memory 0. In this repository, against the clean submodule plus the patch above, the Brushless build is 308 KB flash, 119.5 KB RAM (11.6 KB free) and 54 KB CCM. The rows below are from the paper repository's vendored firmware, which also carries the lab's power-distribution changes. The brushed row is from 2026-10-04, before the rule (not rebuilt):

| Platform | Flash | RAM | CCM |
|---|---|---|---|
| Brushless (`cf21bl.bin`), 2026-11-11 | 309 KB (30%) | 119.5 KB of 128 KB (91%), 11.5 KB free | 54 KB (85%) |
| Brushless (`cf21bl.bin`), 2026-10-04 | 304 KB (29%) | 119 KB of 128 KB (91%), 11.6 KB free | 54 KB (85%) |
| Brushed (`cf2.bin`), 2026-10-04 | 313 KB (30%) | 121 KB of 128 KB (92%), 9.9 KB free | 62 KB (95%) |

The planner object is 24.7 KB. TinyMPC is not linked.

## Test on the desktop first

```bash
apps/tinysdp_ct/test/run_sil.sh
```

This runs the exact app source with stub firmware headers, a simulated clock, perfect tracking and moving obstacles. Seven scenarios, each with the commitment rule and with first-certified execution (14 runs):

| Scenario | What it stresses |
|---|---|
| empty, static sphere, crossing sphere, three spheres | normal operation |
| 0.05 ms planning budget | planning can't finish |
| obstacle appearing mid-flight | sudden change in the world |
| 0.2 m tracking kick | the reset path |

With the rule, every run must also keep Proposition 1 under this perfect tracking: the promised arrival never moves later within a commitment, and a run with a single commitment arrives by its first promise (one period of slack) with executed effort at most the first plan's (2% slack for the 1 ms integration).

Result on 2026-11-11: all 14 pass. Arrival times (s), rule vs first certified, every rule run with its first promise at 4.04–4.08 s:

| Scenario | Rule: arrival, promises, effort / J0 | First certified: arrival, effort / J0 |
|---|---|---|
| empty | 4.00, 1, 0.81 | 4.52, 1.25 |
| static sphere | 4.00, 1, 1.00 | 4.48, 0.95 |
| crossing sphere | 4.00, 1, 0.97 | 4.60, 0.71 |
| three spheres | 4.00, 1, 0.99 | 6.16, 4.60 |
| 0.05 ms planning budget | 4.04, 1, 0.97 | 4.56, 0.78 |
| obstacle appearing mid-flight | 4.04, 3, 1.68 | 5.04, 0.85 |
| 0.2 m tracking kick | 4.40, 2, 1.15 | 4.76, 1.10 |

Effort above 1 with the rule only follows a new promise (the obstacle that appears, the kick), as Proposition 1 allows. Result on 2026-10-04 (before the rule): all 7 pass. Every scenario reaches the goal, and the minimum clearance is at least +0.050 m (the required margin). The test caught three bugs before any flight:
- unsigned time wrap-around: the first setpoint would have jumped to the goal;
- a plan-activation race: plans were dropped every other frame;
- infeasible default limits.

## Fly

```bash
python3 apps/tinysdp_ct/host/fly_tinysdp_ct.py --scenario apps/tinysdp_ct/host/scenarios/00_empty.json --dry-run
python3 apps/tinysdp_ct/host/fly_tinysdp_ct.py --scenario apps/tinysdp_ct/host/scenarios/01_static_sphere.json --bench --build-commit <sha>
python3 apps/tinysdp_ct/host/fly_tinysdp_ct.py --scenario apps/tinysdp_ct/host/scenarios/00_empty.json --commit 1 --build-commit <sha>
```

Modes:
**For a step-by-step test plan, start with `HARDWARE_TESTS.md`.**

- `--dry-run`: no takeoff, `tsdp.run` stays 0. Checks, parameters and logging only.
- `--bench` (**props off**): the motors are never armed, and `tsdp.run = 1` for `--timeout` seconds. The planner runs on board against the streamed obstacles and follows its own reference (`tsdp.resetD` is raised to 100 m for the run and restored), so planning time, stack headroom and the commitment logs are measured without flying. The run ends when the reference reaches the goal.
- default: flight.

`--commit 1` / `--commit 0` sets `tsdp.commit` (commitment rule / first certified) for rule-on vs rule-off runs; without it the firmware's value (1) is kept and read back. `--build-commit` records the source commit the firmware was built from in the meta file. Use both on every recorded run.

The script:
0. runs pre-flight checks: every log variable exists (the right firmware is flashed), battery at least `--min-vbat`, parameters read back;
1. selects PID (default) and resets the estimator;
2. **arms the motors** (the Brushless requires it), then takes off to `start` with the high-level commander;
3. streams obstacles at 20 Hz;
4. sets `tsdp.run = 1`;
5. logs five blocks at 100 Hz (state, track, frame, health, commit) into one CSV row per state sample, plus the true positions of the virtual obstacles at that row's host time;
6. stops at the goal or the timeout, sets `tsdp.run = 0`, lands, and disarms.

When `run` returns to 0 the app relaxes its priority, the firmware tells the high-level commander the current state, and the script sends `go_to` the current position and lands. A geofence breach or Ctrl-C releases and lands; a second Ctrl-C sends the emergency stop.

Each flight writes `flights/<scenario>_<time>.csv` and `.meta.json`: the scenario, arguments, parameters read back, firmware revision and outcome. Analyse it with:

```bash
python3 apps/tinysdp_ct/host/analyze_flight.py flights/<name>.csv --json summary.json
```

The analysis reports:
- on-board planning and frame time (median, p95, max) against the 30 ms budget and the 40 ms period, and late frames;
- the decision mix, iterations and route attempts;
- tracking error;
- certified ρ against the measured deviation δ. With virtual obstacles δ is the tracking error, so this is the hardware version of §4.5;
- true clearance to every obstacle;
- stack headroom, obstacle age and battery;
- the commitment rule (logs with `tsdp.promiseMs`): promises made, the first promise after `run`, how far the promise slid over the run (about 0 when the first promise is kept), promises moved later within a commitment (must be 0 with the rule), certified plans rejected as later than promised or costlier than the tail, and arrival minus the first promise. In the SIL logs, three spheres: rule 1 promise, slid 0.00 s, 89 plans rejected as costlier; first certified 125 promises, slid 2.28 s.

`test/run_sil.sh` runs the same analysis on the simulated flights, so the logging-to-numbers pipeline is checked before any flight.

**Bring-up order:**
1. `--dry-run`: check that params and logs arrive and obstacle packets are accepted (`tsdp.obs`).
2. `00_empty`, slow (cruise 0.4 m/s).
3. `01_static_sphere`.
4. `02_crossing_sphere`.
5. `03_three_spheres`.

Inspect `planUs`, `decision` and `late` after every flight.

## What is verified, and what is not

| Item | Status |
|---|---|
| The firmware builds; the planner and checker logic run correctly in software-in-the-loop | ✅ |
| Planning time on the STM32F405 | ❌ not measured yet. Desktop planning is about 0.1–1.5 ms mean; the MCU will be much slower. Read `tsdp.planUs` on the first flight; if frames run late (`tsdp.late`), lower `tsdp.budgetMs` or the SDP capacity |
| Tracking quality of PID on this reference with the Brushless, and the positioning system | ❌ flight-only. The planner's 4 m/s² acceleration limit is well within the Brushless's thrust margin, but check the logged tracking error first |
| Stack margin of both tasks on the MCU | ❌ logged live (`stackPl`, `stackSp`). Check the first flight's minimum: it should stay well above 0 |
| `tsdp.margin` vs real tracking error | ❌ logged (`trackErr` against `rho`). Set the margin from the first flights' tracking error: the margin is a requirement on tracking, not a guarantee |
