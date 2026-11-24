# Hardware tests: step by step

What we need from the drone, in order of risk. Each step says what to run, what to check, and what to send back. Run them
in order and stop at the first step that fails its check. Paths are from the root of this repository (branch `sdp-ct`).

## Before anything

- A positioning system (Lighthouse or motion capture) so `stateEstimate` is in a fixed world frame. The scenarios' start
  and goal are in that frame: start (0, 0, 0.5) m, goals 1.5 to 2.5 m along +x at 0.5 m height. Clear about 3 m x 3 m.
- Python with `cflib`, a Crazyradio, a charged battery (the script refuses to fly below 3.6 V; start above 3.9 V).
- A net or cage for the flight steps, and one person on Ctrl-C (once: stop, release and land; twice: motor stop).

## Step 0: build, flash, record the version

```bash
git rev-parse --short HEAD                     # write this down: it goes into every run as --build-commit
git -C crazyflie-firmware apply ../apps/tinysdp_ct/firmware-ldflags.patch   # once per checkout (see README)
cd apps/tinysdp_ct
make cf21bl_defconfig < /dev/null && make -j4 < /dev/null
CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
```

Then a dry run (no takeoff, the planner stays off):

```bash
python3 apps/tinysdp_ct/host/fly_tinysdp_ct.py --scenario apps/tinysdp_ct/host/scenarios/00_empty.json --dry-run --build-commit <sha>
```

**Check:** it does not abort with "firmware lacks log variables" (that means an older build without the commitment rule),
and the new `flights/*_dry.meta.json` shows `"tsdp.commit": "1"` under `params`.

## Step 1: bench, propellers OFF

**Remove the propellers.** The motors are never armed in this mode, but take them off anyway. The planner runs on board
against virtual obstacles and follows its own reference, so this measures planning time, stack headroom and the
commitment rule without any flight.

```bash
for c in 1 0; do for r in 1 2 3; do
  python3 apps/tinysdp_ct/host/fly_tinysdp_ct.py --scenario apps/tinysdp_ct/host/scenarios/03_three_spheres.json --bench --commit $c --build-commit <sha>
done; done
for f in flights/*_bench_*.csv; do python3 apps/tinysdp_ct/host/analyze_flight.py "$f" --json "${f%.csv}.summary.json"; done
```

**Check** (from the analysis printout):
- `frame (all)` p95 below 30 ms and `late` 0. This is the on-board planning time against the budget, the main number we
  are missing.
- `stack free` for the planner and setpoint tasks above 0 (report the values: they are the headroom under load).
- With `--commit 1`: `moved later within a commitment 0` and `slid` close to 0 s. With `--commit 0`: `slid` clearly above
  0 (the promise slides without the rule).

## Step 2: short flight, no obstacles, rule on and off

Takes off to 0.5 m, hovers at the start, then the planner flies 1.5 m to the goal and the script lands.

```bash
for c in 1 0; do for r in 1 2 3; do
  python3 apps/tinysdp_ct/host/fly_tinysdp_ct.py --scenario apps/tinysdp_ct/host/scenarios/00_empty.json --commit $c --build-commit <sha>
done; done
```

**Check:** outcome `reached` in every run; `tracking error` small (p95 a few cm); with `--commit 1`, `arrival - first
promise` at most about 0.1 s (the host clock adds some jitter) and `slid` close to 0.

## Step 3: virtual obstacles

The obstacles exist only on the radio link, so nothing physical is in the way. Same pattern, one scenario at a time,
stopping if any run fails its check:

```bash
for s in 01_static_sphere 02_crossing_sphere 03_three_spheres; do for c in 1 0; do for r in 1 2 3; do
  python3 apps/tinysdp_ct/host/fly_tinysdp_ct.py --scenario apps/tinysdp_ct/host/scenarios/$s.json --commit $c --build-commit <sha>
done; done; done
```

**Check:** `true clearance` min above 0 and `samples in contact 0` (the drone never entered a virtual obstacle);
`delta<=rho` fraction 1.0, or close to it (the tracking error stayed within the certified margin: this is what the
safety certificate promises); outcome `reached`.

## What to send back

The whole `flights/` folder: every `*.csv`, `*.meta.json` and `*.summary.json`, including failed or aborted runs, plus one
line per session with the date, the drone, the positioning system and anything unusual. They are archived in the paper
repository under `apps/tinysdp_ct/results/<date>_<step>/`.

## Settings already in the scenarios

Cruise 0.4 to 0.5 m/s, speed limit 1.0 to 1.5 m/s, acceleration limit 3 to 4 m/s^2, certified margin 5 cm, robot radius
8 cm, planning budget 30 ms per 40 ms period. Do not change them between rule-on and rule-off runs.
