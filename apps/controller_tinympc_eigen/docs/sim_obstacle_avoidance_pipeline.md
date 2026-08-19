# Simulation Obstacle-Avoidance Pipeline

This is the current clean simulation path for testing TinyMPC ADMM half-space obstacle
avoidance.

## Goal

1. Provide a reference trajectory for the Crazyflie.
2. Place obstacles along that trajectory.
3. Simulate sector-depth detections from PyBullet geometry.
4. Convert the selected sector into ADMM position half-space constraints.
5. Run the closed-loop sim.
6. Plot the actual trajectory, destination, obstacles, half-spaces, and planned ADMM path.

The simulator is self-contained in this repository. It uses the checked-in TinyMPC-ADMM
sources and firmware-generated dynamics, and does not import `tinympc-vision` or another
sibling checkout. `gym-pybullet-drones` supplies the four-motor CF2X rigid-body plant.
The adapter replaces the stock 27 g mass and inertia with the generated 45 g AI-deck +
Flow-deck model, and maps motor thrusts so the total force and roll/pitch/yaw torques agree
with the generated firmware model. The default perception path uses ground-truth ray depth with optional
noise/dropout/bias. An optional image-level path renders the onboard 160x160 grayscale
camera in PyBullet and runs a supplied GAP8-compatible ONNX model.

Install the Python dependencies with:

```bash
python3 -m pip install -r tools/pybullet_simulation/requirements-vision.txt
```

## Trajectory Format

CSV with required columns:

```csv
t,x,y,z
```

Optional columns:

```csv
vx,vy,vz
```

If velocity columns are absent, the runner estimates them from finite differences. Example:

```text
sim/trajectories/straight.csv
```

## Single Run

```bash
tools/pybullet_simulation/run_depth_constraint_sim.py \
  --out sim_runs/single \
  --trajectory-file sim/trajectories/straight.csv \
  --duration 8.0 \
  --control-mode admm \
  --obstacle center_box,1.55,0.18,1.10,0.16,0.22,0.28 \
  --overwrite
```

## Firmware-fidelity execution path

The defaults match the generated controller: PyBullet integrates the plant at
500 Hz, TinyMPC has 20 ms knots, replans at 50 Hz, and runs five ADMM iterations.
The first four TinyMPC control values are physical per-motor thrust deviations
from hover and are applied directly to the Gym plant. There is no acceleration
or geometric-PID shim in `--control-mode admm`.

```bash
tools/pybullet_simulation/run_depth_constraint_sim.py \
  --out sim_runs/firmware_fidelity \
  --duration 8 \
  --plant-dt 0.002 \
  --model-dt 0.02 \
  --mpc-rate-hz 50 \
  --horizon 20 \
  --start-k 0 \
  --constraint-end-k 20 \
  --camera-rate-hz 20 \
  --control-mode admm \
  --obstacle center_box,1.55,0.18,1.10,0.16,0.22,0.28 \
  --overwrite
```

The host ABI compiles the repository's TinyMPC-ADMM C++ sources and loads the
same generated 20 ms `A/B/Q/R`, costs, hover thrust, bounds, horizon, and ADMM
settings used by the firmware. For a model interval that is an integer multiple
of 20 ms it composes the discrete transition, then recomputes `Pinf`, `Kinf`,
and the ADMM primal cache. It is host-simulation-only and does not change
firmware parameters.

The Crazyflie project's stock Python firmware binding (`cffirmware`, also
packaged by some simulators as `pycffirmware`) does not contain this out-of-tree
TinyMPC app controller. Consequently, routing control through that stock binding
would silently run PID or Mellinger instead. This simulator instead calls the
actual checked-in TinyMPC solver through its small host ABI; Gym handles only
the plant. This is the closest executable boundary without adding the complete
app and its FreeRTOS task dependencies to the upstream SWIG binding.

Outputs:

- `summary.json`: run result and obstacle/goal metadata.
- `closed_loop.csv`: state, commands, active constraint, clearance, ADMM terminal point,
  physical motor thrusts, and Gym motor RPMs.
- `constraints.csv`: horizon half-space rows.
- `planned_horizon.csv`: full ADMM planned horizon for each control step.
- `neural_perception.csv` (neural mode): raw ONNX clearances and confidence
  outputs for all four directions at each camera frame, plus danger flags and
  whether the planar policy activated.

Plot:

```bash
tools/pybullet_simulation/plot_depth_constraint_run.py sim_runs/single
```

This writes:

```text
sim_runs/single/trajectory_topdown.png
```

## Neural camera mode

The neural adapter implements the deployed contract directly: a 160x160
monochrome sensor frame, center crop to 120x160, NCHW input, and a
`1x12x15x20` quantized output. The model and its adjacent
`quantization_manifest.json` are runtime artifacts, so a newly trained model
can be tested without changing simulator code:

```bash
tools/pybullet_simulation/run_depth_constraint_sim.py \
  --out sim_runs/neural \
  --perception-mode neural \
  --onnx-model path/to/sequential_int.onnx \
  --control-mode admm \
  --plot \
  --overwrite
```

The simple neural policy declares each output at -40, -13.3, +13.3, and +40
degrees dangerous when its clearance is below `--neural-danger-clearance`
(0.25 m by default). Confidence does not gate this experimental policy.
When at least two regions are dangerous it places one plane normal to the
drone's forward axis. The boundary uses the closest reported clearance minus
`--neural-plane-margin` (0.10 m by default). Otherwise no vision constraint is
sent to TinyMPC.
This provides an end-to-end integration point, but model accuracy on the simple
PyBullet renderer must be validated before treating the result as flight evidence.
The plot distinguishes the reference path, flown path, ground-truth obstacle
footprints, neural detection points, active constraint planes, and MPC horizons.

## Neural clearance benchmark

The standalone benchmark moves the rendered camera through straight, lateral,
diagonal, slalom, and altitude-changing trajectories against walls, thin posts,
blocks, bars, and gate frames. It compares all four ONNX outputs with geometric
first-surface distance along the model's fixed directions (-40, -13.3, +13.3,
and +40 degrees):

```bash
python3 tools/pybullet_simulation/benchmark_neural_clearance.py \
  --onnx-model ../../../tinympc-perception/releases/gap8-sequential-bothflights-qat-dory-v1/sequential_int.onnx \
  --out sim_runs/neural_clearance_benchmark \
  --frames-per-case 80 \
  --danger-threshold 0.25 \
  --overwrite
```

It writes `samples.csv`, `report.json`, `clearance_accuracy.png`, and
`confidence_reliability.png`. The report includes distance errors, per-scene
and per-direction results, a prediction threshold sweep at a fixed geometric
danger definition, confidence/error correlation, confidence ranking AUCs,
decile bins, and selective-coverage metrics. Confidence values are raw logical
scores rather than calibrated probabilities. These are synthetic
domain-transfer results, not a substitute for scoring synchronized real HM01B0
frames against measured clearance; poor results can come from the network, the
renderer-to-camera domain gap, or both.

## Batch Run

```bash
tools/pybullet_simulation/batch_depth_constraint_sim.py \
  --out sim_runs/batch \
  --cases 12 \
  --trajectory-file sim/trajectories/straight.csv \
  --include-baseline \
  --control-mode admm \
  --overwrite
```

Plot every case:

```bash
tools/pybullet_simulation/plot_depth_constraint_run.py sim_runs/batch
```

Find crashed avoidance runs:

```bash
python3 - <<'PY'
import csv
with open("sim_runs/batch/batch_results.csv") as f:
    for row in csv.DictReader(f):
        if row["variant"] == "avoid" and row["collision"] == "True":
            print(row["run_dir"])
PY
```

## Geometry Note

The half-space is currently generated from the selected sector ray:

```text
a = sector ray direction in world frame
p_obst = p_drone + depth * a
b = a^T p_obst - margin
constraint: a^T p <= b
```

Obstacles are PyBullet boxes, and depth is computed by ray-AABB intersection. The plane is
not a true tangent plane to a cylinder or box face; it is a conservative plane placed before
the measured obstacle point along the selected ray.
