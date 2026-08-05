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

The current closed-loop sim uses ground-truth ray depth from obstacle geometry, with optional
noise/dropout/bias. It does not yet use optical-flow-estimated depth.

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

## 5 Hz multirate experiment

The following is the first experiment for the proposed slow-replanning
architecture. PyBullet integrates the plant at 500 Hz, the MPC model has 40 ms
knots, and the solver replans every 200 ms. Between replans the acceleration/PID
shim follows five consecutive predicted state targets; it does not repeatedly
command the terminal state.

```bash
tools/pybullet_simulation/run_depth_constraint_sim.py \
  --out sim_runs/multirate_5hz \
  --duration 8 \
  --plant-dt 0.002 \
  --model-dt 0.04 \
  --mpc-rate-hz 5 \
  --horizon 20 \
  --start-k 1 \
  --constraint-end-k 11 \
  --camera-rate-hz 25 \
  --target-speed 0.4 \
  --control-mode admm \
  --obstacle center_box,1.55,0.18,1.10,0.16,0.22,0.28 \
  --overwrite
```

The host solver regenerates its `A/B` at the requested model timestep from the
continuous Crazyflie hover model using RK4 plus a discrete-transition
linearization. It then recomputes `Pinf`, `Kinf`, and the ADMM primal-cache
matrices. This follows TinyMPC's documented model workflow: discretize first,
then linearize the discrete transition. It is host-simulation-only; it does not
change the firmware's generated 20 ms data.

`--constraint-end-k 11` is exclusive, so with `--horizon 20` the half-space is
active at the ten future knots 1 through 10. The remainder of the horizon is intentionally
unconstrained, leaving the optimizer a route to the nominal destination.

Outputs:

- `summary.json`: run result and obstacle/goal metadata.
- `closed_loop.csv`: state, commands, active constraint, clearance, ADMM terminal point.
- `constraints.csv`: horizon half-space rows.
- `planned_horizon.csv`: full ADMM planned horizon for each control step.

Plot:

```bash
tools/pybullet_simulation/plot_depth_constraint_run.py sim_runs/single
```

This writes:

```text
sim_runs/single/trajectory_topdown.png
```

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
