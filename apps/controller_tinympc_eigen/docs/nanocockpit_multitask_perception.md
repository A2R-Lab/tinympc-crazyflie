# NanoCockpit multi-task perception integration

This integration targets this repository's `vision` branch controller
(`~/tinympc-crazyflie`) and NanoCockpit's GAP firmware
(`~/tinympc-nanocockpit`). TinyMPC constraints and safety policy live here;
NanoCockpit owns GAP8 inference and transport. It does not use the old CPX
AI-deck path.

## Runtime data path

1. NanoCockpit captures native HM01B0 160×160 monochrome frames.
2. DORY runs the packed eight-channel CNN asynchronously on seven GAP8 cluster
   workers.
3. GAP8 retains the 40×40 corner/gate tensor and 20×20 control maps.
4. The three control maps are conservatively max-pooled to 10×10 and packed as
   uint4 nominal-speed collision, inverse range, and uncertainty values. The
   packet member keeps its historical `obstacle_presence` name for ABI stability.
5. STM32 validates the 172-byte packet's header, dimensions, sequence, and
   CRC32, then publishes it through a seqlock.
6. Model-specific generated affine constants convert the transported integer
   values back to logits and probabilities.
7. The controller shifts the nominal 1.0 m/s collision logit using current
   speed, the TinyMPC horizon, perception/control latency, and map age. It
   conservatively combines this with inverse-range reachability.

Latency increases traversable exposure and reduces remaining TTC; it never
makes a stale map appear safer. The motion-conditioned output estimates
short-horizon collision probability.
It is not a generic ray/obstacle-intersection map.

## Angular constraints

The controller selects the connected safe component nearest the known gate
opening, fits its left and right image boundaries, and shifts them inward by a
range-, uncertainty-, vehicle-radius-, and latency-dependent pixel margin.
For each pixel line `l`, it computes:

```text
n_camera = normalize(K^T l)
```

The sign is chosen so the fitted gate-center ray is feasible. The current
camera attitude rotates each normal into the world frame and it is frozen for
one solve. For TinyMPC state order
`[position(3), attitude-error(3), velocity(3), angular-rate(3)]`, each selected
horizon row is:

```text
-n_world^T p_k - tau_k n_world^T v_k <= -n_world^T p_camera
```

Image constraints occupy half-space slots 1 and 2; the existing modeled
obstacle retains slot 0. The ADMM projector supports position and velocity
coefficients and a nonnegative slack with a strong configurable quadratic
penalty. Maximum slack, total slack, and slack cost are logged.

## Runtime defaults and bring-up

Perception reception, danger computation, and angular obstacle-avoidance
constraints default on:

```text
percept.enable=1
percept.constrain=1
percept.logOnly=0
```

The current RF123 PTQ package is less accurate than the floating-point teacher;
that accuracy loss has been explicitly accepted for obstacle-avoidance use.
Packet validity and age checks suppress stale or malformed maps. Corridor
confidence, conservative pixel margins, and nonnegative strongly penalized
slack keep imperfect image constraints from making TinyMPC infeasible.

For initial tethered testing, `percept.logOnly=1` remains available as a runtime
override without rebuilding. Log `percept.valid`, `percept.corridor`, normal
directions, map age, danger, and slack before untethered operation.

## Host verification and firmware build

```bash
cd ~/tinympc-crazyflie/apps/controller_tinympc_eigen
~/isaacsim-env/bin/python -m pytest -q \
  tools/flow_gap8_validation/equivalence/test_perception_*.py \
  tools/flow_gap8_validation/equivalence/test_gate_pnp_multitask_abi.py

apptainer exec --bind ~/tinympc-crazyflie:~/tinympc-crazyflie \
  --pwd ~/tinympc-crazyflie/apps/controller_tinympc_eigen \
  ~/containers/bitcraze-builder.sif \
  bash -lc 'make clean && make -j8'
```

The generated firmware artifacts are in `build/`. Physical flashing and
closed-loop constraint enabling require the actual Crazyflie/AI-deck hardware.
