# Handheld danger-network capture and replay

This test isolates the camera and deployed danger model from flight control.
The Crazyflie remains disarmed while the AI-deck is carried by hand along an
obstacle-approach trajectory.

## 1. Flash the camera-only GAP8 streamer

The normal inference image does not stream camera frames. From
`apps/controller_tinympc_eigen`, build and flash the dedicated streamer in the
sibling checkout:

```bash
cd ../../../tinympc-nanocockpit/src/gap
./gap8.sh examples/pulp-frontnet clean all STDC_STREAM_ONLY=1
```

This requires the AI-deck JTAG connection. The NINA must be running the
NanoCockpit CPX bridge. Connect the laptop to the AI-deck Wi-Fi network after
flashing.

## 2. Record a handheld trajectory

From `apps/controller_tinympc_eigen`:

```bash
python3 tools/hardware/capture_handheld_danger_frames.py \
  --duration-s 20 \
  --out capture_runs/handheld_head_on_01
```

The recorder does not open CrazyRadio, arm, or send any flight commands. Start
in open space, approach the obstacle head-on, translate laterally past its
edge, and retreat. Move at roughly the intended flight speed. With no
`--duration-s` or `--frames`, stop using Ctrl-C.

## 3. Replay the deployed integer model

```bash
python3 tools/offline_replay_handheld_danger.py \
  capture_runs/handheld_head_on_01 \
  --danger-threshold 0.60
```

Outputs are written under `capture_runs/handheld_head_on_01/danger_replay`:

- `danger.csv`: continuous probability, integer output, obstacle-cell count,
  center-cell state, largest connected cluster, and all 8x10 map values.
- `summary.json`: sequence-level extrema and hit counts.
- `timeline.png`: obstacle-cell count, cluster size, and raw danger probability
  over the complete handheld trajectory.
- `panels/`: input, continuous danger heatmap, and the deployed binary mask.

The laptop proposal treats raw probabilities at or above 0.60 as dangerous.
This corresponds to yellow-or-hotter pixels in the Turbo heatmap. For
the stricter 0.65 experiment, pass `--danger-threshold 0.65`; its exact
quantized equivalent is `q >= 51`. The flight firmware currently uses the
original `q >= 42` cutoff. `danger.csv` reports both the raw probability
decision and the integer firmware decision.

The proposed mask is split into four-connected components. If the desired
center image cell is dangerous, its component is selected; otherwise the
largest component is shown for diagnosis. A grid-aligned bounding box encloses
that component. When the target is inside the box, the nearer viable left or
right edge is drawn as the candidate vertical constraint line. The CSV records
the box coordinates, selected side, and line coordinate.

The 8x10 result becomes rows 1 through 8 of the transmitted controller map.
Rows 0 and 9 are forced unsafe because they lie outside the network crop; they
are intentionally excluded from the laptop cell counts.

The ONNX replay executes the integer encoder and danger-head artifacts from
`../tinympc-nanocockpit/gap8_stdc_release_shared_real_v1`. It exposes the raw
probability before GAP8 thresholds it to a binary map, which is useful for
determining whether indecisive results come from the network itself or the
deployed threshold.
