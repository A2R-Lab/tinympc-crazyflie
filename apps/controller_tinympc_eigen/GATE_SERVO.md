# Gate-assisted straight flight

The gate/corner partitions already run in ESPNet. The new GAP8 sender exports them to STM32 alongside the unchanged collision packets. Both firmware images must be updated for gate navigation.

The normal leg remains 2 m/s on the original heading. Three fresh detections of a rail with a corresponding labeled corner and image-edge support acquire a gate. STM32 first commands zero velocity to settle, then visually centers with a 0.5 m/s maximum translation command. A lone left rail causes a 0.25 m/s search to the right; a lone right rail causes a search to the left. No forward translation is requested during alignment.

Centering requires both rail probabilities >=0.7, at least one image-supported endpoint on each rail, and four confident, consistently ordered corners. Corner quality is spatial-softmax peak mass >=0.02 (uniform mass is 0.0025), not a calibrated detection probability. The image check searches within four pixels of the predicted endpoint for a vertical edge extending into the rail: contrast >=18 on 12 of 16 rows, with slope allowance. These are initial engineering thresholds requiring camera verification, not trained rail segmentation.

Three distinct centered frames (image error less than 5% in both axes), measured speed below 0.15 m/s, roll/pitch within five degrees, and yaw within five degrees of the original heading start PASS. PASS commands 0.5 m/s along that heading until measured displacement reaches one meter. It then starts a new 2 m/s straight leg at the measured exit position, retaining the original heading. It does not return to the original lateral path. The same gate cannot retrigger until fresh observations show no matched rail for one second.

The meter is measured from the aligned starting position, **not from an estimated gate plane**. There is no metric depth estimate, aperture-size clearance check, or guarantee that a distant gate lies inside that one-meter segment. A single visible rail is insufficient to infer the gate center; it can only initiate the lateral search. Centering is image-based and assumes the standard forward-facing, level AI-deck mount.

Obstacle braking retains priority: center danger strictly >0.95, stale/invalid collision input, manual cancellation, or the overall 15-second mission timeout terminate the run. Alignment aborts on stale gate input (>400 ms), ten-second timeout, displacement over 1.5 m from acquisition, or height outside 0.2–1.5 m. Passage allows the gate to leave the camera but retains collision protection and a ten-second passage timeout. Abort leads to the normal zero-velocity brake; the flight script then lands. These limits do not guarantee physical clearance or exact stopping distance.

## Verification logs

Danger braking requires one fresh packet with center >0.95. Once braking settles, three distinct consecutive fresh packets with center <=0.95 resume a new 2 m/s leg from the stopped position. Dangerous, stale, skipped, or reordered packets reset the clear count. The start rejection remains conservative. `espTest.brakeFrames` and `espTest.clearFrames` expose the trigger and resume state in test.csv.

Console prints `GATE SEARCH`, `SLOW`, `ALIGN`, `PASS`, `RESUME`, or `ABORT` on transitions and once per second. `rails` is the verified rail bitmask (1 left, 2 right, 3 both); `center` means a valid center estimate; `err` is normalized image error in thousandths; `distance` is measured passage travel; `age` is gate packet receive age. Abort `why`: 1 invalid state/timing, 2 timeout, 3 stale gate, 4 search/height limit. Raw rail, corner, edge mask and packet health appear in the `gate` log group; control phase, errors and commands appear in `gateNav`.

From this app directory, a **passive recording** uses:

```sh
/Users/char_chen/miniconda3/bin/python tools/run_brake_csv.py --gate-logs --seconds 20
```

The existing explicit `--fly --height 0.5` options launch the mission. With `--gate-logs`, the runner also requires fresh gate telemetry before arming. Extra CSVs (`gate_nav`, `gate_seen`, `gate_top`, `gate_bottom`) record at 10 Hz; existing flight groups remain 20 Hz. Corner coordinates are normalized [0,1], in LT/RT/LB/RB order.

## Validation

Host tests cover packet CRC/ranges/version/duplicates, collision coexistence, edge support, heatmap decoding, rail/corner association, steering signs, unique-frame confirmation, stale/invalid inputs, timeout, measured one-meter completion and retrigger inhibition. STM32 and GAP8 builds are required. These checks do not constitute a gate flight test.
