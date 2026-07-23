# AI-deck optical-flow sim-to-real model

The source-matched path is based on `cookacola/tinympc-nanocockpit`, branch
`gate8-dory-package`, plus the STM32 implementation in this repository.

## Deployed constraints represented

- HM01B0 half-resolution capture, cropped to 160x160, configured at 30 FPS.
- Manual 10 ms integration and fixed gain from the producer configuration.
- One camera owner alternating flow and DORY/CNN frames at 15 Hz each. A flow
  overrun drops the next CNN frame rather than delaying safety sensing.
- GAP8 Shi-Tomasi scan, 48-feature capacity, two-level pyramidal LK, 7x7
  windows, three iterations, error/flow thresholds, arithmetic sector means,
  alpha=0.35 smoothing, and three-update missing-sector holds.
- Hard-coded producer intrinsics fx=89.15584 px and cx=81.10381 px.
- Nine float32 sectors in the exact 168-byte packed message with CRC32.
- Shared USART3 at 115200 baud with a single non-overlapping DMA owner:
  14.583 ms per flow packet and 27.6% combined utilization at 15+15 Hz.
- GAP8 DMA transmits from a stable copy rather than the inference publication
  buffer. Flow and gate packets carry producer sequences; STM32 rejects
  consecutive duplicates and CRC-valid non-finite/out-of-range payloads.
- The STM32 ISR queue holds 512 bytes in ordinary SRAM—more than two complete
  212-byte gate+flow bursts and 44.4 ms of wire traffic. Queue drops, parser
  stack high-water mark, CRC failures, invalid payloads, and duplicates are
  exposed as logs.
- Because the stock CPX AI-deck driver is disabled, the custom RX task now owns
  the AI-deck IO4 reset pulse. USART3 and its queue are initialized before GAP8
  is released, giving deterministic DORY boot/framing without a second UART
  consumer. If a previously working link produces no valid CRC-checked packet
  for two seconds, the RX owner resets the AI-deck; initial boot gets a
  ten-second grace period. Reset count is logged.
- STM32 controller repeatedly reading a 15 Hz payload; only new samples
  vote in the persistent map.
- 500 ms stale-payload cutoff, candidate and map gates, world/body transforms.

## Producer implementation

The implementation is applied to `src/gap/examples/pulp-frontnet/main.c` and
`config.h` in the `gate8-dory-package` checkout. The patch files in this
directory document the initial flow changes; the checkout additionally contains
the final alternating scheduler and single UART service.

The racing extension uses balanced per-sector features, a
foreground-biased robust sector statistic, and puts radial image expansion in
the existing `flow_y_rad_s` field. STM32 uses this statistic for forward-motion
depth when lateral parallax is unobservable and yaw rate is below 0.2 rad/s.

## Verified resource envelope

The actual combined GAP8 source builds with DORY in the official GAP SDK
container. Exact replay of the generated DORY allocator requires 153600 bytes,
so the arena is 160000 bytes rather than the previous 380000. The successful
link reports 143368 bytes of static L2; adding the arena and two 162x161 camera
buffers gives an estimated 355532-byte runtime peak and 168756-byte headroom.

The STM32 firmware also builds cleanly: flash 335232/1032192 (32%), RAM
108428/131072 (83%), and CCM 62248/65536 (95%). CCM is the tight resource and
new controller-side state should not be added there without relocating data.

The 66.7 ms execution-slot check currently uses conservative 50 ms assumptions
for both workloads. It is a static safety audit, not measured GAP8 timing; DORY
deployment does not depend on the unavailable hosted GreenWaves SDK.
An offline GVSOC launch was also attempted with both the default camera model
and a local 162x162 grayscale PGM stream. The SDK camera plugin aborted in
ImageMagick before firmware execution in both cases, so it supplied no valid
cycle measurement. Do not reinterpret the 50 ms audit inputs as measurements.

## Coordinates and signs

- Camera/body `x` is forward and body `y` is left.
- Positive sector azimuth is left; the transmitted azimuth field is normalized
  pinhole coordinate `q=(u-cx)/fx`, converted on STM32 with `atan(q)`.
- Positive yaw rate is counter-clockwise/left. STM32 removes it from measured
  angular flow before dividing by effective translational velocity.
- Horizontal inverse depth uses
  `(bearing_rate-yaw_rate)/(vx*sin(az)-vy*cos(az))`.
- Near the centerline, radial expansion is positive for forward approach and
  inverse depth is `radial_expansion/body_vx`.
- World velocity is rotated into the full body frame with `R(q)^T`; obstacle
  points are then rotated back into the horizontal world map using yaw.

## Camera liveness

The original two-buffer pipeline allowed consumer callbacks to overlap. If a
flow callback suspended while waiting for a slow CNN, the following CNN callback
could overtake it, reinitialize the shared completion event, and launch another
cluster job into the same L2 arena. Consumers are now completion-serialized
while capture and crop retain their buffering.

CNN/flow selection uses a monotonic software delivered-frame sequence instead
of HIMAX's 8-bit hardware frame-count parity. A watchdog only acts while CPI is
actually waiting for a frame: after 250 ms without capture completion, and no
more than once per 500 ms, it stops CPI, returns HM01B0 to standby, reapplies
configuration, restarts streaming, and lets the existing capture request
complete. It does not misclassify a slow consumer/CNN as a sensor failure; the
STM32's two-second whole-deck watchdog remains the outer recovery layer.

HM01B0 model ID is checked at startup, register reads are initialized, and
essential timing/configuration registers are read back after configuration.
This is necessary because GAP SDK 3.8's HIMAX register API always returns
success even if its underlying I2C operation failed. Flow packet flag bit 0
reports an in-process camera recovery while bit 1 reports an I2C/readback
failure. STM32 logs the raw flag field.

Always run `make clean` before the final GAP8 build. The SDK make dependency
graph missed a rapid `camera.h` ABI change during this audit; an incremental
link then diagnosed different `camera_t` sizes across objects. The clean build
recompiled every consumer and eliminated the mismatch.

## Remaining calibration requirements

The nominal/stress Monte Carlo profiles are hypotheses until hardware logs are
collected. Record GAP8 timestamp/dt, per-sector raw and smoothed flow/counts,
CRC/drop counters, message age, body velocity, gyro z, and measured obstacle
truth. Fit focal/extrinsic bias, LK pixel error, gyro bias, velocity bias, packet
loss, and latency distributions from those logs, then replace the profile
ranges in `sim_aideck_stm32_monte_carlo.py`.
# Crazyflow closed-loop workflow

The reproducible vehicle-dynamics run uses Crazyflow's first-principles
Crazyflie model at 500 Hz, its 100 Hz state controller, a MuJoCo collision
box, the source-matched 160x160 camera renderer, the GAP8 sector frontend,
115200-baud packet serialization, and the STM32 estimator mirror:

```sh
python3 -m venv .venv-crazyflow
.venv-crazyflow/bin/pip install -e /path/to/crazyflow
apps/controller_tinympc_eigen/tools/run_crazyflow_pipeline.sh \
  --trials 2 \
  --out apps/controller_tinympc_eigen/tools/results/crazyflow_pipeline.json
```

This is deliberately offline and opens no Crazyflie, radio, UART, or camera
device. The MuJoCo box supplies collision truth but is not exposed to the
avoidance policy.

The closed-loop policy uses an acceleration-limited launch, a dynamically
consistent emergency-braking trajectory, and a sequential quintic sidestep.
The maneuver is latched so persistent map output cannot retrigger or reverse
it. A run is only a safe pass when it clears the box, avoids ground contact,
and exits past the obstacle; detection success is reported separately.

The July 2026 validation covered five noisy seeds at each of 1, 2, and 3 m/s
(2% packet loss, pixel-noise sigma 2). The 30 Hz hybrid pipeline detected and
safely passed in 15/15 runs, with zero box or ground contacts, mean cylinder
error 0.206 m, and 1.09x real-time CPU execution. A separate stress run with
10% packet loss and pixel-noise sigma 5 safely passed 9/9 runs with mean
cylinder error 0.182 m. The deployed 5 Hz producer detected 0/15 and collided
15/15, so it remains unsuitable for this racing scenario.

The deployable alternating 15 Hz flow / 15 Hz CNN schedule was separately
tested for three noisy seeds at each of 1, 2, and 3 m/s. It detected and safely
passed 9/9, with zero contacts and mean cylinder error 0.246 m. Ten-hertz flow
missed the 3 m/s case, making 15 Hz the simulated minimum rather than a target
with generous margin.

These are simulation results, not authorization to fly. The remaining
sim-to-real gaps include real HM01B0 blur/rolling shutter and exposure
behavior, GAP8 execution contention with the gate network, calibration and
time-sync error, aerodynamic ground/wall effects, and unmodelled motor/battery
variation. World velocity is now transformed by the full measured quaternion,
removing the earlier yaw-only roll/pitch bias, but image rotation and vertical
translation compensation still need hardware-log validation.

Principal failure modes are low texture or repeated texture, saturation/motion
blur, pure rotation or insufficient translation, foreground/background mixing
inside a coarse sector, unmodelled pitch/roll image motion, stale pose/velocity
relative to the camera timestamp, and multiple nearby obstacles merging into
the single-cylinder output. A planar textured box is the favorable case; this
is sparse obstacle ranging, not general dense monocular 3D reconstruction.

The STM32 mirror and firmware now count cluster hits only when the UART
seqlock sample changes. Reusing a 30 Hz packet in the 100 Hz controller can no
longer manufacture persistent evidence.

`sim_vision_uart_link.py` exercises the exact 44/168-byte mixed framing with
CRC corruption, truncation, dropped bytes/packets, duplicates, noise, and
bounded resynchronization. A truncated fixed-length message may consume the
immediately following packet before its CRC fails; the parser recovers on
subsequent headers and stale-data gates prevent the lost frame being reused as
fresh evidence.
