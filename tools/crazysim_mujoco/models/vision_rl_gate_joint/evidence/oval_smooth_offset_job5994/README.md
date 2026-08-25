# Softened oval with offset gate (Slurm 5994)

This diagnostic changes only the oval/course geometry. TinyMPC Q/R, cached
banking matrices, controller source, and the opt-in joint policy remain frozen.

The oval minor radius is 0.75 m instead of 0.50 m, reducing peak curvature from
4.00 to 1.78 1/m. The gate is placed at header index 375, where curvature is
0.75 1/m, and shifted 0.20 m to the path's right. The shift is physically
represented in both the NewBee mesh and collision rails.

The calibrated run launched at 1.026 s for a 1.000 s request. It entered a
right banked dodge before reaching the gate and contacted at 6.806 s. The gate
was not crossed, so this run does not demonstrate successful visual servoing.
Slurm job 5994's nonzero status came after analysis when the old video hook
incorrectly called Docker from an Apptainer job. Job 5995 rendered the same
immutable telemetry through Apptainer; `run.sh` now reuses the selected
container engine for future video post-processing.
