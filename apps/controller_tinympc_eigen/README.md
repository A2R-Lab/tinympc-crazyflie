# TinyMPC-ADMM Controller for Crazyflie

This is a TinyMPC-ADMM based controller for the Crazyflie, adapted for the latest firmware API. It's based on the original implementation by Ishaan's team but updated to work with the new crazyflie firmware structure.

## Features

- Uses TinyMPC-ADMM for Model Predictive Control
- Supports trajectory tracking and setpoint following
- Eigen-based matrix operations for efficient computation
- Configurable solver parameters and trajectory options

## Key Changes from Original Implementation

1. Updated controller function signature to use `stabilizerStep_t` instead of `uint32_t tick`
2. Adapted for new firmware build system and structure
3. Maintained compatibility with TinyMPC-ADMM library
4. Added proper includes for new firmware API (`stabilizer_types.h`)
5. Added code generation to generate a custom TinyMPC solver for the Crazyflie
6. Added APIs to interface with TinyMPC library

## Build Instructions

### Code Generation

1. Navigate to this directory
2. Run `make codegen` to generate the solver.

### Building the Firmware

1. Run `make controller` to build the controller
2. Flash the resulting firmware to your Crazyflie

## Configuration

- Trajectory parameters and model matrices are defined in header files (e.g., `params_500hz.h`, `traj_fig8_12.h`)
- Controller parameters can be adjusted in the main controller file
- MPC solver settings (iterations, tolerances) are configurable

## Logging

The controller provides several logging variables for monitoring:
- `ctrlMPC.iters`: Number of solver iterations
- `ctrlMPC.mpcTime`: Solver execution time in microseconds
- `ctrlMPC.primal_residual`: Primal residual from ADMM solver
- `ctrlMPC.dual_residual`: Dual residual from ADMM solver
- `ctrlMPC.ref_x/y/z`: Reference trajectory coordinates

## Notes

- The controller runs at 500Hz by default
- Trajectory execution can be enabled/disabled via `en_traj` parameter
- Hover values are preconfigured for specific Crazyflie units and may need adjustment
