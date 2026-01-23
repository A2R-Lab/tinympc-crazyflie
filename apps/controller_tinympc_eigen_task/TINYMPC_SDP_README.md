# TinyMPC-SDP Crazyflie Implementation

## Overview

This implementation brings TinyMPC-SDP (Semidefinite Programming) obstacle avoidance to the Bitcraze Crazyflie 2.1 drone. It demonstrates real-time collision avoidance using a hybrid approach combining Linear Time-Varying (LTV) constraints with Positive Semidefinite (PSD) relaxation.

## System Configuration

| Parameter | Value |
|-----------|-------|
| Platform | Crazyflie 2.1 Brushless |
| MPC Rate | 25 Hz (40ms period) |
| State Dimension | 12 (position, orientation, velocities) |
| Input Dimension | 4 (motor thrusts) |
| Horizon | 20 steps |
| ADMM Iterations | 5 per solve |
| Solve Time | ~15ms |

## Obstacle Avoidance Architecture

### 1. LTV Linear Constraints
Half-space constraint on base state positions:
```
a^T * [x, y, z] <= b
```
Where `a` is the normal vector pointing from obstacle center to predicted position. Activates when drone approaches the obstacle within activation margin.

### 2. PSD Block (3×3)
Lifts 2D position to a symmetric matrix:
```
M = [1; x; y] * [1; x; y]^T = | 1    x    y   |
                              | x    x²   xy  |
                              | y    xy   y²  |
```
This lifted representation enables encoding quadratic collision constraints as linear constraints in the lifted space.

### 3. Lifted Disk Constraint
The obstacle avoidance constraint in lifted space:
```
M(1,1) + M(2,2) - 2·ox·M(0,1) - 2·oy·M(0,2) >= r² - ox² - oy²
```
Which encodes: `||[x,y] - [ox,oy]||² >= r²` (stay outside disk)

### 4. Cardano Eigenvalue Projection
Projects the 3×3 matrix onto the PSD cone using analytical Cardano formula:
- Computes eigenvalues without iterative solver
- Shifts minimum eigenvalue to ensure all eigenvalues ≥ 0
- Runs every 5 ADMM iterations (once per solve)

### 5. Lifted Gradient Push
When near or violating the constraint, adds gradient to the cost:
```
q(x) -= rho_psd * (x - ox)
q(y) -= rho_psd * (y - oy)
```

## Comparison with Reference Implementation

### Reference: `tiny_psd_dynamic_demo.cpp`

The desktop TinyMPC-SDP demo uses:
- **Full lifted dynamics**: 4D base → 20D lifted (includes all outer products)
- **Lifted A, B matrices**: Dynamics operate on lifted state
- **Linear constraints on lifted state**: `tiny_set_lifted_disks()` creates linear inequalities
- **Certificate verification**: Computes rank-1 gap and lifted margin
- **Planner/Tracker architecture**: Separate PSD planner and LQR tracker

### This Implementation: Crazyflie Embedded

| Aspect | Desktop Demo | Crazyflie |
|--------|-------------|-----------|
| State Lifting | Full 4D → 20D | Partial: 12D base + 3×3 block |
| Dynamics | Lifted (A_lift, B_lift) | Base quadrotor dynamics |
| Obstacle Constraint | Linear on lifted state | Hybrid: LTV + PSD block |
| Eigensolve | Eigen library (full) | Cardano formula (eigenvalues only) |
| Certificate | Rank-1 gap + margin | Not implemented |
| Environment | Desktop simulation | Embedded real-time (STM32F4) |

### Why the Hybrid Approach?

1. **Memory constraints**: Full 20D lifted state would require 20×20 matrices
2. **Compute constraints**: Full eigendecomposition too expensive at 25Hz
3. **Simplicity**: Base dynamics already validated for quadrotor control
4. **Effectiveness**: LTV provides primary avoidance, PSD adds SDP consistency

## File Structure

```
controller_tinympc_eigen_task/
├── src/
│   └── controller_tinympc.cpp      # Main controller, obstacle setup
├── TinyMPC/
│   └── src/tinympc/
│       ├── types.hpp               # PSD data structures (Spsd, Hpsd, etc.)
│       ├── psd_support.hpp         # Cardano eigensolve, helpers
│       ├── admm.hpp                # ADMM function declarations
│       └── admm.cpp                # PSD slack/dual updates, lifted gradient
└── TINYMPC_SDP_README.md           # This file
```

## Key Data Structures

### In `types.hpp`:
```cpp
// PSD types for 2D position lifting (psd_dim=3: [1, x, y])
#define PSD_DIM 3
#define PSD_SVEC_SIZE 6  // 3*(3+1)/2

struct tiny_problem {
    // ... existing fields ...
    
    // PSD variables
    int en_psd;                      // Enable flag
    tiny_MatrixSvecNh Spsd;          // Slack variables (svec form)
    tiny_MatrixSvecNh Spsd_new;      // Updated slack
    tiny_MatrixSvecNh Hpsd;          // Dual variables
    
    // Obstacle parameters
    tinytype psd_obs_x, psd_obs_y, psd_obs_r;
};
```

## Key Functions

### `project_psd_3x3(M)` in `psd_support.hpp`
```cpp
// Compute eigenvalues using Cardano's formula (analytical, no iteration)
eigenvalues_3x3_sym(M, eig);

// Shift minimum eigenvalue to make PSD
if (min_eig < 0) {
    M(0,0) += (-min_eig + eps);
    M(1,1) += (-min_eig + eps);
    M(2,2) += (-min_eig + eps);
}
```

### `update_psd_slack()` in `admm.cpp`
1. Assemble lifted block from current position
2. Add dual variable: `Raw = M + H`
3. Project onto PSD cone
4. Enforce lifted disk constraint (if violated, push xx, yy outward)

### `update_psd_dual()` in `admm.cpp`
```cpp
// Augmented Lagrangian update with under-relaxation
H = H + gamma_psd * (M - Snew);
```

### `update_linear_cost()` PSD section in `admm.cpp`
1. Compute pullback gradient from lifted block to base positions
2. Add lifted obstacle gradient when constraint is active

## Tuning Parameters

| Parameter | Value | Location | Description |
|-----------|-------|----------|-------------|
| `rho_psd` | 10.0 | controller_tinympc.cpp | PSD penalty weight |
| `gamma_psd` | 0.2 | admm.cpp | Dual update relaxation |
| `r_obs` | 0.35m | controller_tinympc.cpp | Obstacle radius |
| `obs_activation_margin` | 0.15m | controller_tinympc.cpp | LTV activation distance |
| `obs_center` | (0.7, 0.1, 0.5) | controller_tinympc.cpp | Obstacle position |

## Demo Maneuver

1. **Takeoff**: Drone hovers at 0.5m using PID
2. **Switch to MPC**: User sets controller to OOT (Out-Of-Tree) #6
3. **Straight-line trajectory**: Fly from (0,0,0.5) toward (1,0,0.5)
4. **Obstacle avoidance**: Swerve around obstacle at (0.7, 0.1, 0.5)
5. **Goal reached**: Continue to x=1.0m
6. **Land**: Kill motors after trajectory completes

## Limitations

1. **Single static obstacle**: Currently only supports one disk obstacle
2. **2D avoidance**: Only x-y plane, z is not lifted
3. **No certificate**: Rank-1 gap not computed (computational constraint)
4. **Approximate projection**: Eigenvalue shift instead of full reconstruction

## Future Work

1. **Full lifted dynamics**: Implement proper 20D state with lifted A, B
2. **Certificate computation**: Add rank-1 gap verification for provable safety
3. **Dynamic obstacles**: Extend to moving obstacle prediction
4. **Multiple obstacles**: Support multiple disk constraints
5. **3D lifting**: Include z in the PSD block for full 3D avoidance

## References

- TinyMPC: https://tinympc.org
- TinyMPC-SDP: Semidefinite Relaxation for Collision Avoidance in Model Predictive Control
- Crazyflie Firmware: https://github.com/bitcraze/crazyflie-firmware
