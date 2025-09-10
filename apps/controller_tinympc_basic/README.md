# TinyMPC Basic Controller

This is a bare bones implementation of a TinyMPC controller that uses the **old TinyMPC API** with the **new Crazyflie firmware base**.

## What We've Accomplished

### 1. Successful Integration
- ✅ Old TinyMPC API (from `Ishaan/tinympc-crazyflie-firmware`) working with new CF firmware
- ✅ Proper out-of-tree controller structure 
- ✅ Clean build without errors
- ✅ All necessary dependencies copied and configured

### 2. Structure
```
controller_tinympc_basic/
├── Makefile                    # Build configuration with correct paths
├── app-config                  # Firmware configuration (enables controller)
├── Kbuild                      # Build system file
├── src/
│   ├── Kbuild                  # Source build configuration
│   ├── controller_tinympc_basic.cpp  # Main controller implementation
│   ├── cpp_compat.h            # C++ compatibility header
│   ├── quadrotor_50hz_params_*.hpp   # TinyMPC parameter files
│   └── quadrotor_50hz_line_9s_xyz.hpp # Trajectory file
└── TinyMPC/                    # Old TinyMPC library (compatible API)
    ├── include/Eigen/          # Eigen library
    └── src/tinympc/            # TinyMPC source
```

### 3. Current Status
- **Controller Mode**: Currently defaults to PID (safe fallback)
- **MPC Mode**: Basic task structure in place, ready for full TinyMPC implementation
- **Build System**: Fully working with new firmware base
- **Dependencies**: All TinyMPC dependencies properly linked

### 4. Next Steps for Full Implementation
1. Copy the actual MPC computation logic from the old implementation
2. Add proper initialization of TinyMPC problem data
3. Implement trajectory following and state estimation integration
4. Add logging and parameter interfaces (with C++ compatibility fixes)
5. Test on hardware

## Building
```bash
cd /home/moises/Documents/A2R/firmware/tinympc-crazyflie/apps/controller_tinympc_basic
make
```

## Key Differences from Original
- **Firmware Base**: Uses new Crazyflie firmware (better organized, more features)
- **Structure**: Out-of-tree controller pattern (cleaner separation)
- **Build System**: Updated build configuration for new firmware
- **Compatibility**: Maintains old TinyMPC API for easy migration

This serves as a solid foundation for migrating the full TinyMPC implementation to the newer Crazyflie firmware base while preserving the working TinyMPC controller logic.

## Parameter Files Explanation

All necessary parameter files have been copied from the original implementation. Here's what each file contains:

### 🎯 **MPC Controller Parameters**
- **`quadrotor_50hz_params_unconstrained.hpp`** - Main MPC parameters for unconstrained optimization (ρ=5.0)
- **`quadrotor_50hz_params_constrained.hpp`** - MPC parameters for constrained optimization (ρ=63.0)
- **`quadrotor_50hz_params.hpp`** - Base 50Hz parameters
- **`quadrotor_50hz_params_2.hpp`** - Alternative parameter set (ρ=5, passive)
- **`quadrotor_50hz_params_3.hpp`** - Alternative parameter set (ρ=5, aggressive)

Each parameter file contains:
- **`Adyn_data`** - Discrete-time system dynamics matrix (A)
- **`Bdyn_data`** - Input matrix (B) 
- **`Q_data`** - State cost weights
- **`Qf_data`** - Terminal state cost weights
- **`R_data`** - Input cost weights
- **`Kinf_data`** - Infinite horizon LQR gain matrix
- **`Pinf_data`** - Infinite horizon Riccati solution
- **`Quu_inv_data`** - Inverse of input Hessian
- **`AmBKt_data`** - Precomputed matrix (A - B*K)^T
- **`coeff_d2p_data`** - Dual-to-primal conversion coefficients
- **`rho_value`** - ADMM penalty parameter

### 🏁 **Different Control Frequencies**
- **`quadrotor_10hz_params.hpp`** - 10Hz control parameters
- **`quadrotor_20hz_params.hpp`** - 20Hz control parameters  
- **`quadrotor_25hz_params.hpp`** - 25Hz control parameters
- **`quadrotor_100hz_params.hpp`** - 100Hz control parameters
- **`quadrotor_250hz_params.hpp`** - 250Hz control parameters
- **`quadrotor_500hz_params.hpp`** - 500Hz control parameters

### 🛤️ **Trajectory Reference Files**
- **`quadrotor_50hz_line_9s_xyz.hpp`** - 9-second straight line trajectory (X,Y,Z positions)
- **`quadrotor_50hz_line_5s.hpp`** - 5-second line trajectory (full state)
- **`quadrotor_50hz_line_8s.hpp`** - 8-second line trajectory (full state)  
- **`quadrotor_50hz_line_10s.hpp`** - 10-second line trajectory (full state)
- **`quadrotor_50hz_line_10s_xyz.hpp`** - 10-second line trajectory (X,Y,Z only)
- **`quadrotor_50hz_ref_circle.hpp`** - Circular trajectory reference
- **`quadrotor_50hz_ref_circle_2_5s.hpp`** - 2.5-second circular trajectory
- **`quadrotor_100hz_ref_hover.hpp`** - Hover reference at 100Hz
- **`quadrotor_20hz_ref_hover.hpp`** - Hover reference at 20Hz

### 🔧 **How These Files Work**

1. **MPC Parameters**: Generated offline using the TinyMPC toolchain, containing pre-solved optimization matrices for different scenarios (constrained vs unconstrained)

2. **Frequency Selection**: Different control rates require different discretization time steps, hence separate parameter files

3. **Trajectory Format**: Reference trajectories contain waypoints for the desired path:
   - Position coordinates (x, y, z) over time
   - Some include full state vectors (position, velocity, orientation)
   - Time-indexed for real-time trajectory following

4. **Usage Pattern**: The controller selects which parameter set and trajectory to use based on:
   - Desired control frequency (50Hz is most common)
   - Whether constraints are active (obstacle avoidance, input limits)
   - Mission type (hover, line, circle trajectories)

### 📁 **File Verification**
✅ All 20 parameter/trajectory files successfully copied  
✅ Build system verified working with all files  
✅ Ready for full TinyMPC implementation
