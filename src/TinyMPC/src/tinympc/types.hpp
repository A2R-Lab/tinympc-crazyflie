#pragma once


#include <Eigen.h>
// #include <Eigen/Core>
// #include <Eigen/LU>

using namespace Eigen;


#ifdef __cplusplus
extern "C" {
#endif

    typedef float tinytype;  // Using float for embedded (Crazyflie STM32)
    typedef Matrix<tinytype, Dynamic, Dynamic> tinyMatrix;
    typedef Matrix<tinytype, Dynamic, 1> tinyVector;

    // typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;
    // typedef Matrix<tinytype, NINPUTS, 1> tiny_VectorNu;
    // typedef Matrix<tinytype, NSTATES, NSTATES> tiny_MatrixNxNx;
    // typedef Matrix<tinytype, NSTATES, NINPUTS> tiny_MatrixNxNu;
    // typedef Matrix<tinytype, NINPUTS, NSTATES> tiny_MatrixNuNx;
    // typedef Matrix<tinytype, NINPUTS, NINPUTS> tiny_MatrixNuNu;

    // typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;       // Nu x Nh
    // typedef Matrix<tinytype, NINPUTS, NHORIZON - 1> tiny_MatrixNuNhm1; // Nu x Nh-1

    /**
     * Solution
     */
    typedef struct {
        int iter;
        int solved;
        tinyMatrix x; // nx x N
        tinyMatrix u; // nu x N-1
    } TinySolution;

    /**
     * Matrices that must be recomputed with changes in time step, rho
     */
    typedef struct {
        tinytype rho;
        tinyMatrix Kinf;       // nu x nx
        tinyMatrix Pinf;       // nx x nx
        tinyMatrix Quu_inv;    // nu x nu
        tinyMatrix AmBKt;      // nx x nx
        
        // PSD cache
        tinytype rho_psd;      // PSD penalty parameter
    } TinyCache;

    /**
     * User settings
     */
    typedef struct {
        tinytype abs_pri_tol;
        tinytype abs_dua_tol;
        int max_iter;
        int check_termination;
        int en_state_bound;
        int en_input_bound;
        
        // PSD settings
        int en_psd;            // Enable PSD constraints (0=off, 1=on)
        int nx0_psd;           // Base state dimension for PSD (e.g., 3 for x,y,z)
        int nu0_psd;           // Base input dimension for PSD (usually 0 for position-only)
    } TinySettings;

    /**
     * Problem variables
     */
    typedef struct {
        int nx; // Number of states
        int nu; // Number of control inputs
        int N;  // Number of knotpoints in the horizon

        // State and input
        tinyMatrix x;    // nx x N
        tinyMatrix u;    // nu x N-1

        // Linear control cost terms
        tinyMatrix q;    // nx x N
        tinyMatrix r;    // nu x N-1

        // Linear Riccati backward pass terms
        tinyMatrix p;    // nx x N
        tinyMatrix d;    // nu x N-1

        // Auxiliary variables
        tinyMatrix v;    // nx x N
        tinyMatrix vnew; // nx x N
        tinyMatrix z;    // nu x N-1
        tinyMatrix znew; // nu x N-1

        // Dual variables
        tinyMatrix g;    // nx x N
        tinyMatrix y;    // nu x N-1



        // Q, R, A, B given by user
        tinyVector Q;       // nx x 1
        tinyVector R;       // nu x 1
        tinyMatrix Adyn;    // nx x nx
        tinyMatrix Bdyn;    // nx x nu

        // State and input bounds
        tinyMatrix x_min;   // nx x N
        tinyMatrix x_max;   // nx x N
        tinyMatrix u_min;   // nu x N-1
        tinyMatrix u_max;   // nu x N-1

        // Reference trajectory to track for one horizon
        tinyMatrix Xref;    // nx x N
        tinyMatrix Uref;    // nu x N-1

        // Temporaries
        tinyVector Qu;      // nu x 1


        
        // Variables for keeping track of solve status
        tinytype primal_residual_state;
        tinytype primal_residual_input;
        tinytype dual_residual_state;
        tinytype dual_residual_input;
        int status;
        int iter;
        
        // ========== PSD (obstacle avoidance) variables ==========
        // Disk obstacles: each disk is (cx, cy, radius)
        // Max 4 obstacles for embedded (can increase if needed)
        tinytype psd_disks[4][3];  // [obstacle_idx][cx, cy, r]
        int psd_num_disks;         // Number of active disk obstacles
        
        // PSD slack variable S (svec form) - one per horizon step
        // For position-only (nx0=2): psd_dim = 1+2 = 3, svec_len = 6
        // For 3D position (nx0=3): psd_dim = 1+3 = 4, svec_len = 10
        tinyMatrix Spsd;           // svec_len x N
        tinyMatrix Spsd_new;       // svec_len x N
        tinyMatrix Hpsd;           // PSD dual variable, svec_len x N
        
        // PSD residuals
        tinytype primal_residual_psd;
        tinytype dual_residual_psd;
    } TinyWorkspace;

    /**
     * Main TinyMPC solver structure that holds all information.
     */
    typedef struct {
        TinySolution *solution; // Solution
        TinySettings *settings; // Problem settings
        TinyCache *cache;       // Problem cache
        TinyWorkspace *work;    // Solver workspace
    } TinySolver;

#ifdef __cplusplus
}
#endif
