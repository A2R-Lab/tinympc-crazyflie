// Codegen for Crazyflie quadrotor MPC with PSD LIFTED solver
// This generates a solver for the lifted state space required for PSD obstacle avoidance

#include <iostream>
#ifdef __MINGW32__
#include <experimental/filesystem>
namespace std_fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace std_fs = std::filesystem;
#endif

#include <tinympc/tiny_api.hpp>
#include <tinympc/codegen.hpp>
#include <tinympc/psd_support.hpp>

// Base system dimensions
#define NX0 4   // x, y, vx, vy
#define NU0 2   // ax, ay

// Lifted system dimensions for PSD
#define NXL (NX0 + NX0*NX0)  // 4 + 16 = 20
#define NUL (NU0 + NX0*NU0 + NU0*NX0 + NU0*NU0)  // 2 + 8 + 8 + 4 = 22
#define NHORIZON 25

extern "C" {

typedef Matrix<tinytype, NXL, NHORIZON, ColMajor> tiny_MatrixNxNh;
typedef Matrix<tinytype, NUL, NHORIZON-1, ColMajor> tiny_MatrixNuNhm1;

std_fs::path output_dir_relative = "tinympc_generated_code_crazyflie_psd/";

int main()
{
    TinySolver *solver;

    const tinytype dt = tinytype(0.01);
    
    // Base planar double integrator dynamics
    tinyMatrix Ad(NX0, NX0);
    Ad << 1, 0, dt, 0,
          0, 1, 0, dt,
          0, 0, 1, 0,
          0, 0, 0, 1;
    
    tinyMatrix Bd(NX0, NU0);
    Bd << 0.5*dt*dt, 0,
          0, 0.5*dt*dt,
          dt, 0,
          0, dt;

    // Build lifted A, B matrices for PSD
    tinyMatrix A, B;
    tiny_build_lifted_from_base(Ad, Bd, A, B);
    
    std::cout << "[PSD Codegen] Base: nx0=" << NX0 << " nu0=" << NU0 << "\n";
    std::cout << "[PSD Codegen] Lifted: nxL=" << A.rows() << " nuL=" << B.cols() << "\n";

    // Cost matrices for lifted system
    tinyMatrix Q = tinyMatrix::Zero(NXL, NXL);
    // Base state costs (position and velocity)
    Q(0,0) = 50.0; Q(1,1) = 50.0;  // position
    Q(2,2) = 5.0;  Q(3,3) = 5.0;   // velocity
    // Small regularization on lifted XX block
    for (int i = NX0; i < NXL; ++i) {
        Q(i,i) = 1e-3;
    }

    tinyMatrix R = tinyMatrix::Zero(NUL, NUL);
    // Base input cost
    R(0,0) = 1.0; R(1,1) = 1.0;
    // Small regularization on lifted input blocks
    for (int i = NU0; i < NUL; ++i) {
        R(i,i) = 1e-2;
    }

    // State bounds
    tinyMatrix x_min = tiny_MatrixNxNh::Constant(-1e9);
    tinyMatrix x_max = tiny_MatrixNxNh::Constant(1e9);
    // Tighter bounds on base states
    x_min.topRows(NX0).setConstant(-30.0);
    x_max.topRows(NX0).setConstant(30.0);

    // Input bounds
    tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-1e9);
    tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(1e9);
    // Tighter bounds on base inputs (acceleration)
    u_min.topRows(NU0).setConstant(-10.0);
    u_max.topRows(NU0).setConstant(10.0);

    int verbose = 1;
    int status = tiny_setup(&solver,
                            A, B, Q, R,
                            tinytype(1.0), NXL, NUL, NHORIZON,
                            x_min, x_max, u_min, u_max,
                            verbose);
    if (status) {
        std::cerr << "[PSD Codegen] tiny_setup failed!\n";
        return 1;
    }

    // Enable PSD mode in solver settings
    tiny_enable_psd(solver, NX0, NU0, tinytype(5.0));
    
    // CRITICAL: Allocate space for 1 state linear constraint (disk avoidance)
    tiny_enable_state_linear(solver, 1);
    std::cout << "[PSD Codegen] Allocated " << solver->work->Alin_x.rows() << " state linear constraint rows\n";

    // Solver options
    solver->settings->abs_pri_tol = 1e-3;
    solver->settings->abs_dua_tol = 1e-3;
    solver->settings->max_iter = 100;
    solver->settings->check_termination = 1; 

    std::cout << "[PSD Codegen] Generating code to: " << std_fs::absolute(output_dir_relative) << "\n";
    tiny_codegen(solver, std_fs::absolute(output_dir_relative).string().c_str(), verbose);

    std::cout << "[PSD Codegen] Done! Solver dimensions: nx=" << NXL << " nu=" << NUL << " N=" << NHORIZON << "\n";

    return 0;
}

} /* extern "C" */
