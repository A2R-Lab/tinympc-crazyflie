// Codegen for Crazyflie quadrotor MPC
// The code will be generated in the `tinympc_generated_code_crazyflie_example` folder

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

#define NX0 4
#define NU0 2
#define NHORIZON 25

extern "C" {

typedef Matrix<tinytype, NX0 + NX0*NX0, NHORIZON, ColMajor> tiny_MatrixNxNh;
typedef Matrix<tinytype, NU0 + NX0*NU0 + NU0*NX0 + NU0*NU0, NHORIZON-1, ColMajor> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NX0 + NX0*NX0, 1> tiny_VectorNx;

std_fs::path output_dir_relative = "tinympc_generated_code_crazyflie_example/";

int main()
{
    TinySolver *solver;

    const tinytype dt = tinytype(0.01);
    Eigen::Matrix<tinytype, NX0, NX0> Ad;
    Ad << 1, 0, dt, 0,
          0, 1, 0, dt,
          0, 0, 1, 0,
          0, 0, 0, 1;
    Eigen::Matrix<tinytype, NX0, NU0> Bd;
    Bd << 0.5*dt*dt, 0,
          0, 0.5*dt*dt,
          dt, 0,
          0, dt;

    tinyMatrix A, B;
    tiny_build_lifted_from_base(Ad, Bd, A, B);

    const int nx = A.rows();
    const int nu = B.cols();
    tinyVector fdyn = tiny_VectorNx::Zero();

    tinyMatrix Q = tinyMatrix::Zero(nx, nx);
    Q(0,0) = 50.0; Q(1,1) = 50.0;
    Q(2,2) = 5.0;  Q(3,3) = 5.0;
    Q.diagonal().segment(NX0, NX0*NX0).array() = tinytype(1e-3);

    tinyMatrix R = tinyMatrix::Zero(nu, nu);
    R.diagonal().head(NU0).array() = tinytype(1.0);
    R.diagonal().segment(NU0, NX0*NU0).array() = tinytype(2.0);
    R.diagonal().segment(NU0 + NX0*NU0, NU0*NX0).array() = tinytype(2.0);
    R.diagonal().segment(NU0 + NX0*NU0 + NU0*NX0, NU0*NU0).array() = tinytype(10.0);

    tinyMatrix x_min = tiny_MatrixNxNh::Constant(-1e9);
    tinyMatrix x_max = tiny_MatrixNxNh::Constant(1e9);
    tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-1e9);
    tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(1e9);

    int verbose = 0;
    int status = tiny_setup(&solver,
                            A, B, fdyn, Q, R,
                            tinytype(1.0), nx, nu, NHORIZON, verbose);
    // Set bound constraints
    status = tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max);

    // Solver options
    solver->settings->abs_pri_tol = 1e-3;
    solver->settings->abs_dua_tol = 1e-3;
    solver->settings->max_iter = 100;
    solver->settings->check_termination = 1; 


    tiny_codegen(solver, std_fs::absolute(output_dir_relative).string().c_str(), verbose);

    return 0;
}

} /* extern "C" */
