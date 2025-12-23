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

#define NSTATES 12
#define NINPUTS 4
#define NHORIZON 25

#include "../../TinyMPC/examples/problem_data/quadrotor_100hz_params.hpp"

extern "C" {

typedef Matrix<tinytype, NINPUTS, NHORIZON-1, ColMajor> tiny_MatrixNuNhm1;
typedef Matrix<tinytype, NSTATES, NHORIZON, ColMajor> tiny_MatrixNxNh;
typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;

std_fs::path output_dir_relative = "tinympc_generated_code_crazyflie_example/";

int main()
{
    TinySolver *solver;

    tinyMatrix Adyn = Map<Matrix<tinytype, NSTATES, NSTATES, RowMajor>>(Adyn_data);
    tinyMatrix Bdyn = Map<Matrix<tinytype, NSTATES, NINPUTS, RowMajor>>(Bdyn_data);
    tinyVector fdyn = tiny_VectorNx::Zero();
    tinyVector Q = Map<Matrix<tinytype, NSTATES, 1>>(Q_data);
    tinyVector R = Map<Matrix<tinytype, NINPUTS, 1>>(R_data);

    tinyMatrix x_min = tiny_MatrixNxNh::Constant(-1e9);
    tinyMatrix x_max = tiny_MatrixNxNh::Constant(1e9);
    tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-1e9);
    tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(1e9);

    // Set up problem
    int verbose = 0;
    int status = tiny_setup(&solver,
                            Adyn, Bdyn, fdyn, Q.asDiagonal(), R.asDiagonal(),
                            rho_value, NSTATES, NINPUTS, NHORIZON, verbose);
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
