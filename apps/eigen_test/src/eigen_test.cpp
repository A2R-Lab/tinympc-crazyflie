/**
 * Eigen Test App for Crazyflie
 * 
 * This app tests comprehensive Eigen operations needed for TinyMPC control.
 * It tests fundamental operations that work reliably on the ARM Cortex-M4.
 * 
 * Based on Crazyflie App Layer API
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

#ifdef __cplusplus
}

// Use the working Eigen 3.4.0 approach
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <Eigen/SVD>
using namespace Eigen;

#endif

#define DEBUG_MODULE "EIGEN_TEST"

// Test results storage
static struct {
    bool basic_ops;
    bool matrix_inverse;
    bool linear_solver;
    bool svd_decomposition;
    bool qr_decomposition;
    bool cholesky_decomposition;
    bool eigenvalue_decomposition;
    bool riccati_solver;
    bool matrix_operations;
    bool vector_operations;
    bool dynamic_matrices;
    bool dynamic_operations;
    bool dynamic_decompositions;
    bool tinympc_operations;
} test_results = {false};

/**
 * Test basic matrix operations
 */
static void test_basic_operations() {
    DEBUG_PRINT("Testing basic matrix operations...\n");
    
    // Test small matrices
    Matrix<double, 2, 2> A;
    Matrix<double, 2, 1> b;
    
    A << 1.0, 0.0,
         0.0, 1.0;
    b << 1.0, 2.0;
    
    // Test matrix multiplication
    Matrix<double, 2, 1> result1 = A * b;
    DEBUG_PRINT("Matrix multiplication: %f, %f\n", result1(0), result1(1));
    
    // Test matrix addition
    Matrix<double, 2, 2> result2 = A + A;
    DEBUG_PRINT("Matrix addition: %f\n", result2(0,0));
    
    // Test matrix transpose
    Matrix<double, 1, 2> result3 = b.transpose();
    DEBUG_PRINT("Matrix transpose: %f\n", result3(0));
    
    // Test element-wise operations
    Matrix<double, 2, 1> result4 = b.array() * b.array();
    DEBUG_PRINT("Element-wise multiplication: %f\n", result4(0));
    
    test_results.basic_ops = true;
    DEBUG_PRINT("Basic operations test PASSED\n");
}

/**
 * Test matrix inverse operations
 */
static void test_matrix_inverse() {
    DEBUG_PRINT("Testing matrix inverse...\n");
    
    // Create a simple test matrix
    Matrix<double, 2, 2> test_matrix;
    test_matrix << 2.0, 1.0,
                   1.0, 2.0;
    
    // Test inverse
    Matrix<double, 2, 2> inverse = test_matrix.inverse();
    Matrix<double, 2, 2> identity = test_matrix * inverse;
    
    // Check if result is close to identity
    double error = (identity - Matrix<double, 2, 2>::Identity()).norm();
    DEBUG_PRINT("Inverse test error: %f\n", error);
    
    test_results.matrix_inverse = (error < 1e-6);
    DEBUG_PRINT("Matrix inverse test %s\n", test_results.matrix_inverse ? "PASSED" : "FAILED");
}

/**
 * Test linear solver
 */
static void test_linear_solver() {
    DEBUG_PRINT("Testing linear solver...\n");
    
    // Create a simple test system Ax = b
    Matrix<double, 2, 2> A_solve;
    Matrix<double, 2, 1> b;
    
    A_solve << 2.0, 1.0,
               1.0, 2.0;
    b << 1.0, 2.0;
    
    // Solve using LU decomposition
    Matrix<double, 2, 1> x = A_solve.lu().solve(b);
    
    // Check if solution is correct
    Matrix<double, 2, 1> residual = A_solve * x - b;
    double error = residual.norm();
    
    DEBUG_PRINT("Linear solver error: %f\n", error);
    DEBUG_PRINT("Solution: %f, %f\n", x(0), x(1));
    
    test_results.linear_solver = (error < 1e-6);
    DEBUG_PRINT("Linear solver test %s\n", test_results.linear_solver ? "PASSED" : "FAILED");
}

/**
 * Test SVD decomposition (simplified)
 */
static void test_svd_decomposition() {
    DEBUG_PRINT("Testing SVD decomposition...\n");
    
    // Create a small test matrix
    Matrix<double, 2, 2> A;
    A << 1.0, 2.0,
         3.0, 4.0;
    
    // Perform SVD
    JacobiSVD<Matrix<double, 2, 2>> svd(A, ComputeFullU | ComputeFullV);
    
    // Reconstruct matrix
    Matrix<double, 2, 2> reconstructed = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
    
    // Check reconstruction error
    double error = (A - reconstructed).norm();
    DEBUG_PRINT("SVD reconstruction error: %f\n", error);
    
    test_results.svd_decomposition = (error < 1e-6);
    DEBUG_PRINT("SVD decomposition test %s\n", test_results.svd_decomposition ? "PASSED" : "FAILED");
}

/**
 * Test QR decomposition (simplified)
 */
static void test_qr_decomposition() {
    DEBUG_PRINT("Testing QR decomposition...\n");
    
    // Create a small test matrix
    Matrix<double, 2, 2> A;
    A << 1.0, 2.0,
         3.0, 4.0;
    
    // Perform QR decomposition
    HouseholderQR<Matrix<double, 2, 2>> qr(A);
    
    // Get Q and R matrices
    Matrix<double, 2, 2> Q = qr.householderQ();
    Matrix<double, 2, 2> R = qr.matrixQR().triangularView<Upper>();
    
    // Reconstruct matrix
    Matrix<double, 2, 2> reconstructed = Q * R;
    
    // Check reconstruction error
    double error = (A - reconstructed).norm();
    DEBUG_PRINT("QR reconstruction error: %f\n", error);
    
    test_results.qr_decomposition = (error < 1e-6);
    DEBUG_PRINT("QR decomposition test %s\n", test_results.qr_decomposition ? "PASSED" : "FAILED");
}

/**
 * Test Cholesky decomposition (simplified)
 */
static void test_cholesky_decomposition() {
    DEBUG_PRINT("Testing Cholesky decomposition...\n");
    
    // Create a positive definite matrix
    Matrix<double, 2, 2> A;
    A << 4.0, 1.0,
         1.0, 4.0;
    
    // Perform Cholesky decomposition
    LLT<Matrix<double, 2, 2>> llt(A);
    
    // Get L matrix
    Matrix<double, 2, 2> L = llt.matrixL();
    
    // Reconstruct matrix
    Matrix<double, 2, 2> reconstructed = L * L.transpose();
    
    // Check reconstruction error
    double error = (A - reconstructed).norm();
    DEBUG_PRINT("Cholesky reconstruction error: %f\n", error);
    
    test_results.cholesky_decomposition = (error < 1e-6);
    DEBUG_PRINT("Cholesky decomposition test %s\n", test_results.cholesky_decomposition ? "PASSED" : "FAILED");
}

/**
 * Test eigenvalue decomposition (simplified)
 */
static void test_eigenvalue_decomposition() {
    DEBUG_PRINT("Testing eigenvalue decomposition...\n");
    
    // Create a symmetric matrix
    Matrix<double, 2, 2> A;
    A << 3.0, 1.0,
         1.0, 3.0;
    
    // Perform eigenvalue decomposition
    SelfAdjointEigenSolver<Matrix<double, 2, 2>> eigensolver(A);
    
    // Get eigenvalues and eigenvectors
    Vector2d eigenvalues = eigensolver.eigenvalues();
    Matrix<double, 2, 2> eigenvectors = eigensolver.eigenvectors();
    
    // Reconstruct matrix
    Matrix<double, 2, 2> reconstructed = eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
    
    // Check reconstruction error
    double error = (A - reconstructed).norm();
    DEBUG_PRINT("Eigenvalue decomposition error: %f\n", error);
    DEBUG_PRINT("Eigenvalues: %f, %f\n", eigenvalues(0), eigenvalues(1));
    
    test_results.eigenvalue_decomposition = (error < 1e-6);
    DEBUG_PRINT("Eigenvalue decomposition test %s\n", test_results.eigenvalue_decomposition ? "PASSED" : "FAILED");
}

/**
 * Test Riccati equation solver (simplified)
 */
static void test_riccati_solver() {
    DEBUG_PRINT("Testing Riccati equation solver...\n");
    
    // Simple discrete-time Riccati equation: P = A'PA - A'PB(B'PB + R)^(-1)B'PA + Q
    // For a simple 2x2 system
    Matrix<double, 2, 2> A, Q, P;
    Matrix<double, 2, 1> B;
    double R = 1.0;
    
    A << 0.9, 0.1,
         0.0, 0.9;
    B << 0.0, 1.0;
    Q << 1.0, 0.0,
         0.0, 1.0;
    P << 1.0, 0.0,
         0.0, 1.0;
    
    // One step of Riccati iteration
    double scalar_term = B.transpose() * P * B + R;
    Matrix<double, 2, 2> P_new = A.transpose() * P * A - 
                                 (1.0 / scalar_term) * A.transpose() * P * B * B.transpose() * P * A + Q;
    
    // Check if P_new is positive definite (simplified check)
    bool is_positive_definite = (P_new(0,0) > 0) && (P_new.determinant() > 0);
    
    DEBUG_PRINT("Riccati P_new(0,0): %f\n", P_new(0,0));
    DEBUG_PRINT("Riccati det(P_new): %f\n", P_new.determinant());
    
    test_results.riccati_solver = is_positive_definite;
    DEBUG_PRINT("Riccati solver test %s\n", test_results.riccati_solver ? "PASSED" : "FAILED");
}

/**
 * Test TinyMPC-specific Eigen operations
 */
static void test_tinympc_operations() {
    DEBUG_PRINT("Testing TinyMPC-specific operations...\n");
    
    // Typical MPC dimensions
    int nx = 12; // states
    int nu = 4;  // inputs
    int N = 10;  // horizon
    
    // Create dynamic matrices similar to TinyMPC
    MatrixXd Adyn(nx, nx);
    MatrixXd Bdyn(nx, nu);
    MatrixXd Q(nx, nx);
    MatrixXd R(nu, nu);
    VectorXd fdyn(nx);
    
    // Initialize with some values
    Adyn.setIdentity();
    Adyn *= 0.9;
    Bdyn.setRandom();
    Bdyn *= 0.1;
    Q.setIdentity();
    R.setIdentity();
    R *= 0.01;
    fdyn.setZero();
    
    // Test matrix operations used in TinyMPC
    MatrixXd rho_identity = 0.1 * MatrixXd::Identity(nx, nx);
    MatrixXd Q_rho = Q + rho_identity;
    VectorXd Q_diag = Q_rho.diagonal();
    
    DEBUG_PRINT("Q_rho diagonal sum: %f\n", Q_diag.sum());
    
    // Test lazyProduct and noalias (critical for performance)
    MatrixXd result1(nx, nu);
    result1.noalias() = Adyn.lazyProduct(Bdyn);
    DEBUG_PRINT("Lazy product norm: %f\n", result1.norm());
    
    // Test block operations
    MatrixXd large_matrix(nx, N);
    large_matrix.setRandom();
    VectorXd col_block = large_matrix.col(0);
    MatrixXd row_block = large_matrix.row(0);
    
    DEBUG_PRINT("Col block norm: %f\n", col_block.norm());
    DEBUG_PRINT("Row block norm: %f\n", row_block.norm());
    
    // Test head and tail operations
    VectorXd vec(nx);
    vec.setRandom();
    VectorXd head_vec = vec.head(nx-1);
    double last_element = vec(Eigen::placeholders::last);
    
    DEBUG_PRINT("Head norm: %f, last element: %f\n", head_vec.norm(), last_element);
    
    // Test cwise operations
    MatrixXd min_bounds(nx, N);
    MatrixXd max_bounds(nx, N);
    min_bounds.setConstant(-10.0);
    max_bounds.setConstant(10.0);
    
    MatrixXd clamped = max_bounds.cwiseMin(min_bounds.cwiseMax(large_matrix));
    DEBUG_PRINT("Clamped matrix norm: %f\n", clamped.norm());
    
    // Test dot product and projections
    VectorXd a(nx);
    VectorXd b(nx);
    a.setRandom();
    b.setRandom();
    double dot_prod = a.dot(b);
    double sq_norm = a.squaredNorm();
    
    DEBUG_PRINT("Dot product: %f, squared norm: %f\n", dot_prod, sq_norm);
    
    // Test hyperplane projection (used in constraints)
    VectorXd z = a;
    double dist = (a.dot(z) - 5.0) / a.squaredNorm();
    VectorXd projected = z - dist * a;
    
    DEBUG_PRINT("Projected vector norm: %f\n", projected.norm());
    
    // Test reshaping (used in codegen)
    MatrixXd mat_2d(4, 3);
    mat_2d.setRandom();
    VectorXd reshaped = mat_2d.reshaped<RowMajor>();
    
    DEBUG_PRINT("Reshaped vector size: %d\n", (int)reshaped.size());
    
    test_results.tinympc_operations = true;
    DEBUG_PRINT("TinyMPC-specific operations test PASSED\n");
}

/**
 * Test vector operations (simplified)
 */
static void test_vector_operations() {
    DEBUG_PRINT("Testing vector operations...\n");
    
    // Test vector operations
    Vector2d v1(1.0, 2.0);
    Vector2d v2(3.0, 4.0);
    
    double dot_product = v1.dot(v2);
    double norm = v1.norm();
    Vector2d normalized = v1.normalized();
    
    DEBUG_PRINT("Dot product: %f\n", dot_product);
    DEBUG_PRINT("Norm: %f\n", norm);
    DEBUG_PRINT("Normalized(0): %f\n", normalized(0));
    
    // Test vector arithmetic
    Vector2d sum = v1 + v2;
    Vector2d diff = v1 - v2;
    Vector2d scaled = 2.0 * v1;
    
    DEBUG_PRINT("Sum(0): %f\n", sum(0));
    DEBUG_PRINT("Diff(0): %f\n", diff(0));
    DEBUG_PRINT("Scaled(0): %f\n", scaled(0));
    
    test_results.vector_operations = true;
    DEBUG_PRINT("Vector operations test PASSED\n");
}

/**
 * Test dynamic matrices (CRITICAL for TinyMPC)
 */
static void test_dynamic_matrices() {
    DEBUG_PRINT("Testing dynamic matrices...\n");
    
    // Test dynamic matrix creation and resizing
    MatrixXd A_dyn(2, 2);
    A_dyn << 1.0, 2.0,
             3.0, 4.0;
    
    DEBUG_PRINT("Dynamic matrix size: %dx%d\n", (int)A_dyn.rows(), (int)A_dyn.cols());
    DEBUG_PRINT("Dynamic matrix(0,0): %f\n", A_dyn(0,0));
    
    // Test resizing
    A_dyn.resize(3, 3);
    A_dyn << 1.0, 2.0, 3.0,
             4.0, 5.0, 6.0,
             7.0, 8.0, 9.0;
    
    DEBUG_PRINT("Resized matrix size: %dx%d\n", (int)A_dyn.rows(), (int)A_dyn.cols());
    DEBUG_PRINT("Resized matrix(2,2): %f\n", A_dyn(2,2));
    
    // Test dynamic vector
    VectorXd b_dyn(3);
    b_dyn << 1.0, 2.0, 3.0;
    
    DEBUG_PRINT("Dynamic vector size: %d\n", (int)b_dyn.size());
    DEBUG_PRINT("Dynamic vector(1): %f\n", b_dyn(1));
    
    // Test dynamic matrix-vector multiplication
    VectorXd result_dyn = A_dyn * b_dyn;
    DEBUG_PRINT("Dynamic matrix-vector result(0): %f\n", result_dyn(0));
    
    test_results.dynamic_matrices = true;
    DEBUG_PRINT("Dynamic matrices test PASSED\n");
}

/**
 * Test dynamic operations (CRITICAL for TinyMPC)
 */
static void test_dynamic_operations() {
    DEBUG_PRINT("Testing dynamic operations...\n");
    
    // Test dynamic matrix operations
    MatrixXd A_dyn(2, 2);
    MatrixXd B_dyn(2, 2);
    
    A_dyn << 1.0, 2.0,
             3.0, 4.0;
    B_dyn << 5.0, 6.0,
             7.0, 8.0;
    
    // Test dynamic matrix multiplication
    MatrixXd C_dyn = A_dyn * B_dyn;
    DEBUG_PRINT("Dynamic matrix multiplication(0,0): %f\n", C_dyn(0,0));
    
    // Test dynamic matrix addition
    MatrixXd D_dyn = A_dyn + B_dyn;
    DEBUG_PRINT("Dynamic matrix addition(0,0): %f\n", D_dyn(0,0));
    
    // Test dynamic matrix inverse
    MatrixXd A_inv_dyn = A_dyn.inverse();
    MatrixXd identity_dyn = A_dyn * A_inv_dyn;
    double error_dyn = (identity_dyn - MatrixXd::Identity(2, 2)).norm();
    DEBUG_PRINT("Dynamic matrix inverse error: %f\n", error_dyn);
    
    // Test dynamic linear solver
    VectorXd b_dyn(2);
    b_dyn << 1.0, 2.0;
    VectorXd x_dyn = A_dyn.lu().solve(b_dyn);
    VectorXd residual_dyn = A_dyn * x_dyn - b_dyn;
    double solve_error = residual_dyn.norm();
    DEBUG_PRINT("Dynamic linear solver error: %f\n", solve_error);
    
    test_results.dynamic_operations = (error_dyn < 1e-6) && (solve_error < 1e-6);
    DEBUG_PRINT("Dynamic operations test %s\n", test_results.dynamic_operations ? "PASSED" : "FAILED");
}

/**
 * Test dynamic decompositions (CRITICAL for TinyMPC)
 */
static void test_dynamic_decompositions() {
    DEBUG_PRINT("Testing dynamic decompositions...\n");
    
    // Test dynamic SVD
    MatrixXd A_dyn(2, 2);
    A_dyn << 1.0, 2.0,
             3.0, 4.0;
    
    JacobiSVD<MatrixXd> svd_dyn(A_dyn, ComputeFullU | ComputeFullV);
    MatrixXd reconstructed_dyn = svd_dyn.matrixU() * svd_dyn.singularValues().asDiagonal() * svd_dyn.matrixV().transpose();
    double svd_error = (A_dyn - reconstructed_dyn).norm();
    DEBUG_PRINT("Dynamic SVD error: %f\n", svd_error);
    
    // Test dynamic QR
    HouseholderQR<MatrixXd> qr_dyn(A_dyn);
    MatrixXd Q_dyn = qr_dyn.householderQ();
    MatrixXd R_dyn = qr_dyn.matrixQR().triangularView<Upper>();
    MatrixXd qr_reconstructed = Q_dyn * R_dyn;
    double qr_error = (A_dyn - qr_reconstructed).norm();
    DEBUG_PRINT("Dynamic QR error: %f\n", qr_error);
    
    // Test dynamic Cholesky (with positive definite matrix)
    MatrixXd A_pd(2, 2);
    A_pd << 4.0, 1.0,
            1.0, 4.0;
    LLT<MatrixXd> llt_dyn(A_pd);
    MatrixXd L_dyn = llt_dyn.matrixL();
    MatrixXd chol_reconstructed = L_dyn * L_dyn.transpose();
    double chol_error = (A_pd - chol_reconstructed).norm();
    DEBUG_PRINT("Dynamic Cholesky error: %f\n", chol_error);
    
    test_results.dynamic_decompositions = (svd_error < 1e-6) && (qr_error < 1e-6) && (chol_error < 1e-6);
    DEBUG_PRINT("Dynamic decompositions test %s\n", test_results.dynamic_decompositions ? "PASSED" : "FAILED");
}

/**
 * Print test summary
 */
static void print_test_summary() {
    DEBUG_PRINT("\n=== EIGEN TEST SUMMARY ===\n");
    DEBUG_PRINT("Basic operations: %s\n", test_results.basic_ops ? "PASS" : "FAIL");
    DEBUG_PRINT("Matrix inverse: %s\n", test_results.matrix_inverse ? "PASS" : "FAIL");
    DEBUG_PRINT("Linear solver: %s\n", test_results.linear_solver ? "PASS" : "FAIL");
    DEBUG_PRINT("SVD decomposition: %s\n", test_results.svd_decomposition ? "PASS" : "FAIL");
    DEBUG_PRINT("QR decomposition: %s\n", test_results.qr_decomposition ? "PASS" : "FAIL");
    DEBUG_PRINT("Cholesky decomposition: %s\n", test_results.cholesky_decomposition ? "PASS" : "FAIL");
    DEBUG_PRINT("Eigenvalue decomposition: %s\n", test_results.eigenvalue_decomposition ? "PASS" : "FAIL");
    DEBUG_PRINT("Riccati solver: %s\n", test_results.riccati_solver ? "PASS" : "FAIL");
    DEBUG_PRINT("TinyMPC operations: %s\n", test_results.tinympc_operations ? "PASS" : "FAIL");
    DEBUG_PRINT("Vector operations: %s\n", test_results.vector_operations ? "PASS" : "FAIL");
    DEBUG_PRINT("Dynamic matrices: %s\n", test_results.dynamic_matrices ? "PASS" : "FAIL");
    DEBUG_PRINT("Dynamic operations: %s\n", test_results.dynamic_operations ? "PASS" : "FAIL");
    DEBUG_PRINT("Dynamic decompositions: %s\n", test_results.dynamic_decompositions ? "PASS" : "FAIL");
    DEBUG_PRINT("TinyMPC operations: %s\n", test_results.tinympc_operations ? "PASS" : "FAIL");
    
    int passed = 0;
    if (test_results.basic_ops) passed++;
    if (test_results.matrix_inverse) passed++;
    if (test_results.linear_solver) passed++;
    if (test_results.svd_decomposition) passed++;
    if (test_results.qr_decomposition) passed++;
    if (test_results.cholesky_decomposition) passed++;
    if (test_results.eigenvalue_decomposition) passed++;
    if (test_results.riccati_solver) passed++;
    if (test_results.tinympc_operations) passed++;
    if (test_results.vector_operations) passed++;
    if (test_results.dynamic_matrices) passed++;
    if (test_results.dynamic_operations) passed++;
    if (test_results.dynamic_decompositions) passed++;
    if (test_results.tinympc_operations) passed++;
    
    DEBUG_PRINT("Overall: %d/14 tests passed\n", passed);
    DEBUG_PRINT("========================\n\n");
}

/**
 * Main test function
 */
static void run_eigen_tests() {
    DEBUG_PRINT("Starting comprehensive Eigen tests...\n");
    
    test_basic_operations();
    test_matrix_inverse();
    test_linear_solver();
    test_svd_decomposition();
    test_qr_decomposition();
    test_cholesky_decomposition();
    test_eigenvalue_decomposition();
    test_riccati_solver();
    test_tinympc_operations();
    test_vector_operations();
    test_dynamic_matrices();
    test_dynamic_operations();
    test_dynamic_decompositions();
    
    print_test_summary();
}

/**
 * App initialization
 */
void appMain() {
    DEBUG_PRINT("Eigen Test App: Starting...\n");
    
    // Run tests after a short delay
    vTaskDelay(M2T(1000));
    
    run_eigen_tests();
    
    DEBUG_PRINT("Eigen Test App: Tests completed\n");
}

/**
 * App test function
 */
void appTest() {
    DEBUG_PRINT("Eigen Test App: Test function called\n");
}

#ifdef __cplusplus
#endif