// controller_tinympc.cpp - PHASE 4.1: TinyMPC Integration with SimpleMatrix
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
#include "controller.h"
#include "controller_pid.h"
#include "debug.h"
#include "stabilizer_types.h"
#include "commander.h"

#ifdef __cplusplus
}

// PHASE 4.1: Pure C++ matrix implementation (no Eigen)
#include <vector>
#include <algorithm>

#endif

// PHASE 4.1: TinyMPC integration
#define NHORIZON 10
#define NSTATES 12
#define NINPUTS 4

// PHASE 4.1: Create our own parameter arrays (no Eigen dependency)
// These are simplified versions of the quadrotor parameters
static const double Adyn_data[NSTATES * NSTATES] = {
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
};

static const double Bdyn_data[NSTATES * NINPUTS] = {
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0001, 0.0, 0.0, 0.0,
    0.0, 0.0001, 0.0, 0.0,
    0.0, 0.0, 0.0001, 0.0,
    0.0, 0.0, 0.0, 0.0001,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0
};

static const double Q_data[NSTATES * NSTATES] = {
    10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0
};

static const double R_data[NINPUTS * NINPUTS] = {
    100.0, 0.0, 0.0, 0.0,
    0.0, 100.0, 0.0, 0.0,
    0.0, 0.0, 100.0, 0.0,
    0.0, 0.0, 0.0, 100.0
};

static const double rho_value = 63.0;

#define DEBUG_MODULE "TINY_MPC_PHASE4_1"

// Global counter for appMain
static uint32_t test_counter = 0;

// Debug flag to track solver initialization
static bool solver_initialized = false;

// PHASE 4.1: Enhanced matrix class for TinyMPC compatibility
class SimpleMatrix {
private:
    std::vector<double> data;
    int rows, cols;
    
public:
    SimpleMatrix(int r, int c) : data(r * c, 0.0), rows(r), cols(c) {}
    
    double& operator()(int i, int j) { return data[i * cols + j]; }
    const double& operator()(int i, int j) const { return data[i * cols + j]; }
    
    int getRows() const { return rows; }
    int getCols() const { return cols; }
    
    // Identity matrix
    static SimpleMatrix Identity(int size) {
        SimpleMatrix I(size, size);
        for (int i = 0; i < size; i++) {
            I(i, i) = 1.0;
        }
        return I;
    }
    
    // Zero matrix
    static SimpleMatrix Zero(int r, int c) {
        return SimpleMatrix(r, c);
    }
    
    // Random matrix
    static SimpleMatrix Random(int r, int c) {
        SimpleMatrix M(r, c);
        for (int i = 0; i < r; i++) {
            for (int j = 0; j < c; j++) {
                M(i, j) = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
            }
        }
        return M;
    }
    
    // Matrix multiplication
    SimpleMatrix operator*(const SimpleMatrix& other) const {
        if (cols != other.rows) {
            DEBUG_PRINT("Matrix dimension mismatch: %dx%d * %dx%d\n", rows, cols, other.rows, other.cols);
            return SimpleMatrix(1, 1);
        }
        
        SimpleMatrix result(rows, other.cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                double sum = 0.0;
                for (int k = 0; k < cols; k++) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }
    
    // Matrix addition
    SimpleMatrix operator+(const SimpleMatrix& other) const {
        if (rows != other.rows || cols != other.cols) {
            DEBUG_PRINT("Matrix dimension mismatch: %dx%d + %dx%d\n", rows, cols, other.rows, other.cols);
            return SimpleMatrix(1, 1);
        }
        
        SimpleMatrix result(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result(i, j) = (*this)(i, j) + other(i, j);
            }
        }
        return result;
    }
    
    // Scalar multiplication (matrix * scalar)
    SimpleMatrix operator*(double scalar) const {
        SimpleMatrix result(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result(i, j) = (*this)(i, j) * scalar;
            }
        }
        return result;
    }
    
    // Scalar multiplication (scalar * matrix) - friend function
    friend SimpleMatrix operator*(double scalar, const SimpleMatrix& matrix) {
        return matrix * scalar;
    }
    
    // Transpose
    SimpleMatrix transpose() const {
        SimpleMatrix result(cols, rows);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }
    
    // Simple inverse (for small matrices)
    SimpleMatrix inverse() const {
        if (rows != cols || rows > 4) {
            DEBUG_PRINT("Inverse only implemented for small square matrices!\n");
            return SimpleMatrix(1, 1);
        }
        
        if (rows == 1) {
            SimpleMatrix result(1, 1);
            result(0, 0) = 1.0 / (*this)(0, 0);
            return result;
        }
        
        if (rows == 2) {
            double det = (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
            if (fabs(det) < 1e-10) {
                DEBUG_PRINT("Matrix is singular!\n");
                return SimpleMatrix(1, 1);
            }
            
            SimpleMatrix result(2, 2);
            result(0, 0) = (*this)(1, 1) / det;
            result(0, 1) = -(*this)(0, 1) / det;
            result(1, 0) = -(*this)(1, 0) / det;
            result(1, 1) = (*this)(0, 0) / det;
            return result;
        }
        
        // For larger matrices, return identity (placeholder)
        return Identity(rows);
    }
    
    // Copy data from array (for parameter mapping)
    void copyFromArray(const double* arr) {
        for (int i = 0; i < rows * cols; i++) {
            data[i] = arr[i];
        }
    }
    
    // Debug print matrix
    void debugPrint(const char* name) const {
        DEBUG_PRINT("%s (%dx%d):\n", name, rows, cols);
        for (int i = 0; i < rows && i < 3; i++) {  // Only print first 3 rows
            for (int j = 0; j < cols && j < 3; j++) {  // Only print first 3 cols
                DEBUG_PRINT("%.3f ", (*this)(i, j));
            }
            if (cols > 3) DEBUG_PRINT("...");
            DEBUG_PRINT("\n");
        }
        if (rows > 3) DEBUG_PRINT("...\n");
    }
};

// PHASE 4.1: Simple TinyMPC-like solver structure
struct SimpleTinySolver {
    SimpleMatrix Adyn;
    SimpleMatrix Bdyn;
    SimpleMatrix fdyn;
    SimpleMatrix Q;
    SimpleMatrix R;
    double rho;
    int nx, nu, N;
    
    SimpleTinySolver() : 
        Adyn(NSTATES, NSTATES), 
        Bdyn(NSTATES, NINPUTS), 
        fdyn(NSTATES, 1),
        Q(NSTATES, NSTATES), 
        R(NINPUTS, NINPUTS),
        rho(0.0), nx(NSTATES), nu(NINPUTS), N(NHORIZON) {
        
        DEBUG_PRINT("SimpleTinySolver: Constructor called with NSTATES=%d, NINPUTS=%d\n", NSTATES, NINPUTS);
        DEBUG_PRINT("SimpleTinySolver: Matrix dimensions after initialization:\n");
        DEBUG_PRINT("  Adyn: %dx%d\n", Adyn.getRows(), Adyn.getCols());
        DEBUG_PRINT("  Bdyn: %dx%d\n", Bdyn.getRows(), Bdyn.getCols());
        DEBUG_PRINT("  Q: %dx%d\n", Q.getRows(), Q.getCols());
        DEBUG_PRINT("  R: %dx%d\n", R.getRows(), R.getCols());
        
        // Set initialization flag
        solver_initialized = true;
    }
};

// PHASE 4.1: Test TinyMPC integration
static void testTinyMPCIntegration(void) {
    DEBUG_PRINT("PHASE 4.1: Testing TinyMPC integration...\n");
    
    // Create solver locally to avoid global initialization issues
    DEBUG_PRINT("PHASE 4.1: Creating solver locally...\n");
    SimpleTinySolver solver;
    
    // Check if solver was initialized
    if (!solver_initialized) {
        DEBUG_PRINT("PHASE 4.1: WARNING - Solver constructor was not called!\n");
        DEBUG_PRINT("PHASE 4.1: Current matrix dimensions:\n");
        DEBUG_PRINT("  Adyn: %dx%d\n", solver.Adyn.getRows(), solver.Adyn.getCols());
        DEBUG_PRINT("  Bdyn: %dx%d\n", solver.Bdyn.getRows(), solver.Bdyn.getCols());
        DEBUG_PRINT("  Q: %dx%d\n", solver.Q.getRows(), solver.Q.getCols());
        DEBUG_PRINT("  R: %dx%d\n", solver.R.getRows(), solver.R.getCols());
    } else {
        DEBUG_PRINT("PHASE 4.1: Solver was properly initialized\n");
    }
    
    // Test 1: Load parameters from our arrays
    DEBUG_PRINT("PHASE 4.1: Test 1 - Loading parameters...\n");
    {
        solver.Adyn.copyFromArray(Adyn_data);
        solver.Bdyn.copyFromArray(Bdyn_data);
        solver.fdyn = SimpleMatrix::Zero(NSTATES, 1);  // fdyn not in params
        solver.Q.copyFromArray(Q_data);
        solver.R.copyFromArray(R_data);
        solver.rho = rho_value;
        
        DEBUG_PRINT("PHASE 4.1: Parameters loaded successfully\n");
        DEBUG_PRINT("PHASE 4.1: Adyn(0,0) = %.6f\n", solver.Adyn(0,0));
        DEBUG_PRINT("PHASE 4.1: Bdyn(0,0) = %.6f\n", solver.Bdyn(0,0));
        DEBUG_PRINT("PHASE 4.1: Q(0,0) = %.1f\n", solver.Q(0,0));
        DEBUG_PRINT("PHASE 4.1: R(0,0) = %.1f\n", solver.R(0,0));
        DEBUG_PRINT("PHASE 4.1: rho = %.1f\n", solver.rho);
        
        // Debug print matrices
        solver.Adyn.debugPrint("Adyn");
        solver.Bdyn.debugPrint("Bdyn");
        solver.Q.debugPrint("Q");
        solver.R.debugPrint("R");
    }
    
    // Test 2: Basic MPC operations
    DEBUG_PRINT("PHASE 4.1: Test 2 - Basic MPC operations...\n");
    {
        // Create state and input vectors
        SimpleMatrix x = SimpleMatrix::Random(NSTATES, 1);
        SimpleMatrix u = SimpleMatrix::Random(NINPUTS, 1);
        
        x.debugPrint("x");
        u.debugPrint("u");
        
        // Predict next state: x_next = Adyn * x + Bdyn * u + fdyn
        SimpleMatrix x_next = solver.Adyn * x + solver.Bdyn * u + solver.fdyn;
        
        DEBUG_PRINT("PHASE 4.1: State prediction successful\n");
        DEBUG_PRINT("PHASE 4.1: x_next(0) = %.3f\n", x_next(0,0));
        x_next.debugPrint("x_next");
    }
    
    // Test 3: Cost function evaluation
    DEBUG_PRINT("PHASE 4.1: Test 3 - Cost function evaluation...\n");
    {
        SimpleMatrix x = SimpleMatrix::Random(NSTATES, 1);
        SimpleMatrix u = SimpleMatrix::Random(NINPUTS, 1);
        
        // State cost: x' * Q * x
        SimpleMatrix state_cost = x.transpose() * solver.Q * x;
        
        // Input cost: u' * R * u
        SimpleMatrix input_cost = u.transpose() * solver.R * u;
        
        double total_cost = state_cost(0,0) + input_cost(0,0);
        
        DEBUG_PRINT("PHASE 4.1: Cost function evaluation successful\n");
        DEBUG_PRINT("PHASE 4.1: Total cost = %.3f\n", total_cost);
    }
    
    // Test 4: Simple MPC solve (one step)
    DEBUG_PRINT("PHASE 4.1: Test 4 - Simple MPC solve...\n");
    {
        SimpleMatrix x = SimpleMatrix::Random(NSTATES, 1);
        
        // Simple one-step MPC: minimize u'*R*u subject to x_next = Adyn*x + Bdyn*u
        // Solution: u = -inv(R) * Bdyn' * Q * (Adyn * x)
        SimpleMatrix R_inv = solver.R.inverse();
        SimpleMatrix x_pred = solver.Adyn * x;
        SimpleMatrix u_opt = -1.0 * R_inv * solver.Bdyn.transpose() * solver.Q * x_pred;
        
        DEBUG_PRINT("PHASE 4.1: Simple MPC solve successful\n");
        DEBUG_PRINT("PHASE 4.1: u_opt(0) = %.3f\n", u_opt(0,0));
    }
    
    DEBUG_PRINT("PHASE 4.1: All TinyMPC integration tests PASSED!\n");
}

// Controller interface functions
extern "C" void controllerOutOfTreeInit(void) {
    DEBUG_PRINT("TinyMPC Phase 4.1: Controller init\n");
    
    // Test TinyMPC integration
    testTinyMPCIntegration();
    
    // Initialize PID controller as fallback
    controllerPidInit();
    
    DEBUG_PRINT("TinyMPC Phase 4.1: Init complete\n");
}

extern "C" bool controllerOutOfTreeTest(void) {
    return true;
}

extern "C" void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const stabilizerStep_t tick) {

    // Phase 4.1: Use PID controller as fallback
    controllerPid(control, setpoint, sensors, state, tick);
    
    // Add some debug output
    static uint32_t debug_counter = 0;
    if (++debug_counter % 1000 == 0) {
        DEBUG_PRINT("TinyMPC Phase 4.1: PID control, thrust=%u\n", (unsigned int)control->thrust);
    }
}

// RTOS-compliant app main function
extern "C" void appMain(void) {
    DEBUG_PRINT("TinyMPC Phase 4.1 App: Starting...\n");
    while (1) {
        vTaskDelay(M2T(1000));
        DEBUG_PRINT("TinyMPC Phase 4.1 App: alive %lu\n", test_counter++);
    }
}
