# Eigen Test App for Crazyflie

This app tests modern Eigen with dynamic memory allocation for TinyMPC control on the Crazyflie platform.

## Overview

The Eigen Test App is designed to verify that modern Eigen (with dynamic memory allocation) can work on the Crazyflie's ARM Cortex-M4 processor. It tests all the operations needed for TinyMPC control:

- Dynamic matrix allocation
- Basic matrix operations (multiplication, addition, transpose)
- Matrix inverse operations
- Riccati equation solving
- Advanced linear algebra (SVD, QR, Cholesky, eigenvalue decomposition)

## Features

- **Dynamic Memory Allocation**: Tests Eigen's dynamic matrix allocation capabilities
- **Matrix Operations**: Comprehensive testing of basic matrix operations
- **Linear Algebra**: Tests advanced operations needed for MPC
- **Riccati Solver**: Tests solving discrete-time Riccati equations
- **Error Handling**: Robust error handling with try-catch blocks
- **Periodic Testing**: Re-runs tests periodically to ensure stability

## Building

```bash
cd apps/eigen_test
make clean
make -j$(nproc)
```

## Flashing

```bash
make cload
```

## Configuration

The app uses the following configuration in `app-config`:

- `CONFIG_APP_ENABLE=y`: Enables the app layer
- `CONFIG_APP_ENABLE_CPP=y`: Enables C++ compilation
- `CONFIG_APP_STACKSIZE=300`: Sets stack size for the app task

## Eigen Version

This app uses the latest Eigen from the official GitLab repository as a submodule. The Eigen version includes:

- Full dynamic matrix support
- Advanced linear algebra operations
- Exception handling support
- C++11 features

## Test Results

The app will output test results to the debug console:

```
=== Starting Eigen Tests ===
Testing dynamic memory allocation...
Dynamic allocation successful: A(12x12), B(12x4), Q(12x12), R(4x4)
Testing matrix operations...
Matrix operations successful: C(0,0)=1.234, y(0)=0.567, D(0,0)=2.345
Testing matrix inverse...
Matrix inverse successful: error=0.000001
Testing Riccati equation solving...
Riccati equation solved: P(0,0)=1.234, K(0,0)=0.567
Testing advanced linear algebra...
Linear algebra successful: SVD rank=4, QR rank=4, Cholesky valid=1, eig min=0.123
=== Test Results ===
Dynamic allocation: PASS
Matrix operations: PASS
Matrix inverse: PASS
Riccati equation: PASS
Linear algebra: PASS
Overall result: ALL TESTS PASSED
```

## Usage for TinyMPC

Once this app is working, it can serve as a foundation for implementing TinyMPC with modern Eigen. The tested operations include:

1. **Dynamic Matrix Allocation**: Required for variable-size MPC problems
2. **Matrix Inverse**: Needed for computing optimal control gains
3. **Riccati Solver**: Core component of LQR/MPC controllers
4. **Linear Algebra**: Required for matrix decompositions and solving

## Troubleshooting

If tests fail, check:

1. **Memory Issues**: The Crazyflie has limited RAM (128KB)
2. **Stack Overflow**: Increase `CONFIG_APP_STACKSIZE` if needed
3. **Compilation Errors**: Ensure C++11 and exceptions are enabled
4. **Runtime Errors**: Check debug output for specific error messages

## Next Steps

After successful testing, this app can be extended to:

1. Implement a full TinyMPC controller
2. Add real-time MPC solving
3. Integrate with the Crazyflie's control system
4. Add parameter tuning capabilities
