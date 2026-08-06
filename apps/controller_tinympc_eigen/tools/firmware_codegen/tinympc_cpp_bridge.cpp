#include "tinympc/tiny_api.hpp"

#include <Eigen/Dense>

namespace {

void copy_matrix_row_major(const tinyMatrix& matrix, double* destination) {
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      destination[row * matrix.cols() + column] = matrix(row, column);
    }
  }
}

void copy_vector(const tinyVector& vector, double* destination) {
  for (Eigen::Index row = 0; row < vector.rows(); ++row) {
    destination[row] = vector(row);
  }
}

}  // namespace

extern "C" int crazyflie_tinympc_precompute(
    const double* a_data,
    const double* b_data,
    const double* f_data,
    const double* q_data,
    const double* r_data,
    double rho,
    int state_dim,
    int input_dim,
    double* kinf_data,
    double* pinf_data,
    double* quu_inverse_data,
    double* ambkt_data,
    double* apf_data,
    double* bpf_data,
    double* c1_data,
    double* c2_data) {
  using RowMajorMatrix =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  const Eigen::Map<const RowMajorMatrix> A(a_data, state_dim, state_dim);
  const Eigen::Map<const RowMajorMatrix> B(b_data, state_dim, input_dim);
  const Eigen::Map<const Eigen::VectorXd> f(f_data, state_dim);
  const Eigen::Map<const Eigen::VectorXd> Q_diagonal(q_data, state_dim);
  const Eigen::Map<const Eigen::VectorXd> R_diagonal(r_data, input_dim);

  TinyCache cache;
  const int status = tiny_precompute_and_set_cache(
      &cache,
      A,
      B,
      f,
      Q_diagonal.asDiagonal(),
      R_diagonal.asDiagonal(),
      state_dim,
      input_dim,
      rho,
      0);
  if (status != 0) {
    return status;
  }

  copy_matrix_row_major(cache.Kinf, kinf_data);
  copy_matrix_row_major(cache.Pinf, pinf_data);
  copy_matrix_row_major(cache.Quu_inv, quu_inverse_data);
  copy_matrix_row_major(cache.AmBKt, ambkt_data);
  copy_vector(cache.APf, apf_data);
  copy_vector(cache.BPf, bpf_data);
  copy_matrix_row_major(cache.C1, c1_data);
  copy_matrix_row_major(cache.C2, c2_data);
  return 0;
}
