#include "constraint_linear.h"

#ifdef __cplusplus
extern "C" {
#endif

enum tiny_ErrorCode tiny_SetInputBound(tiny_AdmmWorkspace* work, Eigen::MatrixMf* Acu, Eigen::VectorMf* lcu, Eigen::VectorMf* ucu) {
  work->stgs->en_cstr_inputs = 1;
  work->data->Acu = Acu;
  (*(work->data->Acu)).setIdentity();
  work->data->lcu = lcu;
  work->data->ucu = ucu;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetStateBound(tiny_AdmmWorkspace* work, Eigen::MatrixNf* Acx, Eigen::VectorNf* lcx, Eigen::VectorNf* ucx) {
  work->stgs->en_cstr_states = 1;
  work->data->Acx = Acx;
  (*(work->data->Acx)).setIdentity();
  work->data->lcx = lcx;
  work->data->ucx = ucx;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_ClearPositionHalfspaces(tiny_AdmmWorkspace* work) {
  const int horizon = work->data->model[0].nhorizon;
  if (horizon > TINY_MAX_HORIZON_KNOTS) {
    return TINY_NOT_SUPPORTED;
  }
  for (int k = 0; k < horizon; ++k) {
    for (int h = 0; h < MAX_HS; ++h) {
      work->data->a_pos_hs[k][h].setZero();
      work->data->a_vel_hs[k][h].setZero();
      work->data->b_hs[k][h] = 0.0f;
      work->data->slack_penalty_hs[k][h] = 0.0f;
      work->data->slack_used_hs[k][h] = 0.0f;
      work->data->en_hs[k][h] = 0;
    }
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetPositionHalfspace(tiny_AdmmWorkspace* work,
                                              int k,
                                              int h,
                                              const Eigen::Vector3f* a,
                                              float b,
                                              int enable) {
  Eigen::Vector3f zero = Eigen::Vector3f::Zero();
  return tiny_SetKinematicHalfspace(
      work, k, h, a, &zero, b, 0.0f, enable);
}

enum tiny_ErrorCode tiny_SetKinematicHalfspace(
    tiny_AdmmWorkspace* work,
    int k,
    int h,
    const Eigen::Vector3f* a_position,
    const Eigen::Vector3f* a_velocity,
    float b,
    float slack_penalty,
    int enable) {
  if (k < 0 || k >= work->data->model[0].nhorizon ||
      k >= TINY_MAX_HORIZON_KNOTS || h < 0 || h >= MAX_HS) {
    return TINY_NOT_SUPPORTED;
  }
  if (!enable || a_position == 0 || a_velocity == 0) {
    work->data->a_pos_hs[k][h].setZero();
    work->data->a_vel_hs[k][h].setZero();
    work->data->b_hs[k][h] = 0.0f;
    work->data->slack_penalty_hs[k][h] = 0.0f;
    work->data->slack_used_hs[k][h] = 0.0f;
    work->data->en_hs[k][h] = 0;
    return TINY_NO_ERROR;
  }

  const float norm = sqrtf(
      a_position->squaredNorm() + a_velocity->squaredNorm());
  if (norm < 1e-6f) {
    work->data->a_pos_hs[k][h].setZero();
    work->data->a_vel_hs[k][h].setZero();
    work->data->b_hs[k][h] = 0.0f;
    work->data->slack_penalty_hs[k][h] = 0.0f;
    work->data->slack_used_hs[k][h] = 0.0f;
    work->data->en_hs[k][h] = 0;
    return TINY_NO_ERROR;
  }

  work->data->a_pos_hs[k][h] = (*a_position) / norm;
  work->data->a_vel_hs[k][h] = (*a_velocity) / norm;
  work->data->b_hs[k][h] = b / norm;
  work->data->slack_penalty_hs[k][h] =
      slack_penalty > 0.0f ? slack_penalty : 0.0f;
  work->data->slack_used_hs[k][h] = 0.0f;
  work->data->en_hs[k][h] = 1;
  return TINY_NO_ERROR;
}

// enum tiny_ErrorCode tiny_ProjectInput(tiny_AdmmWorkspace* work) {
//   int n = work->data->model[0].ninputs;
//   int N = work->data->model[0].ninputs;

//   for (int k = 0; k < N - 1; ++k) {
//     for (int i = 0; i < n; ++i) {

//       work->ZU_new[k].data[i] = T_MIN(T_MAX(z[i],
//                                 work->data->lcu[i]),  // Between lower
//                                 work->data->ucu[i]);  // and upper bounds
//     } 
//   }
//   return TINY_NO_ERROR;
// }

int IsConstrained(tiny_AdmmWorkspace* work) {
  if (!work->stgs->en_cstr_goal && 
      !work->stgs->en_cstr_inputs && 
      !work->stgs->en_cstr_states) {
    return 0; // unconstrained
  }
  return 1;    
}

#ifdef __cplusplus
}
#endif
