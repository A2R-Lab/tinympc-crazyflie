#include "cost_lqr.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

enum tiny_ErrorCode tiny_AddStageCost(tiny_AdmmWorkspace* work, const int k) {
  work->info->obj_val += (0.5 * (work->soln->X[k] - work->data->Xref[k]).transpose() * 
                         (*(work->data->Q)) * (work->soln->X[k] - work->data->Xref[k]) +
                         0.5 * (work->soln->U[k] - work->data->Uref[k]).transpose() * 
                         (*(work->data->R)) * (work->soln->U[k] - work->data->Uref[k])).value();
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_AddTerminalCost(tiny_AdmmWorkspace* work) {
  int N = work->data->model[0].nhorizon;
  if (work->stgs->adaptive_horizon > 0) {
    work->info->obj_val += 0.5 * (work->soln->X[N-1] - work->data->Xref[N-1]).transpose() * 
                          (*(work->soln->Pinf_s)) * (work->soln->X[N-1] - work->data->Xref[N-1]);
  }
  else {
    work->info->obj_val += 0.5 * (work->soln->X[N-1] - work->data->Xref[N-1]).transpose() * 
                          (*(work->soln->Pinf)) * (work->soln->X[N-1] - work->data->Xref[N-1]);
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_UpdateLinearCost(tiny_AdmmWorkspace* work) {
  int N = work->data->model[0].nhorizon;
  for (int k = 0; k < N - 1; ++k) {
    /* Compute q[k] = -Q*Xref[k] */  
    (work->data->q[k]).noalias() = -(*(work->data->Q)).lazyProduct(work->data->Xref[k]);

    /* Compute r[k] = -R*Uref[k] */ 
    (work->data->r[k]).noalias() = -(*(work->data->R)).lazyProduct(work->data->Uref[k]);
  }
  /* Compute q[N-1] = -Pinf*Xref[N-1] */ 
  if (work->stgs->adaptive_horizon > 0) {
    (work->soln->p[N-1]).noalias() = -(*(work->soln->Pinf_s)).lazyProduct(work->data->Xref[N-1]);
  }
  else {
    (work->soln->p[N-1]).noalias() = -(*(work->soln->Pinf)).lazyProduct(work->data->Xref[N-1]);
  }
  // Pinf is the augmented terminal Hessian used by the cached recursion.
  // The reference belongs to the unaugmented terminal objective.
  if (work->stgs->en_cstr_states) {
    if (work->data->state_constraint_weights) {
      work->soln->p[N-1] += work->rho * work->data->state_constraint_weights->cwiseProduct(work->data->Xref[N-1]);
    } else {
      work->soln->p[N-1] += work->rho * work->data->Xref[N-1];
    }
  }
  if (work->data->q_base && work->data->terminal_base) {
    for (int k = 0; k < N - 1; ++k)
      work->data->q_base[k] = work->data->q[k];
    *work->data->terminal_base = work->soln->p[N-1];
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_UpdateConstrainedLinearCost(tiny_AdmmWorkspace* work) {
  int N = work->data->model[0].nhorizon;
  // Rebuild from the immutable objective, never accumulate old penalties.
  // The optional buffers are populated by tiny_UpdateLinearCost at solve entry.
  if (work->data->q_base && work->data->terminal_base) {
    for (int k = 0; k < N - 1; ++k)
      work->data->q[k] = work->data->q_base[k];
    work->soln->p[N-1] = *work->data->terminal_base;
  } else {
    tiny_UpdateLinearCost(work);
  }
  if (work->stgs->en_cstr_inputs) {
    for (int k = 0; k < N - 1; ++k) {
      /* Compute r_tilde[k] = r[k] - ρ*(z[k]-y[k]) */ 
      work->data->r_tilde[k] = work->data->r[k] - work->rho * (work->ZU_new[k] - work->soln->YU[k]);
    }
  }
  if (work->stgs->en_cstr_states) {
    for (int k = 0; k < N - 1; ++k) {
      /* Add state constraint term to q[k]: q[k] += -ρ*(zx[k]-yx[k]) */ 
      const Eigen::VectorNf difference = work->ZX_new[k] - work->soln->YX[k];
      if (work->data->state_constraint_weights)
        work->data->q[k] -= work->rho * work->data->state_constraint_weights->cwiseProduct(difference);
      else
        work->data->q[k] -= work->rho * difference;
    }
    /* Terminal state constraint */
    const Eigen::VectorNf difference = work->ZX_new[N-1] - work->soln->YX[N-1];
    if (work->data->state_constraint_weights)
      work->soln->p[N-1] -= work->rho * work->data->state_constraint_weights->cwiseProduct(difference);
    else
      work->soln->p[N-1] -= work->rho * difference;
  }
  return TINY_NO_ERROR;
}

# ifdef __cplusplus
}
# endif // ifdef __cplusplus
