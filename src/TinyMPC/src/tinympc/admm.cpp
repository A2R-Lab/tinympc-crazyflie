#ifndef TINYMPC_NO_IOSTREAM
#include <iostream>
#endif

#include "admm.hpp"
#include "psd_support.hpp"
#include <cmath>

#define DEBUG_MODULE "TINYALG"

namespace tinympc {

// Project onto half-space { z | a^T z <= b }
static inline tinyVector project_halfspace_leq(const tinyVector& z,
                                               const tinyVector& a,
                                               tinytype b) {
    tinytype anorm2 = a.squaredNorm();
    if (anorm2 <= tinytype(1e-12)) return z; // ill-posed row; skip
    tinytype val = a.dot(z);
    if (val <= b) return z; // already feasible
    tinytype step = (val - b) / anorm2;
    return z - step * a;
}

/**
    * Update linear terms from Riccati backward pass
    */
void backward_pass_grad(TinySolver *solver)
{
    for (int i = solver->work->N - 2; i >= 0; i--)
    {
        (solver->work->d.col(i)).noalias() = solver->cache->Quu_inv * (solver->work->Bdyn.transpose() * solver->work->p.col(i + 1) + solver->work->r.col(i));
        (solver->work->p.col(i)).noalias() = solver->work->q.col(i) + solver->cache->AmBKt.lazyProduct(solver->work->p.col(i + 1)) - (solver->cache->Kinf.transpose()).lazyProduct(solver->work->r.col(i)); 
    }
}

/**
    * Use LQR feedback policy to roll out trajectory
    */
void forward_pass(TinySolver *solver)
{
    for (int i = 0; i < solver->work->N - 1; i++)
    {
        (solver->work->u.col(i)).noalias() = -solver->cache->Kinf.lazyProduct(solver->work->x.col(i)) - solver->work->d.col(i);
        (solver->work->x.col(i + 1)).noalias() = solver->work->Adyn.lazyProduct(solver->work->x.col(i)) + solver->work->Bdyn.lazyProduct(solver->work->u.col(i));
    }
}

/**
    * Project slack (auxiliary) variables into their feasible domain, defined by
    * projection functions related to each constraint
    */
void update_slack(TinySolver *solver)
{
    solver->work->znew = solver->work->u + solver->work->y;
    solver->work->vnew = solver->work->x + solver->work->g;

    // Box constraints on input
    if (solver->settings->en_input_bound)
    {
        solver->work->znew = solver->work->u_max.cwiseMin(solver->work->u_min.cwiseMax(solver->work->znew));
    }

    // Box constraints on state
    if (solver->settings->en_state_bound)
    {
        solver->work->vnew = solver->work->x_max.cwiseMin(solver->work->x_min.cwiseMax(solver->work->vnew));
    }
    
    // Linear constraints on state (halfspace projection)
    if (solver->settings->en_state_linear && solver->work->numStateLinear > 0)
    {
        // Update slack variable for linear constraints
        solver->work->vlnew = solver->work->x + solver->work->gl;
        
        // Project each column onto each halfspace constraint
        for (int i = 0; i < solver->work->N; i++) {
            for (int k = 0; k < solver->work->numStateLinear; k++) {
                tinyVector a = solver->work->Alin_x.row(k).transpose();
                tinytype b = solver->work->blin_x(k);
                solver->work->vlnew.col(i) = project_halfspace_leq(solver->work->vlnew.col(i), a, b);
            }
        }
    }
    
    // PSD constraints (obstacle avoidance)
    if (solver->settings->en_psd)
    {
        tiny_update_slack_psd(solver);
    }
}

/**
    * Update next iteration of dual variables by performing the augmented
    * lagrangian multiplier update
    */
void update_dual(TinySolver *solver)
{
    solver->work->y = solver->work->y + solver->work->u - solver->work->znew;
    solver->work->g = solver->work->g + solver->work->x - solver->work->vnew;
    
    // Linear constraint dual update
    if (solver->settings->en_state_linear && solver->work->numStateLinear > 0)
    {
        solver->work->gl = solver->work->gl + solver->work->x - solver->work->vlnew;
    }
    
    // PSD dual update (obstacle avoidance)
    if (solver->settings->en_psd)
    {
        tiny_update_dual_psd(solver);
    }
}

/**
    * Update linear control cost terms in the Riccati feedback using the changing
    * slack and dual variables from ADMM
    */
void update_linear_cost(TinySolver *solver)
{
    solver->work->r = -(solver->work->Uref.array().colwise() * solver->work->R.array());
    (solver->work->r).noalias() -= solver->cache->rho * (solver->work->znew - solver->work->y);
    
    solver->work->q = -(solver->work->Xref.array().colwise() * solver->work->Q.array());
    (solver->work->q).noalias() -= solver->cache->rho * (solver->work->vnew - solver->work->g);
    
    // Add linear constraint contribution to q
    if (solver->settings->en_state_linear && solver->work->numStateLinear > 0)
    {
        (solver->work->q).noalias() -= solver->cache->rho * (solver->work->vlnew - solver->work->gl);
    }
    
    solver->work->p.col(solver->work->N - 1) = -(solver->work->Xref.col(solver->work->N - 1).transpose().lazyProduct(solver->cache->Pinf));
    (solver->work->p.col(solver->work->N - 1)).noalias() -= solver->cache->rho * (solver->work->vnew.col(solver->work->N - 1) - solver->work->g.col(solver->work->N - 1));
    
    // Add linear constraint contribution to terminal cost
    if (solver->settings->en_state_linear && solver->work->numStateLinear > 0)
    {
        solver->work->p.col(solver->work->N - 1) -= solver->cache->rho * (solver->work->vlnew.col(solver->work->N - 1) - solver->work->gl.col(solver->work->N - 1));
    }
    
    // Add PSD contribution to linear cost (obstacle repulsion)
    if (solver->settings->en_psd)
    {
        tiny_add_psd_to_linear_cost(solver);
    }
}

/**
    * Check for termination condition by evaluating whether the largest absolute
    * primal and dual residuals for states and inputs are below threhold.
    */
bool termination_condition(TinySolver *solver)
{
    if (solver->work->iter % solver->settings->check_termination == 0)
    {
        solver->work->primal_residual_state = (solver->work->x - solver->work->vnew).cwiseAbs().maxCoeff();
        solver->work->dual_residual_state = ((solver->work->v - solver->work->vnew).cwiseAbs().maxCoeff()) * solver->cache->rho;
        solver->work->primal_residual_input = (solver->work->u - solver->work->znew).cwiseAbs().maxCoeff();
        solver->work->dual_residual_input = ((solver->work->z - solver->work->znew).cwiseAbs().maxCoeff()) * solver->cache->rho;

        if (solver->work->primal_residual_state < solver->settings->abs_pri_tol &&
            solver->work->primal_residual_input < solver->settings->abs_pri_tol &&
            solver->work->dual_residual_state < solver->settings->abs_dua_tol &&
            solver->work->dual_residual_input < solver->settings->abs_dua_tol)
        {
            return true;
        }
    }
    return false;
}

/**
    * Solve the MPC problem
    */
int solve(TinySolver *solver)
{
    // Initialize variables
    solver->solution->solved = 0;
    solver->solution->iter = 0;
    solver->work->status = 11; // TINY_UNSOLVED
    solver->work->iter = 0;
    
    // Initialize linear constraint slack if enabled
    if (solver->settings->en_state_linear && solver->work->numStateLinear > 0)
    {
        solver->work->vlnew = solver->work->x;
    }

    for (int i = 0; i < solver->settings->max_iter; i++)
    {
        // Solve linear system with Riccati and roll out to get new trajectory
        forward_pass(solver);

        // Project slack variables into feasible domain
        update_slack(solver);

        // Compute next iteration of dual variables
        update_dual(solver);

        // Update linear control cost terms using reference trajectory, duals, and slack variables
        update_linear_cost(solver);

        solver->work->iter += 1;

        // Check for whether cost is minimized by calculating residuals
        if (termination_condition(solver)) {
            solver->work->status = 1; // TINY_SOLVED
            solver->solution->solved = 1;
            solver->solution->iter = solver->work->iter;
            solver->solution->x = solver->work->vnew;
            solver->solution->u = solver->work->znew;
            return 0;
        }

        // Save previous slack variables
        solver->work->v = solver->work->vnew;
        solver->work->z = solver->work->znew;

        backward_pass_grad(solver);

    }
    solver->solution->iter = solver->work->iter;
    solver->solution->solved = 0;
    solver->solution->x = solver->work->vnew;
    solver->solution->u = solver->work->znew;
    return 1;
}

} // namespace tinympc
