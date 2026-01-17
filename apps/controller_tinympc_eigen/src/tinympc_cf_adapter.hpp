#pragma once

// Crazyflie Eigen wrapper
#include <Eigen.h>
using namespace Eigen;

// You will include the generated header that defines TinySolver and tiny_solver.
// Example names; adjust to your generated file names.
#ifdef __cplusplus
#include "tinympc/tiny_data.hpp"   // should provide: extern TinySolver tiny_solver;
#include "tinympc/tiny_api.hpp"    // should provide: int tiny_solve(TinySolver*);
#endif

// ---------- Adapter configuration ----------
// Choose what you feed to the old fast loop.
// Many ADMM MPC formulations use z as the projected / constraint-satisfying control.
// If your old controller expects ZU_new, you likely want znew.
// If your old controller expects primal u, choose U.
enum class TinympcControlOutputKind {
  ZNEW,   // projected/control-satisfying variable (common for actuation)
  UPRIMAL // primal control trajectory
};

// ---------- Minimal adapter API ----------
// These functions are meant to be called from your controller at runtime.
//
// Typical flow per MPC tick:
//   tinympc_cf_set_x0(x0);
//   tinympc_cf_set_reference(Xref, Uref);
//   tinympc_cf_set_settings(2, 0);   // 2 iters, termination off
//   tinympc_cf_solve();
//   tinympc_cf_get_x_pred_1(x1);
//   tinympc_cf_get_u0(u0, TinympcControlOutputKind::ZNEW);

#ifdef __cplusplus
extern "C" {
#endif

// Optional: call once at boot to set embedded-friendly defaults.
// Safe to call multiple times.
void tinympc_cf_init_defaults(void);

// Set ADMM iteration count and termination check
void tinympc_cf_set_settings(int max_iter, int check_termination);

// Set initial state x0 (size NSTATES x 1)
void tinympc_cf_set_x0(const float* x0, int nstates);

// Set horizon references.
// Xref: NHORIZON states, each length NSTATES.
// Uref: NHORIZON-1 inputs, each length NINPUTS.
void tinympc_cf_set_reference(const float* Xref, int nstates, int nhorizon,
                              const float* Uref, int ninputs);

// Solve one MPC step
int tinympc_cf_solve(void);

// Get predicted state at step k=1 (old controller uses Xhrz[1]).
// out_x1 length NSTATES.
void tinympc_cf_get_x_pred_1(float* out_x1, int nstates);

// Get first control action (u0 or z0 depending on kind).
// out_u0 length NINPUTS.
void tinympc_cf_get_u0(float* out_u0, int ninputs, TinympcControlOutputKind kind);

// ========== PSD (Obstacle Avoidance) API ==========

// Enable PSD constraints for obstacle avoidance.
// nx0: base state dimension (2 for XY, 3 for XYZ)
// nu0: base input dimension (0 if using position-only PSD)
// rho_psd: penalty parameter (typically 1.0 - 10.0)
// Returns 0 on success.
int tinympc_cf_enable_psd(int nx0, int nu0, float rho_psd);

// Set disk obstacles in lifted space. Each disk is (cx, cy, r).
// disks points to 3*count floats.
int tinympc_cf_set_psd_disks(const float* disks, int count);

// Clear PSD disk constraints (disables lifted disk linear constraints).
void tinympc_cf_clear_psd_disks(void);

#ifdef __cplusplus
}
#endif
