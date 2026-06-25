#include "gate_tinympc_host.h"

#include "tinympc/tinympc.h"

#include <Eigen.h>
#include <math.h>
#include <string.h>

using namespace Eigen;

namespace {

#define HOST_DT 0.02f

static MatrixNf A;
static MatrixNMf B;
static MatrixMNf Kinf;
static MatrixNf Pinf;
static MatrixMf Quu_inv;
static MatrixNf AmBKt;
static MatrixNMf coeff_d2p;
static MatrixNf Q;
static MatrixMf R;

static VectorNf Xhrz[NHORIZON];
static VectorMf Uhrz[NHORIZON - 1];
static VectorMf d[NHORIZON - 1];
static VectorNf p[NHORIZON];
static VectorMf YU[NHORIZON];
static VectorNf q[NHORIZON - 1];
static VectorMf r[NHORIZON - 1];
static VectorMf r_tilde[NHORIZON - 1];
static VectorNf Xref[NHORIZON];
static VectorMf Uref[NHORIZON - 1];
static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;
static MatrixNf Acx;
static VectorNf ucx;
static VectorNf lcx;
static VectorMf Qu;
static VectorMf ZU[NHORIZON - 1];
static VectorMf ZU_new[NHORIZON - 1];
static VectorNf YX[NHORIZON];
static VectorNf ZX[NHORIZON];
static VectorNf ZX_new[NHORIZON];
static VectorNf x0;

static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

static bool initialized = false;
static const float u_hover[4] = {0.7f, 0.663f, 0.7373f, 0.633f};

static void zeroArrays(void) {
  for (int k = 0; k < NHORIZON; ++k) {
    Xhrz[k].setZero();
    YX[k].setZero();
    ZX[k].setZero();
    ZX_new[k].setZero();
    p[k].setZero();
    Xref[k].setZero();
    if (k < NHORIZON - 1) {
      Uhrz[k].setZero();
      d[k].setZero();
      YU[k].setZero();
      q[k].setZero();
      r[k].setZero();
      r_tilde[k].setZero();
      Uref[k].setZero();
      ZU[k].setZero();
      ZU_new[k].setZero();
    }
  }
  x0.setZero();
  Qu.setZero();
  Acu.setIdentity();
  Acx.setIdentity();
}

static Vector3f gateCornerPoint(const GateTinyMpcReference* gate_ref, int index) {
  const int base = 3 * index;
  return Vector3f(
      gate_ref->gate_corners_m[base + 0],
      gate_ref->gate_corners_m[base + 1],
      gate_ref->gate_corners_m[base + 2]);
}

static bool normalizeVector(Vector3f* value) {
  const float norm = value->norm();
  if (norm <= 1e-6f) {
    return false;
  }
  *value = *value / norm;
  return true;
}

static Vector3f crossVector(const Vector3f& lhs, const Vector3f& rhs) {
  return Vector3f(
      lhs(1) * rhs(2) - lhs(2) * rhs(1),
      lhs(2) * rhs(0) - lhs(0) * rhs(2),
      lhs(0) * rhs(1) - lhs(1) * rhs(0));
}

static void clearGateTvConstraints(void) {
  if (work.data == 0) {
    return;
  }
  for (int k = 0; k < NHORIZON; ++k) {
    work.data->num_hs[k] = 0;
  }
}

static void configureGateTvConstraints(const GateTinyMpcReference* gate_ref) {
  clearGateTvConstraints();
  if (gate_ref == 0 || !gate_ref->has_corners) {
    return;
  }

  const Vector3f c0 = gateCornerPoint(gate_ref, 0);
  const Vector3f c1 = gateCornerPoint(gate_ref, 1);
  const Vector3f c2 = gateCornerPoint(gate_ref, 2);
  const Vector3f c3 = gateCornerPoint(gate_ref, 3);
  const Vector3f center = 0.25f * (c0 + c1 + c2 + c3);
  Vector3f u = 0.5f * ((c1 - c0) + (c2 - c3));
  Vector3f v = 0.5f * ((c3 - c0) + (c2 - c1));
  if (!normalizeVector(&u) || !normalizeVector(&v)) {
    return;
  }
  Vector3f normal = crossVector(u, v);
  if (!normalizeVector(&normal)) {
    return;
  }

  const float safety_margin_m = 0.03f;
  const float half_width = fmaxf(
      0.05f,
      0.25f * ((c1 - c0).norm() + (c2 - c3).norm()) - safety_margin_m);
  const float half_height = fmaxf(
      0.05f,
      0.25f * ((c3 - c0).norm() + (c2 - c1).norm()) - safety_margin_m);
  const float slab_half_depth_m = 0.45f;

  const Vector3f axes[4] = {u, -u, v, -v};
  const float bounds[4] = {
      u.dot(center) + half_width,
      (-u).dot(center) + half_width,
      v.dot(center) + half_height,
      (-v).dot(center) + half_height,
  };

  for (int k = 1; k < NHORIZON; ++k) {
    const Vector3f p_ref = Xref[k].head(3);
    if (fabsf(normal.dot(p_ref - center)) > slab_half_depth_m) {
      continue;
    }
    work.data->num_hs[k] = TINY_MAX_STATE_HALFSPACES;
    for (int h = 0; h < TINY_MAX_STATE_HALFSPACES; ++h) {
      work.data->a_hs[k][h] = axes[h];
      work.data->b_hs[k][h] = bounds[h];
    }
  }
}

static bool controllerTinyMpcGateSolve(
    const DroneState* gate_state,
    const GateTinyMpcReference* gate_ref,
    const GateControllerConfig* gate_config,
    MotorCommand* command,
    GateControllerDebug* debug) {
  if (gate_state == 0 || gate_ref == 0 || gate_config == 0 || command == 0 || debug == 0) {
    return false;
  }

  x0.setZero();
  x0(0) = gate_state->x;
  x0(1) = gate_state->y;
  x0(2) = gate_state->z;
  x0(6) = gate_state->vx;
  x0(7) = gate_state->vy;
  x0(8) = gate_state->vz;
  x0(9) = gate_state->wx;
  x0(10) = gate_state->wy;
  x0(11) = gate_state->wz;
  const float qw = fabsf(gate_state->qw) > 1e-6f ? gate_state->qw : 1.0f;
  x0(3) = gate_state->qx / qw;
  x0(4) = gate_state->qy / qw;
  x0(5) = gate_state->qz / qw;

  const float speed = gate_ref->target_speed_mps;
  for (int i = 0; i < NHORIZON; ++i) {
    const float t = HOST_DT * static_cast<float>(i);
    const float desired_x = fminf(
        fmaxf(gate_state->x + speed * t, gate_state->x),
        gate_config->gate_x + 0.85f);
    const float progress = fminf(1.0f, fmaxf(0.0f, desired_x / fmaxf(1e-3f, gate_config->gate_x)));
    Xref[i].setZero();
    Xref[i](0) = desired_x;
    Xref[i](1) = gate_state->y + progress * (gate_ref->gate_pose_m[1] - gate_state->y);
    Xref[i](2) = gate_state->z + progress * (gate_ref->gate_pose_m[2] - gate_state->z);
    Xref[i](6) = speed;
    if (i < NHORIZON - 1) {
      Uref[i].setZero();
    }
  }

  configureGateTvConstraints(gate_ref);
  tiny_SetInitialState(&work, &x0);
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  tiny_UpdateLinearCost(&work);
  tiny_SolveAdmm(&work);

  command->motor_delta[0] = ZU_new[0](0);
  command->motor_delta[1] = ZU_new[0](1);
  command->motor_delta[2] = ZU_new[0](2);
  command->motor_delta[3] = ZU_new[0](3);
  command->solver_iterations = info.iter;
  command->solver_success = info.status_val >= 0;
  debug->solver_iterations = info.iter;
  debug->solver_success = command->solver_success;
  return command->solver_success;
}

}  // namespace

extern "C" bool gate_tinympc_host_init(void) {
  if (initialized) {
    return true;
  }

  zeroArrays();
#include "params_100hz.h"

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, HOST_DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 250.0f;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  work.rho = stgs.rho_init;
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, ZX, ZX_new);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, YX, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);

  ucu << 1.0f - u_hover[0], 1.0f - u_hover[1], 1.0f - u_hover[2], 1.0f - u_hover[3];
  lcu << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);
  ucx.setConstant(100.0f);
  lcx.setConstant(-100.0f);
  tiny_SetStateBound(&work, &Acx, &lcx, &ucx);
  clearGateTvConstraints();
  tiny_UpdateLinearCost(&work);

  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 1;
  stgs.max_iter = 8;
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 5e-2f;
  stgs.tol_abs_prim = 5e-2f;

  gate_tinympc_reset();
  gate_tinympc_set_solver(controllerTinyMpcGateSolve);
  initialized = true;
  return true;
}

extern "C" void gate_tinympc_host_reset(void) {
  gate_tinympc_reset();
}

extern "C" MotorCommand gate_tinympc_host_step(
    const DroneState* state,
    const GateVisionPacket* vision,
    const GateControllerConfig* config,
    float dt) {
  if (!initialized) {
    gate_tinympc_host_init();
  }
  return gate_tinympc_step(state, vision, config, dt);
}

extern "C" const GateControllerDebug* gate_tinympc_host_last_debug(void) {
  return gate_tinympc_last_debug();
}
