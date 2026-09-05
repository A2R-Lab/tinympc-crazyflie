#ifndef TINYMPC_BRUSHLESS_ACTUATOR_H
#define TINYMPC_BRUSHLESS_ACTUATOR_H
#include <math.h>

/* Graefe et al., arXiv:2603.05944, Eq.9 and accompanying BrushJAX
 * environment/quadcopter.py. One motor, N; normalized steady rotor speed
 * u=omega/2900. Published fit, NOT calibration of this individual aircraft.
 * The negative low-speed polynomial tail is clipped to zero physical thrust.
 * No battery compensation is implied by this fit. */
#define TINYMPC_BRUSHLESS_TAU_S 0.05f
#define TINYMPC_BRUSHLESS_K_RAD_S 2900.0f
#define TINYMPC_BRUSHLESS_MASS_KG 0.043f
#define TINYMPC_BRUSHLESS_MAX_THRUST_N 0.28835022f

static inline float tinyMpcBrushlessThrust(float u) {
  if (!isfinite(u)) return NAN;
  u = fminf(fmaxf(u, 0.0f), 1.0f);
  return fmaxf(((-0.23009526f * u + 0.56176458f) * u - 0.0433191f) * u, 0.0f);
}

static inline float tinyMpcBrushlessCommand(float thrust_n) {
  if (!isfinite(thrust_n)) return NAN;
  if (thrust_n <= 0.0f) return 0.0f;
  if (thrust_n >= TINYMPC_BRUSHLESS_MAX_THRUST_N) return 1.0f;
  float low = 0.0f, high = 1.0f;
  // Bounded work, monotonic physical branch; sub-uint16 command accuracy.
  for (unsigned i = 0; i < 20; ++i) {
    const float mid = 0.5f * (low + high);
    if (tinyMpcBrushlessThrust(mid) < thrust_n) low = mid;
    else high = mid;
  }
  return 0.5f * (low + high);
}

/* Algebraic command -> thrust -> inverse shortcut. Preserve the zero-thrust
 * dead zone and optional physical-thrust cap of the original round trip. */
static inline float tinyMpcBrushlessCommandToRotor(float command, float command_cap) {
  if (!isfinite(command) || !isfinite(command_cap)) return NAN;
  command = fminf(fmaxf(command, 0.0f), 1.0f);
  if (tinyMpcBrushlessThrust(command) <= 0.0f) return 0.0f;
  return fminf(command, command_cap);
}

static inline float tinyMpcBrushlessRotorStep(float rotor_u, float command, float dt_s) {
  if (!isfinite(rotor_u) || !isfinite(command) || !isfinite(dt_s) || dt_s < 0.0f) return NAN;
  command = fminf(fmaxf(command, 0.0f), 1.0f);
  rotor_u = fminf(fmaxf(rotor_u, 0.0f), 1.0f);
  return command + (rotor_u - command) * expf(-dt_s / TINYMPC_BRUSHLESS_TAU_S);
}
#endif
