#ifndef __TINYRACER_DEBUG_H__
#define __TINYRACER_DEBUG_H__

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t magic, stage, phase, cycle, tick;
} MpcRetainedTrace;
extern volatile MpcRetainedTrace mpc_retained;

// Retained through disarm/controller reset until the next accepted line run;
// microseconds, except sample count. Not retained through MCU reset/power loss.
typedef struct {
  uint32_t samples;
  uint32_t first_prep, first_solve, first_publish, first_worker;
  uint32_t max_prep, max_solve, max_publish, max_worker;
} TinyMpcLineTiming;

typedef struct {
  TinyMpcLineTiming line_timing;
  float ref_x, ref_y, ref_z;
  uint8_t direct_active, fallback_guard, fallback_reason;
  uint32_t plan_age_ms, fallback_age_ms, direct_duration_ms;
  uint32_t snapshot_queue_us, preparation_us, publish_age_us;
  uint32_t ready_plan_age_ms, held_plan_max_ms;
  uint32_t lqr_us, lqr_cycles, lqr_gap_max_ms;
  uint8_t lqr_saturation, lqr_valid;
  float admm_state_gap, admm_input_gap_n;
  uint8_t line_phase, line_reason, line_settled;
  uint8_t line_catch;
  uint8_t line_vision_fresh, line_vision_stop;
  uint16_t line_model_id, line_feedback_model;
  uint8_t line_model_valid, line_feedback_local, line_solver_local;
  float line_pitch_deg, line_body_rate;
  float line_att_roll, line_att_pitch, line_att_q;
  float line_att_ref_pitch, line_att_ref_q, line_att_nom_pitch, line_att_nom_q;
  float line_motor_raw_n[4], line_motor_clip_n[4];
  float line_pitch_torque_raw,line_pitch_torque_applied;
  float line_nominal_q_max;
  float line_qdot, line_alpha_cmd, line_alpha_out, line_tau_out, line_output_gap_n;
  uint32_t line_qdot_dt_ms;
  uint8_t line_pitch_limited;
  uint8_t line_motor_high, line_motor_low, line_att_valid, line_recovery_damping;
  float line_distance, line_speed, line_ref_s, line_ref_v, line_stop_s;
  float horizon_x, horizon_y, horizon_z;
  float cylinder_x, cylinder_y, cylinder_radius;
  float plane_nx, plane_ny, plane_boundary;
  float lateral_offset, plane_violation, consensus_error, slack;
  float square_opening_probability;
  uint32_t solve_us;
  uint32_t worker_us, worker_stack_words, worker_cycles, worker_overruns;
  uint8_t pid_fallback;
  uint8_t rate_active;
  uint8_t altitude_pid_active;
  float altitude_target_z, altitude_thrust;
  uint8_t maneuver_mode;
  uint32_t maneuver_caps;
  float rate_collective, rate_p, rate_q, rate_r;
  uint32_t rate_plan_age_ms;
  uint32_t olgmd_trigger_count, olgmd_positive_trigger_count;
  uint32_t olgmd_stale_trigger_count, olgmd_release_count;
  uint8_t mode, cylinder_active;
  uint8_t square_opening_eligible, square_opening_seen;
  uint8_t olgmd_fresh, olgmd_detected, olgmd_trigger_eligible;
  uint8_t olgmd_brake_latched;
  uint8_t olgmd_terminal_stop, olgmd_phase;
  int8_t pass_side;
} TinyRacerDebugTelemetry;

extern TinyRacerDebugTelemetry tinyRacerDebug;

#endif
