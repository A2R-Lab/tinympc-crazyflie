#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t en_psd_avoidance;
extern uint8_t psd_obs_moving;  // 0=static obstacle, 1=moving obstacle
extern uint8_t psd_max_peers;
extern float psd_peer_radius;
extern float psd_peer_margin;
extern int32_t psd_peer_max_age_ms;
extern float psd_rho;

extern uint32_t psd_replan_stride;
extern uint32_t psd_horizon_guard;
extern float psd_base_on;
extern float psd_base_off;
extern float psd_goal_on_bias;
extern float psd_goal_off_bias;
extern float psd_off_hysteresis;

extern uint8_t psd_log_plan_mode;
extern uint8_t psd_log_plan_status;
extern int32_t psd_log_plan_age;
extern int32_t psd_log_disk_count;
extern uint8_t psd_log_certified;
extern float psd_log_trace_gap;
extern float psd_log_eta_min;

#ifdef __cplusplus
}
#endif
