#ifndef GATE_TINYMPC_CORE_H
#define GATE_TINYMPC_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum GateControlMode {
  GATE_CONTROL_ORACLE_TINYMPC = 0,
  GATE_CONTROL_CAMERA_ASSISTED = 1,
  GATE_CONTROL_TERMINAL_COMMIT = 2,
  GATE_CONTROL_PRECOMMIT_ALIGN = 3,
} GateControlMode;

typedef enum GateControlSource {
  GATE_SOURCE_ORACLE_MARGIN = 0,
  GATE_SOURCE_CAMERA_VALID = 1,
  GATE_SOURCE_CAMERA_PROPAGATED = 2,
  GATE_SOURCE_CAMERA_INVALID = 3,
  GATE_SOURCE_PRECOMMIT_ALIGN = 4,
  GATE_SOURCE_TERMINAL_GATE_COMMIT = 5,
  GATE_SOURCE_FALLBACK_SLOW = 6,
} GateControlSource;

typedef enum GateInvalidReason {
  GATE_INVALID_NONE = 0,
  GATE_INVALID_GATE_CLIPPED_OUT_OF_FRAME = 1,
  GATE_INVALID_BBOX_TOO_LARGE = 2,
  GATE_INVALID_LOW_CONTRAST = 3,
  GATE_INVALID_NO_CONTOUR = 4,
  GATE_INVALID_GEOMETRY_FAILED = 5,
  GATE_INVALID_THRESHOLD_FAILED = 6,
  GATE_INVALID_UNKNOWN = 7,
} GateInvalidReason;

typedef struct DroneState {
  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  float qx;
  float qy;
  float qz;
  float qw;
  float wx;
  float wy;
  float wz;
} DroneState;

typedef struct GateVisionPacket {
  bool valid;
  bool raw_valid;
  GateControlSource source;
  GateInvalidReason invalid_reason;
  float margin;
  float sigma_m;
  float confidence;
  float lateral_error_m;
  float vertical_error_m;
  float y_min;
  float y_max;
  float z_min;
  float z_max;
  float margin_left;
  float margin_right;
  float margin_bottom;
  float margin_top;
  float distance_to_gate_m;
  bool has_bounds;
  bool has_distance;
  bool projection_gate_clipped;
} GateVisionPacket;

typedef struct GateControllerConfig {
  GateControlMode mode;
  float gate_x;
  float gate_y;
  float gate_z;
  float safe_half_width;
  float safe_half_height;
  float target_speed;
  float invalid_packet_speed;
  float terminal_commit_speed;
  float terminal_commit_max_age_s;
  float terminal_commit_distance_m;
  float terminal_commit_min_margin_m;
  float terminal_commit_center_tolerance_m;
  float terminal_commit_bound_shrink_m;
  float precommit_align_speed;
  float precommit_aligned_approach_speed;
  float precommit_align_max_age_s;
  float precommit_align_timeout_s;
  float precommit_align_min_improvement_m;
  float precommit_align_max_sigma_m;
  float precommit_align_bound_shrink_m;
} GateControllerConfig;

typedef struct GateControllerDebug {
  GateControlSource control_source;
  GateControlSource raw_packet_source;
  GateInvalidReason fallback_reason;
  bool solver_success;
  int32_t solver_iterations;
  bool commit_allowed;
  float estimator_age_s;
  float distance_to_gate_m;
  float y_error_m;
  float z_error_m;
  float commit_entry_time_s;
  float distance_to_gate_at_commit;
  float y_error_at_commit;
  float z_error_at_commit;
  float last_min_margin_at_commit;
  float precommit_align_entry_time_s;
  float precommit_align_entry_distance_to_gate_m;
  float precommit_align_entry_y_error_m;
  float precommit_align_entry_z_error_m;
  float precommit_align_entry_estimator_age_s;
  float precommit_align_best_error_norm_m;
} GateControllerDebug;

typedef struct MotorCommand {
  float motor_delta[4];
  GateControlSource control_source;
  bool solver_success;
  int32_t solver_iterations;
} MotorCommand;

typedef struct GateTinyMpcReference {
  bool valid;
  float target_speed_mps;
  float gate_pose_m[3];
  float y_min;
  float y_max;
  float z_min;
  float z_max;
  bool has_bounds;
  GateControlSource source;
} GateTinyMpcReference;

typedef bool (*GateTinyMpcSolveFn)(
    const DroneState* state,
    const GateTinyMpcReference* reference,
    const GateControllerConfig* config,
    MotorCommand* command,
    GateControllerDebug* debug);

void gate_tinympc_reset(void);
void gate_tinympc_set_solver(GateTinyMpcSolveFn solve_fn);
MotorCommand gate_tinympc_step(
    const DroneState* state,
    const GateVisionPacket* vision,
    const GateControllerConfig* config,
    float dt);
const GateControllerDebug* gate_tinympc_last_debug(void);

#ifdef __cplusplus
}
#endif

#endif
