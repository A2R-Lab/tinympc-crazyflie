#include "gate_tinympc_core.h"

#include <math.h>
#include <string.h>

namespace {

struct GateMemory {
  bool has_estimate;
  float last_time_s;
  float last_gate_center_y;
  float last_gate_center_z;
  float last_y_min;
  float last_y_max;
  float last_z_min;
  float last_z_max;
  float last_min_margin;
  float last_sigma;
  float last_confidence;
  bool has_corners;
  float last_gate_corners_m[12];
  bool commit_active;
  float commit_entry_time_s;
  float distance_to_gate_at_commit;
  float y_error_at_commit;
  float z_error_at_commit;
  float last_min_margin_at_commit;
  bool align_active;
  float align_entry_time_s;
  float align_entry_distance_to_gate_m;
  float align_entry_y_error_m;
  float align_entry_z_error_m;
  float align_entry_estimator_age_s;
  float align_best_error_norm_m;
  float align_last_improvement_time_s;
  GateInvalidReason align_fallback_reason;
};

struct EntryMetrics {
  bool allowed;
  bool has_estimate;
  bool aligned;
  float estimator_age_s;
  float distance_to_gate_m;
  float y_error_m;
  float z_error_m;
  float sigma_m;
  float last_min_margin_m;
};

GateMemory memory;
GateControllerDebug last_debug;
GateTinyMpcSolveFn solve_callback = 0;
float controller_time_s = 0.0f;

static bool finite_f(float value) {
  return isfinite(value) != 0;
}

static float distance_to_gate(
    const DroneState* state,
    const GateVisionPacket* vision,
    const GateControllerConfig* config) {
  if ((vision != 0) && vision->has_distance && finite_f(vision->distance_to_gate_m)) {
    return vision->distance_to_gate_m;
  }
  return config->gate_x - state->x;
}

static float min_packet_margin(const GateVisionPacket* vision) {
  float result = vision->margin;
  if (finite_f(vision->margin_left)) {
    result = fminf(result, vision->margin_left);
  }
  if (finite_f(vision->margin_right)) {
    result = fminf(result, vision->margin_right);
  }
  if (finite_f(vision->margin_bottom)) {
    result = fminf(result, vision->margin_bottom);
  }
  if (finite_f(vision->margin_top)) {
    result = fminf(result, vision->margin_top);
  }
  return result;
}

static void store_valid_observation(
    const DroneState* state,
    const GateVisionPacket* vision,
    const GateControllerConfig* config) {
  float y_min = config->gate_y - config->safe_half_width;
  float y_max = config->gate_y + config->safe_half_width;
  float z_min = config->gate_z - config->safe_half_height;
  float z_max = config->gate_z + config->safe_half_height;
  if (vision->has_bounds) {
    y_min = vision->y_min;
    y_max = vision->y_max;
    z_min = vision->z_min;
    z_max = vision->z_max;
  }
  memory.has_estimate = true;
  memory.last_time_s = controller_time_s;
  memory.last_gate_center_y = 0.5f * (y_min + y_max);
  memory.last_gate_center_z = 0.5f * (z_min + z_max);
  memory.last_y_min = y_min;
  memory.last_y_max = y_max;
  memory.last_z_min = z_min;
  memory.last_z_max = z_max;
  memory.last_min_margin = min_packet_margin(vision);
  memory.last_sigma = finite_f(vision->sigma_m) ? vision->sigma_m : 0.0f;
  memory.last_confidence = vision->confidence;
  memory.has_corners = vision->has_corners;
  if (vision->has_corners) {
    memcpy(memory.last_gate_corners_m, vision->gate_corners_m, sizeof(memory.last_gate_corners_m));
  }
  (void)state;
}

static EntryMetrics terminal_entry_metrics(
    const DroneState* state,
    const GateVisionPacket* vision,
    const GateControllerConfig* config,
    float max_age_s,
    float max_sigma_m) {
  EntryMetrics entry;
  memset(&entry, 0, sizeof(entry));
  entry.distance_to_gate_m = distance_to_gate(state, vision, config);
  if (!memory.has_estimate) {
    entry.estimator_age_s = INFINITY;
    entry.y_error_m = INFINITY;
    entry.z_error_m = INFINITY;
    entry.last_min_margin_m = -INFINITY;
    entry.sigma_m = INFINITY;
    return entry;
  }
  entry.has_estimate = true;
  entry.estimator_age_s = controller_time_s - memory.last_time_s;
  entry.y_error_m = state->y - memory.last_gate_center_y;
  entry.z_error_m = state->z - memory.last_gate_center_z;
  entry.sigma_m = memory.last_sigma;
  entry.last_min_margin_m = memory.last_min_margin;
  entry.aligned =
      fabsf(entry.y_error_m) <= config->terminal_commit_center_tolerance_m &&
      fabsf(entry.z_error_m) <= config->terminal_commit_center_tolerance_m;
  entry.allowed =
      entry.estimator_age_s <= max_age_s &&
      entry.distance_to_gate_m <= config->terminal_commit_distance_m &&
      entry.last_min_margin_m >= config->terminal_commit_min_margin_m &&
      entry.aligned &&
      entry.sigma_m <= max_sigma_m;
  return entry;
}

static GateTinyMpcReference reference_from_vision(
    const GateVisionPacket* vision,
    const GateControllerConfig* config) {
  GateTinyMpcReference ref;
  memset(&ref, 0, sizeof(ref));
  ref.valid = vision->valid;
  ref.source = vision->source;
  ref.target_speed_mps = vision->valid ? config->target_speed : config->invalid_packet_speed;
  ref.gate_pose_m[0] = config->gate_x;
  ref.gate_pose_m[1] = config->gate_y;
  ref.gate_pose_m[2] = config->gate_z;
  ref.has_bounds = vision->has_bounds;
  ref.y_min = vision->has_bounds ? vision->y_min : config->gate_y - config->safe_half_width;
  ref.y_max = vision->has_bounds ? vision->y_max : config->gate_y + config->safe_half_width;
  ref.z_min = vision->has_bounds ? vision->z_min : config->gate_z - config->safe_half_height;
  ref.z_max = vision->has_bounds ? vision->z_max : config->gate_z + config->safe_half_height;
  ref.has_corners = vision->has_corners;
  if (vision->has_corners) {
    memcpy(ref.gate_corners_m, vision->gate_corners_m, sizeof(ref.gate_corners_m));
    ref.gate_pose_m[0] = 0.25f * (
        vision->gate_corners_m[0] + vision->gate_corners_m[3] +
        vision->gate_corners_m[6] + vision->gate_corners_m[9]);
    ref.gate_pose_m[1] = 0.25f * (
        vision->gate_corners_m[1] + vision->gate_corners_m[4] +
        vision->gate_corners_m[7] + vision->gate_corners_m[10]);
    ref.gate_pose_m[2] = 0.25f * (
        vision->gate_corners_m[2] + vision->gate_corners_m[5] +
        vision->gate_corners_m[8] + vision->gate_corners_m[11]);
  }
  return ref;
}

static GateTinyMpcReference reference_from_memory(
    GateControlSource source,
    float shrink_m,
    float target_speed_mps,
    const GateControllerConfig* config) {
  GateTinyMpcReference ref;
  memset(&ref, 0, sizeof(ref));
  ref.valid = true;
  ref.source = source;
  ref.target_speed_mps = target_speed_mps;
  ref.y_min = memory.last_y_min + shrink_m;
  ref.y_max = memory.last_y_max - shrink_m;
  ref.z_min = memory.last_z_min + shrink_m;
  ref.z_max = memory.last_z_max - shrink_m;
  ref.has_bounds = true;
  ref.gate_pose_m[0] = config->gate_x;
  ref.gate_pose_m[1] = 0.5f * (ref.y_min + ref.y_max);
  ref.gate_pose_m[2] = 0.5f * (ref.z_min + ref.z_max);
  ref.has_corners = memory.has_corners;
  if (memory.has_corners) {
    memcpy(ref.gate_corners_m, memory.last_gate_corners_m, sizeof(ref.gate_corners_m));
    ref.gate_pose_m[0] = 0.25f * (
        ref.gate_corners_m[0] + ref.gate_corners_m[3] +
        ref.gate_corners_m[6] + ref.gate_corners_m[9]);
    ref.gate_pose_m[1] = 0.25f * (
        ref.gate_corners_m[1] + ref.gate_corners_m[4] +
        ref.gate_corners_m[7] + ref.gate_corners_m[10]);
    ref.gate_pose_m[2] = 0.25f * (
        ref.gate_corners_m[2] + ref.gate_corners_m[5] +
        ref.gate_corners_m[8] + ref.gate_corners_m[11]);
  }
  return ref;
}

static void enter_terminal_commit(const EntryMetrics* entry) {
  memory.commit_active = true;
  memory.align_active = false;
  memory.commit_entry_time_s = controller_time_s;
  memory.distance_to_gate_at_commit = entry->distance_to_gate_m;
  memory.y_error_at_commit = entry->y_error_m;
  memory.z_error_at_commit = entry->z_error_m;
  memory.last_min_margin_at_commit = memory.last_min_margin;
}

static GateTinyMpcReference select_reference(
    const DroneState* state,
    const GateVisionPacket* vision,
    const GateControllerConfig* config,
    EntryMetrics* selected_entry,
    GateInvalidReason* fallback_reason) {
  GateTinyMpcReference ref = reference_from_vision(vision, config);
  *fallback_reason = GATE_INVALID_NONE;
  memset(selected_entry, 0, sizeof(*selected_entry));

  if (config->mode != GATE_CONTROL_TERMINAL_COMMIT && config->mode != GATE_CONTROL_PRECOMMIT_ALIGN) {
    return ref;
  }

  if (vision->raw_valid) {
    store_valid_observation(state, vision, config);
    memory.align_active = false;
    if (!memory.commit_active) {
      return ref;
    }
  }

  const bool clipped =
      vision->invalid_reason == GATE_INVALID_GATE_CLIPPED_OUT_OF_FRAME ||
      vision->projection_gate_clipped;
  if (!memory.commit_active && clipped) {
    const bool precommit = config->mode == GATE_CONTROL_PRECOMMIT_ALIGN;
    const float max_age_s = precommit ? config->precommit_align_max_age_s : config->terminal_commit_max_age_s;
    const float max_sigma_m = precommit ? config->precommit_align_max_sigma_m : INFINITY;
    EntryMetrics entry = terminal_entry_metrics(state, vision, config, max_age_s, max_sigma_m);
    *selected_entry = entry;
    if (entry.allowed) {
      enter_terminal_commit(&entry);
    } else if (precommit && entry.has_estimate) {
      const float error_norm = hypotf(entry.y_error_m, entry.z_error_m);
      const bool should_enter =
          fabsf(entry.z_error_m) > config->terminal_commit_center_tolerance_m ||
          (!vision->valid && !entry.aligned);
      if (!memory.align_active && should_enter) {
        memory.align_active = true;
        memory.align_entry_time_s = controller_time_s;
        memory.align_entry_distance_to_gate_m = entry.distance_to_gate_m;
        memory.align_entry_y_error_m = entry.y_error_m;
        memory.align_entry_z_error_m = entry.z_error_m;
        memory.align_entry_estimator_age_s = entry.estimator_age_s;
        memory.align_best_error_norm_m = error_norm;
        memory.align_last_improvement_time_s = controller_time_s;
        memory.align_fallback_reason = GATE_INVALID_NONE;
      } else if (
          memory.align_active &&
          error_norm < memory.align_best_error_norm_m - config->precommit_align_min_improvement_m) {
        memory.align_best_error_norm_m = error_norm;
        memory.align_last_improvement_time_s = controller_time_s;
      }
      if (memory.align_active) {
        const float no_improve_s = controller_time_s - memory.align_last_improvement_time_s;
        if (!entry.aligned && no_improve_s > config->precommit_align_timeout_s) {
          memory.align_active = false;
          memory.align_fallback_reason = GATE_INVALID_UNKNOWN;
        } else {
          const float speed = entry.aligned ? config->precommit_aligned_approach_speed : config->precommit_align_speed;
          return reference_from_memory(
              GATE_SOURCE_PRECOMMIT_ALIGN,
              config->precommit_align_bound_shrink_m,
              speed,
              config);
        }
      }
    }
  }

  if (memory.commit_active) {
    EntryMetrics entry = terminal_entry_metrics(
        state,
        vision,
        config,
        config->precommit_align_max_age_s,
        config->precommit_align_max_sigma_m);
    *selected_entry = entry;
    return reference_from_memory(
        GATE_SOURCE_TERMINAL_GATE_COMMIT,
        config->terminal_commit_bound_shrink_m,
        config->terminal_commit_speed,
        config);
  }
  if (vision->valid) {
    return ref;
  }
  *fallback_reason = memory.align_fallback_reason != GATE_INVALID_NONE ? memory.align_fallback_reason : vision->invalid_reason;
  ref.valid = false;
  ref.source = GATE_SOURCE_FALLBACK_SLOW;
  ref.target_speed_mps = config->invalid_packet_speed;
  ref.has_bounds = false;
  return ref;
}

static void fill_debug(
    const GateTinyMpcReference* ref,
    const GateVisionPacket* vision,
    const EntryMetrics* entry,
    GateInvalidReason fallback_reason) {
  last_debug.control_source = ref->source;
  last_debug.raw_packet_source = vision->source;
  last_debug.fallback_reason = fallback_reason;
  last_debug.commit_allowed = entry->allowed;
  last_debug.estimator_age_s = entry->estimator_age_s;
  last_debug.distance_to_gate_m = entry->distance_to_gate_m;
  last_debug.y_error_m = entry->y_error_m;
  last_debug.z_error_m = entry->z_error_m;
  last_debug.commit_entry_time_s = memory.commit_entry_time_s;
  last_debug.distance_to_gate_at_commit = memory.distance_to_gate_at_commit;
  last_debug.y_error_at_commit = memory.y_error_at_commit;
  last_debug.z_error_at_commit = memory.z_error_at_commit;
  last_debug.last_min_margin_at_commit = memory.last_min_margin_at_commit;
  last_debug.precommit_align_entry_time_s = memory.align_entry_time_s;
  last_debug.precommit_align_entry_distance_to_gate_m = memory.align_entry_distance_to_gate_m;
  last_debug.precommit_align_entry_y_error_m = memory.align_entry_y_error_m;
  last_debug.precommit_align_entry_z_error_m = memory.align_entry_z_error_m;
  last_debug.precommit_align_entry_estimator_age_s = memory.align_entry_estimator_age_s;
  last_debug.precommit_align_best_error_norm_m = memory.align_best_error_norm_m;
}

}  // namespace

extern "C" void gate_tinympc_reset(void) {
  memset(&memory, 0, sizeof(memory));
  memset(&last_debug, 0, sizeof(last_debug));
  controller_time_s = 0.0f;
}

extern "C" void gate_tinympc_set_solver(GateTinyMpcSolveFn solve_fn) {
  solve_callback = solve_fn;
}

extern "C" MotorCommand gate_tinympc_step(
    const DroneState* state,
    const GateVisionPacket* vision,
    const GateControllerConfig* config,
    float dt) {
  MotorCommand command;
  memset(&command, 0, sizeof(command));
  if (state == 0 || vision == 0 || config == 0) {
    return command;
  }

  EntryMetrics entry;
  GateInvalidReason fallback_reason;
  GateTinyMpcReference ref = select_reference(state, vision, config, &entry, &fallback_reason);
  fill_debug(&ref, vision, &entry, fallback_reason);
  command.control_source = ref.source;
  if (solve_callback != 0) {
    solve_callback(state, &ref, config, &command, &last_debug);
  }
  command.control_source = ref.source;
  command.solver_success = last_debug.solver_success;
  command.solver_iterations = last_debug.solver_iterations;
  controller_time_s += fmaxf(0.0f, dt);
  return command;
}

extern "C" const GateControllerDebug* gate_tinympc_last_debug(void) {
  return &last_debug;
}
