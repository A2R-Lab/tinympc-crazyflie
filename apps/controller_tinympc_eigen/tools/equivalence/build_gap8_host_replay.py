#!/usr/bin/env python3
"""Build a host replay executable from the pushed GAP8 production functions.

The generated translation unit contains verbatim function bodies extracted
from pulp-frontnet/main.c. Platform timing, camera-status, and placement
annotations are stubbed; image processing and persistent filter state are not.
"""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path


FUNCTIONS = (
    "bilinear_sample",
    "build_half_pyramid",
    "corner_score_proxy",
    "select_shi_tomasi_features",
    "robust_near_sample",
    "flow_sample_sigma",
    "lk_track_level",
    "lk_track_pyramid",
    "flow_filter_payload",
    "flow_compute_camera_payload",
)


def extract_function(source: str, name: str) -> str:
    match = re.search(
        rf"(?m)^static\s+(?:inline\s+)?[\w\s\*]+\b{name}\s*\(",
        source,
    )
    if not match:
        raise RuntimeError(f"cannot find production function {name}")
    brace = source.find("{", match.start())
    if brace < 0:
        raise RuntimeError(f"cannot find body for {name}")
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():index + 1]
    raise RuntimeError(f"unterminated body for {name}")


def state_declarations(source: str) -> str:
    start = source.index("#define FLOW_SECTORS")
    end = source.index("static inline float f_abs", start)
    block = source[start:end]
    # The host has no GAP memory-placement attributes.
    return block.replace("PI_L2 ", "").replace("PI_FC_L1 ", "")


HOST_PREAMBLE = r'''
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IMG_W 160
#define IMG_H_CAM 160
#define PI_FC_L1
#define PI_L2
#define FLOW_OBS_SECT_MAX 9
typedef struct __attribute__((packed)) {
  float azimuth_rad;
  float flow_x_rad_s;
  float flow_y_rad_s;
  float flow_sigma_rad_s;
  float confidence;
} flow_obstacle_sector_t;
typedef struct __attribute__((packed)) {
  uint32_t gap8_ts_us;
  uint32_t stm32_ts_echo;
  float dt_s;
  uint8_t n_sectors;
  uint8_t flags;
  uint16_t reserved;
  flow_obstacle_sector_t sector[FLOW_OBS_SECT_MAX];
} flow_obstacle_payload_t;
#define FLOW_TRACK_MAX 32
#define FLOW_TRACK_WIRE_VERSION 1
typedef struct __attribute__((packed)) {
  uint16_t u_q4;
  uint16_t v_q4;
  int16_t du_q8;
  int16_t dv_q8;
  uint16_t lk_err_q8;
  uint16_t fb_err_q8;
} flow_track_wire_t;
typedef struct __attribute__((packed)) {
  uint32_t gap8_ts_us;
  uint32_t stm32_ts_echo;
  uint16_t sequence;
  uint16_t dt_us;
  uint8_t version;
  uint8_t count;
  uint8_t flags;
  uint8_t reserved;
  flow_track_wire_t track[FLOW_TRACK_MAX];
} flow_track_payload_t;

typedef int camera_t;
static camera_t camera;
static uint32_t host_clock_us;
static uint32_t time_get_us(void) { return host_clock_us++; }
static uint32_t camera_get_recovery_count(camera_t *unused) {
  (void)unused; return 0;
}
static uint32_t camera_get_i2c_error_count(camera_t *unused) {
  (void)unused; return 0;
}
static uint32_t stm32_tick_at_gap8_time(uint32_t timestamp_us) {
  return timestamp_us / 1000u;
}
static float host_track_dx_sum;
static float host_track_dy_sum;
static unsigned host_sector_count[FLOW_OBS_SECT_MAX];
static inline float f_abs(float value) {
  return value < 0.0f ? -value : value;
}
'''


HOST_RUNNER = r'''
static void reset_flow_state(void) {
  memset(flow_prev_frame, 0, sizeof(flow_prev_frame));
  memset(flow_cur_frame, 0, sizeof(flow_cur_frame));
  memset(flow_prev_half, 0, sizeof(flow_prev_half));
  memset(flow_cur_half, 0, sizeof(flow_cur_half));
  memset(flow_features, 0, sizeof(flow_features));
  memset(flow_feature_scores, 0, sizeof(flow_feature_scores));
  memset(&flow_camera_payload, 0, sizeof(flow_camera_payload));
  memset(&flow_tx_payload, 0, sizeof(flow_tx_payload));
  memset(flow_smooth_x, 0, sizeof(flow_smooth_x));
  memset(flow_smooth_y, 0, sizeof(flow_smooth_y));
  memset(flow_smooth_conf, 0, sizeof(flow_smooth_conf));
  memset(flow_hold_count, 0, sizeof(flow_hold_count));
  memset(&flow_diag, 0, sizeof(flow_diag));
  memset(&flow_profile, 0, sizeof(flow_profile));
  flow_have_prev = false;
  flow_filter_initialized = false;
  flow_prev_ts_us = 0;
  host_clock_us = 0;
}

typedef struct {
  uint32_t case_id;
  uint32_t frame_id;
  uint32_t timestamp_us;
} replay_header_t;

int main(void) {
  replay_header_t header;
  uint8_t frame[IMG_W * IMG_H_CAM];
  uint32_t active_case = UINT32_MAX;
  printf("case,frame,dt,selected,accepted,rejected,valid_mask,"
         "feature_x_sum,feature_y_sum,feature_score_sum,track_dx_sum,track_dy_sum,"
         "count0,count1,count2,count3,count4,count5,count6,count7,count8,"
         "q0,fx0,fy0,c0,q1,fx1,fy1,c1,q2,fx2,fy2,c2,"
         "q3,fx3,fy3,c3,q4,fx4,fy4,c4,q5,fx5,fy5,c5,"
         "q6,fx6,fy6,c6,q7,fx7,fy7,c7,q8,fx8,fy8,c8,"
         "track_ts,track_echo,track_dt_us,track_count");
  for (int i = 0; i < FLOW_TRACK_MAX; i++) {
    printf(",tu%d,tv%d,tdu%d,tdv%d,tlk%d,tfb%d",
           i, i, i, i, i, i);
  }
  putchar('\n');
  while (fread(&header, sizeof(header), 1, stdin) == 1) {
    if (fread(frame, sizeof(frame), 1, stdin) != 1) {
      fputs("truncated replay frame\n", stderr);
      return 2;
    }
    if (header.case_id != active_case) {
      reset_flow_state();
      active_case = header.case_id;
    }
    flow_obstacle_payload_t payload;
    flow_compute_camera_payload(frame, header.timestamp_us, &payload);
    unsigned feature_x_sum = 0, feature_y_sum = 0, feature_score_sum = 0;
    for (int i = 0; i < flow_diag.selected; i++) {
      feature_x_sum += (unsigned)flow_features[i].x;
      feature_y_sum += (unsigned)flow_features[i].y;
      feature_score_sum += (unsigned)flow_features[i].score;
    }
    printf("%u,%u,%.9g,%u,%u,%u,%u,%u,%u,%u,%.9g,%.9g",
           header.case_id, header.frame_id, payload.dt_s,
           flow_diag.selected, flow_diag.accepted, flow_diag.rejected,
           flow_diag.valid_sector_mask, feature_x_sum, feature_y_sum,
           feature_score_sum, host_track_dx_sum, host_track_dy_sum);
    for (int i = 0; i < FLOW_SECTORS; i++) {
      printf(",%u", host_sector_count[i]);
    }
    for (int i = 0; i < FLOW_SECTORS; i++) {
      printf(",%.9g,%.9g,%.9g,%.9g",
             payload.sector[i].azimuth_rad,
             payload.sector[i].flow_x_rad_s,
             payload.sector[i].flow_y_rad_s,
             payload.sector[i].confidence);
    }
    printf(",%u,%u,%u,%u",
           flow_track_camera_payload.gap8_ts_us,
           flow_track_camera_payload.stm32_ts_echo,
           flow_track_camera_payload.dt_us,
           flow_track_camera_payload.count);
    for (int i = 0; i < FLOW_TRACK_MAX; i++) {
      const flow_track_wire_t *track =
          &flow_track_camera_payload.track[i];
      printf(",%u,%u,%d,%d,%u,%u",
             track->u_q4, track->v_q4, track->du_q8, track->dv_q8,
             track->lk_err_q8, track->fb_err_q8);
    }
    putchar('\n');
    fflush(stdout);
  }
  if (!feof(stdin)) {
    perror("replay input");
    return 3;
  }
  return 0;
}
'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nanocockpit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--generated-c", type=Path)
    args = parser.parse_args()

    app = args.nanocockpit / "src/gap/examples/pulp-frontnet"
    source = (app / "main.c").read_text()
    generated = [HOST_PREAMBLE, state_declarations(source)]
    functions = [extract_function(source, name) for name in FUNCTIONS]
    # Read-only instrumentation of accepted track values. The production
    # statements remain unchanged and the added sums do not feed computation.
    compute_index = FUNCTIONS.index("flow_compute_camera_payload")
    functions[compute_index] = functions[compute_index].replace(
        "  int n_features = 0;",
        "  int n_features = 0;\n"
        "  host_track_dx_sum = 0.0f;\n"
        "  host_track_dy_sum = 0.0f;\n"
        "  memset(host_sector_count, 0, sizeof(host_sector_count));",
    ).replace(
        "      accepted_tracks++;",
        "      accepted_tracks++;\n"
        "      host_track_dx_sum += dx;\n"
        "      host_track_dy_sum += dy;",
    )
    functions[compute_index] = functions[compute_index].replace(
        "      if (flow_count[sector] < FLOW_FEATURES_PER_SECTOR) {",
        "      if (flow_count[sector] < FLOW_FEATURES_PER_SECTOR) {\n"
        "        host_sector_count[sector]++;",
    )
    generated.extend(functions)
    generated.append(HOST_RUNNER)
    generated_text = "\n\n".join(generated)

    generated_c = args.generated_c or args.output.with_suffix(".c")
    generated_c.write_text(generated_text)
    command = [
        "cc", "-std=c11", "-O2", "-Wall", "-Wextra",
        "-Wno-unused-variable", "-Wno-unused-function",
        "-I", str(app), str(generated_c), "-lm", "-o", str(args.output),
    ]
    subprocess.run(command, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
