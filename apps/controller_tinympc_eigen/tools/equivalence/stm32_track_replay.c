/*
 * Host replay wrapper for the exact production STM32 feature-track estimator.
 * Each process represents one independent scene so firmware statics begin from
 * the same zero-initialized state as a fresh boot.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t flow_equiv_tick_ms = 0;
#include "../../src/flowdeck_obstacle_link.c"

#define PREFIX_FIELDS 14
#define TRACK_FIELDS 6

static int parse_values(char *line, double *values, int capacity) {
  int count = 0;
  char *cursor = line;
  while (*cursor != '\0' && count < capacity) {
    char *end = NULL;
    values[count] = strtod(cursor, &end);
    if (end == cursor) break;
    count++;
    cursor = end;
    if (*cursor == ',') cursor++;
    else if (*cursor != '\0' && *cursor != '\n') break;
  }
  return count;
}

int main(void) {
  char line[16384];
  double values[PREFIX_FIELDS + FLOW_TRACK_MAX * TRACK_FIELDS];
  flowObstacleLinkInit();
  puts("case,frame,accepted,reject_motion,reject_geometry,reject_uncertainty,"
       "support,cluster_sigma,validated_sigma,safety_age,baseline,obs_hits,obs_valid,cyl_valid,cyl_conf,"
       "cyl_accepts,map_peak,cyl_world_x,cyl_world_y,cyl_var_x,cyl_var_y,loom_support,loom_rate,"
       "emergency");
  while (fgets(line, sizeof(line), stdin) != NULL) {
    const int expected = PREFIX_FIELDS + FLOW_TRACK_MAX * TRACK_FIELDS;
    if (parse_values(line, values, expected) != expected) return 2;
    const unsigned case_id = (unsigned)values[0];
    const unsigned frame_id = (unsigned)values[1];
    flow_equiv_tick_ms = (uint32_t)values[2];
    const float body_vx = (float)values[3];
    const float body_vy = (float)values[4];
    const float yaw_rate = (float)values[5];
    const float world_x = (float)values[6];
    const float world_y = (float)values[7];
    const float yaw = (float)values[8];

    flow_track_msg_t message;
    memset(&message, 0, sizeof(message));
    message.p.gap8_ts_us = (uint32_t)values[9];
    message.p.stm32_ts_echo = (uint32_t)values[10];
    message.p.sequence = (uint16_t)values[11];
    message.p.dt_us = (uint16_t)values[12];
    message.p.version = FLOW_TRACK_WIRE_VERSION;
    message.p.count = (uint8_t)values[13];
    for (int i = 0; i < FLOW_TRACK_MAX; i++) {
      const int offset = PREFIX_FIELDS + i * TRACK_FIELDS;
      message.p.track[i].u_q4 = (uint16_t)values[offset];
      message.p.track[i].v_q4 = (uint16_t)values[offset + 1];
      message.p.track[i].du_q8 = (int16_t)values[offset + 2];
      message.p.track[i].dv_q8 = (int16_t)values[offset + 3];
      message.p.track[i].lk_err_q8 = (uint16_t)values[offset + 4];
      message.p.track[i].fb_err_q8 = (uint16_t)values[offset + 5];
    }
    flowObstacleLinkRecordState(
        flow_equiv_tick_ms, body_vx, body_vy, yaw_rate,
        world_x, world_y, yaw);
    if (!flowObstacleLinkPublishTracksFromRx(&message)) return 3;
    flowObstacleLinkUpdateDepth(
        body_vx, body_vy, yaw_rate, world_x, world_y, yaw);
    printf("%u,%u,%u,%u,%u,%u,%u,%.9g,%.9g,%u,%.9g,%u,%.9g,%.9g,%.9g,"
           "%u,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%.9g,%u\n",
           case_id, frame_id, g_trackDepthAccepted,
           g_trackDepthRejectedMotion, g_trackDepthRejectedGeometry,
           g_trackDepthRejectedUncertainty, g_trackClusterSupport,
           g_trackClusterSigma, g_trackValidatedSigma, g_trackSafetyAge,
           g_trackBaselineM, g_obsHits, g_obsValid,
           g_cylValid, g_cylConf, g_cylAccepts, g_mapPeak,
           g_cylWorldX, g_cylWorldY,
           g_cylVarX, g_cylVarY, g_trackLoomingSupport,
           g_trackLoomingRate, flowObstacleLinkEmergencyBrake() ? 1u : 0u);
  }
  return 0;
}
