/*
 * Host replay wrapper for the production STM32 obstacle estimator.
 *
 * This translation unit includes the firmware .c file itself. The only
 * substitutions are the RTOS clock and log-registration macros in mocks/.
 * All estimator functions, constants, float storage, and persistent state are
 * therefore compiled from the same source used in the Crazyflie image.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

uint32_t flow_equiv_tick_ms = 0;

#include "../../src/flowdeck_obstacle_link.c"

enum {
  INPUT_PREFIX_FIELDS = 10,
  SECTOR_FIELDS = 4,
};

static int parse_floats(char *line, float *values, int capacity) {
  int count = 0;
  char *cursor = line;
  while (*cursor != '\0' && count < capacity) {
    char *end = NULL;
    values[count] = strtof(cursor, &end);
    if (end == cursor) {
      break;
    }
    count++;
    cursor = end;
    if (*cursor == ',') {
      cursor++;
    } else if (*cursor != '\0' && *cursor != '\n') {
      break;
    }
  }
  return count;
}

int main(void) {
  char line[8192];
  float values[INPUT_PREFIX_FIELDS + FLOW_OBS_SECT_MAX * SECTOR_FIELDS];
  flowObstacleLinkInit();

  puts("case,frame,sample,obs_valid,obs_hits,cluster_start,cluster_count,"
       "obs_range,obs_world_x,obs_world_y,cyl_valid,cyl_conf,cyl_accepts,"
       "cyl_world_x,cyl_world_y,cyl_body_x,cyl_body_y,"
       "valid_mask,r0,r1,r2,r3,r4,r5,r6,r7,r8");

  while (fgets(line, sizeof(line), stdin) != NULL) {
    if (line[0] == '#' || line[0] == '\n') {
      continue;
    }
    const int expected = INPUT_PREFIX_FIELDS +
                         FLOW_OBS_SECT_MAX * SECTOR_FIELDS;
    if (parse_floats(line, values, expected) != expected) {
      fprintf(stderr, "invalid replay row\n");
      return 2;
    }

    const unsigned case_id = (unsigned)values[0];
    const unsigned frame_id = (unsigned)values[1];
    flow_equiv_tick_ms = (uint32_t)values[2];
    const float body_vx = values[3];
    const float body_vy = values[4];
    const float yaw_rate = values[5];
    const float world_x = values[6];
    const float world_y = values[7];
    const float yaw = values[8];
    const float dt = values[9];

    flow_obstacle_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.p.gap8_ts_us = flow_equiv_tick_ms * 1000u;
    msg.p.dt_s = dt;
    msg.p.n_sectors = FLOW_OBS_SECT_MAX;
    msg.p.reserved = (uint16_t)(frame_id + 1u);
    for (int i = 0; i < FLOW_OBS_SECT_MAX; i++) {
      const int j = INPUT_PREFIX_FIELDS + i * SECTOR_FIELDS;
      msg.p.sector[i].azimuth_rad = values[j];
      msg.p.sector[i].flow_x_rad_s = values[j + 1];
      msg.p.sector[i].flow_y_rad_s = values[j + 2];
      msg.p.sector[i].confidence = values[j + 3];
    }
    if (!flowObstacleLinkPublishFromRx(&msg)) {
      fprintf(stderr, "firmware rejected case %u frame %u\n", case_id, frame_id);
      return 3;
    }
    flowObstacleLinkUpdateDepth(body_vx, body_vy, yaw_rate,
                                world_x, world_y, yaw);

    unsigned valid_mask = 0;
    for (int i = 0; i < FLOW_OBS_SECT_MAX; i++) {
      if (g_valid[i] > 0.5f) {
        valid_mask |= 1u << i;
      }
    }
    printf("%u,%u,%u,%.9g,%u,%u,%u,%.9g,%.9g,%.9g,"
           "%.9g,%.9g,%u,%.9g,%.9g,%.9g,%.9g,%u",
           case_id, frame_id, g_seq >> 1, g_obsValid, g_obsHits,
           g_obsClusterStart, g_obsClusterCount, g_obsRange,
           g_obsWorldX, g_obsWorldY, g_cylValid, g_cylConf,
           g_cylAccepts, g_cylWorldX, g_cylWorldY,
           g_cylBodyX, g_cylBodyY, valid_mask);
    for (int i = 0; i < FLOW_OBS_SECT_MAX; i++) {
      printf(",%.9g", g_range[i]);
    }
    putchar('\n');
  }
  return 0;
}
