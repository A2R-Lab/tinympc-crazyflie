/*
 * Host harness for the actual STM32 estimator translation unit.
 *
 * Input is whitespace-separated:
 *   new dt vx vy yaw_rate world_x world_y
 *   followed by 9 * (q flow_x flow_y confidence)
 *
 * One output row is emitted per input row. The firmware source is included so
 * its file-local diagnostic state can be compared without adding test-only
 * symbols to the flight image.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"

TickType_t equivalenceTick = 0;

#include "../../../src/flowdeck_obstacle_link.c"

int main(void) {
  flowObstacleLinkInit();
  flow_obstacle_msg_t message;
  memset(&message, 0, sizeof(message));
  message.p.n_sectors = FLOW_OBS_SECT_MAX;
  uint16_t wire_sequence = 1;
  unsigned frame = 0;

  for (;;) {
    unsigned new_sample;
    float dt, vx, vy, yaw_rate, world_x, world_y, yaw;
    if (scanf("%u %f %f %f %f %f %f %f",
              &new_sample, &dt, &vx, &vy, &yaw_rate,
              &world_x, &world_y, &yaw) != 8) {
      break;
    }
    for (unsigned i = 0; i < FLOW_OBS_SECT_MAX; i++) {
      flow_obstacle_sector_t *sector = &message.p.sector[i];
      if (scanf("%f %f %f %f", &sector->azimuth_rad,
                &sector->flow_x_rad_s, &sector->flow_y_rad_s,
                &sector->confidence) != 4) {
        return 2;
      }
    }

    equivalenceTick += (TickType_t)(dt * 1000.0f + 0.5f);
    if (new_sample) {
      message.p.dt_s = dt;
      message.p.gap8_ts_us += (uint32_t)(dt * 1000000.0f + 0.5f);
      message.p.reserved = wire_sequence++;
      if (!flowObstacleLinkPublishFromRx(&message)) {
        return 3;
      }
    }
    flowObstacleLinkUpdateDepth(vx, vy, yaw_rate, world_x, world_y, yaw);

    printf("%u %u", frame++, new_sample);
    for (unsigned i = 0; i < FLOW_OBS_SECT_MAX; i++) {
      printf(" %.9g %.9g %.9g %.9g %.9g",
             g_valid[i], g_invDepth[i], g_range[i],
             g_bodyX[i], g_bodyY[i]);
    }
    printf(" %u %u %.9g %.9g %.9g %.9g %.9g"
           " %.9g %.9g %.9g %u %.9g %.9g %.9g %.9g %.9g"
           " %u %lu %lu %lu %lu %lu\n",
           g_obsClusterStart, g_obsClusterCount, g_obsValid,
           g_obsRange, g_obsBodyX, g_obsBodyY, g_groupScore,
           g_aggregateDisplacement, g_yawExplainedRatio,
           g_depthDisagreement, g_rejectReason,
           g_cylValid, g_cylConf, g_cylWorldX, g_cylWorldY, g_cylAge,
           g_obsHits,
           (unsigned long)g_rejectLowMotion,
           (unsigned long)g_rejectYaw,
           (unsigned long)g_rejectDepthDisagree,
           (unsigned long)g_rejectDispersion,
           (unsigned long)g_rejectNoGroup);
  }
  return 0;
}
