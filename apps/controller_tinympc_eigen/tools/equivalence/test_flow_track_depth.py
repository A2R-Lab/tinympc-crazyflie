#!/usr/bin/env python3
"""Host tests for the source-identical STM32 per-track depth estimator."""

from __future__ import annotations

import pathlib
import subprocess
import tempfile


APP = pathlib.Path(__file__).resolve().parents[2]
SOURCE = r"""
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

uint32_t flow_equiv_tick_ms = 1000;
#include "src/flowdeck_obstacle_link.c"

static int publish_track(float body_vy, int16_t du_q8,
                         uint16_t fb_q8, uint16_t dt_us) {
  flow_track_msg_t message;
  memset(&message, 0, sizeof(message));
  message.p.gap8_ts_us = flow_equiv_tick_ms * 1000u;
  message.p.stm32_ts_echo = flow_equiv_tick_ms;
  message.p.sequence = (uint16_t)(g_trackRxOk + 1u);
  message.p.dt_us = dt_us;
  message.p.version = FLOW_TRACK_WIRE_VERSION;
  message.p.count = 1;
  message.p.track[0].u_q4 = 1298; /* 81.125 px */
  message.p.track[0].v_q4 = 1174; /* 73.375 px */
  message.p.track[0].du_q8 = du_q8;
  message.p.track[0].fb_err_q8 = fb_q8;
  flowObstacleLinkRecordState(flow_equiv_tick_ms, 0.0f, body_vy, 0.0f,
                              0.0f, 0.0f, 0.0f);
  if (!flowObstacleLinkPublishTracksFromRx(&message)) return 10;
  flowTrackUpdateDepth(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  return 0;
}

static int publish_forward_track(float body_vx, float yaw_rate,
                                 int16_t du_q8) {
  flow_track_msg_t message;
  memset(&message, 0, sizeof(message));
  message.p.gap8_ts_us = flow_equiv_tick_ms * 1000u;
  message.p.stm32_ts_echo = flow_equiv_tick_ms;
  message.p.sequence = (uint16_t)(g_trackRxOk + 1u);
  message.p.dt_us = 65000u;
  message.p.version = FLOW_TRACK_WIRE_VERSION;
  message.p.count = 1;
  message.p.track[0].u_q4 = 1920; /* 120 px, away from the focus of expansion */
  message.p.track[0].v_q4 = 1174;
  message.p.track[0].du_q8 = du_q8;
  message.p.track[0].fb_err_q8 = 26;
  flowObstacleLinkRecordState(flow_equiv_tick_ms, body_vx, 0.0f, yaw_rate,
                              0.0f, 0.0f, 0.0f);
  if (!flowObstacleLinkPublishTracksFromRx(&message)) return 10;
  flowTrackUpdateDepth(body_vx, 0.0f, yaw_rate, 0.0f, 0.0f, 0.0f);
  return 0;
}

int main(void) {
  flowObstacleLinkInit();
  if (publish_track(0.2f, -301, 26, 65535) != 0) return 1;
  if (g_trackDepthAccepted != 1 || !g_trackDepthValid[0]) return 2;
  if (fabsf(g_trackRange[0] - 1.0f) > 0.08f) return 3;
  if (!(g_trackRangeSigma[0] > 0.0f &&
        g_trackRangeSigma[0] < FLOW_OBS_MAX_RANGE_SIGMA_M)) return 4;
  if (!g_trackDepthSyncValid || g_trackDepthSyncErrorMs != 0u) return 5;

  flow_equiv_tick_ms += 67;
  if (publish_track(0.01f, -15, 26, 65535) != 0) return 6;
  if (g_trackDepthAccepted != 0 || g_trackDepthRejectedMotion != 1) return 7;

  flow_equiv_tick_ms += 67;
  if (publish_track(0.2f, -301, 8u * 256u, 65535) != 0) return 8;
  if (g_trackDepthAccepted != 0 ||
      g_trackDepthRejectedUncertainty != 1) return 9;

  flow_equiv_tick_ms += 67;
  if (publish_forward_track(1.0f, 0.0f, 324) != 0) return 11;
  if (g_trackDepthAccepted != 1 || g_trackForwardDepthAccepted != 1 ||
      !g_trackDepthValid[0]) return 12;
  if (fabsf(g_trackRange[0] - 2.2f) > 0.25f) return 13;

  flow_equiv_tick_ms += 67;
  if (publish_forward_track(0.0f, 0.1f, -176) != 0) return 14;
  if (g_trackDepthAccepted != 0 || g_trackDepthValid[0]) return 15;

  flow_equiv_tick_ms += 500;
  if (publish_track(0.2f, -301, 26, 65535) != 0) return 16;
  if (g_trackDepthAccepted != 0 || g_trackDepthValid[0] ||
      g_trackDepthRejectedGeometry != 1) return 17;

  puts("PASS per-track depth: lateral/forward range, yaw, sync, uncertainty, and capture-gap gates");
  return 0;
}
"""


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="flow-track-depth-") as directory:
        tmp = pathlib.Path(directory)
        source = tmp / "test.c"
        executable = tmp / "test"
        source.write_text(SOURCE)
        subprocess.run(
            [
                "cc",
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Wno-unused-variable",
                f"-I{APP}",
                f"-I{APP / 'tools/equivalence/mocks'}",
                f"-I{APP / 'tools/equivalence/stubs'}",
                str(source),
                "-lm",
                "-o",
                str(executable),
            ],
            check=True,
        )
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
