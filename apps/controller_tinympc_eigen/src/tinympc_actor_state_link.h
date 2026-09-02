#pragma once

#include <stdint.h>

#define TINYMPC_ACTOR_STATE_ABI "espnetv2_state24_nominal_preview_body_v1"
#define TINYMPC_ACTOR_STATE_FLOATS 24u

#if defined(CONFIG_PLATFORM_SITL)
void tinyMpcActorStateLinkPublish(
    const float state[TINYMPC_ACTOR_STATE_FLOATS], uint32_t firmware_tick_ms);
#endif

