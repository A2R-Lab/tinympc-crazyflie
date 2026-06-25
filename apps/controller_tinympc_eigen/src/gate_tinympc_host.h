#ifndef GATE_TINYMPC_HOST_H
#define GATE_TINYMPC_HOST_H

#include "gate_tinympc_core.h"

#ifdef __cplusplus
extern "C" {
#endif

bool gate_tinympc_host_init(void);
void gate_tinympc_host_reset(void);
MotorCommand gate_tinympc_host_step(
    const DroneState* state,
    const GateVisionPacket* vision,
    const GateControllerConfig* config,
    float dt);
const GateControllerDebug* gate_tinympc_host_last_debug(void);

#ifdef __cplusplus
}
#endif

#endif
