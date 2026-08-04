/*
 * aideck_protocol.c
 * Elia Cereda <elia.cereda@idsia.ch>
 * Luca Crupi <luca.crupi@idsia.ch>
 * Gabriele Abbate <gabriele.abbate@idsia.ch>
 * Jérôme Guzzi <jerome@idsia.ch>
 *
 * Copyright (C) 2022-2025 IDSIA, USI-SUPSI
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * This software is based on the following publication:
 *    E. Cereda, A. Giusti, D. Palossi. "NanoCockpit: Performance-optimized 
 *    Application Framework for AI-based Autonomous Nanorobotics"
 * We kindly ask for a citation if you use in academic work.
 */

#include "aideck_protocol.h"

#include "uart1.h"
#include "debug.h"
#include "usec_time.h"
#include "crc32.h"

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"

#include <string.h>

#define BUFFER_LENGTH 16

// --- received inference_output_t

void log_inference_output(inference_stamped_t *value) {
  DEBUG_PRINT("timestamp=%lu, x=%.3g, y=%.3g, z=%.3g, phi=%.3g\n", value->stm32_timestamp, (double)value->x,(double)value->y,(double)value->z,(double)value->phi);
}

static void __inference_stamped_cb(void *buffer) {
  inference_stamped_t *value = (inference_stamped_t *)buffer;
  inference_stamped_callback(value);
}

// Provide a default implementation of inference_output_callback that can be overriden by the user with a non-weak implementation,
// so that the firmware compiles even if this is not implemented.
__attribute__((weak)) void inference_stamped_callback(inference_stamped_t *inference) {
  // Do nothing
}

// --- sent state_msg_t

void send_state_msg(state_msg_t *msg) {
  memcpy(msg->header, STATE_MSG_HEADER, sizeof(msg->header));
  msg->checksum = crc32CalculateBuffer(msg, sizeof(*msg) - sizeof(msg->checksum));
  uart1SendDataDmaBlocking(sizeof(state_msg_t), (uint8_t *)msg);
}

// --- sent rng_msg_t

void send_rng_msg(rng_msg_t *msg) {
  memcpy(msg->header, RNG_MSG_HEADER, sizeof(msg->header));
  msg->checksum = crc32CalculateBuffer(msg, sizeof(*msg) - sizeof(msg->checksum));
  uart1SendDataDmaBlocking(sizeof(rng_msg_t), (uint8_t *)msg);
}

// --- sent tof_msg_t

void send_tof_msg(tof_msg_t *msg) {
  memcpy(msg->header, TOF_MSG_HEADER, sizeof(msg->header));
  msg->checksum = crc32CalculateBuffer(msg, sizeof(*msg) - sizeof(msg->checksum));
  uart1SendDataDmaBlocking(sizeof(tof_msg_t), (uint8_t *)msg);
}

input_t inputs[INPUT_NUMBER] = {
  { .header = INFERENCE_STAMPED_HEADER, .callback = __inference_stamped_cb, .size = sizeof(inference_stamped_t) }
};
