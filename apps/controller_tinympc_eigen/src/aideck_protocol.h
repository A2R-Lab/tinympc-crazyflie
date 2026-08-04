/*
 * aideck_protocol.h
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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define HEADER_LENGTH 4
#define REQUEST_TIMEOUT 2000 // number of milliseconds to wait for a confirmation
#define INPUT_NUMBER 1

typedef struct input_s {
  const char *header;
  uint8_t size;
  void (*callback)(void *);
  bool valid;
} input_t;

extern input_t inputs[INPUT_NUMBER];

// --- received inference_output_t

#define INFERENCE_STAMPED_HEADER "\x90\x19\x8\x32"
typedef struct inference_stamped_s {
  uint32_t stm32_timestamp;   // [ticks]
  float x;                    // [m]
  float y;                    // [m]
  float z;                    // [m]
  float phi;                  // [rad]
} __attribute__((packed)) inference_stamped_t;

// To be implemented
void inference_stamped_callback(inference_stamped_t *);

// --- sent state_msg_t

#define STATE_MSG_HEADER "!STA"
typedef struct {
  uint8_t header[4];
  uint32_t timestamp;
  
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;

  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;

  // compressed quaternion, see quatcompress.h (elements stored as xyzw)
  int32_t quat;

  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;

  uint32_t checksum;
} __attribute__((packed)) state_msg_t;

void send_state_msg(state_msg_t *msg);

// -- sent rng_msg_t

#define RNG_MSG_HEADER "!RNG"
typedef struct {
  uint8_t header[4];
  
  uint32_t entropy;

  uint32_t checksum;
} __attribute__((packed)) rng_msg_t;

void send_rng_msg(rng_msg_t *msg);

// -- sent tof_msg_t

#define TOF_MSG_HEADER "!TOF"
typedef struct {
  uint8_t header[4];

  uint8_t resolution;
  uint8_t _padding[3];

  uint8_t data[64];

  uint32_t checksum;
} __attribute__((packed)) tof_msg_t;

void send_tof_msg(tof_msg_t *tof);
