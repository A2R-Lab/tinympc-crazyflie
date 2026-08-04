/*
 * aideck.c
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

#define DEBUG_MODULE "AIDECK"

#include "aideck_protocol.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32fxxx.h"

#include "config.h"
#include "console.h"
#include "debug.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "system.h"
#include "uart1.h"
#include "uart2.h"
#include "timers.h"
#include "worker.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool isInit = false;

// Uncomment when NINA printout read is desired from console
// #define DEBUG_NINA_PRINT

#ifdef DEBUG_NINA_PRINT
static void NinaTask(void *param) {
    systemWaitStart();
    vTaskDelay(M2T(1000));
    DEBUG_PRINT("Starting reading out NINA debugging messages:\n");
    vTaskDelay(M2T(2000));

    // Pull the reset button to get a clean read out of the data
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(10);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);

    // Read out the byte the NINA sends and immediately send it to the console.
    uint8_t byte;
    while (1)
    {
        if (uart1GetDataWithDefaultTimeout(&byte) == true)
        {
            consolePutchar(byte);
        }
        // if(in_byte != old_byte)
        // {
        //   old_byte = in_byte;
        //   uart2SendData(1, &old_byte);
        // }
    }
}
#endif

// Read n bytes from UART, returning the read size before ev. timing out.
static int read_uart_bytes(int size, uint8_t *buffer) {
  uint8_t *byte = buffer;
  for (int i = 0; i < size; i++) {
    if(uart1GetDataWithDefaultTimeout(byte))
    {
      byte++;
    }
    else
    {
      return i;
    }
  }
  return size;
}

// Read UART 1 while looking for structured messages.
// When none are found, print everything to console.
static uint8_t header_buffer[HEADER_LENGTH];
static uint8_t buffer[100];
static void read_uart_message() {
  uint8_t *byte = header_buffer;
  int n = 0;
  input_t *input;
  input_t *begin = (input_t *) inputs;
  input_t *end = begin + INPUT_NUMBER;
  for (input = begin; input < end; input++) input->valid = 1;
  while(n < HEADER_LENGTH) {
    if(uart1GetDataWithDefaultTimeout(byte)) {
      int valid = 0;
      for (input = begin; input < end; input++) {
        if(!(input->valid)) continue;
        if(*byte != (input->header)[n]) {
          input->valid = 0;
        } else{
          valid = 1;
        }
      }
      n++;
      if(valid) {
        // Keep reading
        byte++;
        continue;
      }
    }

    // forward to console and return;
    for (size_t i = 0; i < n; i++) {
      consolePutchar(header_buffer[i]);
    }
    return;
  }
  // Found message
  for (input = begin; input < end; input++) {
    if(input->valid) break;
  }
  // uint8_t buffer[input->size];
  int size = read_uart_bytes(input->size, buffer);
  if( size == input->size ) {
    // DEBUG_PRINT("Should call callback for msg %4s of size %d\n", input->header, size);
    // for (size_t i = 0; i < size; i++) {
    //   DEBUG_PRINT("0x%02x\n", buffer[i]);
    // }
    // Call the corresponding callback
    workerSchedule(input->callback, buffer);
    // input->callback(buffer);
  } else {
    DEBUG_PRINT("Failed to receive message %4s: (%d vs %d bytes received)\n",
                 input->header, size, input->size);
  }
}

static void Gap8Task(void *param) {
    systemWaitStart();
    vTaskDelay(M2T(1000));

    // Pull the reset button to get a clean read out of the data
    pinMode(DECK_GPIO_IO4, OUTPUT);
    digitalWrite(DECK_GPIO_IO4, LOW);
    vTaskDelay(100);
    digitalWrite(DECK_GPIO_IO4, HIGH);
    pinMode(DECK_GPIO_IO4, INPUT_PULLUP);
    // DEBUG_PRINT("Starting UART listener\n");
    while (1) {
        read_uart_message();
    }
}

static void aideckInit(DeckInfo *info) {
    if (isInit)
        return;

    // Intialize the UART for the GAP8
    uart1Init(115200);
    // Initialize task for the GAP8
    xTaskCreate(Gap8Task, AI_DECK_GAP_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);

#ifdef DEBUG_NINA_PRINT
    // Initialize the UART for the NINA
    uart2Init(115200);
    // Initialize task for the NINA
    xTaskCreate(NinaTask, AI_DECK_NINA_TASK_NAME, AI_DECK_TASK_STACKSIZE, NULL,
                AI_DECK_TASK_PRI, NULL);
#endif
  isInit = true;
}

static bool aideckTest() {
    return true;
}

static const DeckDriver aideck_deck = {
    .vid = 0xBC,
    .pid = 0x12,
    .name = "bcAI",

    .usedPeriph = 0,
    .usedGpio = 0, // FIXME: Edit the used GPIOs

    .init = aideckInit,
    .test = aideckTest,
};

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcAIDeck, &isInit)
PARAM_GROUP_STOP(deck)

DECK_DRIVER(aideck_deck);
