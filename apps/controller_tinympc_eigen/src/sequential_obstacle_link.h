/* Four-direction sequential perception receiver (GAP8 -> STM32 UART1). */
#ifndef __SEQUENTIAL_OBSTACLE_LINK_H__
#define __SEQUENTIAL_OBSTACLE_LINK_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEQUENTIAL_OBSTACLE_MSG_HEADER "\x90\x19\x8\x38"
#define SEQUENTIAL_OBSTACLE_HEADER_LEN 4
#define SEQUENTIAL_OBSTACLE_DIRECTIONS 4

typedef struct __attribute__((packed)) {
  uint32_t stm32_timestamp;
  uint16_t sequence;
  uint8_t gate_valid;
  uint8_t reserved;
  float clearance_m[SEQUENTIAL_OBSTACLE_DIRECTIONS];
  float confidence[SEQUENTIAL_OBSTACLE_DIRECTIONS];
} sequential_obstacle_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t header[SEQUENTIAL_OBSTACLE_HEADER_LEN];
  sequential_obstacle_payload_t p;
  uint32_t checksum;
} sequential_obstacle_msg_t;

#ifdef __cplusplus
static_assert(sizeof(sequential_obstacle_payload_t) == 40,
              "sequential obstacle payload ABI changed");
static_assert(sizeof(sequential_obstacle_msg_t) == 48,
              "sequential obstacle packet ABI changed");
#else
_Static_assert(sizeof(sequential_obstacle_payload_t) == 40,
               "sequential obstacle payload ABI changed");
_Static_assert(sizeof(sequential_obstacle_msg_t) == 48,
               "sequential obstacle packet ABI changed");
#endif

void sequentialObstacleLinkInit(void);
bool sequentialObstacleLinkPublishFromRx(const sequential_obstacle_msg_t *msg);
void sequentialObstacleLinkNoteBadRx(void);
void sequentialObstacleLinkNoteCrcErr(void);
bool sequentialObstacleLinkGetLatest(
    float clearance_m[SEQUENTIAL_OBSTACLE_DIRECTIONS],
    float confidence[SEQUENTIAL_OBSTACLE_DIRECTIONS],
    uint8_t *gate_valid, uint32_t *age_ms, uint32_t *stm32_capture_tick,
    uint32_t *sample);

#ifdef __cplusplus
}
#endif
#endif
