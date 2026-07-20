/*
 * gate8_link.h
 * GAP8 AI-deck -> STM32 vision link. Receives 8 gate-corner detections over
 * UART1/USART3, matching the GAP8 producer in tinympc-nanocockpit
 * examples/pulp-frontnet/main.c. v1 is receive-only.
 */
#ifndef __GATE8_LINK_H__
#define __GATE8_LINK_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GATE8_N_CORNERS 8

/* Wire message GAP8 -> STM32. Must stay byte-identical to the GAP8 side. */
#define GATE8_MSG_HEADER "\x90\x19\x8\x33"
#define GATE8_HEADER_LEN 4

typedef struct __attribute__((packed)) {
  uint32_t stm32_timestamp;            /* 0 in v1, no state round-trip */
  float    corner[GATE8_N_CORNERS];    /* dequantized corners, order TL,TR,BR,BL */
} gate8_payload_t;

typedef struct __attribute__((packed)) {
  uint8_t  header[GATE8_HEADER_LEN];
  gate8_payload_t p;
  uint32_t checksum;                   /* CRC32 over header+payload */
} gate8_msg_t;

/* Init UART1/USART3 and start the RX task. Call once at startup. */
void gate8LinkInit(void);

/* Copy the latest corners, seqlock-safe. False until the first valid message. */
bool gate8LinkGetLatest(float corners[GATE8_N_CORNERS], uint32_t *out_age_ms);

/* As above, plus the publish counter of the sample that was copied. The counter
 * increments once per accepted message, so a consumer that must act on each frame
 * exactly once (e.g. an EKF update, which would grow over-confident if the same
 * frame were fused repeatedly) can skip re-reads of a sample it already used. */
bool gate8LinkGetLatestSeq(float corners[GATE8_N_CORNERS], uint32_t *out_age_ms,
                           uint32_t *out_sample);

#ifdef __cplusplus
}
#endif

#endif /* __GATE8_LINK_H__ */
