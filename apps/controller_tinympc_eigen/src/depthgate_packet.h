/* DepthGate telemetry only; never grants flight-control authority. */
#ifndef DEPTHGATE_PACKET_H
#define DEPTHGATE_PACKET_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#define DEPTHGATE_HEADER "\x90\x19\x08\x44"
/* Little endian IEEE754 binary32. Timestamp is GAP-local milliseconds when
 * status bit 1 is set; bit 0 means valid live-camera inference. */
typedef struct __attribute__((packed)) {
  uint32_t source_timestamp_ms;
  uint16_t sequence;
  uint16_t status;
  uint32_t inference_us;
  float inverse_depth[3]; /* left, center, right; 1/meters */
  float corners_xy[8]; /* LT, RT, LB, RB; x/y pixels in 160x128 image */
  float visibility_logits[4]; /* same corner order; sigmoid threshold 0 */
} DepthGatePayload;
typedef struct __attribute__((packed)) {
  uint8_t header[4];
  DepthGatePayload payload;
  uint32_t checksum; /* existing firmware CRC32 over header + payload */
} DepthGatePacket;
_Static_assert(sizeof(DepthGatePayload) == 72, "DepthGate payload ABI");
_Static_assert(sizeof(DepthGatePacket) == 80, "DepthGate packet ABI");
static inline bool depthGatePayloadValid(const DepthGatePayload *p) {
  if ((p->status & ~3u) || !(p->status & 1u)) return false;
  for (int i = 0; i < 3; ++i)
    if (!isfinite(p->inverse_depth[i]) || p->inverse_depth[i] < 0) return false;
  for (int i = 0; i < 8; ++i)
    if (!isfinite(p->corners_xy[i])) return false;
  for (int i = 0; i < 4; ++i)
    if (!isfinite(p->visibility_logits[i])) return false;
  return true;
}
#endif
