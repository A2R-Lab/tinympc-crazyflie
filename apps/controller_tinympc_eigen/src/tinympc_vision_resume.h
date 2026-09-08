#ifndef TINYMPC_VISION_RESUME_H
#define TINYMPC_VISION_RESUME_H
#include <stdbool.h>
#include <stdint.h>

#define TINYMPC_VISION_RESUME_CLEAR_FRAMES 3u

typedef struct {
  uint16_t sequence;
  uint8_t clear_frames;
} tinympcVisionResumeState;

/* Count distinct, consecutive clear inference packets. Repeated controller
 * reads of one packet do not count. Stale, dangerous, skipped, or reordered
 * packets reset confirmation. Sequence wraps from 65535 to 1; zero is invalid. */
static inline bool tinympcVisionResumeUpdate(tinympcVisionResumeState *state,
    bool fresh, bool dangerous, uint16_t sequence)
{
  if (!fresh || dangerous || sequence == 0u) {
    state->clear_frames = 0;
    state->sequence = sequence;
    return false;
  }
  if (sequence == state->sequence) return false;
  const uint16_t expected = state->sequence == 65535u
      ? 1u : (uint16_t)(state->sequence + 1u);
  state->clear_frames = sequence == expected
      ? (uint8_t)(state->clear_frames + 1u) : 1u;
  state->sequence = sequence;
  return state->clear_frames >= TINYMPC_VISION_RESUME_CLEAR_FRAMES;
}
#endif
