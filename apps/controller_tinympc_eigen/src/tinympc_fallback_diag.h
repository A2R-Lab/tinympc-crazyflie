#ifndef TINYMPC_FALLBACK_DIAG_H
#define TINYMPC_FALLBACK_DIAG_H
#include <stdbool.h>
#include <stdint.h>
/* Observation only. Never feeds decisions back into the control guard.
 * Reason bits: 1 motors forbidden, 2 no plan, 4 stale, 8 invalid,
 * 16 pilot target/mode changed, 32 existing fallback latch,
 * 64 explicit line-test cancellation or envelope/trajectory abort. */
typedef struct {
  uint8_t active, guard, reason;
  uint32_t age_ms, fault_age_ms, direct_ms, entered_ms, events;
} TinyMpcFallbackDiag;
static inline void tinyMpcFallbackDiagUpdate(TinyMpcFallbackDiag *d,
    bool reset, bool active, uint8_t guard, uint32_t now_ms, uint32_t age_ms) {
  if (reset) {
    const uint32_t events = d->events;
    d->active = d->guard = d->reason = 0;
    d->age_ms = d->fault_age_ms = d->direct_ms = d->entered_ms = 0;
    d->events = events;
  }
  d->guard = guard;
  d->age_ms = age_ms;
  if (active && !d->active) {
    d->entered_ms = now_ms;
    d->direct_ms = 0;
    ++d->events;
  }
  if (d->active) d->direct_ms = now_ms - d->entered_ms;
  if (!active && d->active) {
    d->reason = guard;
    d->fault_age_ms = age_ms;
    ++d->events;
  }
  d->active = active;
}
#endif
