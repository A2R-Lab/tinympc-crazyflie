#include <assert.h>
#include <stdint.h>

#include "tinympc_direct_plan_replay.h"

int main(void) {
  TinyMpcDirectPlanReplaySelection selection;

  selection = tinyMpcDirectPlanReplaySelect(100u, 100u, 20u, 19u);
  assert(selection.valid && selection.knot == 0u && selection.age_ticks == 0u);

  selection = tinyMpcDirectPlanReplaySelect(119u, 100u, 20u, 19u);
  assert(selection.valid && selection.knot == 0u);
  selection = tinyMpcDirectPlanReplaySelect(120u, 100u, 20u, 19u);
  assert(selection.valid && selection.knot == 1u);

  selection = tinyMpcDirectPlanReplaySelect(479u, 100u, 20u, 19u);
  assert(selection.valid && selection.knot == 18u);
  selection = tinyMpcDirectPlanReplaySelect(480u, 100u, 20u, 19u);
  assert(!selection.valid && selection.knot == 18u);

  selection = tinyMpcDirectPlanReplaySelect(4u, UINT32_MAX - 5u, 5u, 3u);
  assert(selection.valid && selection.age_ticks == 10u && selection.knot == 2u);

  selection = tinyMpcDirectPlanReplaySelect(100u, 100u, 0u, 19u);
  assert(!selection.valid);
  selection = tinyMpcDirectPlanReplaySelect(100u, 100u, 20u, 0u);
  assert(!selection.valid);
  return 0;
}
