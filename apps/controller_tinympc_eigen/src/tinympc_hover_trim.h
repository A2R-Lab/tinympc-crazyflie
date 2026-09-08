#ifndef TINYMPC_HOVER_TRIM_H
#define TINYMPC_HOVER_TRIM_H

/* Fixed model-space motor offsets, confirmed by the user for this aircraft.
 * Applied once on top of generated hover; no runtime PID learning. */
static const float tinympc_hover_default_trim[4] = {
  -0.006f, 0.010f, 0.006f, -0.010f
};

#endif
