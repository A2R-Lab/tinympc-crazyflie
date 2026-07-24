#pragma once

#include "tinympc/types.hpp"

// Keep the trained quad-branch checkpoint byte-for-byte in the source header,
// but instantiate its constants as float for the embedded evaluator.
#define double tinytype
#include "limo_barrier_params_source.hpp"
#undef double
