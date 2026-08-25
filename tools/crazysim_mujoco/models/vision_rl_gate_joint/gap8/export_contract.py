"""Fail-closed checks shared by the NeMO-to-DORY export path."""
from __future__ import annotations

import numpy as np
from onnx import numpy_helper


def signed_int8_weight_ranges(graph) -> dict[str, list[int]]:
    """Return weight ranges, rejecting values pulp-nn would wrap as int8."""
    ranges: dict[str, list[int]] = {}
    for initializer in graph.graph.initializer:
        if not initializer.name.endswith(".weight"):
            continue
        values = numpy_helper.to_array(initializer)
        if not np.all(np.isfinite(values)) or not np.all(values == np.rint(values)):
            raise RuntimeError("non-integral NeMO weight initializer: %s" % initializer.name)
        minimum, maximum = int(values.min()), int(values.max())
        ranges[initializer.name] = [minimum, maximum]
        if minimum < -128 or maximum > 127:
            raise RuntimeError("%s range [%d,%d] exceeds GAP8 signed-int8 storage" %
                               (initializer.name, minimum, maximum))
    if not ranges:
        raise RuntimeError("NeMO graph contains no weight initializers")
    return ranges
