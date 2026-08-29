# IMAV 2022 nanocopter arena assets

The textures used by the Isaac Lab reconstruction are vendored from the
official TU Delft `crazyflie-simulation` repository, branch `imav2022`, commit
`da3636651e43ba7663eb3ed4f73c59f641058cef`. The upstream repository is MIT
licensed. Byte checksums are enforced in `envs/imav22_arena.py`.

The arena now renders the checksum-pinned official `imav2022-gate.dae`
directly, including its original UV map and complete floor stand. Invisible
analytic rail boxes retain the exact 0.40 m opening as the collision contract.
`derived/gate_orange_fabric_from_official_atlas.png` is retained only as
historical provenance from the earlier procedural prototype; it is no longer
used for the rendered gate.

Upstream: <https://github.com/tudelft/crazyflie-simulation>

The organizers published a 10 m x 10 m arena, an inner 8 m x 8 m scoring area,
and a final inventory of four orange poles, two black panels, two flags, and two
orange gates. Gate openings were 0.40 m x 0.40 m with 0.094 m borders and a
1.0 m center height. Exact final-round object coordinates and every physical
flag texture were not published.

The exact 2 m x 2 m traffic-pattern rug visible in final-arena imagery is
included as a non-colliding venue detail with an explicit one-image UV map and
is not counted as a challenge obstacle. The lower black-curtain perimeter,
alpha-masked netting, padded posts, and one equipment cabinet use pinned assets
from the official Cyberzoo model. The ceiling is intentionally omitted so the
HM01B0 validation scene retains the project's neutral Isaac Lab lighting.

The final flags are reconstructed as smooth, double-sided feather banners from
the published arena photographs because the public simulator does not contain
their mesh or artwork. `derived/flag_white_blue_proxy_v1.png` is an explicitly
non-official, restrained white/blue print proxy; it deliberately avoids an
invented sponsor wordmark. The black panels use the official one-metre panel
proportions (1.00 m wide, 1.80 m board above 0.20 m, 0.03 m board thickness,
0.43 m foot depth), full-face UVs, and the pinned official panel photographs.

Accordingly, this project makes two narrower, auditable claims:

1. Geometry dimensions, object counts, and the simulator textures are based on
   the official public material.
2. `competition_reference_layout()` is a deterministic reconstruction, not a
   claim of the undisclosed final placement. `sample_randomized_layout(seed)`
   is the training distribution. The flag silhouette and print remain an
   explicitly labeled photographic reconstruction.

## Complexity modes

The arena contract exposes the three competition conditions directly:

- `gate_only`: two static gates; environmental multiplier 1x.
- `static_full`: two gates plus the eight non-gate final objects; multiplier 5x.
- `relocated_full`: the same full inventory, with gates fixed and one non-gate
  obstacle relocated every 30 seconds; multiplier 10x.

For `relocated_full`, relocation is deterministic from the episode seed and
event index. A candidate pose must remain inside the scoring area, clear all
other objects and start/finish keepouts, remain at least 1.5 m plus its own
radius from the drone, and lie outside the HM01B0's forward 87-degree sector
with a five-degree margin. This models a course change while avoiding the
unsupported claim that the drone tracked continuously moving obstacles.
