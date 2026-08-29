# IMAV22 Isaac Lab visual-fidelity evidence v3

This directory is a remote archival snapshot of the accepted seeded IMAV22
Isaac Lab reconstruction rendered on 2026-08-29. It is evidence and source
provenance, not the runtime scene used by CrazySim.

The four accepted visual corrections are:

1. smooth, double-sided white/blue feather-banner proxies;
2. an explicitly UV-mapped 2 m x 2 m traffic rug;
3. the open-ceiling Cyberzoo curtain/net perimeter with restrained background
   clutter and unchanged neutral Isaac Lab lighting; and
4. official one-metre black-panel proportions, feet, and full-face official
   photographic textures, checked from front, rear, and HM01B0 distance.

## Contents

- `render/`: accepted seed-22 overview, 160 x 120 HM01B0 frames, gate and panel
  close-ups, layout manifest, and render validation.
- `camera_smoke/`: the independent finite camera-contract smoke result.
- `source_snapshot/`: exact environment, renderer, tests, provenance, and the
  derived banner texture used to produce this evidence.
- `SHA256SUMS`: checksums for all archived payload files (excluding this README
  and the checksum manifest itself).

## Validation at capture time

- IMAV22 arena tests: 16 passed.
- Crazyflie project quick validator: 3 passed.
- Seeded finite arena render: passed.
- Standard offscreen camera smoke test: passed.
- Arena HM01B0 grayscale shape: 160 x 120.
- Arena HM01B0 grayscale mean: 102.76 / 255.
- Arena HM01B0 grayscale standard deviation: 71.55.

The final banner artwork was not published. The smooth banner silhouette and
white/blue print are therefore explicitly labeled proxies; official simulator
assets remain checksum-pinned in the source snapshot's provenance contract.
