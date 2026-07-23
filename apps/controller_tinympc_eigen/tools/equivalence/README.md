# 15 Hz estimator equivalence

Run from `apps/controller_tinympc_eigen`:

```sh
python3 tools/equivalence/run_15hz_equivalence.py
```

The script compiles a host executable that directly includes the production
`src/flowdeck_obstacle_link.c` translation unit, runs 216 updates across 18
deterministic 15 Hz cases, and compares its sector estimates, candidate groups,
N-of-M state, rejection intermediates, and cylinder output with
`tools/sim_flow_obstacle_sectors.py`.

The host-only headers under `stubs/` replace FreeRTOS tick and logging plumbing;
they do not replace estimator logic. The generated machine-readable result is
`tools/results/flow_15hz_equivalence.json`.

Equivalence detects C/Python drift. It is not evidence of physical detection
quality, timing safety, or authorization for flight.
