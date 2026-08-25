# Representative physical dataset v3 source snapshot

This directory preserves the minimal untracked IsaacSim source closure used
for the representative v3 gate/obstacle dataset. It is a source overlay, not
an Isaac Sim distribution and not a copy of the rendered dataset.

Start from IsaacSim base commit `045ca8b59622b99a408092124377c66346e8d9c2`,
then overlay the `user_workflows/` and `gap8_perception/` directories in this
snapshot at that workspace root. Set `TINY_MPC_REPO` to this repository before
running the saved exporter launcher. The launcher retains the fixed v3
parameters: 90 episodes, layout start 29000, seed 20260826, 210 frames, and
one ray-tracing subframe. The compact-only launcher is the recovery path for a
complete raw `transitions.jsonl`; it never rerenders samples.

Verify before use:

```bash
python3 verify_snapshot.py
python3 verify_snapshot.py --check-external-root /path/to/isaacsim-workspace
```

The second command additionally verifies the optional course textures. Those
roughly 14 MB of appearance assets remain external to keep this repository
small, but their hashes are locked in `dataset_v3_source_manifest.json`.
NewBee meshes/texture and generated TinyMPC inputs stay in the main repository
and are verified by the same script. Raw frames, NPZs, transitions, job logs,
Isaac/Omniverse installation, and GPU environment remain external provenance.
