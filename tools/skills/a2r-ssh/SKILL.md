---
name: a2r-ssh
description: Connect from Charles's Mac to the A2R main lab computer using the working a2r-main SSH alias, and locate remote TinyMPC or GAP8 work. Use for requests involving a2r-main, the main lab computer, or remote lab training artifacts.
---

# A2R lab SSH

Use the existing local SSH configuration; the connection was verified on 2026-09-10:

```sh
ssh -o BatchMode=yes -o ConnectTimeout=15 a2r-main 'hostname; pwd'
```

The alias resolves to user `cchen`, host `a2r-lab-server.dartmouth.edu`, port `2219`, identity `~/.ssh/a2r_lab_ed25519`, with `IdentitiesOnly yes`. Prefer the alias to spelling out connection details. No password or key contents belong in this skill. If it fails, inspect selected fields from `ssh -G a2r-main` and use bounded SSH diagnostics; do not replace keys or disable host-key verification. A changed host key needs an independently verified fingerprint.

Use remote shell commands through `ssh a2r-main '...'`, and `rsync` or `scp` using `a2r-main:/home/cchen/...`. Read `/home/cchen/AGENTS.md` and any repository instructions before edits. Check Git status/remotes: remote firmware checkouts contain substantial uncommitted experiments. Use a separate build directory or checkout when testing local changes; do not overwrite those checkouts with a sync or reset.

Known locations (recheck when used):

- `/home/cchen/tinympc-crazyflie` — lab firmware repository.
- `/home/cchen/tinympc-nanocockpit` — GAP8 firmware repository.
- `/home/cchen/ddnd-gap8-gates-20260908/STATUS.json` — DDND training/deployment provenance. Latest recovered release on September 10 was `releases/final-11174-liveled-20260909`; the top-level `gap8_app` had older weights. Read status and manifests instead of assuming the newest modification time identifies a model.
- `/home/cchen/containers/bitcraze-builder.sif` — ARM firmware toolchain, used via `apptainer exec`.
- Local Crazyflie checkout: `/Users/charleschen/CMU/Research/TinyMPC`; NanoCockpit is a separate nested Git repository.

For cleanup, distinguish source/checkpoints and training dependencies from disposable installs. The September 10 cleanup removed unused IsaacLab/Isaac Sim environments and package caches; it preserved `drone_rl/outputs/orange_gate9_10cm_100k_20260907` and `depthgate-20260906/data/teacher_cache` because DDND manifests referenced them. Recheck dependencies before future deletion. Prior cleanup or push authorization does not authorize new unrelated deletions or pushes.
