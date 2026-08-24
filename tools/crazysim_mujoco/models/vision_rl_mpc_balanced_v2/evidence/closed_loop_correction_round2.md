# Closed-loop correction round 2

Date: 2026-08-24

This candidate is retained as negative evidence and is not promoted as the
default policy.

## Provenance

- Candidate ONNX SHA-256:
  `208ce074a6ba93d54f5799fca641ab3c9469bb76820e885a6408f16bdb6a2423`
- First correction shard SHA-256:
  `e4f3af69b23d2a186879584574386bf3cc0601b3b9b2ec572335eaef4efb2b47`
- Second correction shard SHA-256:
  `a6834c5c88df09c20641af0ce7f5c9eec00c280bbcbd7dd24d0d6ea90000f86d`
- Slurm jobs: correction generation `5885`; training `5886`
- Training seeds used for the second correction: `707`, `808`, `909`
- Untouched closed-loop evaluation seeds: `1001`, `1102`, `1203`

The procedural, original U-course oracle, first closed-loop correction, and
second closed-loop correction datasets were sampled as four balanced expert
domains. The procedural validation set remained isolated.

## Offline results

- Independent procedural evaluator: accepted
- Procedural decision balanced accuracy: `0.6162`
- Procedural LEFT recall: `0.9400`
- Procedural RIGHT recall: `0.9086`
- Hard TRACK recall: `0.9285`
- PyTorch/ONNX maximum absolute logit error: `4.29e-6`
- Failed-stream direction balanced accuracy after correction: `0.9069`
- Failed-stream LEFT recall: `0.9048`
- Failed-stream RIGHT recall: `0.9091`

## Fresh closed-loop results

The candidate achieved `0/3` U-course successes. Mean horizontal speed was
`0.7484 m/s`, mean cross-track RMSE was `0.4885 m`, and mean time from launch
to contact was `8.6397 s`.

- Seed `1001`: switched LEFT to RIGHT and cleared both early obstacles, then
  touched the S1 outer wall at `5.293 s` after launch.
- Seed `1102`: reached S1 and S2 and completed the 180-degree turn, then
  contacted `static_4` at `16.102 s` after launch. The high angular rate was
  post-contact.
- Seed `1203`: switched LEFT to RIGHT, then touched the S1 outer wall at
  `4.524 s` after launch.

The prior failure was poor RIGHT recognition at `static_2`. That classification
failure improved materially. The new limiting failure is the controller
interface: an opposite-side redirect crosses the lateral reference from
`+0.60 m` to `-0.60 m`, and the dodge state machine doubles the configured
sidestep rate during that crossing. The two wall-contact runs reached about
`0.93 m` cross-track in a corridor whose centerline-to-wall clearance is about
`0.90 m` after the vehicle envelope. Further policy-only retraining is not the
next principled change; the lateral redirect should be made dynamically
feasible and corridor-aware first.

## GAP8/DORY status

The candidate float graph uses only `Conv`, `Relu`, `AveragePool`, `Flatten`,
and `Gemm`, with input `[1,2,160,160]` and output `[1,3]`. Its architecture is
compatible with a DORY-oriented conversion path and appears to fit GAP8 L2,
but this float ONNX is not a deployable DORY artifact. Required qualification
still includes NeMO quantization/QAT, integer ONNX parity, DORY C generation
and tiling, GVSOC checksum validation, a three-logit NanoCockpit output ABI,
and measured sub-33.3-ms inference.
