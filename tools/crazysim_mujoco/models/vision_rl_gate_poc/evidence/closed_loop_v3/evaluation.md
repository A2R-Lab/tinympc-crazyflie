# Gate + obstacle POC evaluation

Gate pass/order is MuJoCo/course ground truth from `summary.json`; neural detections are association evidence only.
Matrix status verified: yes.

## candidate gate obstacle

0/3 course completions; 2/3 contacts/crashes (sum/max contact count 5.000/4.000); minimum clearance 0.741 m; ordered gate passes 0/3 declared-gate runs.

| seed | complete | passpoint | contact/crash | contact max | crash after launch (s) | gate order | min clearance (m) | actions T/L/R | p95 inference (ms) | evidence |
| --- | --- | --- | --- | ---: | ---: | --- | ---: | --- | ---: | --- |
| 2001 | no | no | yes | 1.000 | 9.629 | no | 2.046 | 315/0/4 | 4.382 | complete |
| 2002 | no | yes | no | 0.000 | — | no | 0.741 | 700/0/20 | 4.435 | complete |
| 2003 | no | no | yes | 4.000 | 9.540 | no | 2.046 | 315/0/1 | 4.579 | complete |

Obstacle clearance for gate-contact runs is truncated before the obstacle encounter and is not an avoidance-success result.

## candidate obstacle only

0/3 course completions; 0/3 contacts/crashes (sum/max contact count 0.000/0.000); minimum clearance 0.754 m; ordered gate passes 0/0 declared-gate runs.

| seed | complete | passpoint | contact/crash | contact max | crash after launch (s) | gate order | min clearance (m) | actions T/L/R | p95 inference (ms) | evidence |
| --- | --- | --- | --- | ---: | ---: | --- | ---: | --- | ---: | --- |
| 2001 | no | no | no | 0.000 | — | — | 0.926 | 664/0/56 | 4.280 | complete |
| 2002 | no | no | no | 0.000 | — | — | 0.754 | 673/3/44 | 4.535 | complete |
| 2003 | no | no | no | 0.000 | — | — | 0.845 | 676/4/40 | 4.671 | complete |

## baseline obstacle only

0/3 course completions; 0/3 contacts/crashes (sum/max contact count 0.000/0.000); minimum clearance 0.846 m; ordered gate passes 0/0 declared-gate runs.

| seed | complete | passpoint | contact/crash | contact max | crash after launch (s) | gate order | min clearance (m) | actions T/L/R | p95 inference (ms) | evidence |
| --- | --- | --- | --- | ---: | ---: | --- | ---: | --- | ---: | --- |
| 2001 | no | no | no | 0.000 | — | — | 0.846 | 669/4/47 | 0.367 | complete |
| 2002 | no | no | no | 0.000 | — | — | 0.911 | 680/10/30 | 0.370 | complete |
| 2003 | no | no | no | 0.000 | — | — | 0.908 | 677/6/37 | 0.376 | complete |

## Obstacle-only regression

Candidate and baseline aggregates retain every supplied run. Paired rows below are only same-seed deltas.

| seed | candidate complete | baseline complete | clearance delta (m) |
| --- | --- | --- | ---: |
| 2001 | no | no | 0.081 |
| 2002 | no | no | -0.157 |
| 2003 | no | no | -0.063 |
