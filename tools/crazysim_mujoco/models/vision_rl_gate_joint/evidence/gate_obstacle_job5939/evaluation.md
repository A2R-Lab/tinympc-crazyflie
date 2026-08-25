# Joint gate + obstacle evaluation

Accepted: **yes**.

Physical gate passage comes only from analyzer course-plane results, never detector telemetry. Contact rows explicitly identify analyzer classification, conservative stop-first-contact inference, or unknown telemetry.

## Acceptance checks

- matrix_valid: pass
- eight_of_ten_contact_free_ordered_gate_and_completion: pass
- zero_gate_frame_contacts: pass
- positive_course_completion: pass
- obstacle_only_collision_not_worse: pass
- inference_p95_under_33ms: pass

## Groups

| group | trials | complete | completion | gate pass | frame contacts | obstacle collision rate | min clearance m | mean speed m/s | max run p95 ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| candidate_gate_obstacle | 10 | 10 | 0.9 | 1.0 | 0.0 | 0.0 | 0.39658391377368263 | 0.58054430765112 | 18.167053000070155 |
| candidate_obstacle_only | 10 | 10 | 1.0 | 0.0 | 0.0 | 0.0 | 0.4672066415241303 | 0.5758224402424374 | 19.107018224895008 |
| baseline_obstacle_only | 10 | 10 | 0.8 | 0.0 | 0.0 | 0.0 | 0.6031158454741538 | 0.4980094002913339 | 13.013401092030104 |

## Contact classification

| group | seed | frame | obstacle | source | clearance scope |
| --- | ---: | ---: | ---: | --- | --- |
| candidate_gate_obstacle | 3121 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3122 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3123 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3124 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3125 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3126 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3127 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3128 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3129 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 3130 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3121 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3122 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3123 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3124 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3125 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3126 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3127 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3128 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3129 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 3130 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3121 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3122 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3123 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3124 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3125 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3126 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3127 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3128 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3129 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 3130 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
