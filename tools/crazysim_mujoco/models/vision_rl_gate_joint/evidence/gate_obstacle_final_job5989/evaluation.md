# Joint gate + obstacle evaluation

Accepted: **yes**.

Physical gate passage comes only from analyzer course-plane results, never detector telemetry. Contact rows explicitly identify analyzer classification, conservative stop-first-contact inference, or unknown telemetry.

## Matrix tool provenance

- runner SHA-256: a05d789d1dda48cf634737fc0bbbe9b37bfa49ea4bb044319bad7730260ef53f
- evaluator SHA-256: 0fcf0ee7f20247814152c49768d410f7e9d99edcf47b455ac13aa6614f176f83

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
| candidate_gate_obstacle | 10 | 10 | 1.0 | 1.0 | 0.0 | 0.0 | 0.5575969905728464 | 0.5309460921414695 | 19.538878486491708 |
| candidate_obstacle_only | 10 | 10 | 1.0 | 0.0 | 0.0 | 0.0 | 0.3901021991229531 | 0.5220651686563791 | 17.586535483133048 |
| baseline_obstacle_only | 10 | 10 | 1.0 | 0.0 | 0.0 | 0.0 | 0.40924286037635094 | 0.4899305322029316 | 12.855207175016396 |

## Contact classification

| group | seed | frame | obstacle | source | clearance scope |
| --- | ---: | ---: | ---: | --- | --- |
| candidate_gate_obstacle | 4001 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4002 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4003 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4004 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4005 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4006 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4007 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4008 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4009 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_gate_obstacle | 4010 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4001 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4002 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4003 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4004 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4005 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4006 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4007 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4008 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4009 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| candidate_obstacle_only | 4010 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4001 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4002 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4003 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4004 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4005 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4006 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4007 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4008 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4009 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
| baseline_obstacle_only | 4010 | 0.0 | 0.0 | generic_no_contact | full_observed_window |
