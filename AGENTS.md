# Deployment reports

## Explicit requests only

Create or update a saved deployment report only when the user explicitly asks
for one or asks to use the deployment-report template. Building, testing,
flashing, inspecting memory, or discussing results does not by itself request a
saved report. Do not add automatic reporting to builds, hooks, or CI.

Use the measurements requested for the task and available evidence. A report
request does not require another build, flash, or hardware run, and does not
expand the authorized hardware actions. Do not automatically migrate or refresh
historical reports when code changes. For explicitly requested migrations, preserve
the original measurement time and results; recover provenance from evidence or
mark it unknown instead of substituting the current source/environment.

## JSON format and naming

- Copy [tools/reports/deployments/TEMPLATE.json](tools/reports/deployments/TEMPLATE.json).
  The report is JSON, not Markdown. Preserve the template's field names and
  nesting, matching the original TinySDP deployment report.
- Save it as `tools/reports/deployments/<report-id>/report.json`. Use
  `YYYYMMDDTHHMMSS+HHMM-<app>-<source-sha12>` for the report ID, with the actual
  signed timezone offset (`-HHMM` where appropriate). For example:
  `20261120T093000-0500-tinysdp_ct-0123456789ab`. Use the first 12 characters of
  the full source commit SHA. The JSON `report_id` must match its directory.
- Set `recorded_at` to that timestamp at seconds precision in ISO 8601 with
  timezone offset. Use a unique timestamp; do not overwrite an earlier report.
  Amend one only when asked.
- Keep reports and small supporting logs/patches in Git beside `report.json`.
  Keep large binaries, ELF files, caches, and long telemetry captures outside Git;
  record relevant paths and hashes instead. There is no report generator.

## Fields and provenance

- Keep quantities as numeric byte counts. Use `null` for unknown, unavailable,
  inapplicable, or unmeasured values, not zero or invented numbers. Replace every
  template instruction in `notes` with actual observations or explanations.
- `notes` is an array of strings: include measurement conditions, limitations,
  actual commands and relative evidence paths, and explanations of missing data.
- Retain raw firmware parameter values as strings under `telemetry.parameters`.
  Preserve observed telemetry zeros; explain unsampled values in `notes`.
  Use `[]` for `heap_log_variables` only when checked and none exist; otherwise null.
- The only additions to the original layout are `report_id` and `source`.
  Record `source.commit` before adding the report, and `source.dirty` as a boolean.
  Each `source.submodules` entry has `path`, `commit`, and `dirty` fields.
- For dirty sources, save tracked diffs in the paths listed by `source.patches`.
  Save relevant untracked build inputs separately in `source.untracked_inputs`;
  `git diff` does not capture them. Exclude the report itself from source patches.
- Record configuration paths/settings, build flags, compiler version, container
  image name/ID, and binary SHA-256. List configuration paths/settings in
  `source.configurations`; use a string with name/version for `source.compiler`.
  A commit alone does not identify a dirty build; the binary hash identifies the
  exact image. Use `source.changes` to summarize relevant changes and
  `source.compared_with` to reference a previous report, or null.
- Do not create commits or Git tags unless requested. Optional deployment tags
  use `deploy/<app>/<report-id>` and point at the recorded source commit; tags do
  not replace patches for dirty sources.

## Memory accounting

- Keep both `build_summary_bytes` and `elf_details_bytes`. The former records the
  build's printed figures; the latter includes all ELF allocations/reservations.
  Explain differences in `notes`. Do not count host debug ELF file size as Flash.
- Report the whole firmware. Planner objects, reserved heaps, and task stacks
  are included in region allocations; do not add them again.
- TinySDP's `tsdp.stackPl` and `tsdp.stackSp` report unused-stack high-water marks
  in 32-bit words; multiply by 4 for bytes. They update only during active
  planning/setpoint execution. Idle zeros do not demonstrate stack exhaustion.
- Record workload, duration, and which metrics were measured, not exercised,
  unavailable, or failed in `notes`. Startup free heap is not minimum-ever free
  heap; retain unsupported metrics as null and explain them.

# Commit messages

Use Conventional Commits: `type: description` or `type(scope): description`.

- Use `feat` for features, `fix` for bug fixes, `docs` for documentation,
  `build` for build tools/containers/dependencies, `test` for tests,
  `refactor` for restructuring without behavior changes, and `chore` for maintenance.
- Exception: use `wip` (with an optional scope) only for temporary checkpoints
  on work branches, e.g. `wip(tui): checkpoint drone renaming`. Before merging,
  clean up these checkpoints into focused commits with normal types and
  meaningful messages.
- The scope is optional: it names the affected area, such as `container`, `tui`,
  `tinysdp`, or `reports`. Do not introduce custom types such as `config` by default.
- Keep the title short. Use lowercase types and start the description with a verb
  such as `add`, `fix`, `remove`, or `update`. Omit status labels like `DONE` and
  a trailing full stop.
- Keep each commit focused on one logical change. Use the body to explain the
  reason and relevant testing when needed.
- Reference the deployment report in the body for measured memory or performance
  changes. Do not claim an improvement without supporting measurements.
- Mark breaking changes with `!` before the colon and explain what callers need
  to change. These rules require no tools or hooks and do not authorize committing.

Examples:

```text
feat(tui): add drone renaming
fix(container): preserve Docker colors
docs: explain stack measurements
chore(reports): record Brushless deployment results
```
