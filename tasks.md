# Tasks

## Remaining

1. - [ ] **Timestamp synchronization across log formats.** The wpilog, revlog, and hoot files use different clock sources/epochs. Timestamps are not currently aligned between formats.

## Completed

1. - [x] Parse `.wpilog` binary files (AdvantageKit format)
2. - [x] Electrical analyzer: brownouts, low battery, voltage sags, PDH current spikes, CAN errors, rail faults
3. - [x] Mechanical analyzer: shooter velocity, intake stall, turret drift, swerve heading/translation/pose
4. - [x] Subsystem roll-up and terminal report output
5. - [x] Batch analysis mode
6. - [x] Parse `.revlog` binary files (REV Robotics SPARK motor controller data)
7. - [x] REV log analyzer: sticky faults, bus voltage sag per motor, stall detection, over-temperature, hardware faults, has-reset
8. - [x] Integrate `--revlog` flag into `analyze.py` CLI
9. - [x] Write README with dependencies, install, configuration, and usage
10. - [x] **Analyze and plan to parse `.hoot` files.** Format is proprietary (CTRE Phoenix 6 Signal Logger). Decoded via `owlet` CLI → `.wpilog`.
11. - [x] **Write hoot-to-wpilog conversion wrapper and integrate into the analyzer.** `hoot_converter.py` + `analyzers/hoot.py` for TalonFX, CANcoder, Pigeon2, CANdle.
12. - [x] **Convert and analyze the `rio_*.hoot` file.** Same workflow, tags issues with bus name.
13. - [x] **Refactor CLI to accept a timestamp and auto-discover log files.** `--dir` + `--time` with recursive search across subdirectories.
14. - [x] **Add `can_map.json` for the IssueWithSwerve session.** (Marked complete by user.)
15. - [x] **Deduplicate firmware records in `revlog_parser.py`.** Added `firmware_seen` set to skip repeated entry ID 1 records. Reduced from ~70K to 1 sample per device.
16. - [x] **Add REVLOG/HOOT summary lines to `_summarize()`.** Both subsystems now show categorized summaries (power starved, rebooted, brownout, hw fault, over-temp, current spike, stall, voltage sag, sticky fault) instead of generic "N warning(s)".
17. - [x] **Verify SPARK Status 0 signal bit layout against the official DBC.** Cross-referenced with `spark.public.dbc`. Confirmed: APPLIED_OUTPUT bits [0:15] signed LE (factor 3.082e-5), VOLTAGE bits [16:27] unsigned (factor 7.326e-3), CURRENT bits [28:39] unsigned (factor 3.663e-2), MOTOR_TEMPERATURE bits [40:47] = byte 5. Updated factors to match DBC exactly.
18. - [x] **Decode additional SPARK status frames.** Added decoders for Status 3 (analog sensor), Status 4 (alt encoder), Status 5 (duty cycle encoder), Status 6 (duty cycle raw), Status 7 (I-accumulation). Status 8/9 (setpoint diagnostics) omitted as read-only telemetry. Only Status 0/1/2 appear in the IssueWithSwerve data.
19. - [x] **Add power-starved and motor-reboot detection.** ERR when supply voltage < 7V for > 5s or BootDuringEnable fault detected. Per-motor status table in report output.
20. - [x] **Add `--warn` and `--info` detail levels.** Default detail output shows ERR only. `--warn` adds warnings, `--info` adds all. Voltage sags tiered: ERR < 7V, WARN 7–9V, INFO 9–10V.
21. - [x] **Update README with `--warn`/`--info` flags, voltage severity tiers, and power-starved thresholds.**
22. - [x] **Chronological ordering in detail sections.** Issues within each subsystem sorted by `time_start`.
23. - [x] **Recursive file discovery.** `--dir` searches subdirectories so vendor log files don't need to be in the same folder.
