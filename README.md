# AdvantageKit Log Analyzer

Command-line tool for analyzing FRC robot log files. Parses AdvantageKit `.wpilog`, REV Robotics `.revlog`, and CTRE Phoenix 6 `.hoot` files to surface electrical, mechanical, and motor controller issues.

## Supported File Formats

| Format | Source | Parser |
|---|---|---|
| `.wpilog` | AdvantageKit / WPILib DataLog | Native Python (`parser.py`) |
| `.revlog` | REV Robotics StatusLogger (SPARK MAX/FLEX) | Native Python (`revlog_parser.py`) |
| `.hoot` | CTRE Phoenix 6 Signal Logger (TalonFX, CANcoder, Pigeon2) | Via `owlet` CLI (`hoot_converter.py`) |

## Dependencies

- **Python 3.10+**
- **rich** >= 13.0 — colored terminal output (optional; falls back to plain ANSI)
- **numpy** >= 1.24 — signal processing acceleration (optional; pure-Python fallback)

For `.hoot` file support, CTRE's `owlet` binary is required. It is downloaded automatically on first use from `redist.ctr-electronics.com` and cached in `tools/`.

## Installation

```bash
pip install rich numpy
```

Or with the included requirements file:

```bash
pip install -r requirements.txt
```

Both dependencies are optional. The tool runs with no external packages installed.

## Configuration

### `can_map.json` (optional)

Place a `can_map.json` file in the project root to configure device labels, subsystem assignments, and motor groups. The file supports two formats that can be mixed in the same file.

#### Simple format

Flat key-value pairs for device labeling only:

```json
{
  "PDH:0":  "Front Left Drive",
  "PDH:4":  "Intake Motor",
  "SPARK:16": "FL Drive SPARK",
  "CTRE:TalonFX-2": "FL Drive Falcon"
}
```

#### Extended format

Adds subsystem routing and motor group comparison:

```json
{
  "subsystems": ["ELEVATOR", "SWERVE", "INTAKE"],

  "devices": {
    "CTRE:TalonFX-29": { "name": "Elevator Left",   "subsystem": "ELEVATOR" },
    "CTRE:TalonFX-34": { "name": "Elevator Right",  "subsystem": "ELEVATOR" },
    "CTRE:TalonFX-2":  { "name": "FL Drive Falcon",  "subsystem": "SWERVE" },
    "CTRE:TalonFX-3":  { "name": "FL Steer Falcon",  "subsystem": "SWERVE" },
    "SPARK:16":         { "name": "Indexer",           "subsystem": "INTAKE" },
    "CTRE:Pigeon2-30":  { "name": "IMU",              "subsystem": "SWERVE" },
    "PDH:0":            { "name": "FL Drive",          "subsystem": "SWERVE" }
  },

  "motor_groups": [
    {
      "name": "Elevator",
      "subsystem": "ELEVATOR",
      "motors": ["CTRE:TalonFX-29", "CTRE:TalonFX-34"],
      "relationship": "same_direction",
      "ratio": 1.0
    },
    {
      "name": "Intake Rollers",
      "subsystem": "INTAKE",
      "motors": ["SPARK:16", "SPARK:31"],
      "relationship": "opposite_direction",
      "ratio": 1.0
    }
  ]
}
```

**`subsystems`** — Declares subsystem names that should appear in the report table. These are merged with the defaults (`ELECTRICAL`, `SHOOTER`, `INTAKE`, `TURRET`, `SWERVE`, `CAN`, `REVLOG`, `HOOT`). Without this, CTRE and REV device issues all land under `HOOT` or `REVLOG` since those log formats have no concept of which subsystem a motor belongs to.

**`devices`** — Maps device keys to a name and subsystem. When a device has a subsystem assignment, all issues from that device (faults, voltage sags, current spikes, etc.) appear under that subsystem instead of the generic `HOOT`/`REVLOG`.

**`motor_groups`** — Defines mechanically linked motors for comparison analysis. The analyzer detects:
- **Output divergence** (ERR): one motor driving while the other is off
- **Velocity divergence** (WARN): motors running at different speeds (> 20% difference)
- **Current imbalance** (WARN): one motor drawing > 1.5x the other's current

Motor group fields:
| Field | Description |
|---|---|
| `name` | Group label shown in issue messages |
| `subsystem` | Subsystem to route group issues to |
| `motors` | List of device keys (minimum 2) |
| `relationship` | `same_direction` (default) or `opposite_direction` |
| `ratio` | Gear ratio between motors (default 1.0) |

**Device key formats:**

| Key pattern | Devices |
|---|---|
| `CTRE:<DeviceType-ID>` | CTRE Phoenix 6 devices (e.g., `CTRE:TalonFX-2`, `CTRE:CANcoder-12`, `CTRE:Pigeon2-30`) |
| `SPARK:<device_num>` | REV SPARK MAX/FLEX by CAN ID (e.g., `SPARK:16`) |
| `PDH:<channel>` | PDH current channels 0-23 (e.g., `PDH:0`) |
| `SUBSYSTEM/Motor` | AdvantageKit motor paths (e.g., `SHOOTER/Flywheel`) |

### Analyzer thresholds

Thresholds are defined as constants at the top of each analyzer module:

| File | Threshold | Default |
|---|---|---|
| `analyzers/electrical.py` | Low battery voltage | < 11.0 V for > 1 s |
| `analyzers/electrical.py` | Voltage sag detection | 2.0 V drop in 0.5 s |
| `analyzers/electrical.py` | Sustained current spike | > 40 A for > 0.5 s |
| `analyzers/electrical.py` | Instantaneous current spike | > 60 A |
| `analyzers/revlog.py` | Motor over-temperature | > 80 C for > 2 s |
| `analyzers/revlog.py` | Motor stall | output > 0.3 and velocity < 50 RPM for > 0.5 s |
| `analyzers/revlog.py` | Power starved (per motor) | < 7.0 V for > 5 s |
| `analyzers/revlog.py` | Bus voltage sag (per motor) | < 10.0 V for > 0.5 s |
| `analyzers/hoot.py` | TalonFX over-temperature | > 80 C for > 2 s |
| `analyzers/hoot.py` | Power starved (per device) | < 7.0 V for > 5 s |
| `analyzers/hoot.py` | Supply voltage sag (per device) | < 10.0 V for > 0.5 s |
| `analyzers/hoot.py` | Stator current spike | > 80 A for > 0.5 s |

### Voltage sag severity levels

Voltage sags are classified into three severity tiers across all analyzers:

| Level | Voltage | Detail flag needed |
|---|---|---|
| `ERR` | < 7.0 V | shown by default |
| `WARN` | 7.0 – 9.0 V | `--warn` |
| `INFO` | 9.0 – 10.0 V | `--info` |

### Detail output levels

The detailed issue listing defaults to showing only errors. Use `--warn` or `--info` to include lower-severity issues:

```bash
# Default: only ERR-level issues in detail sections
python3 analyze.py -d logs/ -t 2026-03-24_23-38

# Include warnings
python3 analyze.py -d logs/ -t 2026-03-24_23-38 --warn

# Include everything (info, warnings, errors)
python3 analyze.py -d logs/ -t 2026-03-24_23-38 --info
```

The summary table and motor status table always show all subsystems regardless of detail level — only the per-subsystem detail sections are filtered.

## Usage

There are two ways to specify which log files to analyze: **auto-discovery by timestamp** or **explicit file paths**. Both can be combined with filtering and verbose options.

### Auto-discovery by timestamp (recommended)

Point the analyzer at a root directory and provide a timestamp. It searches recursively through all subdirectories and finds matching `.wpilog`, `.revlog`, and `.hoot` files automatically:

```bash
python3 analyze.py --dir IssueWithSwerve/ --time 2026-03-24_23-38
```

Files do **not** need to be in the same folder. Different FRC vendors place logs in their own directory structures — the search walks the entire directory tree, so you can point `--dir` at a top-level logs directory or even a USB drive root:

```bash
# Search an entire USB drive for logs from a specific match
python3 analyze.py -d /media/usb/ -t 2026-03-24_23-38

# Search the roboRIO log root
python3 analyze.py -d /home/lvuser/logs/ -t 2026-03-24_23-38
```

The timestamp is a prefix match — you can be as specific or as broad as needed:

```bash
# Match all files from a specific second
python3 analyze.py -d logs/ -t 2026-03-24_23-38-19

# Match all files from a specific minute
python3 analyze.py -d logs/ -t 2026-03-24_23-38

# Match all files from a specific date
python3 analyze.py -d logs/ -t 2026-03-24
```

The discovery logic handles the different filename conventions used by each vendor:
- `.wpilog` — `YY-MM-DD_HH-MM-SS.wpilog` (e.g., `26-03-24_23-38-13.wpilog`)
- `.revlog` — `REV_YYYYMMDD_HHMMSS.revlog` (e.g., `REV_20260324_233816.revlog`)
- `.hoot` — `*_YYYY-MM-DD_HH-MM-SS.hoot` (e.g., `rio_2026-03-24_23-38-19.hoot`)

### Explicit file paths

Specify any combination of individual log files directly:

```bash
# WPILog only
python3 analyze.py path/to/logfile.wpilog

# WPILog + REV motor data
python3 analyze.py logfile.wpilog --revlog file.revlog

# WPILog + CTRE hoot logs (multiple hoot files supported)
python3 analyze.py logfile.wpilog --hoot canivore.hoot rio.hoot

# All three sources
python3 analyze.py logfile.wpilog \
    --revlog file.revlog \
    --hoot canivore.hoot rio.hoot

# REV log only (no wpilog required)
python3 analyze.py --revlog file.revlog

# Hoot files only
python3 analyze.py --hoot canivore.hoot rio.hoot
```

### Options

```
-d, --dir DIR          Directory for auto-discovery (use with --time)
-t, --time TIMESTAMP   Timestamp prefix for auto-discovery
-r, --revlog FILE      Path to .revlog file
    --hoot FILE [FILE]  Path to one or more .hoot files
-s, --subsystem NAME   Filter output to one subsystem
-w, --warn             Show warnings and errors in detail sections
-i, --info             Show all issues (info, warnings, errors) in detail sections
-v, --verbose          Show all decoded channels
-b, --batch DIR        Batch-analyze all .wpilog files in a directory
```

### Filter to one subsystem

```bash
python3 analyze.py -d logs/ -t 2026-03-24_23-38 -s HOOT
```

Available subsystems: `ELECTRICAL`, `SHOOTER`, `INTAKE`, `TURRET`, `SWERVE`, `CAN`, `REVLOG`, `HOOT`

### Show all decoded channels

```bash
python3 analyze.py -d logs/ -t 2026-03-24_23-38 --verbose
```

### Batch mode

Analyze all `.wpilog` files in a directory and print a one-line summary for each:

```bash
python3 analyze.py --batch path/to/logs/
```
