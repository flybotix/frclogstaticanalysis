#!/usr/bin/env bash
#
# Delete wpilog, hoot, and revlog files smaller than a standard FRC match.
#
# Thresholds (based on typical ~3-minute match logging rates):
#   wpilog  < 2 MB
#   hoot    < 2 MB
#   revlog  < 1 MB
#
# Usage:
#   ./clean_small_logs.sh <directory>        # dry-run (default)
#   ./clean_small_logs.sh <directory> --delete

set -euo pipefail

# Size thresholds in bytes
WPILOG_MIN=$((2 * 1024 * 1024))   # 2 MB
HOOT_MIN=$((2 * 1024 * 1024))     # 2 MB
REVLOG_MIN=$((1 * 1024 * 1024))   # 1 MB

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <directory> [--delete]"
    exit 1
fi

TARGET_DIR="$1"
DELETE=false
if [[ "${2:-}" == "--delete" ]]; then
    DELETE=true
fi

if [[ ! -d "$TARGET_DIR" ]]; then
    echo "Error: $TARGET_DIR is not a directory"
    exit 1
fi

count=0
total_bytes=0

while IFS= read -r -d '' file; do
    size=$(stat -f%z "$file" 2>/dev/null || stat -c%s "$file" 2>/dev/null)
    ext="${file##*.}"

    case "$ext" in
        wpilog) min=$WPILOG_MIN ;;
        hoot)   min=$HOOT_MIN ;;
        revlog) min=$REVLOG_MIN ;;
        *)      continue ;;
    esac

    if [[ $size -lt $min ]]; then
        if [[ "$DELETE" == true ]]; then
            rm "$file"
            echo "  deleted  $(du -h "$file" 2>/dev/null || echo "${size}B") $file"
        else
            # File still exists in dry-run, so du works
            human=$(du -h "$file" | cut -f1)
            echo "  would delete  ${human}  $file"
        fi
        count=$((count + 1))
        total_bytes=$((total_bytes + size))
    fi
done < <(find "$TARGET_DIR" -type f \( -name "*.wpilog" -o -name "*.hoot" -o -name "*.revlog" \) -print0)

if [[ $count -eq 0 ]]; then
    echo "No undersized log files found."
else
    total_mb=$(echo "scale=1; $total_bytes / 1048576" | bc)
    if [[ "$DELETE" == true ]]; then
        echo "Deleted $count file(s) (${total_mb} MB)"
    else
        echo "Found $count file(s) (${total_mb} MB) — re-run with --delete to remove"
    fi
fi
