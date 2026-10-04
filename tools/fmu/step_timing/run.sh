#!/usr/bin/env bash
# Time an FMU stepped the way the simulator steps it.
#   tools/fmu/step_timing/run.sh <path.fmu> [seconds] [inputVR value stepTime]
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fmu="$1"; shift
work="$(mktemp -d)"; trap 'rm -rf "$work"' EXIT
unzip -q "$fmu" -d "$work/x"
lib="$(find "$work/x/binaries" -name '*.dylib' -o -name '*.so' | head -1)"
token="$(grep -o 'instantiationToken="[^"]*"' "$work/x/modelDescription.xml" | head -1 | sed 's/.*="//;s/"$//')"
"${CC:-clang}" -O2 -o "$work/t" "$here/step_timing.c"
echo "step timing: $(basename "$fmu")"
"$work/t" "$lib" "$token" "$@"
