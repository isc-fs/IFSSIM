#!/usr/bin/env bash
# Prove an FMU survives instantiate -> free -> instantiate in one process.
#
# inspect_fmu.py WARNs that the FMU declares canBeInstantiatedOnlyOncePerProcess.
# That is an honest statement about non-reentrant generated code, and it is not
# the same claim as "cannot be reloaded" -- IFSSIM never needs two cars at once,
# it needs to drop one and load another in the same editor session. The
# inspector's note says that must be PROVEN rather than assumed. This proves it.
#
#   tools/fmu/reload_probe/run.sh [path-to.fmu]
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fmu="${1:-$here/../../../matlab/plant/fmu/IFSSIM_Plant.fmu}"
[ -f "$fmu" ] || { echo "no such FMU: $fmu" >&2; exit 2; }

work="$(mktemp -d)"; trap 'rm -rf "$work"' EXIT
unzip -q "$fmu" -d "$work/x"

lib="$(find "$work/x/binaries" -name '*.dylib' -o -name '*.so' | head -1)"
[ -n "$lib" ] || { echo "no binary in $fmu -- source-only FMU, nothing to load" >&2; exit 2; }

token="$(grep -o 'instantiationToken="[^"]*"' "$work/x/modelDescription.xml" \
         | head -1 | sed 's/.*="//;s/"$//')"
[ -n "$token" ] || { echo "no instantiationToken in modelDescription.xml" >&2; exit 2; }

cc="${CC:-clang}"
"$cc" -O1 -o "$work/probe" "$here/reload_probe.c"
echo "reload probe: $(basename "$fmu")"
"$work/probe" "$lib" "$token"
