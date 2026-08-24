#!/usr/bin/env python3
"""Inspect an .fmu and check it against IFSSIM's plant-boundary requirements.

WHY THIS EXISTS
---------------
docs/fmu_plant_migration.md gates the whole migration on a handful of properties
of the exported FMU. Every one of them is a single attribute in
modelDescription.xml, and every one of them fails SILENTLY if you get it wrong:

  * canGetAndSetFMUState is optional. Without it, deterministic reset is
    impossible — and a known-incomplete reset becomes an unknown-incomplete
    black box, which is strictly worse than what the platform has today.

  * THE SPELLING TRAP. FMI 2.0 spells it `canGetAndSetFMUstate` (lowercase s);
    FMI 3.0 spells it `canGetAndSetFMUState` (capital S). A parser matching one
    reads False on the other and silently disables the state path. This tool
    accepts both, deliberately, and tells you which spelling it found.

  * A variable internal step means a data-dependent substep count, which
    destroys the "N game ticks == N doStep calls" invariant that
    simContinueForTime relies on.

  * canBeInstantiatedOnlyOncePerProcess=true makes editor reload impossible
    without a full process restart. Simulink's supportMultiInstance defaults
    OFF, so this one bites by default rather than by accident.

  * An FMU exported on Windows contains win64 binaries ONLY. There is no
    cross-compile. On a macOS editor and Linux CI that is a hard stop unless
    the FMU ships sourceCode.

Running this on a real .fmu answers, in about a second, three questions the
design doc could not settle from public sources.

USAGE
    python3 tools/fmu/inspect_fmu.py path/to/model.fmu
    python3 tools/fmu/inspect_fmu.py path/to/model.fmu --json

Exits non-zero if any REQUIRED gate fails, so it can gate CI.
"""
from __future__ import annotations

import argparse
import json
import platform
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

# Communication step the platform runs at. The FMU's internal step must divide
# this exactly, or the substep count per macro step is not an integer.
COMM_STEP = 1.0 / 60.0


def host_tuples() -> list[str]:
    """Binary directory names that would satisfy THIS machine.

    FMI 2.0 used coarse names (darwin64); FMI 3.0 switched to
    <arch>-<os> (aarch64-darwin). Both appear in the wild, and a Mac can be
    either Intel or Apple Silicon, so accept every spelling that would work.
    """
    sysname = platform.system().lower()
    mach = platform.machine().lower()
    if sysname == "darwin":
        if mach in ("arm64", "aarch64"):
            return ["aarch64-darwin", "darwin64"]
        return ["x86_64-darwin", "darwin64"]
    if sysname == "linux":
        if mach in ("arm64", "aarch64"):
            return ["aarch64-linux", "linux64"]
        return ["x86_64-linux", "linux64"]
    if sysname == "windows":
        return ["x86_64-windows", "win64"]
    return []


def _bool(v: str | None) -> bool | None:
    if v is None:
        return None
    return v.strip().lower() in ("true", "1")


def inspect(path: Path) -> dict:
    out: dict = {"path": str(path), "checks": [], "info": {}}

    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        if "modelDescription.xml" not in names:
            raise SystemExit("error: no modelDescription.xml — not a valid .fmu")
        root = ET.fromstring(z.read("modelDescription.xml"))

    i = out["info"]
    i["fmiVersion"] = root.get("fmiVersion")
    i["modelName"] = root.get("modelName")
    i["generationTool"] = root.get("generationTool")
    # FMI 2.0 calls it guid; FMI 3.0 renamed it instantiationToken.
    i["token"] = root.get("guid") or root.get("instantiationToken")

    cs = root.find("CoSimulation")
    i["hasCoSimulation"] = cs is not None
    i["modelIdentifier"] = cs.get("modelIdentifier") if cs is not None else None

    # --- the state flag, under both spellings ---
    state_attr, state_val = None, None
    if cs is not None:
        for spelling in ("canGetAndSetFMUState", "canGetAndSetFMUstate"):
            if spelling in cs.attrib:
                state_attr, state_val = spelling, _bool(cs.get(spelling))
                break
    i["stateAttrFound"] = state_attr
    i["canGetAndSetFMUState"] = state_val

    if cs is not None:
        i["canSerializeFMUState"] = _bool(
            cs.get("canSerializeFMUState") or cs.get("canSerializeFMUstate"))
        i["canHandleVariableCommunicationStepSize"] = _bool(
            cs.get("canHandleVariableCommunicationStepSize"))
        i["canBeInstantiatedOnlyOncePerProcess"] = _bool(
            cs.get("canBeInstantiatedOnlyOncePerProcess"))
        i["hasEventMode"] = _bool(cs.get("hasEventMode"))
        fis = cs.get("fixedInternalStepSize")
        i["fixedInternalStepSize"] = float(fis) if fis else None

    # --- packaging ---
    bins = sorted({n.split("/")[1] for n in names
                   if n.startswith("binaries/") and n.count("/") >= 2})
    i["binaries"] = bins
    i["hasSourceCode"] = any(n.startswith("sources/") or n.startswith("sourceCode/")
                             for n in names)
    i["hasResources"] = any(n.startswith("resources/") for n in names)

    # --- variable counts, FMI 2.0 and 3.0 shapes ---
    mv = root.find("ModelVariables")
    causalities: dict[str, int] = {}
    if mv is not None:
        for v in list(mv):
            c = v.get("causality") or "local"
            causalities[c] = causalities.get(c, 0) + 1
    i["variables"] = causalities

    # ---------------- gates ----------------
    def check(name, ok, required, detail):
        out["checks"].append({"name": name, "ok": bool(ok),
                              "required": required, "detail": detail})

    check("Co-Simulation present", cs is not None, True,
          "ME-only FMUs need an external solver the platform does not have")

    check("canGetAndSetFMUState", state_val is True, True,
          f"found as '{state_attr}'" if state_attr else
          "ATTRIBUTE ABSENT — deterministic reset is impossible; in Simulink "
          "this is the 'EnableFMUState'/state-save export option")

    fis = i.get("fixedInternalStepSize")
    if fis:
        ratio = COMM_STEP / fis
        integral = abs(ratio - round(ratio)) < 1e-9
        check("internal step divides 1/60", integral, True,
              f"{COMM_STEP:.6f} / {fis:g} = {ratio:.6f} substeps"
              + ("" if integral else "  NOT an integer"))
    else:
        check("internal step divides 1/60", False, False,
              "fixedInternalStepSize not declared — cannot confirm a fixed "
              "internal step; a data-dependent substep count breaks the "
              "N-ticks==N-doSteps invariant")

    once = i.get("canBeInstantiatedOnlyOncePerProcess")
    check("multi-instance allowed", once is not True, True,
          "true means the editor cannot reload without a process restart; "
          "Simulink's supportMultiInstance defaults OFF")

    ht = host_tuples()
    usable = [b for b in bins if b in ht]
    check(f"binary for this host ({platform.system()}/{platform.machine()})",
          bool(usable) or i["hasSourceCode"], True,
          f"present: {bins or 'NONE'}; this host accepts {ht}"
          + ("; sourceCode present, so it can be compiled" if i["hasSourceCode"] else ""))

    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fmu", type=Path)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    if not a.fmu.exists():
        raise SystemExit(f"error: {a.fmu} does not exist")

    r = inspect(a.fmu)
    if a.json:
        print(json.dumps(r, indent=2))
    else:
        i = r["info"]
        print(f"\n{a.fmu.name}")
        print(f"  FMI version     {i['fmiVersion']}")
        print(f"  model           {i['modelName']}  (id: {i['modelIdentifier']})")
        print(f"  generated by    {i['generationTool']}")
        print(f"  token           {i['token']}")
        print(f"  binaries        {i['binaries'] or 'NONE'}")
        print(f"  sourceCode      {i['hasSourceCode']}     resources: {i['hasResources']}")
        print(f"  internal step   {i.get('fixedInternalStepSize')}")
        print(f"  event mode      {i.get('hasEventMode')}")
        print(f"  variables       {i['variables']}")
        print("\n  gates:")
        for c in r["checks"]:
            tag = "PASS" if c["ok"] else ("FAIL" if c["required"] else "WARN")
            print(f"    [{tag}] {c['name']}")
            print(f"           {c['detail']}")

    failed = [c for c in r["checks"] if c["required"] and not c["ok"]]
    if failed:
        print(f"\n{len(failed)} REQUIRED gate(s) failed.")
        return 1
    print("\nAll required gates passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
