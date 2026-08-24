# FMU test fixtures

Tiny hand-built `.fmu` files. They contain no runnable binary — the "library" is
padding — so they exercise the *package* layer only: ZIP reading,
`modelDescription.xml` parsing, and the gate checks. That is deliberate: those
are the parts that fail silently, and they can be tested without MATLAB, without
a licence, and without launching the editor.

| fixture | what it is for |
|---|---|
| `good_fmi3.fmu` | well-formed FMI 3.0 Co-Simulation; every gate should PASS |
| `trap_fmi2_lowercase_state.fmu` | FMI 2.0 using the **lowercase** `canGetAndSetFMUstate` spelling, and `win64`-only. The state gate must PASS (reading the attribute correctly) while multi-instance and host-binary FAIL. A parser that only matches the FMI 3.0 capital-S spelling reports a false negative here — that is the bug this fixture exists to catch. |
| `deflate_probe.fmu` | every entry DEFLATE-compressed rather than stored, so the zlib inflate path is exercised. A reader that only handles stored entries passes the other two fixtures and fails this one. |

Check them with either implementation — they must agree:

```bash
python3 tools/fmu/inspect_fmu.py tools/fmu/fixtures/good_fmi3.fmu
```

```
inspectFmu <abs-path-to.fmu>      # over the sim RPC, exercises the C++ path
```

The two implementations share no code on purpose. The Python one uses
`zipfile`; the C++ one uses `FFSDSZipReader` over zlib, because the engine's
`FZipArchiveReader` links libzip only under `bBuildEditor` and would fail in a
packaged build. Agreement between them is evidence; a single implementation
agreeing with itself is not.
