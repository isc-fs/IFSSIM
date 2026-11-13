# Parameter overrides

Each `*.yaml` here is a list of pipeline parameter changes that the sim's
Benchmarks page (Parameter overrides) can add to a run. Only the file's
`pipeline:` section is used: component, then parameter, then value, as in a
run spec (`tools/sim_benchmark/launch/spec.py`). The leading comment shows
as the file's description.

```yaml
# A looser residual gate for cone detection.
pipeline:
  cone_detection:
    residual_gate_mse: 0.05
```

Picked files are merged over the preset's own `pipeline:` section, in the
order they were picked: a later file wins. From a shell, pass the file to
`run_spec.py` after the preset: `run_spec.py missions-basic overrides/x.yaml`.
