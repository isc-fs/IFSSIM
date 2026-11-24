R"JSON(
{
  "about": [
    "The run spec fields the Benchmarks page (SFSDSBenchPage.cpp) sets, and the words it shows for them.",
    "The page compiles this in and takes its labels from it; a field it draws that isn't here logs a warning.",
    "tools/sim_benchmark/tests/test_schema.py checks it against run_spec.py --print-schema both ways.",
    "Format: this file is JSON between its first line and its last (the C++ raw string around it).",
    "path: dotted, * matches any one key, panda_* a prefix; the most specific entry wins.",
    "An empty label or help: the setting's name and bench.yaml's help (BenchUi::Words)."
  ],
  "version": 1,
  "benchmarks": {
    "missions": ["Drive the missions", "The pipeline drives the sim through the missions, scored by the referee. The sim and Mission Control must be running."],
    "sim_bag": ["Replay recorded bags", "Perception and SLAM run offline on recorded simulator bags, scored against the sim's ground truth."]
  },
  "fields": [
    {"path": "benchmarks", "tab": "run", "label": "What to test", "help": "Pick one, or start from a preset above."},

    {"path": "benchmarks.missions.settings.missions", "tab": "run", "label": "Missions", "help": "All lit: every mission."},
    {"path": "benchmarks.missions.settings.tracks", "tab": "run", "label": "Tracks", "help": "The tracks picked here replace every mission's own: each picked mission runs on all of them."},
    {"path": "benchmarks.missions.settings.repeats", "tab": "run", "label": "Runs per track", "help": "Each run is scored on its own; run i uses seed i."},
    {"path": "benchmarks.missions.settings.start_noise", "tab": "run", "label": "Vary the start", "help": "Start each run from a slightly different, seeded position."},
    {"path": "benchmarks.*.bags", "tab": "run", "label": "Bags", "help": "Pick one or more, or all."},
    {"path": "benchmarks.*.only", "tab": "run", "label": "Measure", "help": "All lit: all of them."},
    {"path": "benchmarks.*.repeats", "tab": "run", "label": "Runs per bag", "help": "Each run is a job of its own."},
    {"path": "benchmarks.missions.repeats", "tab": "run", "label": "Repeat everything", "help": "The whole benchmark again, each time a job of its own."},
    {"path": "pipeline", "tab": "run", "label": "Parameter changes", "help": "Files of pipeline parameter changes, over the preset's own; a file picked later wins."},
    {"path": "name", "tab": "run", "label": "Name", "help": "How it shows in the queue."},

    {"path": "benchmarks.*.settings.*", "tab": "run", "label": "", "help": ""},
    {"path": "benchmarks.*.settings.laps", "tab": "run", "label": "Laps", "help": "For every mission. Empty: each mission's own."},
    {"path": "benchmarks.*.settings.timeout_s", "tab": "run", "label": "Time limit (s)", "help": "Per run, for every mission. Empty: each mission's own."},
    {"path": "benchmarks.*.settings.stack", "tab": "run", "label": "Pipeline stack", "help": "Its own: started from the code being tested. The running one: the stack that's already up.",
     "options": {"own": "Its own", "running": "The running one"}},
    {"path": "benchmarks.*.settings.replace_stack", "tab": "run", "label": "Stop the running stack", "help": "Stop a running pipeline stack for the benchmark, and start it again after."},
    {"path": "benchmarks.*.settings.panda_start", "tab": "run", "label": "LattePanda: start", "help": "The command that starts the pipeline on the LattePanda.", "needs": "latte_panda"},
    {"path": "benchmarks.*.settings.panda_stop", "tab": "run", "label": "LattePanda: stop", "help": "The command that stops it.", "needs": "latte_panda"},
    {"path": "benchmarks.*.settings.panda_sha", "tab": "run", "label": "LattePanda: commit", "help": "The command that prints the pipeline commit there.", "needs": "latte_panda"},
    {"path": "benchmarks.*.settings.gt_range_m", "tab": "run", "label": "Ground truth range (m)", "help": "Cones within this distance count."},
    {"path": "benchmarks.*.settings.gt_hfov_deg", "tab": "run", "label": "Ground truth view (deg)", "help": "Half the horizontal field of view."},
    {"path": "benchmarks.*.settings.gt_min_range_m", "tab": "run", "label": "Ground truth from (m)", "help": "Cones closer than this are left out."},
    {"path": "benchmarks.*.settings.gt_scan_period_ms", "tab": "run", "label": "LiDAR period (ms)", "help": "Used to time the ground-truth cones."},
    {"path": "benchmarks.*.settings.max_frames", "tab": "run", "label": "First frames only", "help": "Perception on the first N LiDAR frames only."},
    {"path": "benchmarks.*.settings.profile", "tab": "run", "label": "Time each stage", "help": "Perception's per-stage timings."},
    {"path": "benchmarks.*.settings.strategy", "tab": "run", "label": "SLAM strategy", "help": "Empty: trackdrive."},
    {"path": "benchmarks.*.settings.resynth_odom", "tab": "run", "label": "Rebuild odometry", "help": "SLAM rebuilds /odom from the bag's sensors."},

    {"path": "code.ifssim", "tab": "setup", "label": "Sim (IFSSIM)", "help": "The sim's code on the runner. A local run uses this sim, as it is.", "targets": ["runner"]},
    {"path": "benchmarks.*.settings.pipeline_on", "tab": "setup", "label": "Where the pipeline runs", "help": "Not now: the run waits in the queue until it's free.",
     "options": {"bench_pc": "This PC", "latte_panda": "LattePanda"}},
    {"path": "code.pipeline", "tab": "setup", "label": "Pipeline (IFS09)", "help": "Fetched when the run starts. Empty: the commit this checkout's pipeline/ has."},
    {"path": "boards.mode", "tab": "setup", "label": "Boards", "help": "None, the real boards on a HIL bench, or emulated ones (IFS-vHIL) in Docker.",
     "options": {"hil": "Real (HIL)", "vhil": "Emulated (vHIL)"}},
    {"path": "boards.system", "tab": "setup", "label": "System", "help": "IFS-vHIL's systems/<id>.yaml: which boards are emulated.", "needs": "vhil"},
    {"path": "boards.firmware.*", "tab": "setup", "label": "", "help": "A branch, tag, #PR or commit of the board's firmware. Empty: bench.yaml's default."}
  ]
}
)JSON"
