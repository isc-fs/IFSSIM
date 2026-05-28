from __future__ import annotations

import argparse
import json
from pathlib import Path

from report_html import render_index_html, write_run_report


def _latest_result_jsons(root: Path) -> list[Path]:
    return sorted(root.glob("**/results.json"), key=lambda p: p.stat().st_mtime, reverse=True)


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def main() -> None:
    ap = argparse.ArgumentParser(description="Aggregate benchmark outputs into JSON and HTML reports.")
    ap.add_argument("--results-root", default="tools/sim_benchmark/results")
    ap.add_argument("--output-json", default="tools/sim_benchmark/results/benchmark_summary.json")
    ap.add_argument(
        "--with-report",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Write report.html in each run folder (default: on).",
    )
    ap.add_argument(
        "--index-html",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Also write a top-level benchmark_report.html index (default: on).",
    )
    args = ap.parse_args()

    root = Path(args.results_root)
    jsons = _latest_result_jsons(root)
    rows = [_load(p) for p in jsons]
    out_json = Path(args.output_json)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps({"runs": rows}, indent=2))
    print(f"Wrote {out_json}")

    if args.with_report:
        for json_path in jsons:
            row = _load(json_path)
            prof_path = json_path.parent / "profile.json"
            if prof_path.is_file() and not row.get("profile"):
                row["profile"] = _load(prof_path)
            if row.get("module") == "perception" and row.get("gt_eval") and (
                "gt_range_m" not in row
                or "gt_min_range_m" not in row
                or "gt_scan_period_ms" not in row
            ):
                print(
                    f"Note: {json_path.parent.name} predates GT visibility fix — "
                    "re-run run_perception_benchmark.py on the bag, not just generate_report.py."
                )
            if row.get("module") == "slam" and row.get("has_gt_cones") and (
                "gt_range_m" not in row or "gt_min_range_m" not in row
            ):
                print(
                    f"Note: {json_path.parent.name} predates GT visibility fix — "
                    "re-run run_slam_benchmark.py on the bag, not just generate_report.py."
                )
            run_dir = json_path.parent
            report = write_run_report(row, run_dir, root)
            json_path.write_text(json.dumps(row, indent=2))
            print(f"Wrote {report}")

    if args.index_html and rows:
        out_html = root / "benchmark_report.html"
        out_html.write_text(render_index_html(rows, root), encoding="utf-8")
        print(f"Wrote {out_html}")


if __name__ == "__main__":
    main()
