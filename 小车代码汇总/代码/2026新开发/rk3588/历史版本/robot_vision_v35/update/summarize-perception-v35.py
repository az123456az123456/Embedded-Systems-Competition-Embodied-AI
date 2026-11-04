#!/usr/bin/env python3
"""Read measured v35 A/B logs on the computer; Python is not required on RK."""
import argparse
import importlib.util
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("perception_compare", HERE / "compare-performance.py")
compare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compare)
compare.PERF_METRICS += ("eye_fps",)
COUNTERS = ("ai_skipped", "completed_pairs", "discarded_pairs", "published_pairs",
            "expired_pairs", "pending_expired_pairs", "out_of_order_pairs")


def summarize(path):
    text = path.read_text(encoding="utf-8", errors="replace")
    report = compare.summarize(text, str(path))
    rows = [dict(compare.FIELD.findall(line)) for line in text.splitlines() if line.startswith("PERF ")]
    counters = {}
    for name in COUNTERS:
        values = [int(row[name]) for row in rows if name in row and row[name].isdigit()]
        resets = sum(right < left for left, right in zip(values, values[1:]))
        counters[name] = {"samples": len(values), "first": values[0] if values else None,
                          "last": values[-1] if values else None, "counter_resets": resets,
                          "delta": values[-1] - values[0] if len(values) > 1 and not resets else None}
    report["scheduler_counters"] = counters
    scheduler = [line for line in text.splitlines() if line.startswith("STAGE scheduler ")]
    report["scheduler_samples"] = len(scheduler)
    report["scheduler_final_fields"] = dict(compare.FIELD.findall(scheduler[-1])) if scheduler else {}
    report["queue_scope"] = "Eye queue_ms are logged per-pair snapshots. STAGE scheduler means/max are cumulative since process start."
    report["complete_pair_contract"] = "result_fps counts complete same-source stereo frames; eye_fps counts individual eye tasks."
    report["preprocess_backends"] = sorted({match.group(1) for match in re.finditer(r"preprocess_backend=(\d+)", text)})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        reports = [summarize(path) for path in args.logs]
    except OSError as error:
        parser.error(str(error))
    comparisons = [compare.compare(reports[0], report) for report in reports[1:]]
    notes = ["All cases must use the same model checksum, resolution, NPU workers and frequency policy; inspect metadata.txt.",
             "cpu-v35 versus pixel-v11 isolates CPU preprocessing. legacy-v10 also forces the legacy input API and is not an original v10 executable.",
             "A requested RGA case may fall back to CPU; inspect PREPROCESS active/reason and preprocess_backend (0 CPU, 1 RGA, 2 legacy/gather).",
             "Counter deltas exclude the first measured sample as baseline; no missing log intervals or counter resets are invented.",
             "No board throughput is asserted by installing or inspecting this tool; it only summarizes supplied logs."]
    if args.json:
        print(json.dumps({"logs": reports, "comparisons": comparisons, "notes": notes}, ensure_ascii=False, indent=2, allow_nan=False))
    else:
        for report in reports:
            compare.print_report(report)
            print("Scheduler counter deltas between first and last retained PERF:")
            for key, value in report["scheduler_counters"].items():
                print(f"  {key}: delta={value['delta']} samples={value['samples']} resets={value['counter_resets']}")
            print("  " + report["complete_pair_contract"])
            print("  " + report["queue_scope"])
            if report["scheduler_final_fields"]:
                print("  Final scheduler: " + json.dumps(report["scheduler_final_fields"], sort_keys=True))
        for result in comparisons:
            print(f"\nA/B means: {result['baseline']} -> {result['candidate']}")
            for name, metric in result["perf"].items():
                percent = metric["change_percent"]
                percent_text = "n/a" if percent is None else f"{percent:+.2f}%"
                print(f"  {name}: {metric['before_mean']:.3f} -> {metric['after_mean']:.3f} ({percent_text})")
        print("\n" + "\n".join(notes))


if __name__ == "__main__":
    main()
