#!/usr/bin/env python3
"""Summarize one or compare multiple perception PERF/STAGE logs, using stdlib.

Run this tool on a computer, not necessarily on the RK3588. PERF values and
STAGE values are periodic snapshots, not a recording of every captured frame.
Worker busy_ms is cumulative end-to-end task time, not NPU hardware utilization.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import re
import statistics
import sys


MARKER = re.compile(r"(?:^|\s)(PERF|STAGE)\s+")
FIELD = re.compile(r"([A-Za-z_][A-Za-z_0-9]*)=([^\s]+)")
PERF_METRICS = (
    "capture_fps", "preview_fps", "result_fps", "inference_ms",
    "frame_age_ms", "box_age_ms",
)
NOTES = [
    "PERF/STAGE statistics summarize logged snapshots, not every frame.",
    "inference_ms is paired-result latency including queueing, not 1000/result_fps.",
    "Worker busy_ms includes preprocessing, input, inference and output; it is not hardware utilization.",
    "Worker task times use differences between consecutive cumulative counters; the first counter is only a baseline.",
    "New worker window means are weighted by completed window_jobs; their p50/p95 describe window means, not individual-job percentiles.",
    "Without elapsed-time timestamps, no worker occupancy percentage is estimated.",
]


def number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) and result >= 0 else None


def integer(value):
    if not isinstance(value, str) or not re.fullmatch(r"\d+", value):
        return None
    return int(value)


def percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def stats(values):
    if not values:
        return None
    return {
        "count": len(values), "mean": statistics.fmean(values),
        "p50": percentile(values, 0.5), "p95": percentile(values, 0.95),
        "min": min(values), "max": max(values),
    }


def metric_summary(records, metrics=None):
    values = defaultdict(list)
    for record in records:
        keys = metrics if metrics is not None else (
            key for key in record if key.endswith("_ms") or key.endswith("_fps") or key == "ui_hz"
        )
        for key in keys:
            value = number(record.get(key))
            if value is not None:
                values[key].append(value)
    return {key: stats(samples) for key, samples in sorted(values.items())}


def worker_windows(records):
    metrics = defaultdict(list)
    weighted_sum, weights = defaultdict(float), defaultdict(int)
    completed_jobs = 0
    window_count = 0
    for record in records:
        jobs = integer(record.get("window_jobs"))
        if jobs is None:
            continue
        window_count += 1
        completed_jobs += jobs
        duration = number(record.get("window_ms"))
        if duration is not None:
            metrics["window_ms"].append(duration)
        if not jobs:
            continue
        for key, raw in record.items():
            if not (key.startswith("mean_") and key.endswith("_ms")):
                continue
            value = number(raw)
            if value is not None:
                metrics[key].append(value)
                weighted_sum[key] += value * jobs
                weights[key] += jobs
    return {
        "windows": window_count, "completed_jobs": completed_jobs,
        "weighted_means_ms": {key: weighted_sum[key] / weight for key, weight in sorted(weights.items())},
        "window_mean_stats": {key: stats(values) for key, values in sorted(metrics.items())},
    }


def worker_summary(records):
    previous = None
    jobs_total, busy_total, resets, anomalies = 0, 0.0, 0, 0
    interval_means = []
    cores = set()
    for record in records:
        jobs, busy = integer(record.get("jobs")), number(record.get("busy_ms"))
        if jobs is None or busy is None:
            continue
        core = record.get("core")
        if core is not None:
            cores.add(core)
        if previous is not None:
            delta_jobs, delta_busy = jobs - previous[0], busy - previous[1]
            if delta_jobs < 0 or delta_busy < -0.01:
                resets += 1
            elif delta_jobs == 0:
                if abs(delta_busy) > 0.01:
                    anomalies += 1
            else:
                delta_busy = max(0.0, delta_busy)
                jobs_total += delta_jobs
                busy_total += delta_busy
                interval_means.append(delta_busy / delta_jobs)
        previous = jobs, busy
    return {
        "counter_samples": len(records), "cores": sorted(cores),
        "valid_intervals": len(interval_means), "completed_jobs_delta": jobs_total,
        "busy_ms_delta": busy_total,
        "task_ms_weighted": busy_total / jobs_total if jobs_total else None,
        "interval_mean_task_ms": stats(interval_means),
        "counter_resets": resets, "anomalous_intervals": anomalies,
        "window_stages": worker_windows(records),
    }


def summarize(text, name="log", skip_perf=0):
    records, perf_index = [], -1
    for line in text.splitlines():
        marker = MARKER.search(line)
        if not marker:
            continue
        kind = marker.group(1)
        fields = dict(FIELD.findall(line[marker.end():]))
        if not fields:
            continue
        if kind == "PERF":
            perf_index += 1
        records.append((kind, fields, perf_index))
    has_perf = perf_index >= 0
    filtered = [
        (kind, fields) for kind, fields, index in records
        if index >= skip_perf or (not has_perf and skip_perf == 0)
    ]
    perf = [fields for kind, fields in filtered if kind == "PERF"]
    stages = [fields for kind, fields in filtered if kind == "STAGE"]
    eyes, workers = defaultdict(list), defaultdict(list)
    for stage in stages:
        if "eye" in stage:
            eyes[stage["eye"]].append(stage)
        worker = integer(stage.get("npu_worker"))
        if worker is not None:
            workers[str(worker)].append(stage)
    worker_results = {worker: worker_summary(rows) for worker, rows in sorted(workers.items())}
    jobs = sum(worker["completed_jobs_delta"] for worker in worker_results.values())
    busy = sum(worker["busy_ms_delta"] for worker in worker_results.values())
    modes = {
        key: sorted({row[key] for row in perf if key in row})
        for key in ("detect", "stereo", "preview", "npu_workers", "core_binding")
    }
    modes = {key: value for key, value in modes.items() if value}
    return {
        "name": name, "perf_windows": len(perf), "skipped_perf_windows": min(skip_perf, perf_index + 1),
        "modes": modes, "perf": metric_summary(perf, PERF_METRICS),
        "eyes": {eye: metric_summary(rows) for eye, rows in sorted(eyes.items())},
        "camera": metric_summary([row for row in stages if "camera_frame" in row]),
        "ui": metric_summary([row for row in stages if "ui_hz" in row]),
        "workers": worker_results,
        "worker_total": {
            "completed_jobs_delta": jobs, "busy_ms_delta": busy,
            "task_ms_weighted": busy / jobs if jobs else None,
            "window_stages": worker_windows([row for rows in workers.values() for row in rows]),
        },
        "notes": NOTES,
    }


def compare(baseline, candidate):
    result = {}
    for key in PERF_METRICS:
        before = baseline["perf"].get(key, {}).get("mean")
        after = candidate["perf"].get(key, {}).get("mean")
        if before is None or after is None:
            continue
        result[key] = {
            "before_mean": before, "after_mean": after, "delta": after - before,
            "change_percent": 100 * (after - before) / before if before else None,
        }
    differences = {
        key: {"before": baseline["modes"].get(key), "after": candidate["modes"].get(key)}
        for key in baseline["modes"].keys() | candidate["modes"].keys()
        if baseline["modes"].get(key) != candidate["modes"].get(key)
    }
    before_stages = baseline["worker_total"]["window_stages"]["weighted_means_ms"]
    after_stages = candidate["worker_total"]["window_stages"]["weighted_means_ms"]
    stage_deltas = {
        key: {"before_mean": before_stages[key], "after_mean": after_stages[key],
              "delta": after_stages[key] - before_stages[key]}
        for key in sorted(before_stages.keys() & after_stages.keys())
    }
    return {"baseline": baseline["name"], "candidate": candidate["name"], "perf": result,
            "mode_differences": differences, "worker_window_stages": stage_deltas}


def print_metrics(title, metrics):
    if not metrics:
        return
    print(title)
    print(f"  {'metric':<22} {'n':>5} {'mean':>10} {'p50':>10} {'p95':>10}")
    for key, value in metrics.items():
        print(f"  {key:<22} {value['count']:>5} {value['mean']:>10.2f} "
              f"{value['p50']:>10.2f} {value['p95']:>10.2f}")


def print_report(report):
    print(f"\n{report['name']} — {report['perf_windows']} PERF windows "
          f"(skipped {report['skipped_perf_windows']})")
    if report["modes"]:
        print("  " + " ".join(key + "=" + "/".join(value) for key, value in report["modes"].items()))
    print_metrics("PERF", report["perf"])
    if not report["perf_windows"]:
        print("  No PERF windows retained; FPS/age comparisons are unavailable.")
    print_metrics("Camera (logged snapshot timings, ms)", report["camera"])
    for eye, metrics in report["eyes"].items():
        print_metrics(f"Eye {eye} (logged snapshot timings, ms)", metrics)
    print_metrics("UI (ms except ui_hz/video_submit_fps)", report["ui"])
    if report["workers"]:
        print("Worker counter differences (end-to-end task time; not hardware utilization)")
    for worker, values in report["workers"].items():
        mean = values["task_ms_weighted"]
        mean_text = "unavailable (need >=2 valid counters)" if mean is None else f"{mean:.2f} ms/job"
        interval = values["interval_mean_task_ms"]
        tail = f"; interval-mean p50={interval['p50']:.2f}, p95={interval['p95']:.2f} ms/job" if interval else ""
        print(f"  worker={worker} core={','.join(values['cores']) or '?'} "
              f"delta_jobs={values['completed_jobs_delta']} delta_busy_ms={values['busy_ms_delta']:.2f} "
              f"weighted={mean_text}{tail}; resets={values['counter_resets']} "
              f"anomalies={values['anomalous_intervals']}")
    total = report["worker_total"]["task_ms_weighted"]
    if total is not None:
        print(f"  All workers: weighted {total:.2f} ms/single-eye job (from counter differences)")
    windows = report["worker_total"]["window_stages"]
    if windows["windows"]:
        print(f"Completed-job window stages ({windows['completed_jobs']} single-eye jobs; "
              "mean weighted by jobs, p50/p95 of window means)")
        for key, mean in windows["weighted_means_ms"].items():
            distribution = windows["window_mean_stats"][key]
            print(f"  {key:<22} weighted_mean={mean:.2f} "
                  f"window_p50={distribution['p50']:.2f} window_p95={distribution['p95']:.2f}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", type=Path, help="one log for a summary, two or more for A/B comparisons")
    parser.add_argument("--skip-perf", type=int, default=0, metavar="N",
                        help="discard the first N PERF windows and their STAGE snapshots in every log")
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args(argv)
    if args.skip_perf < 0:
        parser.error("--skip-perf must be nonnegative")
    reports = []
    for path in args.logs:
        try:
            content = path.read_text(encoding="utf-8", errors="replace")
        except OSError as error:
            parser.error(f"cannot read {path}: {error}")
        reports.append(summarize(content, str(path), args.skip_perf))
    comparisons = [compare(reports[0], report) for report in reports[1:]]
    if args.json:
        print(json.dumps({"logs": reports, "comparisons": comparisons}, ensure_ascii=False, indent=2, allow_nan=False))
    else:
        for report in reports:
            print_report(report)
        for result in comparisons:
            print(f"\nA/B: {result['baseline']} -> {result['candidate']} (mean of PERF windows)")
            if result["mode_differences"]:
                print("  Conditions differ: " + json.dumps(result["mode_differences"], ensure_ascii=False, sort_keys=True))
            for key, values in result["perf"].items():
                change = values["change_percent"]
                change_text = f"{change:+.1f}%" if change is not None else "percent unavailable (baseline zero)"
                print(f"  {key}: {values['before_mean']:.2f} -> {values['after_mean']:.2f} "
                      f"({change_text})")
            for key, values in result["worker_window_stages"].items():
                print(f"  worker {key}: {values['before_mean']:.2f} -> {values['after_mean']:.2f} ms "
                      f"(delta {values['delta']:+.2f} ms; job weighted)")
        print("\nNotes:")
        for note in NOTES:
            print("  " + note)
    return 0


if __name__ == "__main__":
    sys.exit(main())
