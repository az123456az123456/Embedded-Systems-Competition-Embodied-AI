#!/usr/bin/env python3
"""Host-only fixture test; no camera/NPU/phone or motion hardware is used."""
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/benchmark-perception-v34.sh"
SUMMARY = ROOT / "scripts/summarize-perception-v34.py"
checks = 0


def check(value):
    global checks
    checks += 1
    assert value


def run(*args, env=None):
    return subprocess.run(["sh", str(SCRIPT), *args], capture_output=True, text=True,
                          env=env, timeout=30)


def main():
    check(run("--help").returncode == 0)
    check(run("--duration", "0").returncode != 0)
    check(run("--duration", "1;false").returncode != 0)
    check(run("--unknown").returncode != 0)
    with tempfile.TemporaryDirectory(prefix="v34-benchmark-fixture-") as directory:
        temp = Path(directory)
        model = temp / "fixture.rknn"
        model.write_bytes(b"synthetic model identity, not an RKNN network")
        labels = temp / "labels.txt"
        labels.write_text("fixture\n")
        record = temp / "invocations.txt"
        fixture = temp / "launcher.sh"
        fixture.write_text('''#!/bin/sh
printf '%s|%s|%s\\n' "$PERCEPTION_PREPROCESS" "$PERCEPTION_INPUT" "$*" >> "$BENCHMARK_TEST_RECORD"
model=
while [ "$#" -gt 0 ];do
    case "$1" in --model) model=$2;shift;; esac
    shift
done
printf 'CONFIG model=%s labels=fixture camera=mock view=both phone=off\\n' "$model"
trap 'exit 0' TERM INT
while :;do
    echo 'PERF capture_fps=30 preview_fps=30 result_fps=25 eye_fps=50 inference_ms=80 frame_age_ms=2 box_age_ms=100 ai_skipped=10 completed_pairs=20 discarded_pairs=1 published_pairs=19 expired_pairs=0 pending_expired_pairs=0 out_of_order_pairs=1'
    echo 'STAGE eye=L frame=1 preprocess_ms=4 queue_ms=0.2'
    echo 'STAGE scheduler eye_completed=(20,20) leading_eyes=(10,10)'
    sleep 0.1
done
''')
        output = temp / "output"
        environment = dict(os.environ, BENCHMARK_TEST_RECORD=str(record))
        result = run("--duration", "1", "--warmup", "0", "--launcher", str(fixture),
                     "--output", str(output), "--model", str(model), "--labels", str(labels),
                     "--include-rga", "--headless", env=environment)
        check(result.returncode == 0)
        calls = record.read_text().splitlines()
        check(len(calls) == 4)
        check([line.split("|", 2)[:2] for line in calls] ==
              [["cpu", "auto"], ["pixel", "auto"], ["gather", "legacy"], ["rga", "auto"]])
        for line in calls:
            check("--no-phone" in line and "--no-navigation" in line and "--headless" in line)
            check("--drive" not in line and "--arm" not in line)
        logs = sorted(output.glob("*.measured.log"))
        check(len(logs) == 4)
        check(all("PERF " in log.read_text() for log in logs))
        summary = subprocess.run(["python3", str(SUMMARY), "--json", *map(str, logs)],
                                 capture_output=True, text=True, timeout=10, check=True)
        report = json.loads(summary.stdout)
        check(len(report["logs"]) == 4 and len(report["comparisons"]) == 3)
        for log in report["logs"]:
            check(log["perf"]["result_fps"]["mean"] == 25)
            check(log["perf"]["eye_fps"]["mean"] == 50)
            check(log["scheduler_counters"]["ai_skipped"]["delta"] == 0)
        check("case_model_sha256=" in (output / "metadata.txt").read_text())
        # Refuse a pre-existing app by process name, without terminating it.
        busy = subprocess.Popen(["python3", "-c", "import ctypes,time;ctypes.CDLL(None).prctl(15,b'perception-dash',0,0,0);print('ready',flush=True);time.sleep(20)"],
                                stdout=subprocess.PIPE, text=True)
        try:
            check(busy.stdout.readline().strip() == "ready")
            result = run("--duration", "1", "--warmup", "0", "--launcher", str(fixture),
                         "--output", str(temp / "busy"), "--model", str(model), "--labels", str(labels))
            check(result.returncode != 0 and "already active" in result.stderr)
            check(busy.poll() is None)
        finally:
            busy.terminate()
            busy.wait(timeout=5)
        # An exited child must abort, without entering later comparison modes.
        fixture.write_text("#!/bin/sh\nexit 17\n")
        result = run("--duration", "1", "--warmup", "0", "--launcher", str(fixture),
                     "--output", str(temp / "failure"), "--model", str(model), "--labels", str(labels))
        check(result.returncode != 0 and "first PERF" in result.stderr)
    print(f"PASS v34 benchmark/summary fixture checks={checks}; no hardware speed measured")


if __name__ == "__main__":
    main()
