"""Validate log-window statistics, counter differences and honest tail labels."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/compare-performance.py"
SPEC = importlib.util.spec_from_file_location("compare_performance", SCRIPT)
TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TOOL)


LOG = """unrelated camera startup
[21:01:02] PERF capture_fps=15 preview_fps=14 result_fps=10 inference_ms=160 frame_age_ms=4 box_age_ms=220 detect=on stereo=pipeline3 preview=async npu_workers=3 core_binding=pinned
STAGE camera_frame=100 wait_ms=40 color_ms=25 preview_ms=3
STAGE eye=L frame=97 preprocess_ms=10 input_ms=20 run_ms=80 output_ms=5 total_ms=115 queue_ms=2 worker=0 core=0
STAGE eye=R frame=97 preprocess_ms=6 input_ms=14 run_ms=70 output_ms=4 total_ms=94 queue_ms=50 worker=1 core=1
STAGE npu_worker=0 core=0 jobs=1000 busy_ms=100000
STAGE npu_worker=1 core=1 jobs=2000 busy_ms=500000
PERF capture_fps=17 preview_fps=16 result_fps=14 inference_ms=140 frame_age_ms=6 box_age_ms=180 detect=on stereo=pipeline3 preview=async npu_workers=3 core_binding=pinned
STAGE camera_frame=130 wait_ms=35 color_ms=15 preview_ms=2
STAGE eye=L frame=127 preprocess_ms=8 input_ms=16 run_ms=70 output_ms=4 total_ms=98 queue_ms=1 worker=1 core=1
STAGE eye=R frame=127 preprocess_ms=4 input_ms=12 run_ms=60 output_ms=3 total_ms=79 queue_ms=40 worker=0 core=0
STAGE npu_worker=0 core=0 jobs=1020 busy_ms=102400
STAGE npu_worker=1 core=1 jobs=2010 busy_ms=501000
"""


class CompareTests(unittest.TestCase):
    def test_snapshot_stats_interpolate_percentiles_and_separate_eyes(self):
        report = TOOL.summarize(LOG)
        self.assertEqual(report["perf_windows"], 2)
        self.assertEqual(report["perf"]["result_fps"]["mean"], 12)
        self.assertEqual(report["perf"]["result_fps"]["p50"], 12)
        self.assertAlmostEqual(report["perf"]["result_fps"]["p95"], 13.8)
        self.assertEqual(report["perf"]["box_age_ms"]["mean"], 200)
        self.assertEqual(report["eyes"]["L"]["input_ms"]["mean"], 18)
        self.assertEqual(report["eyes"]["R"]["queue_ms"]["mean"], 45)
        self.assertEqual(report["camera"]["color_ms"]["mean"], 20)

    def test_counters_use_differences_and_job_weighted_mean(self):
        report = TOOL.summarize(LOG)
        self.assertEqual(report["workers"]["0"]["completed_jobs_delta"], 20)
        self.assertEqual(report["workers"]["0"]["task_ms_weighted"], 120)
        self.assertEqual(report["workers"]["1"]["task_ms_weighted"], 100)
        self.assertAlmostEqual(report["worker_total"]["task_ms_weighted"], 3400 / 30)
        self.assertNotIn("utilization", report["workers"]["0"])

    def test_first_cumulative_value_is_only_a_baseline(self):
        report = TOOL.summarize("STAGE npu_worker=0 core=0 jobs=9999 busy_ms=2000000\n")
        self.assertEqual(report["workers"]["0"]["completed_jobs_delta"], 0)
        self.assertIsNone(report["workers"]["0"]["task_ms_weighted"])

    def test_restarts_and_incoherent_counters_do_not_pollute_task_means(self):
        log = "\n".join([
            "STAGE npu_worker=0 core=0 jobs=100 busy_ms=10000",
            "STAGE npu_worker=0 core=0 jobs=120 busy_ms=12400",
            "STAGE npu_worker=0 core=0 jobs=1 busy_ms=100",  # process restart
            "STAGE npu_worker=0 core=0 jobs=3 busy_ms=300",
            "STAGE npu_worker=0 core=0 jobs=3 busy_ms=900",  # incoherent
            "STAGE npu_worker=0 core=0 jobs=4 busy_ms=1000",
        ])
        worker = TOOL.summarize(log)["workers"]["0"]
        self.assertEqual(worker["counter_resets"], 1)
        self.assertEqual(worker["anomalous_intervals"], 1)
        self.assertEqual(worker["completed_jobs_delta"], 23)
        self.assertAlmostEqual(worker["task_ms_weighted"], 2700 / 23)

    def test_skip_warmup_removes_associated_stages_and_counters(self):
        report = TOOL.summarize(LOG, skip_perf=1)
        self.assertEqual(report["perf_windows"], 1)
        self.assertEqual(report["perf"]["result_fps"]["mean"], 14)
        self.assertEqual(report["eyes"]["L"]["input_ms"]["mean"], 16)
        self.assertIsNone(report["workers"]["0"]["task_ms_weighted"])

    def test_invalid_and_unknown_ages_are_not_averaged(self):
        report = TOOL.summarize("PERF capture_fps=nan result_fps=inf frame_age_ms=-1 box_age_ms=-1\n"
                                "PERF capture_fps=15 result_fps=12 box_age_ms=200\n")
        self.assertEqual(report["perf"]["capture_fps"]["count"], 1)
        self.assertNotIn("frame_age_ms", report["perf"])
        self.assertEqual(report["perf"]["box_age_ms"]["mean"], 200)
        json.dumps(report, allow_nan=False)

    def test_window_stage_means_weight_by_completed_jobs_and_ignore_empty_windows(self):
        log = "\n".join([
            "STAGE npu_worker=0 core=0 jobs=100 busy_ms=10000 window_ms=2000 window_jobs=10 mean_eye_ms=100 mean_preprocess_ms=10 mean_input_ms=20 mean_run_ms=65 mean_output_ms=5",
            "STAGE npu_worker=1 core=1 jobs=200 busy_ms=20000 window_ms=2000 window_jobs=30 mean_eye_ms=120 mean_preprocess_ms=8 mean_input_ms=18 mean_run_ms=88 mean_output_ms=6",
            "STAGE npu_worker=2 core=2 jobs=0 busy_ms=0 window_ms=2000 window_jobs=0 mean_eye_ms=0 mean_run_ms=0",
        ])
        stages = TOOL.summarize(log)["worker_total"]["window_stages"]
        self.assertEqual(stages["windows"], 3)
        self.assertEqual(stages["completed_jobs"], 40)
        self.assertEqual(stages["weighted_means_ms"]["mean_eye_ms"], 115)
        self.assertEqual(stages["window_mean_stats"]["mean_eye_ms"]["p50"], 110)
        self.assertEqual(stages["window_mean_stats"]["mean_eye_ms"]["count"], 2)

    def test_cli_json_comparison_and_non_ascii_paths(self):
        with tempfile.TemporaryDirectory(prefix="performance comparison ") as temporary:
            old = Path(temporary) / "旧版.log"
            new = Path(temporary) / "新版.log"
            old.write_text("PERF result_fps=10 preview_fps=15 box_age_ms=200\n")
            new.write_text("PERF result_fps=12 preview_fps=15 box_age_ms=150\n")
            result = subprocess.run([sys.executable, str(SCRIPT), "--json", str(old), str(new)],
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads(result.stdout)
            comparison = payload["comparisons"][0]["perf"]
            self.assertEqual(comparison["result_fps"]["change_percent"], 20)
            self.assertEqual(comparison["box_age_ms"]["change_percent"], -25)
            self.assertIn("not hardware utilization", " ".join(payload["logs"][0]["notes"]))

    def test_cli_text_and_zero_baseline_do_not_claim_hardware_occupancy(self):
        with tempfile.TemporaryDirectory() as temporary:
            old, new = Path(temporary) / "old.log", Path(temporary) / "new.log"
            old.write_text("PERF result_fps=0\nSTAGE npu_worker=0 jobs=10 busy_ms=1000\n")
            new.write_text(LOG)
            result = subprocess.run([sys.executable, str(SCRIPT), str(old), str(new)],
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("percent unavailable (baseline zero)", result.stdout)
            self.assertIn("need >=2 valid counters", result.stdout)
            self.assertIn("not hardware utilization", result.stdout)

    def test_mode_differences_and_window_stage_comparison_are_explicit(self):
        old = TOOL.summarize("PERF result_fps=10 detect=on stereo=pipeline3\n"
                             "STAGE npu_worker=0 jobs=1 busy_ms=100 window_jobs=1 mean_input_ms=20\n", "old")
        new = TOOL.summarize("PERF result_fps=15 detect=off stereo=serial\n"
                             "STAGE npu_worker=0 jobs=1 busy_ms=80 window_jobs=1 mean_input_ms=10\n", "new")
        compared = TOOL.compare(old, new)
        self.assertEqual(compared["mode_differences"]["detect"], {"before": ["on"], "after": ["off"]})
        self.assertEqual(compared["worker_window_stages"]["mean_input_ms"]["delta"], -10)


if __name__ == "__main__":
    unittest.main()
