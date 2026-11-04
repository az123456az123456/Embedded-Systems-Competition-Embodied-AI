#!/usr/bin/env python3
"""Optional developer test of production C++ preprocessing and decoding.

Requires numpy, opencv-python and onnxruntime on the development PC. Nothing is
downloaded. The default model/image are found in this workspace's YOLO folder;
use --onnx MODEL --image IMAGE after copying this source package elsewhere.
This checks implementation consistency, not detection accuracy or mAP gains.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parent.parent


def find_asset(relative):
    for parent in Path(__file__).resolve().parents:
        candidate = parent / relative
        if candidate.is_file():
            return candidate
    return None


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_json(args):
    return json.loads(subprocess.check_output([str(value) for value in args], text=True))


def python_decode(outputs, geometry, confidence, nms, np):
    """Independent vectorized DFL/box decode and greedy class-aware NMS."""
    candidates = []
    per = len(outputs) // 3
    for branch in range(3):
        distribution = outputs[branch * per][0].astype(np.float64)
        scores = outputs[branch * per + 1][0]
        channels, height, width = distribution.shape
        bins = channels // 4
        distribution = distribution.reshape(4, bins, height, width)
        distribution -= distribution.max(axis=1, keepdims=True)
        probability = np.exp(distribution)
        probability /= probability.sum(axis=1, keepdims=True)
        distance = (probability * np.arange(bins)[None, :, None, None]).sum(axis=1)
        cls = scores.argmax(axis=0)
        score = scores.max(axis=0)
        row, col = np.nonzero(np.isfinite(score) & (score >= confidence))
        for y, x in zip(row, col):
            if per == 3 and (not np.isfinite(outputs[branch * per + 2][0, 0, y, x]) or
                             outputs[branch * per + 2][0, 0, y, x] < confidence):
                continue
            sx = geometry["target_w"] / width
            sy = geometry["target_h"] / height
            left = ((x + 0.5 - distance[0, y, x]) * sx - geometry["pad_x"]) / geometry["scale_x"]
            top = ((y + 0.5 - distance[1, y, x]) * sy - geometry["pad_y"]) / geometry["scale_y"]
            right = ((x + 0.5 + distance[2, y, x]) * sx - geometry["pad_x"]) / geometry["scale_x"]
            bottom = ((y + 0.5 + distance[3, y, x]) * sy - geometry["pad_y"]) / geometry["scale_y"]
            left, right = np.clip([left, right], 0, geometry["source_w"])
            top, bottom = np.clip([top, bottom], 0, geometry["source_h"])
            if np.isfinite([left, top, right, bottom]).all() and right > left and bottom > top:
                candidates.append(dict(left=float(left), top=float(top), right=float(right),
                                       bottom=float(bottom), score=float(score[y, x]), class_id=int(cls[y, x])))
    candidates.sort(key=lambda d: d["score"], reverse=True)
    kept = []
    for candidate in candidates[:3000]:
        area = (candidate["right"] - candidate["left"]) * (candidate["bottom"] - candidate["top"])
        for other in kept:
            if candidate["class_id"] != other["class_id"]:
                continue
            intersection = max(0, min(candidate["right"], other["right"]) - max(candidate["left"], other["left"])) * max(
                0, min(candidate["bottom"], other["bottom"]) - max(candidate["top"], other["top"]))
            other_area = (other["right"] - other["left"]) * (other["bottom"] - other["top"])
            if intersection / (area + other_area - intersection) > nms:
                break
        else:
            kept.append(candidate)
            if len(kept) == 128:
                break
    return kept


def compare(cpp, python):
    if len(cpp) != len(python):
        raise AssertionError(f"C++ and Python detection count differ: {len(cpp)} vs {len(python)}")
    max_box_delta = max_score_delta = 0.0
    unmatched = list(python)
    for result in cpp:
        choices = [other for other in unmatched if other["class_id"] == result["class_id"]]
        if not choices:
            raise AssertionError("C++ and Python class IDs differ")
        match = min(choices, key=lambda d: sum(abs(d[k] - result[k]) for k in ("left", "top", "right", "bottom")))
        box_delta = max(abs(result[k] - match[k]) for k in ("left", "top", "right", "bottom"))
        score_delta = abs(result["score"] - match["score"])
        if box_delta > 0.02 or score_delta > 0.00001:
            raise AssertionError(f"Decoder mismatch: box delta {box_delta}, confidence delta {score_delta}")
        max_box_delta = max(max_box_delta, box_delta)
        max_score_delta = max(max_score_delta, score_delta)
        unmatched.remove(match)
    return max_box_delta, max_score_delta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onnx", type=Path, default=find_asset("YOLO/yolov8s_airockchip.onnx"))
    bundled_image = find_asset("YOLO/rknn_model_zoo/examples/yolov8/model/bus.jpg")
    parser.add_argument("--image", type=Path, default=bundled_image)
    parser.add_argument("--probe", type=Path, default=ROOT / "build/onnx-decode-probe")
    parser.add_argument("--report", type=Path, default=ROOT / "reports/onnx_validation.json")
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--nms", type=float, default=0.45)
    parser.add_argument("--classes", type=int, default=80)
    parser.add_argument("--require-classes", help="Comma-separated numeric IDs; defaults to 0,5 for bundled bus.jpg")
    args = parser.parse_args()
    for field in ("onnx", "image", "probe"):
        value = getattr(args, field)
        if value is None or not value.is_file():
            parser.error(f"Missing {field}; provide --{field} with an existing local file")
        setattr(args, field, value.resolve())
    if not (math.isfinite(args.confidence) and 0 <= args.confidence <= 1 and
            math.isfinite(args.nms) and 0 <= args.nms <= 1):
        parser.error("confidence and NMS must be finite values from 0 to 1")
    try:
        import cv2
        import numpy as np
        import onnxruntime as ort
    except ImportError as error:
        parser.error(f"Optional development dependencies unavailable: {error}; install numpy, opencv-python, onnxruntime separately")
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error("Cannot read the supplied image")
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    options = ort.SessionOptions()
    options.intra_op_num_threads = min(4, os.cpu_count() or 1)
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.onnx), sess_options=options, providers=["CPUExecutionProvider"])
    if len(session.get_inputs()) != 1 or len(session.get_outputs()) not in (6, 9):
        raise AssertionError("Expected the Rockchip 6/9-output model, not standard [1,84,8400] YOLO export")
    input_meta = session.get_inputs()[0]
    if input_meta.type != "tensor(float)" or len(input_meta.shape) != 4 or input_meta.shape[:2] != [1, 3]:
        raise AssertionError("Expected fixed batch-one RGB NCHW float ONNX input")
    target_h, target_w = input_meta.shape[2:]
    if not isinstance(target_h, int) or not isinstance(target_w, int):
        raise AssertionError("Dynamic ONNX dimensions are unsupported in this sanity test")
    require = ([int(value) for value in args.require_classes.split(",") if value.strip()]
               if args.require_classes is not None else
               ([0, 5] if bundled_image and args.image == bundled_image.resolve() else []))
    cases = [("whole_image", rgb)]
    if rgb.shape[1] > 64 and rgb.shape[0] > 64:
        # The bundled image is already 640 square. Enlarge it before cropping
        # so these cases actually exercise bilinear sampling and rounded
        # per-axis inverse scales, rather than only padding unscaled pixels.
        enlarged = cv2.resize(rgb, (rgb.shape[1] * 3 // 2 + 1, rgb.shape[0] * 3 // 2 + 1),
                              interpolation=cv2.INTER_LINEAR)
        cases.extend([("portrait_roi", enlarged[:, :enlarged.shape[1] // 2 + 1]),
                      ("landscape_roi", enlarged[:enlarged.shape[0] // 2 + 1, :])])
    report = dict(status="passed", purpose="Implementation parity and sample sanity; not mAP evaluation or hardware/RKNN validation",
                  model_sha256=sha256(args.onnx), image_sha256=sha256(args.image), model=args.onnx.name,
                  image=args.image.name, confidence=args.confidence, nms=args.nms,
                  onnxruntime=ort.__version__, required_whole_image_classes=require, cases=[])
    with tempfile.TemporaryDirectory(prefix="perception-onnx-") as temporary:
        directory = Path(temporary)
        for name, source in cases:
            source = np.ascontiguousarray(source)
            source_h, source_w = source.shape[:2]
            raw, processed = directory / "source.bin", directory / "letterbox.bin"
            source.tofile(raw)
            geometry = run_json([args.probe, "preprocess", raw, processed, source_w, source_h, target_w, target_h])
            cpp_pixels = np.fromfile(processed, dtype=np.uint8).reshape(target_h, target_w, 3)
            reference = np.full((target_h, target_w, 3), 114, dtype=np.uint8)
            resized = cv2.resize(source, (geometry["resized_w"], geometry["resized_h"]), interpolation=cv2.INTER_LINEAR)
            px, py = geometry["pad_x"], geometry["pad_y"]
            reference[py:py + resized.shape[0], px:px + resized.shape[1]] = resized
            difference = np.abs(cpp_pixels.astype(np.int16) - reference.astype(np.int16))
            if difference.max() > 2:
                raise AssertionError(f"{name}: C++ bilinear preprocessing disagrees with OpenCV by {difference.max()} levels")
            tensor = np.ascontiguousarray(cpp_pixels.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
            outputs = session.run(None, {input_meta.name: tensor})
            per = len(outputs) // 3
            bins = outputs[0].shape[1] // 4
            for index, output in enumerate(outputs):
                branch, part = divmod(index, per)
                channels = bins * 4 if part == 0 else (args.classes if part == 1 else 1)
                expected = (1, channels, target_h // (8 << branch), target_w // (8 << branch))
                if output.shape != expected or not np.isfinite(output).all():
                    raise AssertionError(f"{name}: invalid output {index} shape/values: {output.shape}; expected {expected}")
                np.asarray(output, dtype="<f4").tofile(directory / f"output_{index}.bin")
            decoded = run_json([args.probe, "decode", directory, source_w, source_h, target_w, target_h,
                                args.classes, bins, len(outputs), args.confidence, args.nms])["detections"]
            python_results = python_decode(outputs, geometry, args.confidence, args.nms, np)
            box_delta, score_delta = compare(decoded, python_results)
            for detection in decoded:
                if not (0 <= detection["left"] < detection["right"] <= source_w and
                        0 <= detection["top"] < detection["bottom"] <= source_h and
                        args.confidence <= detection["score"] <= 1):
                    raise AssertionError(f"{name}: invalid ROI box or probability: {detection}")
            if name == "whole_image":
                if not decoded:
                    raise AssertionError("No sample detections; check model/input preprocessing")
                missing = set(require) - {d["class_id"] for d in decoded}
                if missing:
                    raise AssertionError(f"Missing required sample classes {sorted(missing)}")
            report["cases"].append(dict(name=name, geometry=geometry, detections=decoded,
                                        python_detection_count=len(python_results), max_box_delta_pixels=box_delta,
                                        max_score_delta=score_delta, preprocessing_opencv_max_difference=int(difference.max()),
                                        preprocessing_opencv_mean_difference=float(difference.mean())))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("PASS real ONNX pipeline: " + ", ".join(f"{case['name']} {len(case['detections'])} detections" for case in report["cases"]))
    print(f"C++/independent Python decoder agreement recorded in {args.report}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (AssertionError, subprocess.CalledProcessError, ValueError, OSError) as error:
        print(f"FAIL ONNX pipeline: {error}", file=sys.stderr)
        sys.exit(1)
