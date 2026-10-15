"""Convert a compatible Rockchip YOLOv8 ONNX on a development PC, never on the UI path."""
import argparse
import hashlib
import json
from pathlib import Path


def geometry(model):
    inputs = model.graph.input
    if len(inputs) != 1:
        raise ValueError("Expected one static RGB model input")
    shape = [d.dim_value for d in inputs[0].type.tensor_type.shape.dim]
    if len(shape) != 4 or shape[:2] != [1, 3] or not all(shape):
        raise ValueError("Expected static NCHW RGB input [1,3,H,W]")
    outputs = [[d.dim_value for d in o.type.tensor_type.shape.dim] for o in model.graph.output]
    if len(outputs) not in (6, 9):
        raise ValueError("Need 6/9 Rockchip YOLOv8 branch outputs; standard [1,84,8400] is incompatible")
    per_branch = len(outputs) // 3
    classes = outputs[1][1] if len(outputs[1]) == 4 else 0
    for branch in range(3):
        stride = 8 << branch
        boxes, scores = outputs[branch * per_branch:branch * per_branch + 2]
        if (len(boxes) != 4 or len(scores) != 4 or boxes[0] != 1 or scores[0] != 1
                or not 8 <= boxes[1] <= 256 or boxes[1] % 4
                or scores[1] != classes or classes < 1 or boxes[2:] != scores[2:]
                or boxes[2] * stride != shape[2] or boxes[3] * stride != shape[3]):
            raise ValueError(f"Incompatible branch {branch}: {boxes}, {scores}")
        if per_branch == 3 and outputs[branch * per_branch + 2] != [1, 1, *boxes[2:]]:
            raise ValueError("Invalid score-sum shape")
    return {"input": shape, "outputs": outputs, "classes": classes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("onnx", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--dataset", type=Path, help="Representative calibration list; enables INT8. Omit for non-INT8.")
    parser.add_argument("--check-only", action="store_true", help="Validate ONNX without importing RKNN toolkit")
    args = parser.parse_args()
    import onnx
    metadata = geometry(onnx.load(str(args.onnx), load_external_data=False))
    metadata.update(platform="rk3588", quantized=bool(args.dataset), onnx_sha256=hashlib.sha256(args.onnx.read_bytes()).hexdigest())
    if args.check_only:
        print(json.dumps(metadata, indent=2))
        return
    if args.dataset and not args.dataset.is_file():
        parser.error("Calibration list does not exist")
    from rknn.api import RKNN
    toolkit = RKNN()
    try:
        def check(code, operation):
            if code != 0:
                raise RuntimeError(f"{operation} failed: {code}")
        check(toolkit.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform="rk3588"), "config")
        check(toolkit.load_onnx(model=str(args.onnx)), "load_onnx")
        check(toolkit.build(do_quantization=bool(args.dataset), dataset=str(args.dataset) if args.dataset else None), "build")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        check(toolkit.export_rknn(str(args.output)), "export_rknn")
        metadata["rknn_sha256"] = hashlib.sha256(args.output.read_bytes()).hexdigest()
        args.output.with_suffix(args.output.suffix + ".json").write_text(json.dumps(metadata, indent=2) + "\n")
    finally:
        toolkit.release()


if __name__ == "__main__":
    main()
