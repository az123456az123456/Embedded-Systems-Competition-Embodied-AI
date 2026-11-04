"""Map exports shared by simulation and sensor-only replay."""
from __future__ import annotations
from dataclasses import asdict
import json
from pathlib import Path
import shutil


def export_navigation(navigator, config, directory: Path, summary: dict, log_path: Path | None = None) -> Path:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    names = ("map.pgm", "map.yaml", "map.json", "summary.json", "config.json")
    if log_path is not None:
        source = Path(log_path).resolve()
        if any((directory / name).resolve() == source for name in names):
            raise ValueError("导出目标与传感器源日志重名，请选择其它目录")
    resolved_targets = [(directory / name).resolve() for name in (*names, "telemetry.jsonl")]
    if len(set(resolved_targets)) != len(resolved_targets):
        raise ValueError("导出文件之间存在符号链接别名，请选择空目录")
    grid = navigator.grid
    # Image origin is top-left; occupancy grid and YAML world origin are bottom-left.
    pixels = bytearray()
    for row in reversed(range(grid.rows)):
        for col in range(grid.cols):
            state = grid.cell_state(col, row)
            pixels.append(0 if state == 1 else 254 if state == 0 else 205)
    (directory / "map.pgm").write_bytes(f"P5\n{grid.cols} {grid.rows}\n255\n".encode("ascii") + pixels)
    (directory / "map.yaml").write_text(
        f"image: map.pgm\nresolution: {grid.resolution}\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n", encoding="utf-8")
    (directory / "map.json").write_text(json.dumps({
        "width_m": grid.width, "height_m": grid.height, "resolution_m": grid.resolution,
        "cols": grid.cols, "rows": grid.rows, "origin": "bottom-left",
        "log_odds": grid.values, "seen": grid.seen,
        "trajectory": [asdict(p) for p in navigator.trajectory]}, ensure_ascii=False), encoding="utf-8")
    (directory / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (directory / "config.json").write_text(json.dumps(asdict(config), indent=2), encoding="utf-8")
    if log_path is not None:
        source = Path(log_path)
        destination = directory / "telemetry.jsonl"
        if source.resolve() != destination.resolve():
            shutil.copyfile(source, destination)
    return directory
