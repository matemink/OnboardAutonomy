#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import Counter
from collections.abc import Iterable
from itertools import pairwise
from pathlib import Path
from typing import Any


def percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def read_pts(path: Path) -> list[float]:
    timestamps: list[float] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        timestamps.append(float(line))
    return timestamps


def read_metadata(path: Path) -> list[dict[str, Any]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, list):
        raise ValueError("camera metadata root must be a JSON array")
    return [entry for entry in value if isinstance(entry, dict)]


def read_process_samples(path: Path) -> list[tuple[float, int, int]]:
    samples: list[tuple[float, int, int]] = []
    if not path.exists():
        return samples
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("elapsed_ms"):
            continue
        elapsed_ms, cpu_ticks, rss_kib = line.split("\t")
        samples.append((float(elapsed_ms), int(cpu_ticks), int(rss_kib)))
    return samples


def numeric_values(
    metadata: list[dict[str, Any]],
    key: str,
) -> list[float]:
    return [
        float(entry[key])
        for entry in metadata
        if isinstance(entry.get(key), (int, float))
    ]


def metric_summary(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "minimum": min(values),
        "average": statistics.fmean(values),
        "maximum": max(values),
    }


def estimate_missing_frames(
    intervals_ms: list[float],
    expected_interval_ms: float,
) -> int:
    missing = 0
    for interval in intervals_ms:
        represented_slots = max(1, round(interval / expected_interval_ms))
        missing += represented_slots - 1
    return missing


def build_summary(
    *,
    camera_model: str,
    width: int,
    height: int,
    target_fps: float,
    requested_frames: int,
    capture_status: int,
    timestamps_ms: list[float],
    metadata: list[dict[str, Any]],
    process_samples: list[tuple[float, int, int]],
    clock_ticks_per_second: int,
) -> dict[str, Any]:
    intervals_ms = [
        current - previous
        for previous, current in pairwise(timestamps_ms)
        if current > previous
    ]
    duration_seconds = (
        (timestamps_ms[-1] - timestamps_ms[0]) / 1000.0
        if len(timestamps_ms) >= 2
        else 0.0
    )
    measured_fps = (
        (len(timestamps_ms) - 1) / duration_seconds
        if duration_seconds > 0.0
        else 0.0
    )
    expected_interval_ms = 1000.0 / target_fps
    estimated_dropped_frames = estimate_missing_frames(
        intervals_ms,
        expected_interval_ms,
    )

    average_cpu_percent: float | None = None
    peak_rss_mib: float | None = None
    if process_samples:
        peak_rss_mib = max(sample[2] for sample in process_samples) / 1024.0
    if len(process_samples) >= 2:
        first = process_samples[0]
        last = process_samples[-1]
        sampled_seconds = (last[0] - first[0]) / 1000.0
        cpu_seconds = (
            (last[1] - first[1]) / clock_ticks_per_second
        )
        if sampled_seconds > 0.0:
            average_cpu_percent = cpu_seconds / sampled_seconds * 100.0

    maximum_allowed_drops = max(1, math.floor(requested_frames * 0.01))
    checks = {
        "capture_exit_success": capture_status == 0,
        "requested_frame_count_reached":
            len(timestamps_ms) == requested_frames,
        "metadata_count_matches":
            len(metadata) == len(timestamps_ms),
        "framerate_within_five_percent":
            measured_fps >= target_fps * 0.95,
        "estimated_drop_rate_at_most_one_percent":
            estimated_dropped_frames <= maximum_allowed_drops,
        "resource_samples_available": len(process_samples) >= 2,
    }

    autofocus_states = Counter(
        str(entry["AfState"])
        for entry in metadata
        if "AfState" in entry
    )

    return {
        "schema_version": 1,
        "result": "PASS" if all(checks.values()) else "FAIL",
        "camera": {
            "model": camera_model,
            "width": width,
            "height": height,
            "pixel_format": "YUV420",
            "target_fps": target_fps,
        },
        "capture": {
            "exit_status": capture_status,
            "requested_frames": requested_frames,
            "captured_frames": len(timestamps_ms),
            "metadata_frames": len(metadata),
            "duration_seconds": duration_seconds,
            "measured_fps": measured_fps,
            "estimated_dropped_frames": estimated_dropped_frames,
            "frame_interval_ms": {
                "minimum": min(intervals_ms) if intervals_ms else None,
                "average": (
                    statistics.fmean(intervals_ms)
                    if intervals_ms
                    else None
                ),
                "p95": percentile(intervals_ms, 0.95),
                "maximum": max(intervals_ms) if intervals_ms else None,
            },
        },
        "resources": {
            "sample_count": len(process_samples),
            "average_process_cpu_percent": average_cpu_percent,
            "peak_rss_mib": peak_rss_mib,
        },
        "sensor": {
            "temperature_c": metric_summary(
                numeric_values(metadata, "SensorTemperature")
            ),
            "exposure_time_us": metric_summary(
                numeric_values(metadata, "ExposureTime")
            ),
            "analogue_gain": metric_summary(
                numeric_values(metadata, "AnalogueGain")
            ),
            "lux": metric_summary(numeric_values(metadata, "Lux")),
            "focus_fom": metric_summary(
                numeric_values(metadata, "FocusFoM")
            ),
            "autofocus_states": dict(sorted(autofocus_states.items())),
        },
        "checks": checks,
    }


def format_number(value: float | None, digits: int = 2) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def render_markdown(summary: dict[str, Any]) -> str:
    camera = summary["camera"]
    capture = summary["capture"]
    resources = summary["resources"]
    sensor = summary["sensor"]
    intervals = capture["frame_interval_ms"]
    temperature = sensor["temperature_c"]

    lines = [
        "# OnboardAutonomy Camera Module 3 Benchmark",
        "",
        f"Result: **{summary['result']}**",
        "",
        "| Metric | Value |",
        "|---|---:|",
        f"| Camera | `{camera['model']}` |",
        (
            f"| Stream | {camera['width']}x{camera['height']} "
            f"{camera['pixel_format']} @ {camera['target_fps']:.2f} FPS |"
        ),
        (
            f"| Frames | {capture['captured_frames']} / "
            f"{capture['requested_frames']} |"
        ),
        f"| Measured FPS | {capture['measured_fps']:.3f} |",
        (
            "| Estimated dropped frames | "
            f"{capture['estimated_dropped_frames']} |"
        ),
        (
            "| Frame interval p95 | "
            f"{format_number(intervals['p95'], 3)} ms |"
        ),
        (
            "| Average process CPU | "
            f"{format_number(resources['average_process_cpu_percent'])}% |"
        ),
        (
            "| Peak RSS | "
            f"{format_number(resources['peak_rss_mib'])} MiB |"
        ),
        (
            "| Sensor temperature | "
            f"{format_number(temperature['average'] if temperature else None)}"
            " C |"
        ),
        "",
        "## Acceptance checks",
        "",
    ]
    for name, passed in summary["checks"].items():
        marker = "PASS" if passed else "FAIL"
        lines.append(f"- [{marker}] `{name}`")
    lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pts", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--camera-model", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--target-fps", type=float, required=True)
    parser.add_argument("--requested-frames", type=int, required=True)
    parser.add_argument("--capture-status", type=int, required=True)
    parser.add_argument("--clock-ticks", type=int, required=True)
    parser.add_argument("--report-json", type=Path, required=True)
    parser.add_argument("--report-markdown", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = build_summary(
        camera_model=args.camera_model,
        width=args.width,
        height=args.height,
        target_fps=args.target_fps,
        requested_frames=args.requested_frames,
        capture_status=args.capture_status,
        timestamps_ms=read_pts(args.pts),
        metadata=read_metadata(args.metadata),
        process_samples=read_process_samples(args.samples),
        clock_ticks_per_second=args.clock_ticks,
    )

    args.report_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    args.report_markdown.write_text(
        render_markdown(summary),
        encoding="utf-8",
    )
    print(render_markdown(summary))
    return 0 if summary["result"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
