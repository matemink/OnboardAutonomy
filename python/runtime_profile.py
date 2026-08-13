#!/usr/bin/env python3
"""Analyze bounded Raspberry Pi runtime resource samples."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections.abc import Iterable
from dataclasses import asdict, dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RuntimeSample:
    elapsed_ms: float
    cumulative_cpu_ticks: int
    rss_kib: int
    process_count: int
    temperature_c: float | None
    throttled: int | None


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


def read_samples(path: Path) -> list[RuntimeSample]:
    samples: list[RuntimeSample] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("elapsed_ms"):
            continue
        (
            elapsed_ms,
            cpu_ticks,
            rss_kib,
            process_count,
            temperature_millic,
            throttled_hex,
        ) = line.split("\t")
        samples.append(
            RuntimeSample(
                elapsed_ms=float(elapsed_ms),
                cumulative_cpu_ticks=int(cpu_ticks),
                rss_kib=int(rss_kib),
                process_count=int(process_count),
                temperature_c=(
                    None
                    if temperature_millic == "-"
                    else float(temperature_millic) / 1000.0
                ),
                throttled=(
                    None
                    if throttled_hex == "-"
                    else int(throttled_hex, 16)
                ),
            )
        )
    return samples


def build_summary(
    *,
    samples: list[RuntimeSample],
    requested_duration_seconds: float,
    runtime_status: int,
    clock_ticks_per_second: int,
    architecture: str,
    kernel: str,
) -> dict[str, Any]:
    if requested_duration_seconds <= 0.0:
        raise ValueError("requested_duration_seconds must be positive")
    if clock_ticks_per_second <= 0:
        raise ValueError("clock_ticks_per_second must be positive")

    duration_seconds = samples[-1].elapsed_ms / 1000.0 if samples else 0.0
    average_cpu_percent = None
    if duration_seconds > 0.0:
        average_cpu_percent = (
            samples[-1].cumulative_cpu_ticks
            / clock_ticks_per_second
            / duration_seconds
            * 100.0
        )

    interval_cpu_percent: list[float] = []
    for previous, current in pairwise(samples):
        elapsed_seconds = (current.elapsed_ms - previous.elapsed_ms) / 1000.0
        tick_delta = (
            current.cumulative_cpu_ticks - previous.cumulative_cpu_ticks
        )
        if elapsed_seconds > 0.0 and tick_delta >= 0:
            interval_cpu_percent.append(
                tick_delta
                / clock_ticks_per_second
                / elapsed_seconds
                * 100.0
            )

    temperatures = [
        sample.temperature_c
        for sample in samples
        if sample.temperature_c is not None
    ]
    throttling_values = [
        sample.throttled
        for sample in samples
        if sample.throttled is not None
    ]
    throttled_union = None
    if throttling_values:
        throttled_union = 0
        for value in throttling_values:
            throttled_union |= value

    checks = {
        "architecture_is_aarch64": architecture in {"aarch64", "arm64"},
        "sample_window_complete":
            duration_seconds >= requested_duration_seconds * 0.9,
        "process_samples_available": len(samples) >= 2,
        "runtime_exit_expected": runtime_status in {0, 130, 143},
        "throttling_data_available": throttled_union is not None,
        "no_throttling_observed": throttled_union == 0,
    }

    return {
        "schema_version": 1,
        "result": "PASS" if all(checks.values()) else "FAIL",
        "platform": {
            "architecture": architecture,
            "kernel": kernel,
        },
        "run": {
            "requested_duration_seconds": requested_duration_seconds,
            "sampled_duration_seconds": duration_seconds,
            "runtime_exit_status": runtime_status,
            "sample_count": len(samples),
        },
        "resources": {
            "average_process_group_cpu_percent": average_cpu_percent,
            "p95_process_group_cpu_percent": percentile(
                interval_cpu_percent,
                0.95,
            ),
            "peak_process_group_rss_mib": (
                max(sample.rss_kib for sample in samples) / 1024.0
                if samples
                else None
            ),
            "peak_process_count": (
                max(sample.process_count for sample in samples)
                if samples
                else 0
            ),
        },
        "thermal": {
            "average_temperature_c": (
                statistics.fmean(temperatures) if temperatures else None
            ),
            "maximum_temperature_c": (
                max(temperatures) if temperatures else None
            ),
            "throttled_bit_union": throttled_union,
        },
        "checks": checks,
        "samples": [asdict(sample) for sample in samples],
    }


def format_number(value: float | None, digits: int = 2) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def render_markdown(summary: dict[str, Any]) -> str:
    platform = summary["platform"]
    run = summary["run"]
    resources = summary["resources"]
    thermal = summary["thermal"]
    throttled = thermal["throttled_bit_union"]
    lines = [
        "# OnboardAutonomy Raspberry Pi Runtime Profile",
        "",
        f"Result: **{summary['result']}**",
        "",
        "| Metric | Value |",
        "|---|---:|",
        f"| Architecture | `{platform['architecture']}` |",
        f"| Kernel | `{platform['kernel']}` |",
        (
            "| Sampled duration | "
            f"{run['sampled_duration_seconds']:.2f} / "
            f"{run['requested_duration_seconds']:.2f} s |"
        ),
        f"| Runtime exit status | {run['runtime_exit_status']} |",
        (
            "| Average process-group CPU | "
            f"{format_number(resources['average_process_group_cpu_percent'])}% |"
        ),
        (
            "| p95 process-group CPU | "
            f"{format_number(resources['p95_process_group_cpu_percent'])}% |"
        ),
        (
            "| Peak process-group RSS | "
            f"{format_number(resources['peak_process_group_rss_mib'])} MiB |"
        ),
        f"| Peak process count | {resources['peak_process_count']} |",
        (
            "| Maximum SoC temperature | "
            f"{format_number(thermal['maximum_temperature_c'])} C |"
        ),
        (
            "| Throttled bit union | "
            f"{'n/a' if throttled is None else hex(throttled)} |"
        ),
        "",
        "## Acceptance checks",
        "",
    ]
    for name, passed in summary["checks"].items():
        lines.append(f"- [{'PASS' if passed else 'FAIL'}] `{name}`")
    lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--runtime-status", type=int, required=True)
    parser.add_argument("--clock-ticks", type=int, required=True)
    parser.add_argument("--architecture", required=True)
    parser.add_argument("--kernel", required=True)
    parser.add_argument("--report-json", type=Path, required=True)
    parser.add_argument("--report-markdown", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = build_summary(
        samples=read_samples(args.samples),
        requested_duration_seconds=args.duration,
        runtime_status=args.runtime_status,
        clock_ticks_per_second=args.clock_ticks,
        architecture=args.architecture,
        kernel=args.kernel,
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
