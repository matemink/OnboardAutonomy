#!/usr/bin/env python3
"""Bound retained OnboardAutonomy JSONL logs by count and total bytes."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
from typing import TextIO


@dataclass(frozen=True)
class RotationResult:
    kept: tuple[str, ...]
    removed: tuple[str, ...]
    retained_bytes: int


def rotate_jsonl_logs(
    directory: Path,
    max_files: int,
    max_total_bytes: int,
) -> RotationResult:
    if max_files < 1:
        raise ValueError("max_files must be at least 1")
    if max_total_bytes < 1:
        raise ValueError("max_total_bytes must be at least 1")

    directory.mkdir(parents=True, exist_ok=True)
    candidates = sorted(
        (
            path
            for path in directory.glob("telemetry-*.jsonl")
            if path.is_file() and not path.is_symlink()
        ),
        key=lambda path: (path.stat().st_mtime_ns, path.name),
        reverse=True,
    )

    kept: list[str] = []
    removed: list[str] = []
    retained_bytes = 0
    for path in candidates:
        size = path.stat().st_size
        keep_newest = not kept
        within_count = len(kept) < max_files
        within_bytes = retained_bytes + size <= max_total_bytes
        if keep_newest or (within_count and within_bytes):
            kept.append(path.name)
            retained_bytes += size
            continue

        path.unlink()
        removed.append(path.name)

    return RotationResult(
        kept=tuple(kept),
        removed=tuple(removed),
        retained_bytes=retained_bytes,
    )


def stream_jsonl_logs(
    source: TextIO,
    mirror: TextIO,
    directory: Path,
    stem: str,
    max_files: int,
    max_total_bytes: int,
    max_file_bytes: int,
) -> RotationResult:
    if max_file_bytes < 1:
        raise ValueError("max_file_bytes must be at least 1")

    result = rotate_jsonl_logs(directory, max_files, max_total_bytes)
    sequence = 0
    active_size = 0
    active_file = None

    try:
        for line in source:
            payload = line.encode("utf-8")
            if active_file is None or (
                active_size > 0
                and active_size + len(payload) > max_file_bytes
            ):
                if active_file is not None:
                    active_file.close()
                    active_file = None
                while active_file is None:
                    suffix = "" if sequence == 0 else f"-{sequence:03d}"
                    path = directory / f"{stem}{suffix}.jsonl"
                    sequence += 1
                    try:
                        active_file = path.open("xb")
                    except FileExistsError:
                        continue
                active_size = 0

            active_file.write(payload)
            active_file.flush()
            active_size += len(payload)
            mirror.write(line)
            mirror.flush()
            result = rotate_jsonl_logs(
                directory,
                max_files,
                max_total_bytes,
            )
    finally:
        if active_file is not None:
            active_file.close()

    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--max-files", type=int, default=20)
    parser.add_argument(
        "--max-total-bytes",
        type=int,
        default=100 * 1024 * 1024,
    )
    parser.add_argument("--max-file-bytes", type=int, default=10 * 1024 * 1024)
    parser.add_argument("--stream", action="store_true")
    parser.add_argument("--stem")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    directory = args.directory.expanduser()
    try:
        if args.stream:
            stem = args.stem or (
                "telemetry-"
                + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                + f"-{os.getpid()}"
            )
            stream_jsonl_logs(
                sys.stdin,
                sys.stdout,
                directory,
                stem,
                args.max_files,
                args.max_total_bytes,
                args.max_file_bytes,
            )
        else:
            result = rotate_jsonl_logs(
                directory,
                args.max_files,
                args.max_total_bytes,
            )
            print(json.dumps(asdict(result), sort_keys=True))
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
