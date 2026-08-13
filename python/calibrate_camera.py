#!/usr/bin/env python3

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2 as cv
import numpy as np

DISTORTION_ORDER = ("k1", "k2", "p1", "p2", "k3")


@dataclass(frozen=True)
class CalibrationViews:
    image_size: tuple[int, int]
    object_points: list[np.ndarray]
    image_points: list[np.ndarray]
    accepted: list[Path]
    rejected: list[dict[str, str]]


@dataclass(frozen=True)
class CalibrationSolution:
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    rotation_vectors: Sequence[np.ndarray]
    translation_vectors: Sequence[np.ndarray]
    rms_error_px: float
    per_view_rms_error_px: list[float]


def parse_pattern_size(value: str) -> tuple[int, int]:
    normalized = value.lower().replace("*", "x")
    parts = normalized.split("x")
    if len(parts) != 2:
        raise ValueError("pattern must use COLSxROWS, for example 9x6")
    try:
        columns, rows = (int(part) for part in parts)
    except ValueError as error:
        raise ValueError("pattern dimensions must be integers") from error
    if columns < 2 or rows < 2:
        raise ValueError("pattern must have at least 2x2 inner corners")
    return columns, rows


def checkerboard_object_points(
    pattern_size: tuple[int, int],
    square_size_m: float,
) -> np.ndarray:
    if not math.isfinite(square_size_m) or square_size_m <= 0.0:
        raise ValueError("square size must be a positive finite value")
    columns, rows = pattern_size
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] = grid.astype(np.float32) * square_size_m
    return points


def discover_images(directory: Path, image_glob: str) -> list[Path]:
    if not directory.is_dir():
        raise ValueError(f"image directory does not exist: {directory}")
    images = sorted(path for path in directory.glob(image_glob) if path.is_file())
    if not images:
        raise ValueError(
            f"no calibration images matched {image_glob!r} in {directory}"
        )
    return images


def detect_views(
    image_paths: Sequence[Path],
    pattern_size: tuple[int, int],
    square_size_m: float,
) -> CalibrationViews:
    template = checkerboard_object_points(pattern_size, square_size_m)
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    accepted: list[Path] = []
    rejected: list[dict[str, str]] = []
    image_size: tuple[int, int] | None = None
    flags = (
        cv.CALIB_CB_NORMALIZE_IMAGE
        | cv.CALIB_CB_EXHAUSTIVE
        | cv.CALIB_CB_ACCURACY
    )

    for path in image_paths:
        image = cv.imread(str(path), cv.IMREAD_GRAYSCALE)
        if image is None:
            rejected.append({"file": path.name, "reason": "unreadable image"})
            continue
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            rejected.append(
                {
                    "file": path.name,
                    "reason": (
                        f"resolution {current_size[0]}x{current_size[1]} does "
                        f"not match {image_size[0]}x{image_size[1]}"
                    ),
                }
            )
            continue

        found, corners = cv.findChessboardCornersSB(
            image,
            pattern_size,
            flags=flags,
        )
        if not found or corners is None:
            rejected.append(
                {"file": path.name, "reason": "complete checkerboard not found"}
            )
            continue
        object_points.append(template.copy())
        image_points.append(corners.astype(np.float32))
        accepted.append(path)

    if image_size is None:
        raise ValueError("none of the calibration images could be decoded")
    return CalibrationViews(
        image_size=image_size,
        object_points=object_points,
        image_points=image_points,
        accepted=accepted,
        rejected=rejected,
    )


def calibrate_points(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    image_size: tuple[int, int],
) -> CalibrationSolution:
    if len(object_points) != len(image_points) or not object_points:
        raise ValueError("object and image point views must be non-empty and aligned")
    if image_size[0] <= 0 or image_size[1] <= 0:
        raise ValueError("image size must be positive")

    rms_error, camera_matrix, distortion, rotations, translations = (
        cv.calibrateCamera(
            list(object_points),
            list(image_points),
            image_size,
            None,
            None,
        )
    )
    distortion_values = distortion.reshape(-1)
    if distortion_values.size != len(DISTORTION_ORDER):
        raise RuntimeError(
            "expected the five-coefficient OpenCV pinhole distortion model"
        )

    per_view_errors: list[float] = []
    for world_points, observed, rotation, translation in zip(
        object_points,
        image_points,
        rotations,
        translations,
        strict=True,
    ):
        projected, _ = cv.projectPoints(
            world_points,
            rotation,
            translation,
            camera_matrix,
            distortion,
        )
        residuals = observed.reshape(-1, 2) - projected.reshape(-1, 2)
        squared_distance = np.sum(residuals * residuals, axis=1)
        per_view_errors.append(float(np.sqrt(np.mean(squared_distance))))

    return CalibrationSolution(
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_values,
        rotation_vectors=rotations,
        translation_vectors=translations,
        rms_error_px=float(rms_error),
        per_view_rms_error_px=per_view_errors,
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_document(
    *,
    camera_model: str,
    focus_mode: str,
    lens_position: str,
    pattern_size: tuple[int, int],
    square_size_m: float,
    minimum_views: int,
    maximum_rms_error_px: float,
    maximum_view_error_px: float,
    views: CalibrationViews,
    solution: CalibrationSolution,
) -> dict[str, Any]:
    if len(views.accepted) != len(solution.per_view_rms_error_px):
        raise ValueError("accepted views and reprojection errors are not aligned")
    matrix = solution.camera_matrix
    distortion = solution.distortion_coefficients
    maximum_view_error = max(solution.per_view_rms_error_px)
    checks = {
        "minimum_views_reached": len(views.accepted) >= minimum_views,
        "rms_error_within_limit": (
            solution.rms_error_px <= maximum_rms_error_px
        ),
        "every_view_error_within_limit": (
            maximum_view_error <= maximum_view_error_px
        ),
    }
    accepted = [
        {
            "file": path.name,
            "sha256": sha256(path),
            "rms_reprojection_error_px": error,
        }
        for path, error in zip(
            views.accepted,
            solution.per_view_rms_error_px,
            strict=True,
        )
    ]
    return {
        "schema_version": 1,
        "result": "PASS" if all(checks.values()) else "FAIL",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "camera": {
            "model": camera_model,
            "width": views.image_size[0],
            "height": views.image_size[1],
            "focus_mode": focus_mode,
            "lens_position": lens_position,
        },
        "model": "opencv_pinhole_brown_conrady_5",
        "pattern": {
            "inner_corners_columns": pattern_size[0],
            "inner_corners_rows": pattern_size[1],
            "square_size_m": square_size_m,
        },
        "intrinsics": {
            "fx_px": float(matrix[0, 0]),
            "fy_px": float(matrix[1, 1]),
            "cx_px": float(matrix[0, 2]),
            "cy_px": float(matrix[1, 2]),
            "camera_matrix": [
                [float(value) for value in row]
                for row in matrix.tolist()
            ],
        },
        "distortion": {
            "coefficient_order": list(DISTORTION_ORDER),
            "coefficients": [float(value) for value in distortion],
        },
        "quality": {
            "rms_reprojection_error_px": solution.rms_error_px,
            "mean_view_rms_error_px": statistics.fmean(
                solution.per_view_rms_error_px
            ),
            "maximum_view_rms_error_px": maximum_view_error,
            "maximum_allowed_rms_error_px": maximum_rms_error_px,
            "maximum_allowed_view_error_px": maximum_view_error_px,
            "checks": checks,
        },
        "views": {
            "provided": len(views.accepted) + len(views.rejected),
            "accepted": accepted,
            "rejected": views.rejected,
        },
        "tool": {
            "name": "OnboardAutonomy camera calibration",
            "opencv_version": cv.__version__,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate a camera from checkerboard images.",
    )
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--image-glob", default="*.jpg")
    parser.add_argument("--pattern", default="9x6")
    parser.add_argument("--square-size-mm", type=float, default=25.0)
    parser.add_argument("--camera-model", default="imx708_wide")
    parser.add_argument("--focus-mode", default="manual")
    parser.add_argument("--lens-position", default="default")
    parser.add_argument("--minimum-views", type=int, default=10)
    parser.add_argument("--maximum-rms-error-px", type=float, default=1.0)
    parser.add_argument("--maximum-view-error-px", type=float, default=1.5)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        pattern_size = parse_pattern_size(args.pattern)
        if args.minimum_views < 3:
            raise ValueError("minimum views must be at least 3")
        square_size_m = args.square_size_mm / 1000.0
        images = discover_images(args.images, args.image_glob)
        views = detect_views(images, pattern_size, square_size_m)
        if len(views.accepted) < args.minimum_views:
            raise ValueError(
                f"only {len(views.accepted)} complete views were detected; "
                f"at least {args.minimum_views} are required"
            )
        solution = calibrate_points(
            views.object_points,
            views.image_points,
            views.image_size,
        )
        document = build_document(
            camera_model=args.camera_model,
            focus_mode=args.focus_mode,
            lens_position=args.lens_position,
            pattern_size=pattern_size,
            square_size_m=square_size_m,
            minimum_views=args.minimum_views,
            maximum_rms_error_px=args.maximum_rms_error_px,
            maximum_view_error_px=args.maximum_view_error_px,
            views=views,
            solution=solution,
        )
    except (ValueError, RuntimeError, cv.error) as error:
        print(f"Calibration failed: {error}", file=sys.stderr)
        return 2

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("OnboardAutonomy camera calibration")
    print(f"  Result: {document['result']}")
    print(
        "  Views: "
        f"{len(views.accepted)} accepted, {len(views.rejected)} rejected"
    )
    print(f"  RMS reprojection error: {solution.rms_error_px:.4f} px")
    print(f"  Output: {args.output}")
    return 0 if document["result"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
