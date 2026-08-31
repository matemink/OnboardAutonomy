#!/usr/bin/env python3

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

IMPLEMENTATION_SUFFIXES = {".cc", ".cpp", ".cxx"}
HEADER_SUFFIXES = {".h", ".hh", ".hpp", ".hxx", ".inc", ".inl", ".ipp"}
FULL_SCAN_FILES = {
    ".clang-tidy",
    ".github/workflows/ci.yml",
    "scripts/clang_tidy_scope.py",
    "scripts/run_static_analysis.sh",
}


@dataclass(frozen=True)
class AnalysisScope:
    mode: str
    files: tuple[str, ...] = ()


def requires_full_scan(path: PurePosixPath) -> bool:
    path_text = path.as_posix()
    return (
        path_text in FULL_SCAN_FILES
        or path.name == "CMakeLists.txt"
        or path.suffix == ".cmake"
        or path.parts[:1] == ("cmake",)
        or path.suffix in HEADER_SUFFIXES
    )


def select_scope(project_root: Path, changed_paths: list[str]) -> AnalysisScope:
    implementations: list[str] = []

    for changed_path in sorted(set(changed_paths)):
        path = PurePosixPath(changed_path)
        if requires_full_scan(path):
            return AnalysisScope("full")

        if path.parts[:1] != ("src",) or path.suffix not in IMPLEMENTATION_SUFFIXES:
            continue

        if not (project_root / Path(*path.parts)).is_file():
            return AnalysisScope("full")
        implementations.append(path.as_posix())

    if implementations:
        return AnalysisScope("partial", tuple(implementations))
    return AnalysisScope("skip")


def changed_paths(project_root: Path, base_ref: str) -> list[str] | None:
    verification = subprocess.run(
        ["git", "rev-parse", "--verify", f"{base_ref}^{{commit}}"],
        cwd=project_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if verification.returncode != 0:
        return None

    result = subprocess.run(
        ["git", "diff", "--name-only", f"{base_ref}...HEAD"],
        cwd=project_root,
        check=True,
        capture_output=True,
        text=True,
    )
    return [line for line in result.stdout.splitlines() if line]


def determine_scope(project_root: Path, base_ref: str | None) -> AnalysisScope:
    if not base_ref:
        return AnalysisScope("full")

    paths = changed_paths(project_root, base_ref)
    if paths is None:
        return AnalysisScope("full")
    return select_scope(project_root, paths)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--base-ref")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    project_root = args.project_root.resolve()
    scope = determine_scope(project_root, args.base_ref)
    print(scope.mode)
    for path in scope.files:
        print(path)


if __name__ == "__main__":
    main()
