#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-${HOME}/build/onboard_autonomy-tidy}"

if ! command -v clang-tidy-18 >/dev/null; then
    echo "clang-tidy-18 is required. Run: sudo bash scripts/install_quality_tools.sh" >&2
    exit 1
fi

cmake \
    -S "${project_root}" \
    -B "${build_dir}" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_TESTING=OFF

run-clang-tidy-18 \
    -p "${build_dir}" \
    -j "${CLANG_TIDY_JOBS:-2}" \
    -quiet \
    -config-file="${project_root}/.clang-tidy" \
    "${project_root}/src/.*\\.cpp$"
