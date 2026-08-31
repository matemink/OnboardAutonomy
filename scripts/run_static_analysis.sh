#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-${HOME}/build/onboard_autonomy-tidy}"
base_ref="${CLANG_TIDY_BASE_REF:-}"

scope_args=(--project-root "${project_root}")
if [[ -n "${base_ref}" ]]; then
    scope_args+=(--base-ref "${base_ref}")
fi

scope_output="$(python3 "${project_root}/scripts/clang_tidy_scope.py" \
    "${scope_args[@]}")"
mapfile -t scope_lines <<< "${scope_output}"
scope_mode="${scope_lines[0]}"

if [[ "${scope_mode}" == "skip" ]]; then
    echo "No C++ implementation or global analysis inputs changed; skipping clang-tidy."
    exit 0
fi

analysis_patterns=("${project_root}/src/.*\\.(cc|cpp|cxx)$")
if [[ "${scope_mode}" == "partial" ]]; then
    analysis_patterns=()
    for relative_path in "${scope_lines[@]:1}"; do
        analysis_patterns+=("${project_root}/${relative_path}")
    done
    printf 'Running clang-tidy on %d changed implementation file(s).\n' \
        "${#analysis_patterns[@]}"
else
    echo "Running full-project clang-tidy."
fi

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
    "${analysis_patterns[@]}"
