#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${1:-${HOME}/build/onboard_autonomy-uml}"
output_dir="${project_root}/docs/diagrams/generated"

if ! command -v clang-uml >/dev/null; then
    echo "clang-uml is required. Run: sudo bash scripts/install_quality_tools.sh" >&2
    exit 1
fi

cmake \
    -S "${project_root}" \
    -B "${build_dir}" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_TESTING=OFF

mkdir -p "${output_dir}"
for diagram in architecture_packages autonomy_core runtime_wiring; do
    echo "Generating ${diagram}..."
    clang-uml \
        --config "${project_root}/.clang-uml" \
        --compile-database "${build_dir}" \
        --output-directory "${output_dir}" \
        --diagram-name "${diagram}" \
        --generator mermaid \
        --thread-count "${CLANG_UML_JOBS:-2}"
done
