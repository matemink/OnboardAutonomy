#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly build_dir="${ONBOARD_AUTONOMY_GAZEBO_GUI_BUILD_DIR:-${HOME}/build/onboard_autonomy_gazebo_gui}"

if [[ ! -f "${build_dir}/build.ninja" ]]; then
    cmake \
        -S "${project_dir}/simulation/gui" \
        -B "${build_dir}" \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo >&2
fi
cmake --build "${build_dir}" --parallel >&2

printf '%s\n' "${build_dir}"
