#!/usr/bin/env bash

set -euo pipefail

readonly gazebo_source_dir="${ARDUPILOT_GAZEBO_SOURCE_DIR:-${HOME}/src/ardupilot_gazebo}"
readonly gazebo_build_dir="${ARDUPILOT_GAZEBO_BUILD_DIR:-${HOME}/build/ardupilot_gazebo}"
readonly world_file="${ONBOARD_AUTONOMY_GAZEBO_WORLD:-${gazebo_source_dir}/worlds/iris_runway.sdf}"
readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"

if ! command -v gz >/dev/null 2>&1; then
    printf 'Gazebo is not installed. Run scripts/install_gazebo_harmonic.sh first.\n' >&2
    exit 1
fi

if [[ ! -f "${gazebo_build_dir}/libArduPilotPlugin.so" ]]; then
    printf 'ArduPilot Gazebo plugin is not built: %s\n' \
        "${gazebo_build_dir}" >&2
    exit 1
fi

if [[ ! -f "${world_file}" ]]; then
    printf 'Gazebo world is missing: %s\n' "${world_file}" >&2
    exit 1
fi

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="${gazebo_build_dir}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${project_dir}/simulation/models:${project_dir}/simulation/worlds:${gazebo_source_dir}/models:${gazebo_source_dir}/worlds:${GZ_SIM_RESOURCE_PATH:-}"

source "${script_dir}/require_gazebo_gpu.sh"

printf 'OnboardAutonomy Gazebo world: %s\n' "${world_file}"

declare -a gazebo_args=(-v4 -r)
if [[ "${ONBOARD_AUTONOMY_GAZEBO_HEADLESS:-0}" == "1" ]]; then
    gazebo_args+=(-s)
fi

exec gz sim "${gazebo_args[@]}" "${world_file}"
