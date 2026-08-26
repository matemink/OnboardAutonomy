#!/usr/bin/env bash

set -euo pipefail

readonly gazebo_source_dir="${ARDUPILOT_GAZEBO_SOURCE_DIR:-${HOME}/src/ardupilot_gazebo}"
readonly gazebo_build_dir="${ARDUPILOT_GAZEBO_BUILD_DIR:-${HOME}/build/ardupilot_gazebo}"
readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly gui_config="${project_dir}/simulation/gui/onboard_autonomy.config"

if [[ "${ONBOARD_AUTONOMY_GAZEBO_WEATHER:-0}" == "1" ]]; then
    source "${script_dir}/weather_profile.sh"
else
    export ONBOARD_AUTONOMY_WIND_SPEED_M_S=0
    export ONBOARD_AUTONOMY_WIND_FROM_DEG=0
    export ONBOARD_AUTONOMY_WIND_TURBULENCE_M_S=0
fi

gui_plugin_build_dir="$(
    bash "${script_dir}/build_gazebo_gui_plugins.sh"
)"

if ! command -v gz >/dev/null 2>&1; then
    printf 'Gazebo is not installed. Run scripts/install_gazebo_harmonic.sh first.\n' >&2
    exit 1
fi

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="${gazebo_build_dir}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${project_dir}/simulation/models:${project_dir}/simulation/worlds:${gazebo_source_dir}/models:${gazebo_source_dir}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_GUI_PLUGIN_PATH="${gui_plugin_build_dir}:${GZ_GUI_PLUGIN_PATH:-}"

source "${script_dir}/require_gazebo_gpu.sh"

printf 'Connecting Gazebo GUI to the running simulation server\n'
exec gz sim -g -v4 --gui-config "${gui_config}"
