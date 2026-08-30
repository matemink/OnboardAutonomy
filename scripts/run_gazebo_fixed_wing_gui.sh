#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly sitl_models_dir="${project_dir}/.local/vendor/SITL_Models/Gazebo"

export GZ_SIM_RESOURCE_PATH="${sitl_models_dir}/models:${sitl_models_dir}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
exec "${script_dir}/run_gazebo_gui.sh" "$@"
