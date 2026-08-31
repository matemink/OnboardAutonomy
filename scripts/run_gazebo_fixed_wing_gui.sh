#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly sitl_models_dir="${project_dir}/.local/vendor/SITL_Models/Gazebo"
readonly generated_models_dir="${project_dir}/.local/generated"

export GZ_SIM_RESOURCE_PATH="${generated_models_dir}:${sitl_models_dir}/models:${sitl_models_dir}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
exec bash "${script_dir}/run_gazebo_gui.sh" "$@"
