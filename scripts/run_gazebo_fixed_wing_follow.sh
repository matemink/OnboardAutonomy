#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly sitl_models_dir="${project_dir}/.local/vendor/SITL_Models/Gazebo"
readonly world_file="${project_dir}/simulation/worlds/fixed_wing_follow.sdf"

if [[ ! -f "${sitl_models_dir}/models/skywalker_x8/model.sdf" ]]; then
    printf 'Skywalker X8 assets are missing. Run:\n' >&2
    printf '  bash scripts/prepare_skywalker_x8_sitl.sh\n' >&2
    exit 1
fi

export GZ_SIM_RESOURCE_PATH="${sitl_models_dir}/models:${sitl_models_dir}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export ONBOARD_AUTONOMY_GAZEBO_WORLD="${world_file}"
exec "${script_dir}/run_gazebo_iris.sh" "$@"
