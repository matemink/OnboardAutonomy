#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly x8_model="${project_dir}/.local/vendor/SITL_Models/Gazebo/models/skywalker_x8/model.sdf"
readonly generated_model_dir="${project_dir}/.local/generated/shahed_136_arduplane"

bash "${script_dir}/prepare_skywalker_x8_sitl.sh"

python3 "${script_dir}/generate_shahed_136_physics_model.py" \
    --source "${x8_model}" \
    --output "${generated_model_dir}"

if [[ ! -f "${generated_model_dir}/model.sdf" || \
      ! -f "${generated_model_dir}/model.config" ]]; then
    printf 'Generated Shahed-136 model is incomplete: %s\n' \
        "${generated_model_dir}" >&2
    exit 1
fi

printf 'Fixed-wing follow SITL is ready.\n'
printf '  Pursuer: Skywalker X8 / ArduPlane instance 0\n'
printf '  Target:  Shahed-136 approximation / ArduPlane instance 1\n'
