#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly model_file="${project_dir}/simulation/models/scripted_fixed_wing_target/model.sdf"
readonly world_name="${ONBOARD_AUTONOMY_GAZEBO_WORLD_NAME:-apriltag_landing}"
readonly create_service="/world/${world_name}/create"
readonly target_name="Zephyr_Fixed_Wing_Target"

if [[ ! -f "${model_file}" ]]; then
    printf 'Aerial target model is missing: %s\n' "${model_file}" >&2
    exit 1
fi

service_ready=0
for _ in {1..100}; do
    if gz service -l 2>/dev/null | grep -Fxq "${create_service}"; then
        service_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${service_ready}" != "1" ]]; then
    printf 'Gazebo create service did not become ready: %s\n' \
        "${create_service}" >&2
    exit 1
fi

request="sdf_filename: \"${model_file}\", name: \"${target_name}\", allow_renaming: false"
response="$(
    gz service \
        -s "${create_service}" \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "${request}"
)"

if [[ "${response}" != *'data: true'* ]]; then
    printf 'Gazebo rejected the aerial target spawn request:\n%s\n' \
        "${response}" >&2
    exit 1
fi

printf 'Gazebo aerial target spawned: %s\n' "${target_name}"
printf '  Altitude: 12 m\n'
printf '  Route: 20 m radius circle at 12 m/s\n'
