#!/usr/bin/env bash

set -euo pipefail

readonly sitl_models_commit="25bc38ed8c6c0345840159a8cbc0b02781d52f3c"
readonly sitl_models_url="https://github.com/ArduPilot/SITL_Models.git"
readonly ardupilot_dir="${ARDUPILOT_DIR:-${HOME}/src/ardupilot-Copter-4.6.3}"
readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly vendor_root="${project_dir}/.local/vendor"
readonly sitl_models_dir="${vendor_root}/SITL_Models"
readonly x8_model="${sitl_models_dir}/Gazebo/models/skywalker_x8/model.sdf"
readonly x8_params="${sitl_models_dir}/Gazebo/config/skywalker_x8.param"
readonly arduplane="${ardupilot_dir}/build/sitl/bin/arduplane"

mkdir -p "${vendor_root}"

if [[ ! -d "${sitl_models_dir}/.git" ]]; then
    mkdir -p "${sitl_models_dir}"
    git -C "${sitl_models_dir}" init
    git -C "${sitl_models_dir}" remote add origin "${sitl_models_url}"
    git -C "${sitl_models_dir}" fetch --depth 1 origin "${sitl_models_commit}"
    git -C "${sitl_models_dir}" checkout --detach FETCH_HEAD
fi

actual_commit="$(git -C "${sitl_models_dir}" rev-parse HEAD)"
if [[ "${actual_commit}" != "${sitl_models_commit}" ]]; then
    printf 'Unexpected SITL_Models commit: %s\n' "${actual_commit}" >&2
    printf 'Expected: %s\n' "${sitl_models_commit}" >&2
    exit 1
fi

if [[ ! -f "${x8_model}" || ! -f "${x8_params}" ]]; then
    printf 'Pinned Skywalker X8 model is incomplete: %s\n' \
        "${sitl_models_dir}" >&2
    exit 1
fi

if [[ ! -d "${ardupilot_dir}" ]]; then
    printf 'ArduPilot source is missing: %s\n' "${ardupilot_dir}" >&2
    printf 'Run scripts/install_ardupilot_sitl.sh first.\n' >&2
    exit 1
fi

if [[ ! -x "${arduplane}" ]]; then
    printf 'Building ArduPlane SITL once...\n'
    (
        cd "${ardupilot_dir}"
        ./waf configure --board sitl
        ./waf plane
    )
fi

printf 'Skywalker X8 SITL is ready.\n'
printf '  Assets: %s\n' "${sitl_models_commit}"
printf '  Binary: %s\n' "${arduplane}"
