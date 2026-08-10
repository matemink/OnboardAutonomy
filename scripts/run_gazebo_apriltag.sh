#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly default_world="${project_dir}/simulation/worlds/apriltag_landing.sdf"

export ONBOARD_AUTONOMY_GAZEBO_WORLD="${ONBOARD_AUTONOMY_GAZEBO_WORLD:-${default_world}}"
exec "${script_dir}/run_gazebo_iris.sh" "$@"
