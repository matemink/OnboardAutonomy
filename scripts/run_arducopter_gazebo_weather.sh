#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"

export ONBOARD_AUTONOMY_SITL_WEATHER_DEFAULTS="${project_dir}/config/onboard_autonomy-gazebo-weather.parm"
exec "${script_dir}/run_arducopter_gazebo.sh" "$@"
