#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source "${script_dir}/weather_profile.sh"

export ONBOARD_AUTONOMY_SIM_WIND_PROFILE="${ONBOARD_AUTONOMY_WIND_SPEED_M_S} ${ONBOARD_AUTONOMY_WIND_FROM_DEG} ${ONBOARD_AUTONOMY_WIND_TURBULENCE_M_S}"
exec "${script_dir}/run_onboard_autonomy_gazebo_vision.sh" "$@"
