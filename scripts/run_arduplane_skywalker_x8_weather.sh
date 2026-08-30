#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source "${script_dir}/weather_profile.sh"

export ONBOARD_AUTONOMY_SITL_WEATHER_DEFAULTS="${weather_profile_file}"
exec bash "${script_dir}/run_arduplane_skywalker_x8.sh" "$@"
