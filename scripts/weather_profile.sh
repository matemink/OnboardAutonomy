#!/usr/bin/env bash

readonly weather_profile_script_dir="$(
    cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd
)"
readonly weather_profile_project_dir="$(
    cd -- "${weather_profile_script_dir}/.." && pwd
)"
weather_profile_file="${ONBOARD_AUTONOMY_WEATHER_PROFILE:-${weather_profile_project_dir}/config/onboard_autonomy-gazebo-weather.parm}"

read_weather_parameter() {
    local readonly name="$1"
    awk -v name="${name}" '
        $1 == name { print $2; found = 1; exit }
        END { if (!found) exit 1 }
    ' "${weather_profile_file}"
}

if [[ ! -f "${weather_profile_file}" ]]; then
    printf 'Weather profile not found: %s\n' "${weather_profile_file}" >&2
    exit 1
fi

weather_profile_file="$(
    cd -- "$(dirname -- "${weather_profile_file}")" && pwd
)/$(basename -- "${weather_profile_file}")"
readonly weather_profile_file

ONBOARD_AUTONOMY_WIND_SPEED_M_S="$(read_weather_parameter SIM_WIND_SPD)"
ONBOARD_AUTONOMY_WIND_FROM_DEG="$(read_weather_parameter SIM_WIND_DIR)"
ONBOARD_AUTONOMY_WIND_TURBULENCE_M_S="$(
    read_weather_parameter SIM_WIND_TURB
)"

readonly weather_unsigned_decimal='^[0-9]+([.][0-9]+)?$'
if [[ ! "${ONBOARD_AUTONOMY_WIND_SPEED_M_S}" =~ ${weather_unsigned_decimal} ||
      ! "${ONBOARD_AUTONOMY_WIND_FROM_DEG}" =~ ${weather_unsigned_decimal} ||
      ! "${ONBOARD_AUTONOMY_WIND_TURBULENCE_M_S}" =~ ${weather_unsigned_decimal} ]]; then
    printf 'Weather profile values must be non-negative numbers: %s\n' \
        "${weather_profile_file}" >&2
    exit 1
fi

if ! awk -v direction="${ONBOARD_AUTONOMY_WIND_FROM_DEG}" \
    'BEGIN { exit !(direction >= 0 && direction < 360) }'; then
    printf 'SIM_WIND_DIR must be in [0, 360): %s\n' \
        "${ONBOARD_AUTONOMY_WIND_FROM_DEG}" >&2
    exit 1
fi

export ONBOARD_AUTONOMY_WIND_SPEED_M_S
export ONBOARD_AUTONOMY_WIND_FROM_DEG
export ONBOARD_AUTONOMY_WIND_TURBULENCE_M_S

weather_gazebo_vector() {
    awk \
        -v speed="${ONBOARD_AUTONOMY_WIND_SPEED_M_S}" \
        -v direction_from="${ONBOARD_AUTONOMY_WIND_FROM_DEG}" '
        BEGIN {
            radians = direction_from * atan2(0, -1) / 180
            printf "%.6f %.6f\n", \
                -speed * sin(radians), -speed * cos(radians)
        }
    '
}
