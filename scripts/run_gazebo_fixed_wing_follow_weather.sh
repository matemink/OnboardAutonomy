#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly wind_topic="/world/fixed_wing_follow/wind"

source "${script_dir}/weather_profile.sh"

read -r wind_x_m_s wind_y_m_s < <(weather_gazebo_vector)
readonly wind_message="linear_velocity: {x: ${wind_x_m_s}, y: ${wind_y_m_s}, z: 0.0}, enable_wind: true"

gazebo_pid=''

stop_gazebo() {
    if [[ -z "${gazebo_pid}" ]] || ! kill -0 "${gazebo_pid}" 2>/dev/null; then
        return
    fi

    kill -TERM "${gazebo_pid}" 2>/dev/null || true
    wait "${gazebo_pid}" 2>/dev/null || true
}

trap stop_gazebo EXIT INT TERM

"${script_dir}/run_gazebo_fixed_wing_follow.sh" "$@" &
gazebo_pid=$!

wind_ready=0
for _ in {1..100}; do
    if ! kill -0 "${gazebo_pid}" 2>/dev/null; then
        wait "${gazebo_pid}"
        exit $?
    fi

    if gz topic -l 2>/dev/null | grep -Fxq "${wind_topic}"; then
        wind_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${wind_ready}" != "1" ]]; then
    printf 'Gazebo wind topic did not become ready: %s\n' "${wind_topic}" >&2
    exit 1
fi

gz topic -t "${wind_topic}" -m gz.msgs.Wind -p "${wind_message}"
printf 'Fixed-wing Gazebo weather: %s m/s from %s deg, turbulence %s m/s\n' \
    "${ONBOARD_AUTONOMY_WIND_SPEED_M_S}" \
    "${ONBOARD_AUTONOMY_WIND_FROM_DEG}" \
    "${ONBOARD_AUTONOMY_WIND_TURBULENCE_M_S}"

wait "${gazebo_pid}"
