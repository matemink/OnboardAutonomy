#!/usr/bin/env bash

set -euo pipefail

if [[ -t 1 ]]; then
    printf '\033]0;Skywalker X8 ArduPlane SITL\007'
fi

readonly ardupilot_dir="${ARDUPILOT_DIR:-${HOME}/src/ardupilot-Copter-4.6.3}"
readonly mavproxy="${MAVPROXY:-${HOME}/venv-ardupilot/bin/mavproxy.py}"
readonly arduplane="${ardupilot_dir}/build/sitl/bin/arduplane"
readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly x8_defaults="${project_dir}/.local/vendor/SITL_Models/Gazebo/config/skywalker_x8.param"
readonly weather_defaults="${ONBOARD_AUTONOMY_SITL_WEATHER_DEFAULTS:-}"

if [[ ! -x "${arduplane}" ]]; then
    printf 'ArduPlane SITL is not built. Run:\n' >&2
    printf '  bash scripts/prepare_skywalker_x8_sitl.sh\n' >&2
    exit 1
fi

if [[ ! -x "${mavproxy}" ]]; then
    printf 'MAVProxy is not installed: %s\n' "${mavproxy}" >&2
    exit 1
fi

if [[ ! -f "${x8_defaults}" ]]; then
    printf 'Skywalker X8 defaults are missing: %s\n' "${x8_defaults}" >&2
    exit 1
fi

defaults="${x8_defaults}"
if [[ -n "${weather_defaults}" ]]; then
    if [[ ! -f "${weather_defaults}" ]]; then
        printf 'SITL weather defaults are missing: %s\n' \
            "${weather_defaults}" >&2
        exit 1
    fi
    defaults+=",${weather_defaults}"
fi

cd "${ardupilot_dir}"

autopilot_pid=''

stop_autopilot() {
    if [[ -z "${autopilot_pid}" ]] || \
        ! kill -0 "${autopilot_pid}" 2>/dev/null; then
        return
    fi

    kill -TERM "${autopilot_pid}" 2>/dev/null || true
    wait "${autopilot_pid}" 2>/dev/null || true
}

trap stop_autopilot EXIT INT TERM

"${arduplane}" \
    -S \
    --wipe \
    --model JSON \
    --speedup 1 \
    --slave 0 \
    --defaults "${defaults}" \
    --sim-address=127.0.0.1 \
    -I0 &
autopilot_pid=$!

printf '\nSkywalker X8 MAVProxy console\n'
printf 'This iteration validates only the ArduPlane/Gazebo baseline.\n\n'

mavproxy_args=(
    --master=tcp:127.0.0.1:5760
    --sitl=127.0.0.1:5501
    --out="udp:127.0.0.1:${ONBOARD_AUTONOMY_MAVLINK_OUT_PORT:-14550}"
    --streamrate=-1
)

if [[ -n "${ONBOARD_AUTONOMY_MAVPROXY_STATE_DIR:-}" ]]; then
    mkdir -p "${ONBOARD_AUTONOMY_MAVPROXY_STATE_DIR}"
    mavproxy_args+=(--state-basedir="${ONBOARD_AUTONOMY_MAVPROXY_STATE_DIR}")
fi

"${mavproxy}" "${mavproxy_args[@]}"
