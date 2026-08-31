#!/usr/bin/env bash

set -euo pipefail

if [[ -t 1 ]]; then
    printf '\033]0;Shahed-136 ArduPlane SITL\007'
fi

readonly ardupilot_dir="${ARDUPILOT_DIR:-${HOME}/src/ardupilot-Copter-4.6.3}"
readonly mavproxy="${MAVPROXY:-${HOME}/venv-ardupilot/bin/mavproxy.py}"
readonly arduplane="${ardupilot_dir}/build/sitl/bin/arduplane"
readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly target_defaults="${project_dir}/config/shahed-136-physics-approx.parm"
readonly weather_defaults="${ONBOARD_AUTONOMY_SITL_WEATHER_DEFAULTS:-}"

if [[ ! -x "${arduplane}" ]]; then
    printf 'ArduPlane SITL is not built. Run:\n' >&2
    printf '  bash scripts/prepare_fixed_wing_follow_sitl.sh\n' >&2
    exit 1
fi

if [[ ! -x "${mavproxy}" ]]; then
    printf 'MAVProxy is not installed: %s\n' "${mavproxy}" >&2
    exit 1
fi

defaults="${target_defaults}"
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
    -I1 &
autopilot_pid=$!

printf '\nShahed-136 MAVProxy console\n'
printf 'Physics baseline only: no autonomous route is loaded yet.\n\n'

mavproxy_args=(
    --master=tcp:127.0.0.1:5770
    --sitl=127.0.0.1:5511
    --out="udp:127.0.0.1:${ONBOARD_AUTONOMY_SHAHED_MAVLINK_OUT_PORT:-14560}"
    --streamrate=-1
)

if [[ -n "${ONBOARD_AUTONOMY_MAVPROXY_STATE_DIR:-}" ]]; then
    mkdir -p "${ONBOARD_AUTONOMY_MAVPROXY_STATE_DIR}"
    mavproxy_args+=(--state-basedir="${ONBOARD_AUTONOMY_MAVPROXY_STATE_DIR}")
fi

"${mavproxy}" "${mavproxy_args[@]}"
