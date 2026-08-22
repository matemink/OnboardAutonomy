#!/usr/bin/env bash

set -euo pipefail

ONBOARD_AUTONOMY_BUILD_DIR="${ONBOARD_AUTONOMY_BUILD_DIR:-${HOME}/build/onboard_autonomy}"
COMPANION="${ONBOARD_AUTONOMY_BUILD_DIR}/onboard_autonomy"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"

if [[ ! -x "${COMPANION}" ]]; then
    printf 'OnboardAutonomy is not built: %s\n' "${COMPANION}" >&2
    exit 1
fi

arguments=(
    --transport udp
    --sitl
    --udp-bind 127.0.0.1
    --udp-port 14550
    --snapshot-ms 1000
)

if [[ -n "${ONBOARD_AUTONOMY_SIM_WIND_PROFILE:-}" ]]; then
    read -r wind_speed wind_direction wind_turbulence extra <<< \
        "${ONBOARD_AUTONOMY_SIM_WIND_PROFILE}"
    if [[ -z "${wind_speed:-}" || -z "${wind_direction:-}" ||
          -z "${wind_turbulence:-}" || -n "${extra:-}" ]]; then
        printf '%s\n' \
            'ONBOARD_AUTONOMY_SIM_WIND_PROFILE requires: MPS FROM_DEG GUST' \
            >&2
        exit 1
    fi
    arguments+=(
        --sim-wind
        "${wind_speed}"
        "${wind_direction}"
        "${wind_turbulence}"
    )
fi

if [[ "${ONBOARD_AUTONOMY_GAZEBO_VISION:-0}" == "1" ]]; then
    arguments+=(
        --camera
        --camera-source gstreamer
        --camera-udp-port "${ONBOARD_AUTONOMY_CAMERA_UDP_PORT:-5601}"
        --camera-width 640
        --camera-height 480
        --apriltag
        --camera-calibration
        "${project_dir}/config/gazebo-landing-camera-640x480.json"
        --camera-extrinsics
        "${project_dir}/config/gazebo-landing-camera-extrinsics.json"
        --apriltag-size-mm 2000
        --camera-preview
        --camera-preview-port
        "${ONBOARD_AUTONOMY_CAMERA_PREVIEW_PORT:-8080}"
    )
    if [[ -n "${ONBOARD_AUTONOMY_FORWARD_CAMERA_UDP_PORT:-}" ]]; then
        arguments+=(
            --forward-camera-udp-port
            "${ONBOARD_AUTONOMY_FORWARD_CAMERA_UDP_PORT}"
        )
    fi
fi

if [[ "${ONBOARD_AUTONOMY_AUTONOMOUS:-0}" == "1" ]]; then
    arguments+=(--autonomous)
fi

if [[ "${ONBOARD_AUTONOMY_EXIT_AFTER_AUTONOMY:-0}" == "1" ]]; then
    arguments+=(--exit-after-autonomy)
fi

if [[ "${ONBOARD_AUTONOMY_JSON:-0}" == "1" ]]; then
    arguments+=(--json)
fi

if [[ "${ONBOARD_AUTONOMY_INTERACTIVE:-0}" == "1" ]]; then
    arguments+=(--interactive)
fi

exec "${COMPANION}" "${arguments[@]}"
