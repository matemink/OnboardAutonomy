#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly enable_topic="${ONBOARD_AUTONOMY_CAMERA_ENABLE_TOPIC:-/world/apriltag_landing/model/Holybro_S500/link/Raspberry_Pi_Camera_Module_3_Wide/sensor/Raspberry_Pi_Camera_Module_3_Wide/image/enable_streaming}"

export GZ_VERSION="${GZ_VERSION:-harmonic}"

set_camera_streaming() {
    local readonly enabled="$1"
    timeout 2s gz topic \
        -t "${enable_topic}" \
        -m gz.msgs.Boolean \
        -p "data: ${enabled}"
}

cleanup() {
    set_camera_streaming false >/dev/null 2>&1 || true
}

wait_for_camera() {
    for _ in {1..60}; do
        if gz topic -l | grep -Fxq "${enable_topic}"; then
            return
        fi
        sleep 0.5
    done

    printf 'Gazebo landing camera is not available: %s\n' \
        "${enable_topic}" >&2
    exit 1
}

trap cleanup EXIT INT TERM

printf 'Enabling Gazebo landing camera\n'
printf '  Topic: %s\n' "${enable_topic}"
printf '  Stream: RTP/H.264 UDP %s\n' \
    "${ONBOARD_AUTONOMY_CAMERA_UDP_PORT:-5601}"
printf '  Preview: http://localhost:%s/\n\n' \
    "${ONBOARD_AUTONOMY_CAMERA_PREVIEW_PORT:-8080}"

wait_for_camera
set_camera_streaming true

ONBOARD_AUTONOMY_GAZEBO_VISION=1 \
    "${script_dir}/run_onboard_autonomy_sitl.sh"
