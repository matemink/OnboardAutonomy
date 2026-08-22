#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly downward_enable_topic="${ONBOARD_AUTONOMY_CAMERA_ENABLE_TOPIC:-/world/apriltag_landing/model/Holybro_S500/link/Raspberry_Pi_Camera_Module_3_Wide/sensor/Raspberry_Pi_Camera_Module_3_Wide/image/enable_streaming}"
readonly forward_enable_topic="${ONBOARD_AUTONOMY_FORWARD_CAMERA_ENABLE_TOPIC:-/world/apriltag_landing/model/Holybro_S500/link/Raspberry_Pi_Camera_Module_3_Wide_Forward/sensor/Raspberry_Pi_Camera_Module_3_Wide_Forward/image/enable_streaming}"
readonly downward_camera_udp_port="5601"
readonly forward_camera_udp_port="5602"

export GZ_VERSION="${GZ_VERSION:-harmonic}"

set_camera_streaming() {
    local readonly topic="$1"
    local readonly enabled="$2"
    timeout 2s gz topic \
        -t "${topic}" \
        -m gz.msgs.Boolean \
        -p "data: ${enabled}"
}

cleanup() {
    set_camera_streaming "${downward_enable_topic}" false \
        >/dev/null 2>&1 || true
    set_camera_streaming "${forward_enable_topic}" false \
        >/dev/null 2>&1 || true
}

wait_for_camera() {
    local readonly label="$1"
    local readonly topic="$2"
    for _ in {1..60}; do
        if gz topic -l | grep -Fxq "${topic}"; then
            return
        fi
        sleep 0.5
    done

    printf 'Gazebo %s camera is not available: %s\n' \
        "${label}" "${topic}" >&2
    exit 1
}

trap cleanup EXIT INT TERM

printf 'Enabling Gazebo cameras\n'
printf '  Downward topic: %s\n' "${downward_enable_topic}"
printf '  Downward stream: RTP/H.264 UDP %s\n' \
    "${downward_camera_udp_port}"
printf '  Forward topic: %s\n' "${forward_enable_topic}"
printf '  Forward stream: RTP/H.264 UDP %s\n' \
    "${forward_camera_udp_port}"
printf '  Preview: http://localhost:%s/\n\n' \
    "${ONBOARD_AUTONOMY_CAMERA_PREVIEW_PORT:-8080}"

wait_for_camera downward "${downward_enable_topic}"
wait_for_camera forward "${forward_enable_topic}"
set_camera_streaming "${downward_enable_topic}" true
set_camera_streaming "${forward_enable_topic}" true

ONBOARD_AUTONOMY_GAZEBO_VISION=1 \
ONBOARD_AUTONOMY_CAMERA_UDP_PORT="${downward_camera_udp_port}" \
ONBOARD_AUTONOMY_FORWARD_CAMERA_UDP_PORT="${forward_camera_udp_port}" \
ONBOARD_AUTONOMY_SIM_WIND_PROFILE="${ONBOARD_AUTONOMY_SIM_WIND_PROFILE:-0.0 0.0 0.0}" \
    "${script_dir}/run_onboard_autonomy_sitl.sh"
