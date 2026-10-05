#!/usr/bin/env bash

set -euo pipefail

readonly width="${ONBOARD_AUTONOMY_CAMERA_WIDTH:-640}"
readonly height="${ONBOARD_AUTONOMY_CAMERA_HEIGHT:-480}"
readonly view_count="${ONBOARD_AUTONOMY_CALIBRATION_VIEWS:-24}"
readonly capture_fps="${ONBOARD_AUTONOMY_CALIBRATION_FPS:-1}"
readonly sensor_mode="${ONBOARD_AUTONOMY_CAMERA_SENSOR_MODE:-2304:1296:10:P}"
readonly pattern="${ONBOARD_AUTONOMY_CALIBRATION_PATTERN:-9x6}"
readonly square_size_mm="${ONBOARD_AUTONOMY_CALIBRATION_SQUARE_MM:-25}"
readonly lens_position="${ONBOARD_AUTONOMY_CAMERA_LENS_POSITION:-default}"
readonly state_root="${ONBOARD_AUTONOMY_CALIBRATION_STATE_DIR:-${HOME}/.local/state/onboard_autonomy/calibration}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
calibrator="${ONBOARD_AUTONOMY_CAMERA_CALIBRATOR:-${script_dir}/calibrate_camera.py}"
python_executable="${ONBOARD_AUTONOMY_PYTHON:-python3}"
requirements="${script_dir}/../requirements.txt"
if [[ ! -f "${calibrator}" ]]; then
    calibrator="${script_dir}/../python/calibrate_camera.py"
fi
if [[ ! -f "${requirements}" ]]; then
    requirements="${script_dir}/../python/requirements.txt"
fi

require_positive_integer() {
    local readonly name="$1"
    local readonly value="$2"
    if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]]; then
        printf '%s must be a positive integer, got: %s\n' \
            "${name}" "${value}" >&2
        exit 2
    fi
}

require_positive_integer ONBOARD_AUTONOMY_CAMERA_WIDTH "${width}"
require_positive_integer ONBOARD_AUTONOMY_CAMERA_HEIGHT "${height}"
require_positive_integer ONBOARD_AUTONOMY_CALIBRATION_VIEWS "${view_count}"
require_positive_integer ONBOARD_AUTONOMY_CALIBRATION_FPS "${capture_fps}"
if [[ ! "${sensor_mode}" =~ ^[1-9][0-9]*:[1-9][0-9]*:(8|10|12|14|16):[PU]$ ]]; then
    printf 'ONBOARD_AUTONOMY_CAMERA_SENSOR_MODE must be WIDTH:HEIGHT:DEPTH:P or :U.\n' >&2
    exit 2
fi

if [[ ! "${lens_position}" =~ ^(default|[0-9]+([.][0-9]+)?)$ ]]; then
    printf 'ONBOARD_AUTONOMY_CAMERA_LENS_POSITION must be default or a non-negative number.\n' >&2
    exit 2
fi

if ! command -v rpicam-vid >/dev/null 2>&1 ||
   ! command -v rpicam-hello >/dev/null 2>&1; then
    printf 'rpicam-vid and rpicam-hello are required for calibration capture.\n' >&2
    exit 3
fi
if [[ ! -f "${calibrator}" ]]; then
    printf 'Calibration analyzer not found: %s\n' "${calibrator}" >&2
    exit 3
fi

camera_listing="$(rpicam-hello --list-cameras 2>&1 || true)"
camera_model="$(
    awk '$1 == "0" && $2 == ":" { print $3; exit }' \
        <<<"${camera_listing}"
)"
if [[ -z "${camera_model}" ]]; then
    printf 'No Raspberry Pi camera was detected.\n' >&2
    exit 4
fi

run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$"
run_dir="${state_root}/${run_id}"
image_dir="${run_dir}/images"
mkdir -p "${image_dir}"

printf 'OnboardAutonomy Camera Module 3 calibration capture\n'
printf '  Camera: %s\n' "${camera_model}"
printf '  Resolution: %sx%s\n' "${width}" "${height}"
printf '  Sensor mode: %s\n' "${sensor_mode}"
printf '  Focus: manual, lens position %s\n' "${lens_position}"
printf '  Checkerboard: %s inner corners, %s mm squares\n' \
    "${pattern}" "${square_size_mm}"
printf '  Views: %s at %s FPS\n' "${view_count}" "${capture_fps}"
printf '  Output: %s\n\n' "${run_dir}"
printf 'Move and tilt the flat checkerboard between captures.\n'
printf 'Keep the complete board visible and cover image edges and corners.\n'
printf 'Capture starts in 5 seconds.\n\n'
sleep 5

rpicam-vid \
    --camera 0 \
    --nopreview \
    --width "${width}" \
    --height "${height}" \
    --frames "${view_count}" \
    --framerate "${capture_fps}" \
    --mode "${sensor_mode}" \
    --codec mjpeg \
    --quality 100 \
    --segment 1 \
    --output "${image_dir}/view%03d.jpg" \
    --autofocus-mode manual \
    --lens-position "${lens_position}"

if ! "${python_executable}" -c 'import cv2, numpy' >/dev/null 2>&1; then
    printf '\nImages captured, but Python OpenCV is not installed.\n'
    if [[ -f "${requirements}" ]]; then
        printf 'Install the packaged requirements in a virtual environment:\n'
        printf '  python3 -m venv .venv\n'
        printf '  .venv/bin/python -m pip install -r %s\n' "${requirements}"
        printf 'Then rerun with ONBOARD_AUTONOMY_PYTHON=.venv/bin/python.\n'
    fi
    exit 5
fi

"${python_executable}" "${calibrator}" \
    --images "${image_dir}" \
    --pattern "${pattern}" \
    --square-size-mm "${square_size_mm}" \
    --camera-model "${camera_model}" \
    --sensor-mode "${sensor_mode}" \
    --focus-mode manual \
    --lens-position "${lens_position}" \
    --output "${run_dir}/calibration.json"
