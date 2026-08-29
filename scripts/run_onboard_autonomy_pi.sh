#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
binary="${ONBOARD_AUTONOMY_BINARY:-${script_dir}/onboard_autonomy}"
if [[ ! -x "${binary}" ]]; then
    project_binary="${script_dir}/../build/onboard_autonomy"
    if [[ -x "${project_binary}" ]]; then
        binary="${project_binary}"
    fi
fi

if [[ ! -x "${binary}" ]]; then
    printf 'OnboardAutonomy binary not found: %s\n' "${binary}" >&2
    exit 1
fi

declare -a candidates=()
declare -A resolved_candidates=()
shopt -s nullglob

if [[ -n "${ONBOARD_AUTONOMY_SERIAL:-}" ]]; then
    candidates=("${ONBOARD_AUTONOMY_SERIAL}")
else
    for device in \
        /dev/serial/by-id/* \
        /dev/ttyACM* \
        /dev/ttyUSB* \
        /dev/ttyAMA0; do
        if [[ ! -e "${device}" ]]; then
            continue
        fi
        resolved="$(readlink -f "${device}" 2>/dev/null || true)"
        if [[ -z "${resolved}" ||
              -n "${resolved_candidates[${resolved}]:-}" ]]; then
            continue
        fi
        resolved_candidates["${resolved}"]=1
        candidates+=("${device}")
    done
fi

if (( ${#candidates[@]} == 0 )); then
    printf 'No Pixhawk serial candidate found.\n' >&2
    printf 'Connect USB or configure Pi 5 GPIO UART, then run diagnostics.\n' >&2
    exit 2
fi

if (( ${#candidates[@]} > 1 )); then
    printf 'Multiple serial devices found; refusing to guess:\n' >&2
    printf '  %s\n' "${candidates[@]}" >&2
    printf 'Select one explicitly:\n' >&2
    printf '  ONBOARD_AUTONOMY_SERIAL=/dev/ttyAMA0 %s\n' "$0" >&2
    exit 3
fi

device="${candidates[0]}"
if [[ ! -r "${device}" || ! -w "${device}" ]]; then
    printf 'Serial device is not readable/writable: %s\n' "${device}" >&2
    printf 'Add the user to dialout, then log in again:\n' >&2
    printf '  sudo usermod -aG dialout "$USER"\n' >&2
    exit 4
fi

baud="${ONBOARD_AUTONOMY_BAUD:-115200}"
snapshot_ms="${ONBOARD_AUTONOMY_SNAPSHOT_MS:-1000}"
camera_enabled="${ONBOARD_AUTONOMY_CAMERA_ENABLED:-1}"
camera_width="${ONBOARD_AUTONOMY_CAMERA_WIDTH:-640}"
camera_height="${ONBOARD_AUTONOMY_CAMERA_HEIGHT:-480}"
camera_fps="${ONBOARD_AUTONOMY_CAMERA_FPS:-30}"
apriltag_enabled="${ONBOARD_AUTONOMY_APRILTAG_ENABLED:-0}"
apriltag_size_mm="${ONBOARD_AUTONOMY_APRILTAG_SIZE_MM:-}"
camera_calibration="${ONBOARD_AUTONOMY_CAMERA_CALIBRATION:-${script_dir}/../share/onboard_autonomy/camera-module-3-wide-640x480.json}"
preview_enabled="${ONBOARD_AUTONOMY_CAMERA_PREVIEW_ENABLED:-1}"
preview_port="${ONBOARD_AUTONOMY_CAMERA_PREVIEW_PORT:-8080}"
log_dir="${ONBOARD_AUTONOMY_LOG_DIR:-${HOME}/.local/state/onboard_autonomy}"
log_max_files="${ONBOARD_AUTONOMY_LOG_MAX_FILES:-20}"
log_max_total_bytes="${ONBOARD_AUTONOMY_LOG_MAX_TOTAL_BYTES:-104857600}"
log_max_file_bytes="${ONBOARD_AUTONOMY_LOG_MAX_FILE_BYTES:-10485760}"
log_rotator="${ONBOARD_AUTONOMY_LOG_ROTATOR:-${script_dir}/rotate_jsonl_logs.py}"
if [[ ! -f "${log_rotator}" ]]; then
    project_rotator="${script_dir}/../python/rotate_jsonl_logs.py"
    if [[ -f "${project_rotator}" ]]; then
        log_rotator="${project_rotator}"
    fi
fi
if [[ ! -f "${log_rotator}" ]]; then
    printf 'JSONL log rotator not found: %s\n' "${log_rotator}" >&2
    exit 6
fi
mkdir -p "${log_dir}"
log_stem="telemetry-$(date -u +%Y%m%dT%H%M%SZ)-$$"

printf 'OnboardAutonomy hardware bench\n'
printf '  Mode:   OBSERVE ONLY\n'
printf '  Link:   %s at %s baud\n' "${device}" "${baud}"
printf '  Logs:   %s/%s*.jsonl\n' "${log_dir}" "${log_stem}"
printf '  Safety: autonomous motion is disabled on serial hardware\n\n'

declare -a camera_arguments=()
if [[ "${camera_enabled}" == "1" ]]; then
    camera_arguments=(
        --camera
        --camera-width "${camera_width}"
        --camera-height "${camera_height}"
        --camera-fps "${camera_fps}"
    )
    if [[ "${apriltag_enabled}" == "1" ]]; then
        camera_arguments+=(--apriltag)
        if [[ -n "${apriltag_size_mm}" ]]; then
            if [[ ! -f "${camera_calibration}" ]]; then
                printf 'Camera calibration not found: %s\n' \
                    "${camera_calibration}" >&2
                exit 5
            fi
            camera_arguments+=(
                --camera-calibration "${camera_calibration}"
                --apriltag-size-mm "${apriltag_size_mm}"
            )
        fi
    fi
    if [[ "${preview_enabled}" == "1" ]]; then
        camera_arguments+=(
            --camera-preview
            --camera-preview-port "${preview_port}"
        )
    fi
    printf '  Camera: %sx%s YUV420 at %s FPS\n\n' \
        "${camera_width}" "${camera_height}" "${camera_fps}"
    if [[ "${apriltag_enabled}" == "1" ]]; then
        printf '  Vision: AprilTag tagStandard41h12\n\n'
        if [[ -n "${apriltag_size_mm}" ]]; then
            printf '  Pose: %s mm detection span / %s\n\n' \
                "${apriltag_size_mm}" "${camera_calibration}"
        fi
    fi
    if [[ "${preview_enabled}" == "1" ]]; then
        printf '  Preview: http://companionpi.local:%s/\n\n' \
            "${preview_port}"
    fi
fi

"${binary}" \
    --transport serial \
    --serial-device "${device}" \
    --baud "${baud}" \
    --snapshot-ms "${snapshot_ms}" \
    "${camera_arguments[@]}" \
    --json |
    python3 "${log_rotator}" \
        --stream \
        --directory "${log_dir}" \
        --stem "${log_stem}" \
        --max-files "${log_max_files}" \
        --max-total-bytes "${log_max_total_bytes}" \
        --max-file-bytes "${log_max_file_bytes}"
