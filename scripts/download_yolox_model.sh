#!/usr/bin/env bash

set -euo pipefail

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly project_dir="$(cd -- "${script_dir}/.." && pwd)"
readonly model_dir="${project_dir}/.local/models"
readonly model_file="${model_dir}/object_detection_yolox_2022nov.onnx"
readonly model_url="https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/object_detection_yolox/object_detection_yolox_2022nov.onnx"
readonly model_sha256="c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063"

mkdir -p "${model_dir}"
if [[ -f "${model_file}" ]] &&
   printf '%s  %s\n' "${model_sha256}" "${model_file}" |
       sha256sum --check --status; then
    printf 'YOLOX model already verified: %s\n' "${model_file}"
    exit 0
fi

readonly temporary_file="${model_file}.download"
trap 'rm -f "${temporary_file}"' EXIT
curl --fail --location --output "${temporary_file}" "${model_url}"
printf '%s  %s\n' "${model_sha256}" "${temporary_file}" |
    sha256sum --check --status || {
        printf 'Downloaded YOLOX model failed SHA-256 verification\n' >&2
        exit 1
    }
mv "${temporary_file}" "${model_file}"
printf 'YOLOX model downloaded and verified: %s\n' "${model_file}"
