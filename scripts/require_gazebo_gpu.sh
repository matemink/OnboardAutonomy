#!/usr/bin/env bash

# Gazebo under WSLg can silently fall back to llvmpipe. Never run the demo in
# that state: camera rendering would consume the CPU and make timing unreliable.
if [[ -e /dev/dxg ]]; then
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
fi

if ! command -v glxinfo >/dev/null 2>&1; then
    printf 'Gazebo GPU check requires glxinfo (package: mesa-utils).\n' >&2
    exit 1
fi

if ! gazebo_gpu_info="$(glxinfo -B 2>&1)"; then
    printf 'Gazebo GPU check could not create an OpenGL context:\n%s\n' \
        "${gazebo_gpu_info}" >&2
    exit 1
fi

gazebo_gpu_renderer="$(
    sed -n 's/^OpenGL renderer string: //p' <<<"${gazebo_gpu_info}" | head -n 1
)"

if grep -Eiq 'llvmpipe|softpipe|swrast|software rasterizer' \
        <<<"${gazebo_gpu_info}"; then
    printf 'Gazebo refused to start with CPU rendering: %s\n' \
        "${gazebo_gpu_renderer:-unknown renderer}" >&2
    printf 'Run "wsl --shutdown", then start the demo again.\n' >&2
    exit 1
fi

if [[ -e /dev/dxg ]] &&
   { ! grep -Fq 'Accelerated: yes' <<<"${gazebo_gpu_info}" ||
     ! grep -Fq 'D3D12 (NVIDIA' <<<"${gazebo_gpu_info}"; }; then
    printf 'Gazebo refused to start without accelerated NVIDIA D3D12.\n' >&2
    printf 'Detected renderer: %s\n' \
        "${gazebo_gpu_renderer:-unknown renderer}" >&2
    printf 'Run "wsl --shutdown", then start the demo again.\n' >&2
    exit 1
fi

printf 'Gazebo GPU renderer: %s\n' \
    "${gazebo_gpu_renderer:-hardware accelerated}"
