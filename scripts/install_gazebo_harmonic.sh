#!/usr/bin/env bash

set -euo pipefail

readonly gazebo_repo_url="https://github.com/ArduPilot/ardupilot_gazebo.git"
readonly gazebo_plugin_commit="65937b77aace16735df6f192badb0e6b4eddd056"
readonly gazebo_source_dir="${ARDUPILOT_GAZEBO_SOURCE_DIR:-${HOME}/src/ardupilot_gazebo}"
readonly gazebo_build_dir="${ARDUPILOT_GAZEBO_BUILD_DIR:-${HOME}/build/ardupilot_gazebo}"
readonly install_mode="${1:-all}"

if [[ "${install_mode}" != "all" &&
      "${install_mode}" != "packages-only" &&
      "${install_mode}" != "plugin-only" ]]; then
    printf 'Usage: %s [all|packages-only|plugin-only]\n' "$0" >&2
    exit 2
fi

install_gazebo_packages() {
    local key_file
    local architecture
    local codename

    sudo apt-get update
    sudo apt-get install -y \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gnupg \
        lsb-release \
        mesa-utils \
        ninja-build \
        pkg-config

    key_file="$(mktemp)"
    curl -fsSL \
        https://packages.osrfoundation.org/gazebo.gpg \
        -o "${key_file}"
    sudo install \
        -m 0644 \
        "${key_file}" \
        /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    rm -f "${key_file}"

    architecture="$(dpkg --print-architecture)"
    codename="$(lsb_release -cs)"
    printf '%s\n' \
        "deb [arch=${architecture} signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${codename} main" |
        sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

    sudo apt-get update
    sudo apt-get install -y \
        gz-harmonic \
        libgz-sim8-dev \
        qtdeclarative5-dev \
        qtquickcontrols2-5-dev \
        rapidjson-dev \
        libopencv-dev \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-gl
}

prepare_plugin_source() {
    mkdir -p "$(dirname -- "${gazebo_source_dir}")"

    if [[ -e "${gazebo_source_dir}" &&
          ! -d "${gazebo_source_dir}/.git" ]]; then
        printf 'Plugin source path exists but is not a Git checkout: %s\n' \
            "${gazebo_source_dir}" >&2
        exit 1
    fi

    if [[ ! -d "${gazebo_source_dir}/.git" ]]; then
        git clone \
            --filter=blob:none \
            --no-checkout \
            "${gazebo_repo_url}" \
            "${gazebo_source_dir}"
        git -C "${gazebo_source_dir}" fetch \
            --depth 1 \
            origin \
            "${gazebo_plugin_commit}"
        git -C "${gazebo_source_dir}" checkout \
            --detach \
            "${gazebo_plugin_commit}"
    fi

    local actual_commit
    actual_commit="$(git -C "${gazebo_source_dir}" rev-parse HEAD)"
    if [[ "${actual_commit}" != "${gazebo_plugin_commit}" ]]; then
        printf 'Unexpected ardupilot_gazebo commit: %s\n' \
            "${actual_commit}" >&2
        printf 'Expected: %s\n' "${gazebo_plugin_commit}" >&2
        exit 1
    fi
}

build_plugin() {
    export GZ_VERSION=harmonic
    cmake \
        -S "${gazebo_source_dir}" \
        -B "${gazebo_build_dir}" \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build "${gazebo_build_dir}" --parallel
}

if [[ "${install_mode}" != "plugin-only" ]]; then
    install_gazebo_packages
fi

if [[ "${install_mode}" != "packages-only" ]]; then
    prepare_plugin_source
    build_plugin
fi

if [[ "${install_mode}" == "packages-only" ]]; then
    printf '\nGazebo Harmonic system packages are ready.\n'
else
    printf '\nGazebo Harmonic and the ArduPilot plugin are ready.\n'
    printf 'Plugin source: %s\n' "${gazebo_source_dir}"
    printf 'Plugin build:  %s\n' "${gazebo_build_dir}"
    printf 'Pinned commit: %s\n' "${gazebo_plugin_commit}"
fi
