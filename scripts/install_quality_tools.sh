#!/usr/bin/env bash

set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

apt-get update
apt-get install -y clang-format-18 clang-tidy-18

clang-tidy-18 --version | head -n 2
