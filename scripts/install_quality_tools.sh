#!/usr/bin/env bash

set -euo pipefail

readonly clang_uml_version="0.6.3"
readonly clang_uml_package="clang-uml_${clang_uml_version}-0ubuntu1ppa1.noble_amd64.deb"
readonly clang_uml_url="https://github.com/bkryza/clang-uml/releases/download/${clang_uml_version}/${clang_uml_package}"

if [[ "$(id -u)" -ne 0 ]]; then
    echo "Run this installer with sudo." >&2
    exit 1
fi

apt-get update
apt-get install -y clang-format-18 clang-tidy-18 curl graphviz

temporary_package="$(mktemp --suffix=.deb)"
trap 'rm -f "${temporary_package}"' EXIT
curl --fail --location --output "${temporary_package}" "${clang_uml_url}"
apt-get install -y "${temporary_package}"

clang-tidy-18 --version | head -n 2
clang-uml --version
