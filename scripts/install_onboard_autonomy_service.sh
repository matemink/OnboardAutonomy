#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd -- "${script_dir}/.." && pwd)"
install_root="/opt/onboard-autonomy"
configuration_dir="/etc/onboard-autonomy"
unit_name="onboard-autonomy@.service"
enable_service=false

if [[ "${1:-}" == "--enable" ]]; then
    enable_service=true
elif [[ $# -gt 0 ]]; then
    printf 'Usage: sudo %s [--enable]\n' "$0" >&2
    exit 2
fi

if [[ "${EUID}" -ne 0 ]]; then
    printf 'Run this installer through sudo.\n' >&2
    exit 1
fi

service_user="${ONBOARD_AUTONOMY_SERVICE_USER:-${SUDO_USER:-}}"
if [[ -z "${service_user}" || "${service_user}" == "root" ]]; then
    printf 'Set ONBOARD_AUTONOMY_SERVICE_USER to a non-root Pi user.\n' >&2
    exit 1
fi
if ! id "${service_user}" >/dev/null 2>&1; then
    printf 'Service user does not exist: %s\n' "${service_user}" >&2
    exit 1
fi

binary="${package_dir}/bin/onboard_autonomy"
unit="${package_dir}/share/onboard_autonomy/systemd/${unit_name}"
environment_example="${package_dir}/share/onboard_autonomy/systemd/onboard-autonomy.env.example"
if [[ ! -x "${binary}" || ! -f "${unit}" ||
      ! -f "${environment_example}" ]]; then
    printf 'Incomplete OnboardAutonomy package: %s\n' "${package_dir}" >&2
    exit 1
fi

install -d -m 0755 "${install_root}"
instance="onboard-autonomy@${service_user}.service"
was_active=false
if systemctl is-active --quiet "${instance}"; then
    was_active=true
fi
# Also stop an activating/restarting unit before replacing its executable.
if [[ "$(systemctl show "${instance}" --property=LoadState --value)" != "not-found" ]]; then
    systemctl stop "${instance}"
fi
cp -a "${package_dir}/." "${install_root}/"
install -d -m 0755 "${configuration_dir}"
if [[ ! -f "${configuration_dir}/onboard-autonomy.env" ]]; then
    install -m 0644 \
        "${environment_example}" \
        "${configuration_dir}/onboard-autonomy.env"
fi
install -m 0644 "${unit}" "/etc/systemd/system/${unit_name}"

systemctl daemon-reload
printf 'Installed OnboardAutonomy for user %s.\n' "${service_user}"
printf 'Configuration: %s\n' \
    "${configuration_dir}/onboard-autonomy.env"

if [[ "${enable_service}" == true ]]; then
    systemctl enable --now \
        "onboard-autonomy@${service_user}.service"
    systemctl --no-pager --full status \
        "onboard-autonomy@${service_user}.service" || true
elif [[ "${was_active}" == true ]]; then
    systemctl start "${instance}"
else
    printf 'Review the configuration, then enable with:\n'
    printf '  sudo systemctl enable --now onboard-autonomy@%s.service\n' \
        "${service_user}"
fi
