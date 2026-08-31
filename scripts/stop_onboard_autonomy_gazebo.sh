#!/usr/bin/env bash

set -euo pipefail

readonly graceful_wait_attempts=30
readonly wait_interval_seconds=0.1

readonly -a exact_process_names=(
    arducopter
    arduplane
    gz
)

readonly -a command_patterns=(
    '^gz sim '
    '/onboard_autonomy --transport udp --sitl'
    'mavproxy.py --master=tcp:127.0.0.1:5760'
    'mavproxy.py --master=tcp:127.0.0.1:5770'
    'scripts/run_arduplane_shahed_136'
    'scripts/run_arduplane_skywalker_x8'
    'scripts/run_arducopter_gazebo'
    'scripts/run_gazebo_fixed_wing'
    'scripts/run_gazebo_apriltag'
    'scripts/run_gazebo_gui.sh'
    'scripts/run_gazebo_iris.sh'
    'scripts/run_onboard_autonomy_gazebo'
)

has_demo_processes() {
    local process_name
    local command_pattern

    for process_name in "${exact_process_names[@]}"; do
        if pgrep -x "${process_name}" >/dev/null 2>&1; then
            return 0
        fi
    done

    for command_pattern in "${command_patterns[@]}"; do
        if pgrep -f "${command_pattern}" >/dev/null 2>&1; then
            return 0
        fi
    done

    return 1
}

signal_demo_processes() {
    local readonly signal="$1"
    local process_name
    local command_pattern

    for command_pattern in "${command_patterns[@]}"; do
        pkill "-${signal}" -f "${command_pattern}" 2>/dev/null || true
    done

    for process_name in "${exact_process_names[@]}"; do
        pkill "-${signal}" -x "${process_name}" 2>/dev/null || true
    done
}

if ! has_demo_processes; then
    printf 'No previous OnboardAutonomy Gazebo demo is running.\n'
    exit 0
fi

printf 'Stopping previous OnboardAutonomy Gazebo demo...\n'
signal_demo_processes TERM

for ((attempt = 0; attempt < graceful_wait_attempts; ++attempt)); do
    if ! has_demo_processes; then
        printf 'Previous demo stopped cleanly.\n'
        exit 0
    fi
    sleep "${wait_interval_seconds}"
done

printf 'Forcing only the remaining demo processes to stop...\n'
signal_demo_processes KILL
sleep "${wait_interval_seconds}"

if has_demo_processes; then
    printf 'Some OnboardAutonomy Gazebo demo processes are still running.\n' >&2
    exit 1
fi

printf 'Previous demo stopped.\n'
