#!/usr/bin/env bash

set -euo pipefail

readonly duration_seconds="${ONBOARD_AUTONOMY_PROFILE_SECONDS:-60}"
readonly sample_seconds="${ONBOARD_AUTONOMY_PROFILE_SAMPLE_SECONDS:-0.2}"
readonly default_state_root="${HOME}/.local/state/onboard_autonomy/profiles"
readonly state_root="${ONBOARD_AUTONOMY_PROFILE_STATE_DIR:-${default_state_root}}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
default_launcher="${script_dir}/run_onboard_autonomy_pi.sh"
launcher="${ONBOARD_AUTONOMY_PROFILE_LAUNCHER:-${default_launcher}}"
analyzer="${ONBOARD_AUTONOMY_PROFILE_ANALYZER:-${script_dir}/runtime_profile.py}"
if [[ ! -f "${analyzer}" ]]; then
    analyzer="${script_dir}/../python/runtime_profile.py"
fi

if [[ ! "${duration_seconds}" =~ ^[1-9][0-9]*$ ]]; then
    printf 'ONBOARD_AUTONOMY_PROFILE_SECONDS must be a positive integer.\n' >&2
    exit 2
fi
if [[ ! "${sample_seconds}" =~ ^0\.[0-9]*[1-9][0-9]*$|^[1-9][0-9]*(\.[0-9]+)?$ ]]; then
    printf 'ONBOARD_AUTONOMY_PROFILE_SAMPLE_SECONDS must be positive.\n' >&2
    exit 2
fi
if [[ ! -x "${launcher}" ]]; then
    printf 'Runtime launcher is not executable: %s\n' "${launcher}" >&2
    exit 3
fi
if [[ ! -f "${analyzer}" ]]; then
    printf 'Runtime profile analyzer not found: %s\n' "${analyzer}" >&2
    exit 3
fi
for command in awk date getconf ps python3 setsid; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        printf 'Required profiling command is missing: %s\n' "${command}" >&2
        exit 3
    fi
done

run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$"
run_dir="${state_root}/${run_id}"
mkdir -p "${run_dir}"
samples_file="${run_dir}/process-group-samples.tsv"
runtime_output="${run_dir}/runtime.stdout.log"
runtime_error="${run_dir}/runtime.stderr.log"
report_json="${run_dir}/report.json"
report_markdown="${run_dir}/report.md"
printf 'elapsed_ms\tcumulative_cpu_ticks\trss_kib\tprocess_count\t' \
    >"${samples_file}"
printf 'temperature_millic\tthrottled_hex\n' >>"${samples_file}"

profile_pid=''
cleanup() {
    if [[ -n "${profile_pid}" ]] &&
       kill -0 "${profile_pid}" 2>/dev/null; then
        kill -TERM -- "-${profile_pid}" 2>/dev/null || true
        sleep 0.2
        if kill -0 "${profile_pid}" 2>/dev/null; then
            kill -KILL -- "-${profile_pid}" 2>/dev/null || true
        fi
        wait "${profile_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

printf 'OnboardAutonomy Raspberry Pi runtime profile\n'
printf '  Duration: %s seconds\n' "${duration_seconds}"
printf '  Sample interval: %s seconds\n' "${sample_seconds}"
printf '  Output: %s\n\n' "${run_dir}"

setsid "${launcher}" >"${runtime_output}" 2>"${runtime_error}" &
profile_pid=$!
start_ns="$(date +%s%N)"
duration_ns="$((duration_seconds * 1000000000))"
declare -A previous_ticks=()
cumulative_cpu_ticks=0
window_complete=0

while kill -0 "${profile_pid}" 2>/dev/null; do
    now_ns="$(date +%s%N)"
    elapsed_ns="$((now_ns - start_ns))"
    if (( elapsed_ns > duration_ns )); then
        window_complete=1
        break
    fi

    rss_kib=0
    process_count=0
    declare -A current_ticks=()
    while read -r pid process_group; do
        if [[ "${process_group}" != "${profile_pid}" ]]; then
            continue
        fi
        cpu_ticks="$(
            awk '{ print $14 + $15 }' "/proc/${pid}/stat" 2>/dev/null || true
        )"
        process_rss="$(
            awk '/^VmRSS:/ { print $2 }' "/proc/${pid}/status" \
                2>/dev/null || true
        )"
        if [[ -z "${cpu_ticks}" || -z "${process_rss}" ]]; then
            continue
        fi
        previous="${previous_ticks[${pid}]:-0}"
        if (( cpu_ticks >= previous )); then
            cumulative_cpu_ticks="$((
                cumulative_cpu_ticks + cpu_ticks - previous
            ))"
        else
            cumulative_cpu_ticks="$((cumulative_cpu_ticks + cpu_ticks))"
        fi
        current_ticks["${pid}"]="${cpu_ticks}"
        rss_kib="$((rss_kib + process_rss))"
        process_count="$((process_count + 1))"
    done < <(ps -eo pid=,pgid=)
    previous_ticks=()
    for pid in "${!current_ticks[@]}"; do
        previous_ticks["${pid}"]="${current_ticks[${pid}]}"
    done

    temperature_millic='-'
    if [[ -r /sys/class/thermal/thermal_zone0/temp ]]; then
        temperature_millic="$(
            tr -d '[:space:]' </sys/class/thermal/thermal_zone0/temp
        )"
    fi
    throttled_hex='-'
    if command -v vcgencmd >/dev/null 2>&1; then
        throttled_output="$(vcgencmd get_throttled 2>/dev/null || true)"
        if [[ "${throttled_output}" =~ 0x[0-9a-fA-F]+ ]]; then
            throttled_hex="${BASH_REMATCH[0]}"
        fi
    fi

    printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$((elapsed_ns / 1000000))" \
        "${cumulative_cpu_ticks}" \
        "${rss_kib}" \
        "${process_count}" \
        "${temperature_millic}" \
        "${throttled_hex}" \
        >>"${samples_file}"
    sleep "${sample_seconds}"
done

observed_ns="$(( $(date +%s%N) - start_ns ))"
observed_seconds="$(awk -v ns="${observed_ns}" 'BEGIN { printf "%.9f", ns / 1000000000 }')"
declare -a window_arguments=()
if [[ "${window_complete}" == "1" ]]; then
    window_arguments=(--window-complete)
fi

if kill -0 "${profile_pid}" 2>/dev/null; then
    kill -INT -- "-${profile_pid}" 2>/dev/null || true
    for _ in {1..50}; do
        if ! kill -0 "${profile_pid}" 2>/dev/null; then
            break
        fi
        sleep 0.1
    done
    if kill -0 "${profile_pid}" 2>/dev/null; then
        kill -TERM -- "-${profile_pid}" 2>/dev/null || true
        for _ in {1..20}; do
            if ! kill -0 "${profile_pid}" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
    fi
    if kill -0 "${profile_pid}" 2>/dev/null; then
        kill -KILL -- "-${profile_pid}" 2>/dev/null || true
    fi
fi

set +e
wait "${profile_pid}"
runtime_status=$?
profile_pid=''
set -e

set +e
python3 "${analyzer}" \
    --samples "${samples_file}" \
    --duration "${duration_seconds}" \
    --sample-interval "${sample_seconds}" \
    --observed-duration "${observed_seconds}" \
    "${window_arguments[@]}" \
    --runtime-status "${runtime_status}" \
    --clock-ticks "$(getconf CLK_TCK)" \
    --architecture "$(uname -m)" \
    --kernel "$(uname -r)" \
    --report-json "${report_json}" \
    --report-markdown "${report_markdown}"
analysis_status=$?
set -e

printf '\nArtifacts:\n'
printf '  %s\n' \
    "${report_markdown}" \
    "${report_json}" \
    "${samples_file}" \
    "${runtime_output}" \
    "${runtime_error}"
exit "${analysis_status}"
