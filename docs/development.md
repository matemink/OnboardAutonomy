# Development and SITL runbook

This runbook contains the detailed host build, generated-telemetry,
ArduCopter SITL, integration-test, and failure-injection commands. The
project overview remains in the repository README.

## Host requirements

The verified development host is Ubuntu 24.04 under WSL2. Native Ubuntu
and 64-bit Raspberry Pi OS use the same CMake workflow.

Install the project toolchain:

```bash
sudo bash scripts/bootstrap_ubuntu.sh
```

## Configure, build, and test

The launch scripts default to `${HOME}/build/onboard_autonomy`, so the
following layout works for both manual and scripted runs:

```bash
cmake \
    -S . \
    -B "${HOME}/build/onboard_autonomy" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug
cmake --build "${HOME}/build/onboard_autonomy" --parallel
ctest \
    --test-dir "${HOME}/build/onboard_autonomy" \
    --output-on-failure
python3 -m unittest discover -s python/tests -v
```

CMake downloads pinned generated MAVLink C headers and the other source
dependencies into the build directory. MAVLink framing is not
reimplemented by this project.

## Static analysis

Install the developer-only quality tools on Ubuntu 24.04 or WSL2:

```bash
sudo bash scripts/install_quality_tools.sh
```

Run the same `clang-tidy` checks used by CI:

```bash
bash scripts/run_static_analysis.sh
```

Every finding from an enabled `clang-tidy` check fails the command and the CI
job. This includes correctness, concurrency, performance, portability,
function-size, cognitive-complexity, and the enforceable C++ Core Guidelines
checks provided by clang-tidy 18. The enabled checks and limits are defined in
`.clang-tidy`; changing them requires an explicit review rather than silently
accepting a violation.

Pull requests analyze only changed implementation files. Changes to headers,
CMake, `.clang-tidy`, the CI workflow, or the analyzer scripts trigger a full
scan because they can affect every translation unit. Pull requests without
C++-relevant changes skip `clang-tidy`; pushes to `main` always run the full
scan. Set `CLANG_TIDY_BASE_REF` locally to apply the same incremental selection.

The Core Guidelines profile excludes checks that conflict with required C API
boundaries (POSIX, GStreamer, AprilTag, MAVLink, and `argc`/`argv`), intentional
non-owning reference members, and matrix indexing. Magic numbers are blocking:
domain, protocol, timing, and configuration values must be expressed as named
constants. Local suppressions are allowed only at an unavoidable boundary and
must state the reason next to the suppression.

Install the Python development dependencies and run the same Ruff analysis as
CI:

```bash
python3 -m pip install -r python/requirements-dev.txt
ruff check python scripts
```

Every Ruff finding is blocking. The enabled rules are defined in
`pyproject.toml`.

## Generated MAVLink telemetry

Start OnboardAutonomy:

```bash
"${HOME}/build/onboard_autonomy/onboard_autonomy" \
    --transport udp \
    --udp-bind 127.0.0.1 \
    --udp-port 14550
```

Prepare the Python environment in a second terminal:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r python/requirements.txt
```

Send a healthy state or one isolated failure:

```bash
python python/scenario_runner.py --scenario healthy
python python/scenario_runner.py --scenario no-gps
python python/scenario_runner.py --scenario low-battery
python python/scenario_runner.py --scenario prearm
```

The generated path is useful for fast protocol and domain checks. It is
not a substitute for ArduPilot integration testing.

## Install ArduCopter SITL

The installer pins ArduPilot `Copter-4.6.3` at the commit recorded in the
script, initializes its submodules, and installs Ubuntu prerequisites:

```bash
bash scripts/install_ardupilot_sitl.sh
```

Build ArduCopter in the shell opened by the installer:

```bash
cd "${HOME}/src/ardupilot-Copter-4.6.3"
./waf configure --board sitl
./waf copter
```

The scripts expect MAVProxy at
`${HOME}/venv-ardupilot/bin/mavproxy.py`. Override `ARDUPILOT_DIR` or
`MAVPROXY` when using a different verified installation.

## Run ArduCopter and OnboardAutonomy

Start the flight controller in one terminal:

```bash
bash scripts/run_arducopter_sitl.sh
```

Start the companion runtime in another:

```bash
bash scripts/run_onboard_autonomy_sitl.sh
```

ArduCopter and OnboardAutonomy exchange MAVLink through
`udp://127.0.0.1:14550`. OnboardAutonomy discovers the vehicle system ID,
broadcasts an onboard-controller heartbeat as component `191`, and
requests its required message rates.

On Windows, `StartOnboardAutonomyDemo.cmd` opens both processes in
separate WSL terminals.

## Automated integration checks

Run the bounded generated-telemetry integration check:

```bash
python python/run_integration_check.py \
    --companion "${HOME}/build/onboard_autonomy/onboard_autonomy"
```

Run the complete ArduCopter/MAVProxy/OnboardAutonomy smoke test:

```bash
python python/run_sitl_smoke_test.py
```

The harness starts every process, waits on protocol evidence instead of
fixed sleeps, writes logs under `artifacts/sitl-smoke/`, and terminates
its process group when the check completes.

Run the production autonomy flight and independently verified companion-link
failsafe flight:

```bash
.venv/bin/python python/autonomy_sitl_acceptance.py \
    --companion "${HOME}/build/onboard_autonomy/onboard_autonomy"
.venv/bin/python python/link_failsafe_sitl_acceptance.py \
    --companion "${HOME}/build/onboard_autonomy/onboard_autonomy"
```

The second harness inserts a controllable UDP relay. It cuts both MAVLink
directions only after verified takeoff, then requires OnboardAutonomy to record
heartbeat loss and ArduPilot to enter LAND independently. The tlog must contain
no LAND or RTL command from companion component `191`.

Verify camera-process and stream recovery without restarting OnboardAutonomy:

```bash
.venv/bin/python python/camera_recovery_acceptance.py \
    --companion "${HOME}/build/onboard_autonomy/onboard_autonomy"
```

The harness starts the Gazebo camera, records decoded frames, stops the entire
producer, requires a visible `reconnecting` state, restarts Gazebo, and accepts
only after the same companion process consumes new frames.

## Failure injection

Each scenario first requires a healthy baseline and then changes one
observable condition:

```bash
python python/run_sitl_smoke_test.py --scenario heartbeat-loss
python python/run_sitl_smoke_test.py --scenario gps-loss
python python/run_sitl_smoke_test.py --scenario low-battery
python python/run_sitl_smoke_test.py --scenario prearm
```

| Scenario | Injection | Expected observation |
| --- | --- | --- |
| `heartbeat-loss` | Stop the MAVLink forwarding path | Connection becomes stale |
| `gps-loss` | Disable simulated GPS | Link remains live; GPS becomes not ready |
| `low-battery` | Drain the simulated battery | Battery crosses the readiness threshold |
| `prearm` | Set an invalid SITL motor-spin parameter | ArduPilot emits a real PreArm failure |

The isolated PreArm test never arms the vehicle or drives motors. Its
parameter change exists only in the temporary SITL EEPROM for that run.

## Structured diagnostics

Pass `--json` to emit JSON Lines instead of the operator console:

```bash
"${HOME}/build/onboard_autonomy/onboard_autonomy" \
    --transport udp \
    --udp-port 14550 \
    --json
```

Snapshot records preserve the existing top-level telemetry fields and add
`record_type`, wall-clock time, link activity, and elapsed runtime. Event
records capture controller and camera loss/recovery, target acquisition/loss,
mission and failsafe phase changes, motion-safety changes, and MAVLink command
results.

Structured logging can also run alongside the normal console:

```bash
"${HOME}/build/onboard_autonomy/onboard_autonomy" \
    --transport udp \
    --udp-port 14550 \
    --diagnostic-log artifacts/flight.jsonl
```

Heartbeat freshness controls `connected`; stale data is not reported as
healthy. Console rendering and diagnostic serialization are independent
snapshot consumers.
