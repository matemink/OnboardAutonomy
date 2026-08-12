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

## Static analysis and architecture diagrams

Install the developer-only quality tools on Ubuntu 24.04 or WSL2:

```bash
sudo bash scripts/install_quality_tools.sh
```

Run the same `clang-tidy` checks used by CI:

```bash
bash scripts/run_static_analysis.sh
```

Correctness, concurrency, performance, and portability findings fail the
command. Function-size and cognitive-complexity findings remain visible as
architecture warnings while the existing large functions are refactored in
small, behavior-preserving changes.

Regenerate the Mermaid class and package diagrams from the C++ AST:

```bash
bash scripts/generate_diagrams.sh
```

The generated files are written to `docs/diagrams/generated/`. The diagrams
use CMake's `compile_commands.json`, so they reflect actual compiled types and
relationships rather than a hand-maintained architecture sketch.

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

## Machine-readable mode

Pass `--json` to emit one snapshot per interval instead of the operator
console:

```bash
"${HOME}/build/onboard_autonomy/onboard_autonomy" \
    --transport udp \
    --udp-port 14550 \
    --json
```

Heartbeat freshness controls `connected`; stale data is not reported as
healthy. JSON mode is the stable boundary used by Python integration
tests and hardware logging.
