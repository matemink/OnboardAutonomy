# Gazebo camera simulation

Gazebo Harmonic owns the world and camera sensors. ArduPilot SITL owns vehicle
state and flight control. OnboardAutonomy consumes MAVLink and camera frames.
Physical endpoints remain in observation mode.

## Setup

Build the runtime and ArduCopter SITL using the
[development runbook](development.md), then install the pinned Gazebo plugin:

```bash
bash scripts/install_gazebo_harmonic.sh
```

Launch the camera world and flight controller in separate terminals:

```bash
bash scripts/run_gazebo_camera.sh
bash scripts/run_arducopter_gazebo.sh
ONBOARD_AUTONOMY_INTERACTIVE=1 bash scripts/run_onboard_autonomy_gazebo_vision.sh
```

On Windows, `StartOnboardAutonomyGazeboDemo.cmd` starts the weather camera
profile, console, and browser preview. `StopOnboardAutonomyGazeboDemo.cmd`
terminates the demo processes. The default demo supplies camera observations;
no flight starts automatically.

## Streams and scenes

| Resource | Purpose |
| --- | --- |
| `camera_observation.sdf` | Basic camera world |
| `camera_showcase.sdf` | Camera showcase world |
| `camera_showcase_storm.sdf` | Weather showcase world |
| UDP 5601 | Downward camera frames |
| UDP 5602 | Forward camera frames |
| HTTP 8080 | Runtime camera preview |
| UDP 14550 | Companion MAVLink endpoint |

The model is `iris_with_cameras`. Camera streams are independent and report
failure separately. The downward stream is an observation feed. The world
contains no marker landing pad.

The optional forward OpenCV DNN detector reads a separately downloaded model:

```bash
bash scripts/download_yolox_model.sh
```

The downloader checks the pinned model hash. Weights remain in `.local/models`;
without them, capture and preview remain available. The generic model's labels
are an integration baseline and do not establish aircraft-specific accuracy.

## Weather and rendering

```bash
bash scripts/run_gazebo_camera_weather.sh
bash scripts/run_arducopter_gazebo_weather.sh
ONBOARD_AUTONOMY_INTERACTIVE=1 bash scripts/run_onboard_autonomy_gazebo_weather_vision.sh
```

`config/onboard_autonomy-gazebo-weather.parm` supplies the shared wind profile.
The GUI wind indicator shows configured values, not instantaneous noisy sensor
measurements. Gazebo's server and GUI are separate so capture does not depend
on the GUI lifecycle. WSL launchers check the configured GPU renderer before
starting the scene.

## Existing interactive simulation workflow

The aerial-observation launcher uses `--aerial-observation` with an explicit
`--sitl` assertion and a forward camera. Interactive mode waits for operator
input; the console presents the configured action and `R` for return to launch.
`Q` exits the application. Requests remain subject to the existing telemetry,
startup, and failsafe state machines.

The former marker options, marker assets, and landing acceptance script have
been removed. `config/onboard_autonomy-gazebo.parm` disables precision landing.
Ordinary ArduPilot landing and return-to-launch remain available.

## Recovery checks

```bash
python python/run_integration_check.py --companion "$HOME/build/onboard_autonomy/onboard_autonomy"
python python/camera_recovery_acceptance.py --companion "$HOME/build/onboard_autonomy/onboard_autonomy"
python python/link_failsafe_sitl_acceptance.py --companion "$HOME/build/onboard_autonomy/onboard_autonomy"
```

The first check uses generated UDP telemetry. Camera recovery stops and
restarts the producer and requires new frames in the same companion process.
The link-failsafe harness cuts the telemetry relay and verifies the flight
controller's independent fallback from protocol evidence. These integration
checks require their respective local runtime, simulator, and Python dependencies;
unit tests do not establish that a new Gazebo flight has been executed.

The Windows camera demo checks for a running Gazebo session and occupied camera,
telemetry, SITL, and preview endpoints before it starts. It asks you to close an
existing session instead of terminating other simulations. Set
`ONBOARD_AUTONOMY_BUILD_DIR` to the Linux build directory to select the console
binary; an unset value uses the development runbook's default build directory.
