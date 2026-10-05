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
terminates the demo processes.
The Windows launcher tags its children with this checkout's path. The stop
script signals only tagged processes owned by the current Linux user; other
Gazebo sessions are left running. For terminals that should share this cleanup,
prefix a launch command with `python3 scripts/demo_process.py run --`.
Untagged sessions started by older launchers must be stopped in their own terminals.

The default demo supplies camera observations;
no flight starts automatically. Only the forward stream starts by default;
the browser shows one camera panel. To also enable the downward observation feed:

```bash
ONBOARD_AUTONOMY_DOWNWARD_CAMERA=1 bash scripts/run_onboard_autonomy_gazebo_vision.sh
```

On Windows, set `ONBOARD_AUTONOMY_DOWNWARD_CAMERA=1` before running the demo
launcher. Disabled cameras have no receiver or browser polling loop.

## Streams and scenes

| Resource | Purpose |
| --- | --- |
| `camera_observation.sdf` | Basic camera world |
| `camera_showcase.sdf` | Camera showcase world |
| `camera_showcase_storm.sdf` | Weather showcase world |
| UDP 5601 | Optional downward camera frames |
| UDP 5602 | Forward camera frames |
| HTTP 8080 | Runtime camera preview |
| UDP 14550 | Companion MAVLink endpoint |

The model is `iris_with_cameras`. Camera streams are independent and report
failure separately. The optional downward stream is an observation feed. The world
contains no marker landing pad.

## Weather and rendering

```bash
bash scripts/run_gazebo_camera_weather.sh
bash scripts/run_arducopter_gazebo_weather.sh
ONBOARD_AUTONOMY_INTERACTIVE=1 bash scripts/run_onboard_autonomy_gazebo_weather_vision.sh
```

`config/onboard_autonomy-gazebo-weather.parm` supplies the shared wind speed
and direction seed. Its turbulence value controls SITL only; Gazebo gusts
and noise remain configured separately in the world SDF. The GUI labels
this value as SITL turbulence, so it does not imply matching turbulence models.
The Pixhawk/Pi identity frames carry no physical mass; camera links retain
their sensor mounts, and the barometer keeps its existing topic and pose.
The GUI wind indicator shows configured values, not instantaneous noisy sensor
measurements. Gazebo's server and GUI are separate so capture does not depend
on the GUI lifecycle. WSL launchers check the configured GPU renderer before
starting the scene.

## Console

The companion observes telemetry and camera frames. `Q` exits an interactive
session; `Ctrl+C` exits a passive session. It has no flight-start, tracking,
return-to-launch, or automatic takeoff actions. The former two-aircraft scenario
and its models, flight controllers, and launchers have been removed.

## Recovery checks

```bash
python python/run_integration_check.py --companion "$HOME/build/onboard_autonomy/onboard_autonomy"
python python/camera_recovery_acceptance.py --companion "$HOME/build/onboard_autonomy/onboard_autonomy"
```

The first check uses generated UDP telemetry. Camera recovery stops and
restarts the producer and requires new frames in the same companion process.
These integration
checks require their respective local runtime, simulator, and Python dependencies;
unit tests do not establish that a new Gazebo flight has been executed.

The Windows camera demo checks for a running Gazebo session and occupied camera,
telemetry, SITL, and preview endpoints before it starts. It asks you to close an
existing session instead of terminating other simulations. Set
`ONBOARD_AUTONOMY_BUILD_DIR` to the Linux build directory to select the console
binary; an unset value uses the development runbook's default build directory.
