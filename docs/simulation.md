# Gazebo simulation runbook

Gazebo owns the 3D world, sensors, and vehicle physics. ArduCopter SITL
owns the flight-control loop. OnboardAutonomy sends MAVLink commands,
observes acknowledgements and telemetry, and advances the guarded autonomy
runtime.

## Prerequisites

Build ArduCopter SITL first by following
[development.md](development.md). Then install Gazebo Harmonic and build
the pinned official ArduPilot Gazebo plugin:

```bash
bash scripts/install_gazebo_harmonic.sh
```

The installer records and verifies the plugin commit rather than
building an arbitrary moving branch.

## Start the simulation

Run the three processes in separate terminals:

```bash
bash scripts/run_gazebo_apriltag.sh
bash scripts/run_arducopter_gazebo.sh
ONBOARD_AUTONOMY_INTERACTIVE=1 \
ONBOARD_AUTONOMY_AUTONOMOUS=1 \
    bash scripts/run_onboard_autonomy_gazebo_vision.sh
```

On Windows, `StartOnboardAutonomyGazeboDemo.cmd` launches the interactive
weather profile in visible WSL/WSLg windows, enables console input, waits for
the first simulated camera frame, and opens the preview in the default browser.
The launcher keeps the rendering-sensor server separate from the WSLg GUI
client so camera processing does not depend on the GUI lifecycle.

The simulated Holybro S500 carries two visible and functional Camera Module 3
representations. The downward camera streams H.264/RTP to UDP `5601` and feeds
AprilTag landing detection. The forward camera streams independently to UDP
`5602` and can feed an OpenCV DNN detector.

### Forward detector model

The forward detector uses the pretrained
[YOLOX-S ONNX model from OpenCV Zoo](https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox).
The model is provided by OpenCV under the Apache 2.0 license and was evaluated
on the COCO 2017 validation dataset. COCO defines 80 common object categories,
including `person`, `car`, `airplane`, `bird`, and `kite`. It has no dedicated
`drone` category, so the simulated fixed-wing target may receive one of the
closest generic labels. This detector is therefore a reproducible integration
baseline, not a drone-specific recognition model.

The model weights are not committed to this repository. Download the pinned
version after cloning:

```bash
bash scripts/download_yolox_model.sh
```

The downloader fetches `object_detection_yolox_2022nov.onnx`, verifies its
expected SHA-256 digest, and stores it in the gitignored `.local/models`
directory. Re-running the command is safe: an existing valid file is reused.
Without the model, the simulation and both camera streams still run, but the
forward detection overlay is disabled. A failure of either camera stream is
reported independently in the preview.

`StartOnboardAutonomyAerialTracking.cmd` starts the same proven demo and then
spawns `Zephyr_Fixed_Wing_Target` at 12 m. It follows a deterministic 20 m
radius loop at the ArduPlane default cruise speed of 12 m/s through Gazebo
Harmonic's `VelocityControl`. The textured Zephyr mesh comes from the official
`ardupilot_gazebo` project; its attribution and LGPL-3.0 license are stored next
to the model. This launcher selects the SITL-only aerial-observation mission:
the S500 performs the guarded GUIDED/ARM/TAKEOFF sequence and then holds while
the forward detector remains active. It does not enter the AprilTag precision
landing runtime.

The Zephyr is still a visual stimulus, not a second ArduPlane SITL instance.
YOLOX may report generic COCO labels for both the aircraft and visually similar
objects. The lightweight grass texture and lower-contrast directional light
reduce the unrealistically sharp silhouette that the old flat ground produced.
Guidance still accepts only `airplane`, requires three spatially continuous
observations, and expires a
stale lock after 500 ms. A confirmed off-centre target produces a bounded
relative yaw correction; an ambiguous or lost target produces no command and
keeps the S500 in GUIDED hold.

OnboardAutonomy does not receive the scripted model pose or infer 3D range.
Forward pursuit and standoff guidance remain separate later increments and
cannot run until a range estimate with explicit uncertainty exists.

The interactive profile uses a 3 m/s west wind. Gazebo varies its magnitude
and direction, adds vertical turbulence, and applies the resulting force to
wind-enabled vehicle links. ArduPilot receives matching `SIM_WIND_*` defaults.
To change the default profile, edit one file:

```text
config/onboard_autonomy-gazebo-weather.parm
```

`SIM_WIND_SPD` is the base speed in m/s, `SIM_WIND_DIR` is the direction the
wind comes from in degrees (`0` north, `90` east, `180` south, `270` west),
and `SIM_WIND_TURB` is ArduPilot's turbulence amount in m/s. Gazebo, ArduPilot,
and the wind-vane HUD read these same values on the next launch.
For a temporary strong-wind experiment, copy that file under another name,
change the three `SIM_WIND_*` values, and pass the copy to the Windows launcher:

```powershell
Copy-Item config/onboard_autonomy-gazebo-weather.parm config/local-strong-wind.parm
StartOnboardAutonomyGazeboDemo.cmd config/local-strong-wind.parm
```

This keeps the checked-in moderate profile unchanged while the selected file
is shared by Gazebo, ArduPilot, the companion runtime, and the wind-vane HUD.

Run the same profile manually with:

```bash
bash scripts/run_gazebo_apriltag_weather.sh
bash scripts/run_arducopter_gazebo_weather.sh
ONBOARD_AUTONOMY_INTERACTIVE=1 \
ONBOARD_AUTONOMY_AUTONOMOUS=1 \
    bash scripts/run_onboard_autonomy_gazebo_weather_vision.sh
```

The Gazebo 3D window shows a compact wind-vane HUD with the configured speed,
source-to-destination direction, and turbulence. The companion console stays
focused on flight state. The HUD is configuration evidence, not a live
anemometer reading: Gazebo applies time-varying turbulence inside its physics
system but does not publish that instantaneous noisy vector to the GUI.

For a manual weather GUI launch, use:

```bash
ONBOARD_AUTONOMY_GAZEBO_WEATHER=1 bash scripts/run_gazebo_gui.sh
```

The launcher incrementally builds the project-owned `WindIndicator` plugin
and loads `simulation/gui/onboard_autonomy.config`. A calm GUI launch omits the
weather environment flag and displays `CALM` without a direction arrow.

The `Pixhawk_6C_barometer` sensor publishes atmospheric pressure at 50 Hz on
`/onboard_autonomy/sensors/pixhawk_6c/air_pressure`. This Gazebo topic is
observable evidence, not the pressure input to ArduPilotPlugin 4.6.3. The
flight controller's barometer remains ArduPilot's SITL backend, generated from
simulated altitude with the noise configured in the weather parameter file.

| Process | Responsibility |
| --- | --- |
| Gazebo Harmonic | Holybro S500 development-rig identity, world physics, and simulated camera |
| ArduCopter SITL | Stabilization, navigation modes, arming, and landing |
| MAVProxy | MAVLink routing and an optional flight console |
| OnboardAutonomy | Flight startup, vision decisions, safety supervision, commands, and operator TUI |

## Autonomous flight

`--autonomous` starts one production-shaped path rather than a menu of
scripted demonstration routes:

1. Wait for a connected ArduPilot multicopter, complete telemetry setup,
   and pre-arm readiness.
2. Request GUIDED, arm, and take off to 8 m from a point 3 m horizontally
   offset from the landing pad, confirming each command with `COMMAND_ACK`
   and the resulting vehicle state.
3. Convert the fresh AprilTag track to body-FRD, create a short-lived
   desired motion, and pass it through the independent safety supervisor.
4. Stream approved `LANDING_TARGET` messages at 10 Hz and request LAND only
   after one second of continuous target availability.
5. Stop stale target output immediately. Before LAND, five seconds without
   a target triggers an ordinary fallback LAND instead of indefinite hover.
6. Below 1.5 m, require 0.5 seconds of alignment within 0.25 m. If the full
   marker then leaves the camera view, latch vision corrections off and
   complete the terminal descent without reacting to partial reacquisition.
7. Finish only when ArduPilot reports the vehicle disarmed.

After a completed or failed run, interactive mode lets `S` start the same
guarded autonomy sequence again without restarting the simulator. The
request is blocked while the vehicle is armed or a run is already active.
`Q` exits the runtime. It does not select an alternate flight plan.

Run the non-interactive acceptance flight with compact JSON telemetry and
exit automatically after completion or failure:

```bash
ONBOARD_AUTONOMY_AUTONOMOUS=1 \
ONBOARD_AUTONOMY_EXIT_AFTER_AUTONOMY=1 \
ONBOARD_AUTONOMY_JSON=1 \
    bash scripts/run_onboard_autonomy_gazebo_vision.sh
```

For a reproducible acceptance run, let the external Python harness start all
three processes, capture artifacts, validate the final production snapshot,
and inspect MAVProxy's tlog independently:

```bash
source ~/venv-ardupilot/bin/activate
python python/autonomy_sitl_acceptance.py
python python/autonomy_sitl_acceptance.py --runs 10
python python/autonomy_sitl_acceptance.py --weather
```

The default command is the calm deterministic regression. `--weather` is the
explicit gust stress test; its artifacts record the selected environment
profile and the same independent landing-accuracy evidence.

The harness never sends flight commands. It fails unless the production C++
runtime reaches completed startup and autonomous landing, reports disarmed,
and leaves independent evidence for all four accepted commands, sustained
body-FRD `LANDING_TARGET`, expected modes, and a final position within 0.25 m
of the landing-pad center derived from the Gazebo world coordinates. It also
rejects sustained large center crossings and requires an explicit
terminal-descent handoff. Batch mode
stores each tlog and reports median, worst, and population standard deviation.

## Operator console

The terminal presents a bounded MS-DOS-style control panel rather than
an unbounded log stream. It keeps visible:

- the Raspberry Pi 5 and Pixhawk 6C link;
- current mode, arm state, altitude, GPS, battery, and warnings;
- flight-startup and continuous-autonomy phases;
- the latest complete MAVLink frame sent and received;
- semantic command, acknowledgement, and state-confirmation events.

TX and RX wires pulse only while a fresh complete MAVLink frame is
observed. The animation is presentation state and does not alter protocol
or domain behavior.

## Simulated landing camera

The project world names the simulated vehicle and its onboard components
after the physical development rig: `Holybro_S500`, `Pixhawk_6C`,
`Raspberry_Pi_5`, and `Raspberry_Pi_Camera_Module_3_Wide`. It mounts the
fixed downward camera under the airframe and places a `tagStandard41h12`
landing pad 3 m from the vehicle's start. Gazebo sends `640x480` H.264 over
RTP to UDP port `5601`; `GStreamerCameraSource` decodes it to I420 and
publishes the same `CameraFrame` type used by Camera Module 3.

The names map the virtual components to the real bench, but they do not claim
hardware emulation: the current Gazebo geometry and flight dynamics remain
the pinned official Iris model, Pixhawk behavior comes from ArduCopter SITL,
and OnboardAutonomy runs as the Raspberry Pi 5 companion process.

The simulator calibration is derived from the SDF field of view and stored
in `config/gazebo-landing-camera-640x480.json`. The pad texture, two-metre
detection span, camera geometry, and calibration agreement are guarded by
`python/tests/test_gazebo_apriltag_world.py`.

The camera mount is described independently in
`config/gazebo-landing-camera-extrinsics.json`. It rotates OpenCV camera
optical coordinates `[right, down, forward]` into MAVLink body FRD and adds
the measured 0.16 m camera offset below the simulated body origin.

The deterministic Gazebo profile uses ArduPilot's documented RawSensor
precision-land estimator (`PLND_EST_TYPE=0`). The simulated pinhole camera is
noise-free enough that combining companion-frame EMA with ArduPilot's Kalman
estimator created a moving-frame lag and sustained overshoot. This setting is
not a recommendation for the physical camera; real hardware requires measured
noise and latency before selecting its estimator and tuning.

Open the OnboardAutonomy preview at `http://localhost:8080/`. A complete tag
is not expected while the vehicle rests directly on top of the pad because
the camera is too close to see all four corners. Validate acquisition after
takeoff.

The verified production acceptance run and its known close-range limitation
are recorded in
[precision-landing-sitl.md](evidence/precision-landing-sitl.md).

## Reference gimbal stream

The official Iris world exposes an RTP/H.264 gimbal-camera stream on UDP
port `5600`. Verify a bounded 60-frame decode:

```bash
bash scripts/check_gazebo_camera_stream.sh
```

Open the stream in a WSLg window:

```bash
bash scripts/view_gazebo_camera.sh
```

Run only one receiver at a time. The check enables the stream, decodes a
bounded frame count, and disables it during cleanup.

## WSLg rendering

The launcher selects the D3D12 Mesa path when `/dev/dxg` is available.
If Gazebo opens without a visible scene or falls back to slow software
rendering, restart the WSL environment before retrying.

The decorative-airfield experiment was rolled back so an unsuccessful GUI
path is not presented as a working feature.

## Safety boundary

Automated motion is enabled only when the operator explicitly
passes `--sitl` over UDP. UDP by itself may represent a real MAVLink router,
Wi-Fi bridge, or Ethernet bridge and therefore remains observation-only.
The project SITL launcher supplies the assertion; serial rejects `--sitl`.
The Gazebo command must not be reused as a motor-test procedure on physical
hardware.
