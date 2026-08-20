# Raspberry Pi 5 and Pixhawk 6C bench setup

This procedure validates ARM64 Linux deployment and physical MAVLink
communication without running motors. OnboardAutonomy may request telemetry
message rates, but the serial hardware mode cannot start ARM, TAKEOFF,
LAND or autonomous precision-landing behavior.

## Safety

1. Remove every propeller.
2. Do not connect the flight battery for the first USB test.
3. Do not run arming or motor-test commands.
4. Keep the Pixhawk on a stable, non-conductive surface.
5. Use USB first. Do not wire TELEM/UART until the USB path is verified.

Milestone 1 reads telemetry only.

## Verify the operating system

Run on the Raspberry Pi:

```bash
cat /etc/os-release
uname -m
```

The expected architecture is `aarch64`. If it reports `armv7l`, install
a 64-bit Raspberry Pi OS before the ARM deployment milestone.

## Verify Camera Module 3

On current Raspberry Pi OS images:

```bash
rpicam-hello --list-cameras
```

The output should list an IMX708 camera. Do not troubleshoot the camera
and Pixhawk simultaneously; validate one device at a time.

Run the bounded raw-camera benchmark from the deployed package:

```bash
bin/benchmark_pi_camera.sh
```

The default profile captures 300 `1280x720 YUV420` frames at 30 FPS.
It writes PTS, per-frame metadata, process samples, the raw `rpicam` log,
and JSON/Markdown reports under:

```text
~/.local/state/onboard_autonomy/camera/<run-id>/
```

This baseline measures cadence, estimated frame gaps, CPU, RSS, and
sensor metadata independently from OnboardAutonomy.

The deployed runtime receiver can then be tested together with Pixhawk:

```bash
bin/run_onboard_autonomy_pi.sh
```

The launcher enables `640x480 YUV420 @ 30 FPS` camera reception by
default. The `camera` object in each JSON line reports measured FPS,
processing drops, frame age, and `FrameWallClock` to application
latency. Disable it for a telemetry-only run with:

```bash
ONBOARD_AUTONOMY_CAMERA_ENABLED=0 bin/run_onboard_autonomy_pi.sh
```

## Install the Milestone 1 toolchain

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git ninja-build python3-venv
```

Camera and vision packages are added in a later milestone:

```bash
sudo apt-get install -y gstreamer1.0-tools libopencv-dev
```

## Connect Pixhawk by USB

Connect the Pixhawk 6C USB-C port to a USB host port on the Raspberry Pi.
Then inspect serial devices:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

ArduPilot USB commonly appears as `/dev/ttyACM0`. Grant the current user
serial access:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership.

## Build and run

### Packaged ARM64 candidate

On the Ubuntu development host:

```bash
bash scripts/package_pi5_release.sh
```

Transfer `artifacts/onboard_autonomy-pi5-arm64.tar.gz` to the Pi, then:

```bash
tar -xzf onboard_autonomy-pi5-arm64.tar.gz
cd onboard_autonomy-pi5
bin/diagnose_pi_hardware.sh
bin/run_onboard_autonomy_pi.sh
```

The diagnostic checks architecture, `dialout`, Camera Module 3, serial
candidates, ARM64 ELF format, and runtime libraries. The launcher:

- prefers stable `/dev/serial/by-id` names;
- accepts exactly one serial candidate and refuses to guess otherwise;
- defaults to 115200 baud;
- runs `--transport serial --serial-device ... --json --camera` by default;
- stores JSONL under `~/.local/state/onboard_autonomy`.

Override an ambiguous serial device explicitly:

```bash
ONBOARD_AUTONOMY_SERIAL=/dev/serial/by-id/usb-... \
    bin/run_onboard_autonomy_pi.sh
```

### Install as a boot service

The packaged installer copies the release to `/opt/onboard-autonomy`,
installs a hardened non-root `systemd` unit, and preserves an existing
configuration during upgrades:

```bash
sudo bin/install_onboard_autonomy_service.sh
sudo nano /etc/onboard-autonomy/onboard-autonomy.env
sudo systemctl enable --now onboard-autonomy@"$USER".service
```

Inspect the service and follow its operator output with:

```bash
systemctl status onboard-autonomy@"$USER".service
journalctl -u onboard-autonomy@"$USER".service -f
```

The service runs as the selected Pi user rather than `root`. If no serial
candidate exists at boot, `systemd` retries the launcher every three seconds,
so connecting the Pixhawk later does not require a manual restart. Camera
process and stream recovery happens inside the runtime.

Telemetry JSONL files remain under
`~/.local/state/onboard_autonomy`. The default policy keeps at most 20 files,
10 MiB per file, and 100 MiB in total. Each JSON line is flushed immediately
and mirrored to `journald`; these limits can be changed in the environment
file through `ONBOARD_AUTONOMY_LOG_MAX_FILES`,
`ONBOARD_AUTONOMY_LOG_MAX_FILE_BYTES`, and
`ONBOARD_AUTONOMY_LOG_MAX_TOTAL_BYTES`.

To stop or remove automatic startup:

```bash
sudo systemctl disable --now onboard-autonomy@"$USER".service
```

### Profile the complete runtime

Stop the managed service first so only one runtime owns the camera and serial
device, then run the bounded process-group profiler from the package:

```bash
sudo systemctl stop onboard-autonomy@"$USER".service
bin/profile_onboard_autonomy_pi.sh
```

The default 60-second run measures the C++ runtime and all of its child
processes together, including `rpicam`, camera preview, and the Python JSONL
sink. It records average/p95 CPU, aggregate peak RSS, process count, SoC
temperature, and Raspberry Pi throttling bits under:

```text
~/.local/state/onboard_autonomy/profiles/<run-id>/
```

The report fails if it is not running on ARM64, the runtime exits early, the
sample window is incomplete, `vcgencmd` data is absent, or any throttling bit
is observed. Change the duration only when needed:

```bash
ONBOARD_AUTONOMY_PROFILE_SECONDS=120 \
  bin/profile_onboard_autonomy_pi.sh
```

Restart the managed service after the profile:

```bash
sudo systemctl start onboard-autonomy@"$USER".service
```

The cross-built binary is a deployment candidate, not an ABI promise.
Ubuntu and Raspberry Pi OS can ship different glibc versions. If the
diagnostic reports missing runtime support, perform the native build
below instead of copying random libraries.

### Native Raspberry Pi build

From the OnboardAutonomy source directory on the Pi:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
ONBOARD_AUTONOMY_BINARY=./build/onboard_autonomy \
    scripts/run_onboard_autonomy_pi.sh
```

Expected behavior:

- JSONL snapshots and transition events are written to a timestamped log.
- `connected` changes to `true` after an ArduPilot heartbeat.
- GPS and battery remain not ready if the corresponding messages are
  absent.
- Disconnecting USB changes `connected` to `false` after the freshness
  timeout. The Linux transport detects device hangup and retries the same
  stable path once per second without restarting the process. After heartbeat
  recovery, all six telemetry-rate requests start again. Prefer a
  `/dev/serial/by-id/...` path because `/dev/ttyACM0` can be renumbered after
  reconnecting USB.
- `camera.phase` changes from `starting` to `streaming`.
- On the verified Pi 5 and IMX708 Wide setup, the runtime sustained
  `30.013 FPS` with zero processing drops and approximately `10 ms`
sensor-to-application latency.

It also enables a 10 FPS grayscale diagnostic preview:

```text
http://companionpi.local:8080/
```

The page shows the exact Y plane consumed by the AprilTag detector and
overlays only confirmed targets. It is unauthenticated HTTP intended for
the local trusted bench network. Disable it with:

```bash
ONBOARD_AUTONOMY_CAMERA_PREVIEW_ENABLED=0 \
  bin/run_onboard_autonomy_pi.sh
```

## Camera calibration

Print `assets/calibration/checkerboard-9x6-25mm-a4.svg` at 100% scale and
verify its 100 mm reference line with a ruler. Then capture the calibration
views on the Raspberry Pi:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r python/requirements.txt
ONBOARD_AUTONOMY_PYTHON=.venv/bin/python \
  bash scripts/capture_camera_calibration.sh
```

In the extracted ARM64 package, use its root-level `requirements.txt`
instead of `python/requirements.txt`.

The capture uses `640x480` and the same fixed `manual/default` hyperfocal
lens policy as the runtime. The analyzer accepts only complete
checkerboard views, calculates pinhole intrinsics and Brown-Conrady
distortion, records input hashes and per-view reprojection errors, and
fails when the quality gate is not met. Do not commit guessed or generic
Camera Module 3 intrinsics as if they belonged to the physical camera.

## Windows launcher

`StartOnboardAutonomyPixhawk.cmd` opens the Raspberry Pi runtime over SSH
and then opens the local camera-preview page. Machine-specific values
belong in the ignored `OnboardAutonomyLocal.cmd` file at the repository
root:

```bat
set "ONBOARD_AUTONOMY_PI_HOST=companionpi.local"
set "ONBOARD_AUTONOMY_PI_USER=companion"
set "ONBOARD_AUTONOMY_SSH_KEY=%USERPROFILE%\.ssh\onboard_autonomy_ed25519"
set "ONBOARD_AUTONOMY_REMOTE_ROOT=/home/companion/onboard_autonomy-pi5"
set "ONBOARD_AUTONOMY_SERIAL=/dev/serial/by-id/<your-pixhawk-device>"
```

The launcher lets the Pi auto-detect one serial device when
`ONBOARD_AUTONOMY_SERIAL` is unset. It also reads an existing ignored
`CompanionLabLocal.cmd` and maps its legacy variables during the rename
transition; this compatibility path is not part of the public runtime
configuration.

## Verified Pixhawk 6C TELEM2 UART

The physical Raspberry Pi 5/Pixhawk 6C path is verified at 57600 baud. Power
both computers off before changing any wire, remove propellers, and keep the
Pi and Pixhawk on their normal independent power paths.

| Pixhawk 6C TELEM2 | Raspberry Pi 5 |
| --- | --- |
| Pin 6, GND | Physical pin 6, GND |
| Pin 3, UART5 RX | Physical pin 8, GPIO14 TX |
| Pin 2, UART5 TX | Physical pin 10, GPIO15 RX |

Do not connect TELEM2 `+5 V`; CTS and RTS are unused by this three-wire path.
After applying `bin/configure_pi5_uart.sh` and rebooting the Pi, run:

```bash
ONBOARD_AUTONOMY_SERIAL=/dev/ttyAMA0 \
ONBOARD_AUTONOMY_BAUD=57600 \
  bin/run_onboard_autonomy_pi.sh
```

The baud rate must match the ArduPilot `SERIAL2_BAUD` configuration. The
recorded controller used 57600; do not assume this value for a different
aircraft. See `docs/evidence/uart-hardware.md` for the physical acceptance
result and its safety limitations.
