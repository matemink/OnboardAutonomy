# Serial recovery evidence

## Scope

The automated Linux acceptance test exercises the same POSIX adapter used for
Pixhawk USB/UART. A separate physical run verifies the behavior on Raspberry
Pi 5 and Pixhawk 6C hardware.

## Sequence

1. Create a Linux pseudo-terminal and expose its slave through a stable
   temporary symlink.
2. Open the symlink through `PosixSerialTransport` and receive bytes from the
   first master endpoint.
3. Close the master endpoint and assert that `POLLHUP` is non-fatal.
4. Replace the symlink target with a second pseudo-terminal.
5. Wait past the configured reconnect interval and send bytes through the same
   transport object.
6. Assert that the second master endpoint receives the complete frame.
7. Separately drive all six telemetry requests to `active`, report a lost
   connection, and assert that a reconnect restarts with `SYS_STATUS`.

## Automated result

The complete C++ suite passed 50 consecutive `ctest --repeat until-fail`
runs on Ubuntu 24.04 under WSL2. The transport object survived descriptor
hangup and reused the stable device path; telemetry configuration discarded
the old session and restarted from request 1 of 6.

## Physical USB result

The physical acceptance run was completed on 2026-08-10 using the stable
Pixhawk path
`/dev/serial/by-id/usb-Holybro_Pixhawk6C-bdshot_39004E001051333230323637-if00`.
The flight controller was unplugged only after the link had reached
`telemetry_setup=active (6/6)`, then reconnected while the same
OnboardAutonomy process continued running.

Across 416 captured snapshots, the observed state transitions were:

```text
connected=true  telemetry_setup=configuring (0/6)
connected=true  telemetry_setup=active (6/6)
connected=false telemetry_setup=waiting_for_vehicle (0/6)
connected=true  telemetry_setup=active (6/6)
```

The process was not restarted. It discarded the previous telemetry session,
reopened the stable device path, received the returning heartbeat, and
repeated all six acknowledged message-rate requests. Every transition
reported `motion_commands_allowed=false`.

The source capture is generated as
`artifacts/usb-recovery-v1.0-final.jsonl`; the compact transition evidence is
kept here instead of committing the large runtime log.
