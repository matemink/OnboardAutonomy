# Pixhawk TELEM2 UART evidence

Verified on 2026-08-10 with Raspberry Pi 5 and a Holybro Pixhawk 6C running
ArduCopter 4.5.6.

## Physical path

The Pixhawk was powered through its normal power module and flight battery.
The Raspberry Pi used its own USB-C supply. No USB data cable connected the
two computers during this acceptance run.

| Pixhawk 6C TELEM2 | Signal | Raspberry Pi 5 header |
| --- | --- | --- |
| Pin 6, black | GND | Physical pin 6, GND |
| Pin 3, yellow | UART5 RX | Physical pin 8, GPIO14 TX |
| Pin 2, green | UART5 TX | Physical pin 10, GPIO15 RX |

The `+5 V`, CTS, and RTS contacts were not connected. Both UART endpoints use
3.3 V signaling, and TX/RX are crossed.

## Acceptance

The Pi exposed the configured controller UART as `/dev/ttyAMA0`; the runtime
user belonged to `dialout`. The Pixhawk USB device was absent, proving that a
USB path could not satisfy the test accidentally.

Common rates were tested without changing flight-controller parameters:

| Baud | Result |
| ---: | --- |
| 921600 | No heartbeat |
| 115200 | No heartbeat |
| 57600 | Connected |
| 460800 | No heartbeat |

At 57600 baud, OnboardAutonomy received system/component IDs `1/1`, decoded
the Pixhawk firmware and board metadata, and completed all six requested
MAVLink stream acknowledgements. The final state was:

```text
connected=true
telemetry_setup=active (6/6)
camera=streaming
motion_commands_allowed=false
```

The result demonstrates bidirectional MAVLink over the physical TELEM2/GPIO
UART path: receiving a heartbeat alone cannot complete six acknowledged
message-rate commands.

## Safety finding

The runtime independently read the GCS failsafe parameters and rejected
autonomous startup because `FS_GCS_ENABLE` was not the required value `5`
(`Always LAND`). This is an expected safety gate, not a UART failure. The
bench run did not alter the real flight-controller parameter and did not send
arm, motor-test, takeoff, or movement commands.

The source capture is generated as
`artifacts/uart-telem2-57600-v1.0.jsonl`; large runtime logs remain excluded
from the repository.
