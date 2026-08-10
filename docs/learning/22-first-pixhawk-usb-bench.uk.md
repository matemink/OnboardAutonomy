# Перший реальний Pixhawk 6C USB bench

## Мета ітерації

Замінити ArduCopter SITL реальним Pixhawk 6C, не підключаючи польотну
батарею і не надсилаючи команди руху:

```text
OnboardAutonomy on Raspberry Pi 5
    -> USB CDC serial
    -> Holybro Pixhawk 6C
    -> ArduPilot
```

## Перша hardware-проблема

Перший USB-A -> USB-C кабель подавав живлення, тому Pixhawk світився,
але Raspberry Pi бачив лише власні USB root hubs. У kernel log не було
навіть спроби USB enumeration.

Це доводить лише таке:

```text
VBUS + GND працюють
D+ + D- не працюють або відсутні
```

Після заміни на data cable Linux одразу побачив:

```text
Holybro Pixhawk6C-bdshot
```

Під час старту також було видно короткий bootloader stage:

```text
PX4 BL FMU v6C.x
```

Назва bootloader не означає, що на контролері запущено PX4. Після
завантаження firmware USB product змінився на ArduPilot device.

## Два serial interfaces

Pixhawk створив два CDC ACM devices:

```text
if00 -> /dev/ttyACM0 -> MAVLink
if02 -> /dev/ttyACM1 -> SLCAN
```

OnboardAutonomy використовував стабільний `/dev/serial/by-id/...-if00`,
а не нестабільне ім'я `/dev/ttyACM0`.

Launcher навмисно не вгадує, якщо бачить кілька serial candidates.
Для bench потрібний endpoint передано явно:

```bash
ONBOARD_AUTONOMY_SERIAL=/dev/serial/by-id/...-if00 \
ONBOARD_AUTONOMY_BAUD=115200 \
    bin/run_onboard_autonomy_pi.sh
```

## Перший реальний MAVLink

Pixhawk передав:

```text
system_id:       1
component_id:    1
vehicle_type:    2 (quadrotor)
autopilot_type:  3 (ArduPilot)
system_status:   3 (standby)
armed:           false
GPS fix type:    1
satellites:      0
```

Також отримано реальні повідомлення:

```text
PreArm: RC not found
PreArm: Battery 1 below minimum arming voltage
```

Це перший доказ, що той самий C++ data path працює не лише із SITL.

## Hardware trace знайшов domain bug

Перший JSON показав суперечність:

```text
battery_voltage_v:    0.00
battery_remaining_pct: 99
battery_ready:        true
```

Попередня логіка перевіряла лише `voltage > 0` та percentage. Pixhawk
передавав кілька мілівольт біля нуля, які після округлення виглядали як
`0.00 V`, але формально проходили перевірку.

Не можна виправляти це універсальним вигаданим порогом на кшталт
`3 V`, бо реальний мінімум залежить від конфігурації батареї.

## Read-only BATT_ARM_VOLT path

Офіційна документація ArduPilot визначає `BATT_ARM_VOLT` як джерело
перевірки `Battery below minimum arming voltage`.

Додано мінімальний parameter data path:

```text
MavlinkEncoder
    -> PARAM_REQUEST_READ("BATT_ARM_VOLT")
Pixhawk
    -> PARAM_VALUE
MavlinkDecoder
    -> VehicleState::on_battery_arming_voltage()
VehicleState
    -> voltage >= configured threshold
```

На відміну від `COMMAND_LONG`, `PARAM_REQUEST_READ` не очікує
`COMMAND_ACK`. Успішною відповіддю є `PARAM_VALUE` з тим самим
`param_id`.

MAVLink `param_id` має фіксовані 16 байтів. Encoder використовує:

```cpp
constexpr std::array<char, 16> kBatteryArmingVoltageParameter{
    'B', 'A', 'T', 'T', '_', 'A', 'R', 'M', '_', 'V', 'O', 'L', 'T',
};
```

Zero-filled `std::array` важливий: generated MAVLink packer читає всі
16 байтів. Передача коротшого string literal викликала ARM64 GCC
`-Warray-bounds`, хоча x86 debug build її не помітив.

## Нова battery readiness

Battery component готовий лише коли одночасно:

1. Battery telemetry свіжа.
2. `MAV_SYS_STATUS_SENSOR_BATTERY` enabled і healthy.
3. Отримано реальний `BATT_ARM_VOLT`.
4. Voltage не нижча за цей поріг, якщо він не дорівнює нулю.
5. Percentage не нижче 20%, якщо autopilot його надає.
6. Немає активного warning, що містить `battery`.

До відповіді `PARAM_VALUE` батарея залишається not ready, тому
початкового хибного зеленого стану більше немає.

## Підтверджений результат

Реальний Pixhawk повернув:

```text
battery_arming_voltage_v: 14.70
battery_voltage_v:         0.00
battery_ready:             false
armable:                   false
```

Цей стан був правильним уже до повторного `STATUSTEXT`.

Hardware log:

```text
artifacts/hardware/pixhawk6c-usb-20260727T201836Z.jsonl
```

## Tests і build gates

- Повний native x86-64 C++ suite пройшов.
- Додано encoder test для `PARAM_REQUEST_READ`.
- Додано decoder test для `PARAM_VALUE`.
- Додано domain regressions для unhealthy battery sensor, battery
  PreArm warning і voltage нижче `BATT_ARM_VOLT`.
- ARM64 GCC 12 build пройшов без warning.
- ABI gate: `GLIBC_2.34`, `GLIBCXX_3.4.29`.
- Фінальний package SHA-256:
  `7da7820414a4fab2fae158b6bbf4bfbb927b392c5785f0ae1c79fbb71d110c00`.

## Safety boundary

Hardware launcher дозволив лише:

- companion `HEARTBEAT`;
- `MAV_CMD_SET_MESSAGE_INTERVAL`;
- `PARAM_REQUEST_READ`.

Serial mode не дозволив `ARM`, `TAKEOFF`, `LAND`, route або scenario
commands.

## UART status

Pi 5 UART software configuration уже підготовлена:

```text
dtoverlay=uart0-pi5
```

На момент першого USB bench фізичне TELEM2 wiring ще не було перевірене.
Подальший hardware acceptance 2026-08-10 підтвердив `/dev/ttyAMA0` на
GPIO14/GPIO15, перехресне TX/RX-з'єднання та двонаправлений MAVLink на
57600 baud: OnboardAutonomy отримав heartbeat і завершив усі 6/6 запитів
частоти телеметрії. Деталі та межі твердження записані в
`docs/evidence/uart-hardware.md`.

Офіційні джерела:

- https://ardupilot.org/copter/docs/common-prearm-safety-checks.html
- https://mavlink.io/en/services/parameter.html
- https://mavlink.io/en/messages/common.html
- https://ardupilot.org/dev/docs/USB-IDs.html
