# 🌱 Soil Testing Kit — ESP32 BLE GATT Firmware

A multi-sensor embedded firmware for the ESP32, built with **ESP-IDF v5.x** and **FreeRTOS**. It reads soil health parameters via RS485 Modbus RTU, GPS coordinates via NMEA, and ambient light via ADC — then streams all data to a Flutter app over **BLE GATT** as a compact 31-byte binary packet.

---

## Table of Contents

- [Hardware](#hardware)
- [Pin Configuration](#pin-configuration)
- [BLE Protocol](#ble-protocol)
- [Binary Packet Format](#binary-packet-format)
- [Sensor Details](#sensor-details)
- [LED Indicators](#led-indicators)
- [Button Behaviour](#button-behaviour)
- [FreeRTOS Task Architecture](#freertos-task-architecture)
- [Building & Flashing](#building--flashing)
- [Flutter Integration Notes](#flutter-integration-notes)

---

## Hardware

| Component | Interface | Notes |
|-----------|-----------|-------|
| ESP32 | — | Dual-core, BLE + Wi-Fi SoC |
| 7-in-1 Soil Sensor | RS485 / Modbus RTU | Temp, Moisture, EC, pH, N, P, K |
| MAX485 Transceiver | GPIO (DE/RE) | Half-duplex RS485 driver |
| NEO-6M GPS Module | UART2 | NMEA 0183 sentences |
| LDR (Light Sensor) | ADC1 CH6 (GPIO34) | Ambient light percentage |
| RGB LED | GPIO 12/13/14 | Status indicator |
| Push Button | GPIO 32 | Long press to disconnect/reconnect |

---

## Pin Configuration

```
GPS (UART2)
  RX  → GPIO 16
  TX  → GPIO 17
  Baud: 9600

Soil Sensor (UART1 / RS485)
  TX  → GPIO 18
  RX  → GPIO 19
  DE/RE → GPIO 4
  Baud: 9600

RGB LED
  RED   → GPIO 12
  BLUE  → GPIO 13
  GREEN → GPIO 14

LDR
  ADC1 CH6 → GPIO 34

Button
  GPIO 32  (active HIGH, external pull-down)
```

---

## BLE Protocol

| Property | Value |
|----------|-------|
| Device Name | `SoilTestKit` |
| Service UUID | `0x00FF` |
| DATA Characteristic | `0xFF01` — NOTIFY + READ |
| CMD Characteristic | `0xFF02` — WRITE (no response) |
| MTU | 500 bytes |
| Advertising type | Undirected connectable (`ADV_IND`) |

### Connecting

1. Scan for a device advertising name `SoilTestKit`.
2. Connect and discover services.
3. Enable notifications on `0xFF01` (write `0x0100` to its CCCD).
4. The device begins sending a 31-byte packet on every sensor cycle (~2 s).

### CMD Characteristic (`0xFF02`)

Write `GET` (3 bytes, no null terminator required) to trigger an **immediate** out-of-cycle push of the latest sensor values.

---

## Binary Packet Format

Every notification on `0xFF01` is exactly **31 bytes**, little-endian.

```
Offset  Size  Type      Field            Scaling / Notes
------  ----  -------   ---------------  ------------------------------------
  0      1    uint8     Start byte       0xAA (fixed)
  1      1    uint8     Version          0x01
  2      1    uint8     Packet ID        Monotonically incrementing (wraps at 255)
  3      1    uint8     Payload length   25 (fixed)
  4      2    uint16    LDR (light)      Raw × 100  →  divide by 100.0 for %
  6      2    uint16    Temperature      Raw × 100  →  divide by 100.0 for °C
  8      2    uint16    pH               Raw × 100  →  divide by 100.0
 10      2    uint16    Moisture         Raw × 100  →  divide by 100.0 for %
 12      2    uint16    Conductivity     µS/cm, as-is
 14      2    uint16    Nitrogen         mg/kg, as-is
 16      2    uint16    Phosphorus       mg/kg, as-is
 18      2    uint16    Potassium        mg/kg, as-is
 20      4    int32     Latitude         Degrees × 1 000 000  (e.g. 6927079 → 6.927079°)
 24      4    int32     Longitude        Degrees × 1 000 000  (e.g. 79861243 → 79.861243°)
 28      1    uint8     GPS fix valid    0x01 = valid fix, 0x00 = no fix
 29      1    uint8     Checksum         XOR of bytes 0–28
 30      1    uint8     End byte         0x55 (fixed)
```

> **Checksum validation:** XOR all bytes from index 0 to 28 inclusive. The result must equal byte 29.

---

## Sensor Details

### 7-in-1 Soil Sensor (Modbus RTU)

Each parameter is read with a separate single-register request (function code `0x03`).

| Parameter | Register | Scaling |
|-----------|----------|---------|
| Temperature | `0x0013` | ÷ 10.0 → °C |
| Moisture | `0x0012` | ÷ 10.0 → % |
| EC (Conductivity) | `0x0015` | as-is → µS/cm |
| pH | `0x0006` | ÷ 100.0 |
| Nitrogen | `0x001E` | as-is → mg/kg |
| Phosphorus | `0x001F` | as-is → mg/kg |
| Potassium | `0x0020` | as-is → mg/kg |

**Retry behaviour:** Each register query is attempted up to 3 times (`MODBUS_RETRIES`). On total failure the firmware applies a 2-second bus recovery pause before moving to the next register. Failed fields retain their **last known good value** rather than resetting to zero.

### GPS (NEO-6M, UART2)

Parses `$GPRMC` and `$GPGGA` NMEA sentences. Checksum is validated before any data is accepted. Coordinates, speed, altitude, satellite count, UTC time, and date are extracted and stored. The green LED pulses for 3 s on the **first valid fix** after power-on.

### LDR (GPIO34 / ADC1 CH6)

Sampled with `adc_oneshot`. Raw 12-bit value (0–4095) is converted to a percentage:

```
ldr_pct = (raw / 4095.0) × 100.0
```

Higher percentage = more light.

---

## LED Indicators

| LED | Behaviour | Meaning |
|-----|-----------|---------|
| 🔴 Red | Solid 1 s on boot | Power-on self-test |
| 🔵 Blue | Fast blink (200 ms) | Searching / advertising |
| 🔵 Blue | Solid | BLE connected |
| 🟢 Green | Solid 3 s | First GPS fix acquired |

---

## Button Behaviour

The button on **GPIO 32** (active HIGH) supports a single gesture:

| Gesture | Action |
|---------|--------|
| Hold ≥ 2 seconds | If connected: disconnect the current BLE client. If disconnected: restart advertising. |

---

## FreeRTOS Task Architecture

Three tasks run concurrently. Sensor data is shared via a **FreeRTOS mutex**; inter-task signalling uses an **event group**.

```
Core 0                          Core 1
──────────────────────────────  ──────────────────────────────
sensor_task  (priority 5)       bt_led_task  (priority 3)
  • Reads 7 Modbus registers       • Manages RGB LED states
  • Reads LDR via ADC              • Polls button
  • Updates g_sensor (mutex)       • Calls ble_send_packet()
  • Sets EVT_SENSOR_READY          • Handles BLE state machine

gps_task     (priority 4)
  • Reads NMEA from UART2
  • Updates g_gps (mutex)
```

| Event bit | Source | Consumer |
|-----------|--------|----------|
| `EVT_SENSOR_READY` | `sensor_task` after each cycle | `bt_led_task` triggers BLE notify |
| `EVT_BLE_SEND_NOW` | CCCD enable / `GET` command | `bt_led_task` triggers immediate notify |

---

## Building & Flashing

> Requires **ESP-IDF v5.x** installed (e.g. at `C:\Espressif\` on Windows).

```bash
# Set target (delete sdkconfig and build/ first if switching from Classic BT)
idf.py set-target esp32

# Build
idf.py build

# Flash and monitor
idf.py -p COMx flash monitor
```

**Important `sdkconfig` options** (enable via `idf.py menuconfig` or set directly):

```
CONFIG_BT_ENABLED=y
CONFIG_BT_BLE_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=y
CONFIG_BT_CLASSIC_BT_ENABLED=n    # Classic BT NOT used — BLE only
CONFIG_FREERTOS_UNICORE=n          # Dual-core required
```

> If BT config changes are not taking effect, **delete `sdkconfig` and the `build/` folder** before rebuilding.

---

## Flutter Integration Notes

- Use the [`flutter_blue_plus`](https://pub.dev/packages/flutter_blue_plus) package.
- Scan for `SoilTestKit`, connect, and subscribe to characteristic `0xFF01`.
- Decode each 31-byte notification using the packet table above.
- Validate the XOR checksum before processing.
- Write `GET` (UTF-8 bytes `[0x47, 0x45, 0x54]`) to `0xFF02` to request an immediate update.
- GPS coordinates are `int32` (divide by `1,000,000.0` to get decimal degrees).
- All multi-byte fields are **little-endian**.

---

## License

This project is proprietary to **Advanced Research Computing (ARC)**. All rights reserved.