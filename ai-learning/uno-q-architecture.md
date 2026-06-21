# Arduino UNO Q — Architecture & Technical Reference

> **Source**: [UNO Q User Manual](https://docs.arduino.cc/tutorials/uno-q/user-manual/)
> **Last Updated**: 2026-06-20

---

## 1. Dual-Processor Architecture

The UNO Q is a **hybrid single-board computer** with two independent processors on one board:

| Component | Chip | Core | Clock | OS | Role |
|-----------|------|------|-------|----|------|
| **MPU** (Microprocessor) | Qualcomm® QRB2210 | Quad-core Arm® Cortex®-A53 | 2.0 GHz | Debian Linux | High-level compute, AI/ML, networking, Python |
| **MCU** (Microcontroller) | STM32U585 | Arm® Cortex®-M33 | 160 MHz | Zephyr RTOS | Real-time I/O, Arduino sketches, sensor reads |

### Inter-Processor Communication: The Bridge (RPC)
- The MPU and MCU talk via **RPC (Remote Procedure Call)** using the `Bridge` library.
- Under the hood, a Linux service called `arduino-router` manages bidirectional MessagePack RPC over serial (`/dev/ttyHS1` on Linux, `Serial1` on MCU).
- **Star Topology**: Multiple Linux processes can communicate with the MCU simultaneously.
- **⚠️ Reserved**: `/dev/ttyHS1` (Linux) and `Serial1` (MCU) are locked by the router — DO NOT open in user code.

---

## 2. Hardware Specs

### Memory
- **eMMC Storage**: 16 GB or 32 GB options
- **LPDDR4 RAM**: 2 GB or 4 GB options (4 GB recommended for standalone SBC mode)

### Wireless (WCBN3536A)
- **Wi-Fi® 5**: Dual-band 2.4/5 GHz
- **Bluetooth® 5.1**
- Both with onboard antennas

### GPU
- Adreno™ 702 GPU @ 845 MHz (3D graphics acceleration)
- Dual ISPs supporting up to 25 MP @ 30 fps

### MCU Flash/RAM
- 2 MB flash, 786 KB SRAM on STM32U585

### Form Factor
- Classic **Arduino UNO** form factor — compatible with UNO shields

---

## 3. Pinout Summary

### Digital Pins (STM32 MCU) — 47 total
- 22 exposed via UNO-style headers (D0–D21)
- 25 via JMISC connector
- Notable: D0/D1 = UART RX/TX, D4/D5 = CAN Bus, D10-D13 = SPI, D20/D21 = I2C

### Analog Pins — 6x 14-bit ADC
- A0–A5 (PA4, PA5, PA6, PA7, PC1, PC0)
- Configurable resolution: 8, 10, 12, or 14 bits
- Default VREF: 3.3V (changeable via `analogReference()`)

### DAC — 2 channels
- DAC0 (A0/PA4), DAC1 (A1/PA5)
- 8-12 bit resolution

### PWM — 6 channels
- D3, D5, D6, D9, D10, D11
- Fixed 500 Hz frequency
- 8-bit default resolution (configurable)

### RGB LEDs — 4 total
- **LEDs 1 & 2**: MPU-controlled via `/sys/class/leds/`
- **LEDs 3 & 4**: MCU-controlled via `digitalWrite()` (active LOW)

### LED Matrix
- 8×13 blue LED matrix, MCU-controlled
- Supports 8 levels of grayscale (3-bit)

---

## 4. Communication Protocols

| Protocol | Pins | Arduino Object | Notes |
|----------|------|---------------|-------|
| **SPI** | D10(SS), D11(MOSI), D12(MISO), D13(SCK) | `SPI` | Standard Arduino SPI library |
| **I2C** | D20(SDA), D21(SCL) | `Wire` | UNO-style headers |
| **I2C (Qwiic)** | Internal | `Wire1` | 3.3V only, Qwiic connector |
| **UART** | D0(RX), D1(TX) | `Serial1` | ⚠️ Serial1 reserved by router |
| **Serial Monitor** | N/A | `Serial` | Prints to App Lab console via RPC |
| **CAN Bus** | D4(TX), D5(RX) | FDCAN1 | CAN Bus capable |

---

## 5. Power Options

- **USB-C®**: 5 VDC 3A (15W)
- **5V Pin**: External +5 VDC supply
- **VIN Pin**: External +7-24 VDC supply
- Uses Qualcomm® PM4145 PMIC

---

## 6. USB-C Capabilities

| Feature | Spec |
|---------|------|
| USB Power (Sink) | 5 VDC 3A (15W) |
| USB Standard | USB 3.1 Gen 1 (5 Gb/s) |
| Display | DisplayPort over USB-C |
| Via dongle: HDMI | ✅ |
| Via dongle: Camera | ✅ |
| Via dongle: Audio | ✅ (USB or 3.5mm) |
| Via dongle: Ethernet | ✅ |
| Via dongle: HID | ✅ (keyboard/mouse) |
| Via dongle: Storage | ✅ (microSD/USB drive) |

---

## 7. Development Tools

### Arduino App Lab (Primary)
- Unified IDE for both MPU (Python) and MCU (Arduino sketch)
- Pre-installed on the UNO Q for SBC mode
- Also available as PC-hosted desktop app
- Supports Network Mode (remote access via mDNS)
- Uses "Apps" with `sketch/sketch.ino` + `python/main.py`

### Arduino IDE (MCU only)
- Programs only the STM32 MCU side
- Requires "Arduino UNO Q Zephyr Core" from Boards Manager
- Package URL: `https://downloads.arduino.cc/packages/package_zephyr_index.json`

---

## 8. Key Libraries

| Library | Purpose |
|---------|---------|
| `Arduino_RouterBridge.h` | Bridge RPC between MPU/MCU |
| `Arduino_LED_Matrix.h` | 8×13 LED matrix control |
| `arduino.app_utils` (Python) | App framework, Bridge, Leds module |
| `SPI.h` | SPI communication |
| `Wire.h` | I2C communication |
