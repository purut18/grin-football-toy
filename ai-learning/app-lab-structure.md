# Arduino App Lab — App Structure & Deployment Guide

> **Source**: [UNO Q User Manual](https://docs.arduino.cc/tutorials/uno-q/user-manual/)
> **Last Updated**: 2026-06-20

---

## 1. What is an "App"?

An **App** in Arduino App Lab is a self-contained project that bundles:
- An **Arduino sketch** (C/C++) for the STM32 MCU (real-time I/O, sensors, actuators)
- A **Python script** for the Qualcomm QRB2210 MPU (AI/ML, networking, high-level logic)

These two components communicate via the **Bridge RPC** system.

---

## 2. App Folder Structure

Every App created in Arduino App Lab follows this **strict directory structure**:

```
<app-name>/
├── sketch/
│   └── sketch.ino          # Arduino sketch for STM32 MCU (Zephyr RTOS)
├── python/
│   └── main.py             # Python script for Qualcomm MPU (Debian Linux)
└── metadata.json           # (Optional) App metadata — name, version, etc.
```

### Key Rules:
1. The Arduino sketch MUST be at `sketch/sketch.ino` — this exact path.
2. The Python script MUST be at `python/main.py` — this exact path.
3. The folder name IS the app name (used by `arduino-app-cli`).
4. Additional Python modules can be placed alongside `main.py` in `python/`.
5. Additional Arduino libraries/headers can go in `sketch/` alongside `sketch.ino`.

---

## 3. MCU Side: `sketch/sketch.ino`

Standard Arduino C/C++ sketch running on **STM32U585 via Zephyr RTOS**.

### Typical Structure:
```cpp
#include <Arduino_RouterBridge.h>  // Bridge RPC library (if MPU communication needed)
#include <Arduino_LED_Matrix.h>     // LED matrix (if needed)

void setup() {
    Serial.begin(9600);             // Prints to App Lab console
    Bridge.begin();                 // Initialize RPC bridge to MPU
    
    // Register functions callable from Python/MPU
    Bridge.provide("function_name", local_function);
    
    // Pin configuration
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    // Real-time sensor reading, actuator control
}
```

### Important Notes:
- `Serial` → prints to Arduino App Lab Console (not hardware UART)
- `Serial1` → hardware UART on D0/D1 — **RESERVED by arduino-router** (do NOT use)
- `Bridge.provide()` → exposes MCU function to MPU (runs in background RPC thread)
- `Bridge.provide_safe()` → same but runs in main `loop()` context (safe for Arduino APIs)
- `Bridge.call("method", args)` → calls MPU-side function from MCU
- **⚠️ Do NOT use** `Bridge.call()`, `Monitor.print()`, or `Serial.print()` inside `provide()` callbacks — causes deadlocks

---

## 4. MPU Side: `python/main.py`

Python script running on **Qualcomm QRB2210 under Debian Linux**.

### Typical Structure:
```python
import time
from arduino.app_utils import App      # App lifecycle framework
from arduino.app_utils import Bridge   # RPC bridge to MCU
from arduino.app_utils import Leds     # LED control module

def loop():
    """Main application loop — called repeatedly by the App framework."""
    # Call MCU-side functions via Bridge RPC
    Bridge.call("function_name", arg1, arg2)
    time.sleep(1)

# Entry point — starts the app loop
App.run(user_loop=loop)
```

### Available Python Modules:
| Module | Import | Purpose |
|--------|--------|---------|
| `App` | `from arduino.app_utils import App` | App lifecycle management |
| `Bridge` | `from arduino.app_utils import Bridge` | RPC calls to/from MCU |
| `Leds` | `from arduino.app_utils import Leds` | Control MPU RGB LEDs 1 & 2 |

### Leds API:
```python
# Arguments: (R, G, B) — values 0 or 1
Leds.set_led1_color(1, 0, 0)  # LED 1 = red
Leds.set_led2_color(0, 1, 0)  # LED 2 = green
```

### Direct LED control (via sysfs):
```python
# LED 1 segments
"/sys/class/leds/red:user/brightness"
"/sys/class/leds/green:user/brightness"
"/sys/class/leds/blue:user/brightness"

# LED 2 segments
"/sys/class/leds/red:panic/brightness"
"/sys/class/leds/green:wlan/brightness"
"/sys/class/leds/blue:bt/brightness"
```

---

## 5. Running & Deploying Apps

### From Arduino App Lab UI:
1. Open your App in the Arduino App Lab
2. Click **Run** button (top-right corner)
3. The app is compiled, uploaded to MCU, and Python script starts on MPU

### Run at Startup (Auto-boot):
1. Open your custom App
2. Click the ▼ arrow next to Run button
3. Toggle **"Run at startup"** → ON
4. A `DEFAULT` badge appears next to the app name

### Via CLI (on UNO Q terminal):
```bash
# Set an app as the default startup app
arduino-app-cli properties set default user NAME_OF_YOUR_APP
```

### Important:
- You CANNOT set built-in Examples as startup apps directly — must **Copy and edit** first.
- The app runs on the UNO Q's internal eMMC storage.

---

## 6. File Transfer to UNO Q

The UNO Q can be accessed via:

| Method | Tool | Use Case |
|--------|------|----------|
| **USB-C** | Arduino App Lab (PC-hosted) | Primary development |
| **Network** | Arduino App Lab (Network Mode) | Remote development via mDNS |
| **SSH** | `ssh user@<ip>` | File transfer, terminal access |
| **ADB** | `adb push/pull` | Android Debug Bridge file ops |

### ADB File Transfer:
```bash
# Push a folder to the UNO Q
adb push ./my-app/ /home/user/apps/my-app/

# Pull files from UNO Q
adb pull /home/user/apps/my-app/ ./local-copy/

# Open a shell on the UNO Q
adb shell
```

### SSH File Transfer:
```bash
# Copy files via SCP
scp -r ./my-app/ user@<uno-q-ip>:/home/user/apps/

# SSH into the board
ssh user@<uno-q-ip>
```

---

## 7. Bridge RPC Deep Dive

### MCU → MPU (calling Python from Arduino):
```cpp
// In sketch.ino
auto result = Bridge.call("python_function", arg1, arg2);
```

### MPU → MCU (calling Arduino from Python):
```python
# In main.py
Bridge.call("arduino_function", arg1, arg2)
```

### MCU exposing functions to MPU:
```cpp
// In sketch.ino setup()
Bridge.provide("set_led", set_led_handler);       // Runs in RPC thread (fast but unsafe for Arduino APIs)
Bridge.provide_safe("set_led", set_led_handler);   // Runs in main loop (safe for Arduino APIs)
```

### Advanced: Direct Unix Socket RPC
- Router socket: `/var/run/arduino-router.sock`
- Protocol: MessagePack RPC over Unix Domain Socket
- Can be used from any language (Python, C++, Rust, Go) without Arduino App Lab
- Useful for integrating into existing Linux applications

---

## 8. Router Service Management

```bash
# Check status
systemctl status arduino-router

# Restart (if communication stuck)
sudo systemctl restart arduino-router

# View logs
journalctl -u arduino-router

# Enable verbose logging
sudo nano /etc/systemd/system/arduino-router.service
# Add --verbose to ExecStart line
sudo systemctl daemon-reload
sudo systemctl restart arduino-router
```
