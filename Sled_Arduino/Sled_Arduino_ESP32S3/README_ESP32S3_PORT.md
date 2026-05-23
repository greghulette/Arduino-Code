# Sled Controller - ESP32-S3 Port Guide

## Overview
This document explains the key changes made to port the Sled Controller from Arduino to ESP32-S3 with WCB 3.2 hardware.

## Key Changes Made

### 1. **Pin Mapping for WCB 3.2**
The ESP32-S3 uses different GPIO numbering. The code now uses WCB 3.2 pinout:

| Function | Original | ESP32-S3 (WCB 3.2) | Notes |
|----------|----------|------------------|-------|
| NeoPixel Data | GPIO 3 | GPIO 4 (S1 TX) | Serial1 TX, repurposed as data line |
| Button Input | GPIO 2 | GPIO 6 (S2 TX) | Serial2 TX, repurposed as input |
| LED Pins | 4-13 | 7-16 | Adjusted to avoid ESP32-S3 reserved pins |

**Reserved/Unavailable ESP32-S3 Pins:**
- GPIO 0, 45, 46, 48: Reserved for bootloader/flash
- GPIO 17-18: Serial0 (USB-CDC)
- GPIO 19-20: JTAG/reserved

### 2. **Removed Dependencies**
- **Removed:** `Toggle.h` library dependency
- **Reason:** Using built-in debouncing with `millis()` instead (more reliable on ESP32)
- **Benefit:** Simpler, fewer dependencies

### 3. **Serial Console**
- Changed from hardware Serial (UART1/TX1) to **USB-CDC Serial**
- On ESP32-S3, USB-CDC provides debug output via USB connection
- Baud rate: **115200** (matches USB-CDC default)

### 4. **Button Debouncing**
- Implemented software debouncing (20ms delay) instead of Toggle library
- Uses standard `digitalRead()` and `millis()`
- More compatible with ESP32-S3

### 5. **Code Improvements**
- Better Serial logging for debugging on USB
- Added status messages on startup and mode changes
- Simplified pattern initialization with explicit Index/TotalSteps reset

## Setup Instructions

### Hardware Setup
1. Connect ESP32-S3-DevKitC-1 to WCB 3.2 board (dual-row compatible)
2. **NeoPixel Strip:** Connect data line to GPIO 4
3. **Push Button:** Connect between GPIO 6 and GND (uses INPUT_PULLUP)
4. **Regular LEDs:** Connect anodes through current-limiting resistors (330Ω-1kΩ) to GPIO 7-16, cathodes to GND
5. **USB:** Connect ESP32-S3 to computer via USB-C

### Software Setup (Arduino IDE)

#### 1. Install Board Support
- Open Arduino IDE → File → Preferences
- Add to "Additional Boards Manager URLs":
  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```
- Tools → Board Manager → Search "ESP32" → Install latest version
- Select: Tools → Board → **ESP32S3 Dev Module**

#### 2. Install Libraries
- Sketch → Include Library → Manage Libraries
- Search and install:
  - **Adafruit NeoPixel** (by Adafruit)

#### 3. Configure Board Settings
- Tools → Board: **ESP32S3 Dev Module**
- Tools → USB CDC On Boot: **Enabled** (for USB Serial console)
- Tools → USB Mode: **Default** (or Hardware CDC)
- Tools → Port: Select your ESP32-S3 COM port

#### 4. Upload
- Copy code from `Sled_Arduino_ESP32S3.ino`
- Open sketch in Arduino IDE
- Click Upload

### Serial Monitor
- After upload, open Tools → Serial Monitor
- Baud: **115200**
- Should see startup message: `=== Sled Controller (ESP32-S3) Starting ===`

## Behavior

### Button Press Modes (cycles through)
1. **FADE (Blue→Off):** Smooth fade of all NeoPixels
2. **FADE (Red→Off):** Red fade pattern
3. **COLOR_WIPE (White):** Pixels fill with white
4. **COLOR_WIPE (Red):** Pixels fill with red
5. **COLOR_WIPE (Blue):** Pixels fill with blue
6. **THEATER_CHASE (Blue/Red):** Moving theater chase pattern
7. *Cycles back to mode 0*

### LED Blinking
- Random LED blinks on/off every 400ms (uses GPIO 7-16)

## Troubleshooting

### No Serial Output
- Ensure "USB CDC On Boot" is **Enabled** in board settings
- Try pressing the RST button on the ESP32-S3
- Check Windows Device Manager for COM port

### NeoPixels Not Responding
- Verify GPIO 4 connection to data line
- Check power supply (27 pixels = ~1.6A max)
- Confirm Adafruit_NeoPixel library is installed

### Button Not Working
- Verify GPIO 6 connection to button and GND
- Check that INPUT_PULLUP is working (pull-up resistor may be needed if erratic)
- Monitor Serial output for "Button pressed" messages

### LED Pins Not Blinking
- Verify GPIO 7-16 are connected properly
- Check for competing library timers/PWM

## Next Steps

### For WCB Integration
When ready to add WCB wireless communication:
1. Include `WCBClient.h` (from the WCBClient library in your workspace)
2. Initialize WCB in setup: `wcb.begin(Serial1);` (or appropriate serial port)
3. Send commands via `wcb.send()` or `wcb.broadcast()`

### For Additional Features
- Add IMU sensor integration on I2C
- Implement OTA firmware updates
- Add config storage in NVS (Non-Volatile Storage)

## Pin Reference for Future Expansion

Available ESP32-S3 GPIO for expansion:
- **PWM capable:** 0-21, 25-27, 35-36, 39-42, 47
- **ADC capable:** 1-10, 11-20, 35-37, 39-42
- **I2C:** GPIO 21 (SDA), 22 (SCL) by default, or remap others
- **SPI:** GPIO 8-11 (default), can be remapped

---

**Document Version:** 1.0  
**Date:** 2024  
**ESP32-S3 Variant:** ESP32-S3-DevKitC-1  
**WCB Version:** 3.2
