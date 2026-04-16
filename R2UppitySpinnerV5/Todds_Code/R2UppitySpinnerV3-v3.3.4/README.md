# R2 Uppity Spinner V3

ESP32-based periscope lifter and rotary controller for R2-D2 astromech droids. Supports the **Greg Hulette** and **IA-Parts** periscope lifter mechanisms with TB9051FTG motor drivers.

[![Uppity Spinner](https://i.vimeocdn.com/video/1153816619-6fef8819cf32b562e0519537a46baed562bb51651010442a9ccdd9909c40952e-d_640x360)](https://vimeo.com/558277516)

## Features

- Full web interface for control, calibration, diagnostics, and rescue
- Serial (Marcduino) and WiFi command interfaces
- Automatic safety maneuver on boot (home rotary, lower lifter)
- Soft-stop pulsed drive for lead-screw deceleration
- Encoder-based position tracking with drift correction
- Lifter distance rejection to guard against EMI-corrupted measurements
- Rescue page with 60-second safety override for field recovery
- OTA firmware updates
- Droid Remote (SMQ) wireless controller support
- Multi-lifter ID addressing
- Stored sequence playback (up to 100 sequences)
- Random animation mode with configurable speed/intervals

## Libraries Required

- [Reeltwo](https://github.com/reeltwo/Reeltwo) — Core droid framework (WiFi, web server, OTA, SMQ remote)
- [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) — RGB status LED
- [PCF8574](https://github.com/reeltwo/PCF8574) — I2C GPIO expander (optional)

## Hardware

### Microcontroller

**ESP32-WROOM-32** (or compatible). Dual-core: Core 0 handles WiFi/web, Core 1 handles motor control.

### Motor Drivers

Two **TB9051FTG** drivers — one for lifter, one for rotary.

### Pin Assignments

| Function | Pin |
|----------|-----|
| Lifter Encoder A/B | 34 / 35 |
| Lifter PWM1 / PWM2 | 32 / 33 |
| Lifter DIAG | 36 |
| Lifter Top Limit | 18 |
| Lifter Bottom Limit | 19 |
| Rotary Encoder A/B | 27 / 13 |
| Rotary PWM1 / PWM2 | 25 / 26 |
| Rotary DIAG | 39 |
| Rotary Home Limit | 23 |
| Status LED (NeoPixel) | 5 |
| RC PPM Input | 14 |
| Marcduino Serial RX | 16 |
| I2C SDA / SCL | 21 / 22 |
| PCF8574 Interrupt | 17 |

When using the I2C GPIO expander (PCF8574 at address 0x20), limit switches and motor enable pins are remapped to the expander, freeing ESP32 pins 18/19/23 for SPI (SD card).

### Recommended Motors (Pololu)

| Voltage | Lifter | Rotary |
|---------|--------|--------|
| 12V | [#4841](https://www.pololu.com/product/4841) | [#4847](https://www.pololu.com/product/4847) |
| 6V | [#4801](https://www.pololu.com/product/4801) | [#4807](https://www.pololu.com/product/4807) |

6V motors give a snappy, fast periscope. 12V motors give a slower, more deliberate motion.

### Supported Lifter Mechanisms

| Parameter | Greg Hulette | IA-Parts |
|-----------|-------------|----------|
| Minimum Power | 65% | 30% |
| Seek-to-Bottom Power | 40% | 30% |
| Full Travel Distance | ~450 ticks | ~845 ticks |

Both are auto-detected during calibration.

## Getting Started

1. Wire the ESP32, motor drivers, encoders, and limit switches per the pin table above.
2. Flash the firmware via USB or OTA.
3. Connect to the WiFi access point (default SSID: `R2Uppity`, password: `Astromech`).
4. Open `http://192.168.4.1/` in a browser.
5. Run calibration from the Setup page or send `#PSC` over serial.

Default serial baud rate is **9600**. Default I2C address is **0x20**.

## Web Interface

Access the web UI at `http://192.168.4.1/` (AP mode) or `http://R2Uppity.local/` (station mode with mDNS).

### Periscope Page (`/periscope`)

Main control interface:
- **Lift joystick** — raise/lower with real-time height percentage
- **Rotary dial** — rotate periscope with live degree readout
- **Speed sliders** — independent lift and rotary speed control
- **Light kit selector** — modes 0–7 (full cycle, off, Obi Wan, Yoda, Sith, search light, Dagobah, sparkle)
- **Preset buttons** — trigger stored sequences 0–9
- **Action buttons** — Random Move, Home, etc.
- **Status bar** — height, rotation, safety state, motor state, last command

### Rescue Page (`/rescue`)

Emergency manual control for field recovery:
- **Raw lift/rotary joysticks** — direct motor jog, bypasses sequencing
- **Safety override** — hold button 3 seconds to activate 60-second bypass of safety checks (allows descent when rotary not homed, rotary spin below safe height)
- **Motor enable/disable** toggle
- **Fault clearing**
- **Rotary home set** and **safe spin height** adjustment

### Log Page (`/log`)

Live debug output from the ESP32 serial buffer, auto-refreshing every 500ms.

### Setup Page (`/setup`)

Configuration hub with links to:
- **Calibrate** — guided lifter/rotary calibration wizard
- **Marcduino** — command reference
- **Parameters** — motor calibration parameter editor
- **WiFi** — network mode and credentials
- **Remote** — Droid Remote pairing
- **Firmware** — OTA upload

## Web API

| Endpoint | Method | Parameters | Description |
|----------|--------|------------|-------------|
| `/api/status` | GET | — | JSON: height, ticks, rotation, safety/fault/motor/limit states, override timer, RSSI |
| `/api/cmd` | GET | `c=<command>` | Execute a Marcduino command |
| `/api/lift` | GET | `t=<-1.0 to 1.0>` | Raw lift jog throttle |
| `/api/rotary` | GET | `s=<-1.0 to 1.0>` | Raw rotary spin speed |
| `/api/estop` | GET | — | Emergency stop all motors |
| `/api/action` | GET | `do=<action>` | Actions: `clearfault`, `enablemotors`, `homerotary`, `safety`, `sethome`, `setsafeheight`, `clearenc`, `override_on`, `override_off`, `clearprefs`, `reboot` |
| `/api/log` | GET | `since=<index>` | Streaming log lines |
| `/upload/firmware` | POST | multipart | OTA firmware upload |

## Serial Commands

### Lifter Commands

Commands start with `:P` followed by a subcommand. Multiple commands can be chained with colons:

    :PP100:W2:P0    (raise, wait 2s, lower)

| Command | Description |
|---------|-------------|
| `:PP<0-100>[,speed]` | Move lifter to position % (0=down, 100=up) |
| `:PPR[,speed]` | Random position |
| `:PH[speed]` | Home (rotate to 0°, lower to bottom) |
| `:PA<degrees>[,speed][,maxspeed]` | Rotate to absolute degrees |
| `:PAR[,speed][,maxspeed]` | Random absolute rotation |
| `:PD<degrees>[,speed][,maxspeed]` | Rotate relative degrees (+CCW, -CW) |
| `:PDR[,speed][,maxspeed]` | Random relative rotation |
| `:PR<speed>` | Continuous spin (+CCW, -CW). 0 = stop/home |
| `:PM[,liftSpd,rotSpd,minInt,maxInt]` | Random animation mode |
| `:PW[R]<seconds>` | Wait (R = randomize 1..N) |
| `:PL<0-7>` | Light kit mode |
| `:PS<0-100>` | Play stored sequence |

### Configuration Commands

| Command | Description |
|---------|-------------|
| `#PSC` | Run calibration (stores results to preferences) |
| `#PZERO` | Erase all stored sequences |
| `#PFACTORY` | Factory reset (clear all preferences and sequences) |
| `#PL` | List stored sequences |
| `#PD<n>` | Delete sequence n |
| `#PS<n>:<seq>` | Store sequence (e.g. `#PS1:H`) |
| `#PID<n>` | Set lifter ID (0–255) for multi-lifter systems |
| `#PBAUD<rate>` | Change baud rate (persistent, takes effect after reboot) |
| `#PR` | Toggle rotary enable/disable (reboots) |
| `#PNCL` / `#PNOL` | Set lifter limit switch normally closed / normally open |
| `#PNCR` / `#PNOR` | Set rotary limit switch normally closed / normally open |
| `#PCONFIG` | Display full configuration |
| `#PSTATUS` | Show WiFi/remote status |
| `#PWIFI[0\|1]` | Enable/disable WiFi (toggle if no arg) |
| `#PWIFIRESET` | Reset WiFi to defaults and reboot |
| `#PREMOTE[0\|1]` | Enable/disable Droid Remote (reboots) |
| `#PRNAME<host>` | Set Remote hostname (reboots) |
| `#PRSECRET<secret>` | Set Remote shared secret (reboots) |
| `#PRPAIR` | Start 60s pairing window |
| `#PRUNPAIR` | Unpair all remotes (reboots) |
| `#PDEBUG[0\|1]` | Enable/disable verbose debug output |
| `#PRESTART` | Reboot |

### Light Kit Modes

| Mode | Name | Description |
|------|------|-------------|
| 0 | Full Cycle | Random color, pattern, and speed |
| 1 | Off | All LEDs off |
| 2 | Obi Wan | Blue top LEDs, blue sides, random white main |
| 3 | Yoda | Green fade on/off |
| 4 | Sith | Red flash |
| 5 | Search Light | All white, center bright LED on |
| 6 | Dagobah | Screen-accurate: white main, white sides, red lower, blinking red rear |
| 7 | Sparkle | Random white flashing |

### Stored Sequence Examples

    #PS1:H                        (home)
    #PS2:P100                     (periscope up)
    #PS3:P100:L5:R30              (search light, spin CCW)
    #PS4:P100,100:L7:M,80,80,2,4 (random fast with sparkle)
    #PS5:P100:L7:M,50,40,5,5     (random slow with sparkle)
    #PS6:A0                       (face forward)
    #PS7:P100:L5:R-30             (search light, spin CW)
    #PS8:H:P50:W2:P85,35:A90,25:W2:A270,20,100:W2:P100,100:L5:R50:W4:H  (sneaky periscope)

### Multi-Lifter Addressing

Assign each lifter a unique ID with `#PID<n>`. Prefix commands with the ID to target a specific lifter:

    :P1S1    (lifter #1: play sequence 1)
    :PS1     (all lifters: play sequence 1)
    #P1CONFIG (show config for lifter #1)

## Safety Systems

### Safety Maneuver

Runs automatically on boot: lowers lifter to bottom limit, rotates to home (0°). Must complete before any motion commands are accepted. Prevents the periscope head from hitting the dome during rotation.

### Rotary Safe Height

Rotary commands are blocked unless the lifter is above 50% of full travel (configurable). This prevents the periscope head from colliding with the dome interior.

### Soft Stops (Pulsed Drive)

The lead-screw mechanism self-locks when power is removed but maintains momentum under continuous drive. Approaching limit switches, the firmware uses pulsed drive (3ms on / 1ms off) instead of continuous PWM to mechanically brake every 4ms, preventing violent limit switch impacts.

### Encoder Drift Correction

After a seek completes, the firmware holds the target position. Every 5 seconds it checks if the encoder has drifted more than 5% (configurable via `driftpct` preference) and re-seeks if needed. Gives up after 3 consecutive failures to prevent infinite retry loops.

### Distance Rejection

During calibration and safety maneuver, the measured lifter distance is validated against the stored calibrated value. Measurements under 50 ticks or differing more than 20% from stored are rejected to guard against EMI-corrupted encoder readings.

### Rescue Override

When the periscope is stuck (e.g., rotary not homed, preventing descent), the rescue page provides a 60-second safety override activated by a 3-second long-press. During override, lifter descent and rotary spin are allowed regardless of safety state. Auto-cancels after 60 seconds.

### Motor Fault Detection

The TB9051FTG DIAG pins are monitored. A LOW signal indicates a driver fault (overcurrent, overtemperature). Faults block motor commands and are reported in the web status.

## WiFi Configuration

### Access Point Mode (Default)

- **SSID:** `R2Uppity` (auto-suffixed with MAC on some builds)
- **Password:** `Astromech`
- **IP:** `192.168.4.1`

### Station Mode

Connect to an existing WiFi network. Configure via the `/wifi` web page or `#PWIFIRESET`.

### mDNS

When enabled, the board is reachable at `http://R2Uppity.local/` (or custom hostname).

### Droid Remote

Wireless controller via SMQ protocol (Reeltwo library). Pair with `#PRPAIR`, configure hostname with `#PRNAME`, secret with `#PRSECRET`.

## Assembling the PCB

### Part 1
[![Part1](https://img.youtube.com/vi/x4_3irdV4C8/hqdefault.jpg)](https://www.youtube.com/watch?v=x4_3irdV4C8)

### Part 2
[![Part2](https://img.youtube.com/vi/MdSRJsYx1T8/hqdefault.jpg)](https://www.youtube.com/watch?v=MdSRJsYx1T8)

## Troubleshooting

### EMI from Metal Dome

Placing a metal dome over the droid can cause electromagnetic interference on encoder signals, leading to undercounted ticks, phantom negative positions, and false limit triggers. Mitigations:

- **Hardware:** Twist encoder and limit switch wires tightly. Shield or route wires away from metal surfaces.
- **Software (built-in):** Position clamping prevents negative values. Distance rejection guards against corrupted calibration. Double encoder reset with settling delay at limits. Drift correction with failure limit.

### Periscope Won't Lower

If the rotary isn't homed, the safety system blocks descent. Use the Rescue page's safety override (3-second long-press) to bypass for 60 seconds.

### Calibration Distance Seems Wrong

If the measured distance differs >20% from the previously stored value, it's automatically rejected. Run calibration with the dome removed to get a clean measurement, then the stored value will be used even under dome EMI.
