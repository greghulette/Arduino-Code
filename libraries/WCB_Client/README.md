# WCB_Client

An Arduino library for ESP32 devices that lets them join a [Wireless Communication Board (WCB)](https://github.com/greghulette/Wireless_Communication_Board-WCB) ESP-NOW network as a first-class peer — sending commands, receiving commands, forwarding raw bytes to Pololu Maestro servo controllers, and participating in the ETM heartbeat system.

---

## What is a WCB?

WCB (Wireless Communication Board) is an ESP32-based wireless networking system designed for prop and animatronic control. Multiple WCB boards form a mesh over ESP-NOW, each with a unique ID. They exchange text commands (to trigger actions, run scripts, control servos) and raw serial data (to drive Pololu Maestro servo controllers wirelessly).

This library lets any ESP32 — a Kyber controller, a custom prop brain, a stage cue device — join that network without any hardware modification to the WCB boards.

---

## Requirements

- **Hardware:** Any ESP32 or ESP32-S3 board
- **Arduino IDE:** 2.x recommended
- **Board package:** `esp32 by Espressif Systems` v3.x
- **WCB firmware:** 6.0 or later on all WCB boards in the network

---

## Installation

1. Download or clone this repository
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library** and select the folder, or copy the folder into your Arduino `libraries/` directory
3. Restart Arduino IDE

---

## Dependencies

**WCB_Client itself has no external dependencies** — it uses only built-in ESP32 Arduino libraries (`WiFi.h`, `esp_now.h`).

**If you are using WCBStream for Maestro control**, you also need:

- **PololuMaestro** by Pololu — install via **Sketch → Include Library → Manage Libraries...**, search `PololuMaestro`

---

## Network Credentials

All devices on the same WCB network must share four values. Find them by querying any WCB over serial or using the WCB Config Tool:

| Parameter | WCB Command | Description |
|-----------|-------------|-------------|
| `mac_oct2` | `?WCBM` | MAC address octet 2 identifying the network |
| `mac_oct3` | `?WCBM` | MAC address octet 3 identifying the network |
| `password` | `?WCBP` | ESP-NOW network password |
| `wcb_quantity` | `?WCBQ` | Total number of WCB boards |

---

## Device ID Notes

- IDs **1 through `wcb_quantity`** are standard WCB slots. WCB boards pre-register these MACs so packets arrive correctly. Your device ID must be ≤ `wcb_quantity`.
- ID **20** is the special out-of-band slot — it doesn't consume a WCB number and works even if `wcb_quantity` is less than 20, but requires `?SPECIAL,ON` to be set on the WCB boards.

---

## Quick Start

### Send and receive text commands

```cpp
#include <WCB_Client.h>

const uint8_t MAC_OCT2     = 0x5C;
const uint8_t MAC_OCT3     = 0x57;
const char*   PASSWORD     = "MyNetworkPassword";
const uint8_t WCB_QUANTITY = 3;
const uint8_t DEVICE_ID    = 20;

void onCommandReceived(uint8_t sender, const char* cmd) {
    Serial.printf("From WCB%d: %s\n", sender, cmd);
}

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
              onCommandReceived);

void setup() {
    Serial.begin(115200);
    wcb.begin();
}

void loop() {
    wcb.update();              // must call every loop — drives heartbeats
    wcb.broadcast(":PP100");   // send to all WCBs at once
    wcb.send(2, ":LEDON");     // send to WCB2 only
    delay(5000);
}
```

### Forward Maestro servo commands wirelessly — unicast

Send to one specific WCB and serial port. The receiving WCB writes the bytes directly to the requested port — **no `?KYBER,REMOTE` configuration is required**. The `?KYBER,REMOTE` flag is only checked on the broadcast path; unicast bypasses it entirely.

```cpp
#include <WCB_Client.h>
#include <WCBStream.h>
#include <PololuMaestro.h>

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// WCB_Client must be declared before WCBStream
WCBStream maestroStream(2, 1);   // → WCB2, serial port 1
MiniMaestro maestro(maestroStream, Maestro::noResetPin, /*deviceID=*/1);

void setup() {
    Serial.begin(115200);
    wcb.begin();
}

void loop() {
    wcb.update();                    // also flushes WCBStream automatically
    maestro.setTarget(0, 6000);      // 1500µs — centre position
    delay(1000);
    maestro.setTarget(0, 4000);      // 1000µs — full left
    delay(1000);
}
```

### Forward Maestro servo commands wirelessly — broadcast

Send to every WCB that has a Maestro configured. Requires `?KYBER,REMOTE` on each receiving WCB.

```cpp
#include <WCB_Client.h>
#include <WCBStream.h>
#include <PololuMaestro.h>

WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID);

// broadcast — every WCB with ?KYBER,REMOTE configured will receive it
WCBStream maestroStream(broadcast);
MiniMaestro maestro(maestroStream, Maestro::noResetPin, /*deviceID=*/1);

void setup() {
    Serial.begin(115200);
    wcb.begin();
}

void loop() {
    wcb.update();
    maestro.setTarget(0, 6000);   // reaches all configured Maestros simultaneously
    delay(1000);
}
```

---

## API Reference

### WCB_Client

| Method | Description |
|--------|-------------|
| `WCB_Client(oct2, oct3, password, quantity, deviceID, [cmdCb], [statusCb])` | Constructor. `deviceID` must match your WCB system (1–`quantity` for a standard slot, or `20` for the out-of-band special slot). |
| `begin()` | Initialize WiFi and ESP-NOW, set custom MAC, register peers. Call once from `setup()`. |
| `update()` | Call every `loop()`. Drives heartbeats, offline detection, and WCBStream flushing. |
| `send(wcbID, command)` | Unicast a text command to one WCB. |
| `broadcast(command)` | Broadcast a text command to all WCBs simultaneously. |
| `sendRaw(wcbID, port, data, len)` | Unicast raw bytes to a specific WCB's serial port. |
| `sendKyber(data, len)` | Broadcast raw bytes to all WCBs — any with `?KYBER,REMOTE` will forward to their Maestro. |
| `monitorRaw(serial, target, port, [gap_ms])` | Watch a UART; flush buffered bytes via `sendRaw()` or `sendKyber()` (pass `broadcast` as target) when silent for `gap_ms`. |
| `monitorSerial(serial, target, [terminator])` | Watch a UART for newline-terminated text commands; forward each line via `send()` or `broadcast()`. |
| `isOnline(wcbID)` | Returns `true` if the specified WCB has sent a heartbeat recently. |
| `onCommand(callback)` | Register or replace the received-command callback. |
| `onStatusChange(callback)` | Register or replace the online/offline status callback. |
| `setChecksum(bool)` | Enable/disable CRC32 checksums. Must match `?ETM,CHKSM` setting on WCBs (default: on). |

### WCBStream

A `Stream` adapter that intercepts writes from the Pololu Maestro library and forwards them wirelessly instead of to a physical serial port.

**WCB_Client must be declared before WCBStream at global scope.** WCBStream self-registers with WCB_Client on construction so `wcb.update()` flushes all streams automatically — no extra calls needed in `loop()`.

```cpp
WCBStream stream(broadcast);         // broadcast to all WCBs with Kyber_Remote
WCBStream stream(wcbID, port);       // unicast to a specific WCB:port
WCBStream stream(wcbID, port, 5);    // unicast with 5ms inter-frame gap
```

Pass the `WCBStream` to the Pololu library in place of a serial port:

```cpp
MiniMaestro maestro(stream, Maestro::noResetPin, deviceNumber);
```

### Constants

| Constant | Value | Use |
|----------|-------|-----|
| `broadcast` | 0 | Pass as `target_wcb` to use Kyber broadcast path |
| `WCB_TARGET_BROADCAST` | 0 | Target ID for text command broadcasts |
| `WCB_TARGET_RAW_SERIAL` | 97 | Target ID for unicast raw serial forwarding |
| `WCB_TARGET_KYBER` | 98 | Target ID for Kyber broadcast raw forwarding |
| `WCB_SPECIAL_ID` | 20 | Out-of-band device slot (no WCB slot consumed) |

---

## Callbacks

Both callbacks are optional. If you only need to send commands and never receive them, you can omit both entirely.

### Command callback

Called whenever a text command arrives that is addressed to your device or broadcast to all devices.

```cpp
void onCommandReceived(uint8_t senderID, const char* command) {
    // senderID : which WCB sent the command (1–20)
    // command  : the command string, clean — checksum already stripped
}
```

**`senderID`** is the WCB number of the sender. Use this to respond differently depending on who sent the command, or to log which WCB triggered an action.

**`command`** is a null-terminated C string containing the exact command text. If checksums are enabled, the CRC32 suffix has already been verified and removed before your callback is called — you always receive the clean command.

```cpp
void onCommandReceived(uint8_t senderID, const char* command) {
    Serial.printf("From WCB%d: %s\n", senderID, command);

    if (strcmp(command, ":TRIGGER") == 0) {
        fireEffect();
    } else if (strncmp(command, ":VOL", 4) == 0) {
        int vol = atoi(command + 4);   // e.g. ":VOL75" → 75
        setVolume(vol);
    }
}
```

### Status callback

Called whenever a WCB transitions between online and offline. A WCB is considered offline after approximately 30 seconds of missed heartbeats (3 × the 10-second heartbeat interval, matching WCB firmware defaults).

```cpp
void onStatusChanged(uint8_t wcbID, bool online) {
    // wcbID  : which WCB changed state (1–WCB_QUANTITY)
    // online : true = just came online, false = just went offline
}
```

**`wcbID`** is the WCB number that changed state.

**`online`** is `true` when the WCB just sent its first heartbeat after being offline (or on startup), and `false` when heartbeats have stopped arriving for ~30 seconds.

```cpp
void onStatusChanged(uint8_t wcbID, bool online) {
    if (online) {
        Serial.printf("WCB%d is online\n", wcbID);
        // Re-send initial state, sync positions, etc.
    } else {
        Serial.printf("WCB%d went offline\n", wcbID);
        // Stop sending commands to this WCB, trigger a warning, etc.
    }
}
```

### Registering callbacks

**In the constructor** — both are optional positional arguments. Pass `nullptr` to skip one:

```cpp
// Command callback only
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
              onCommandReceived);

// Status callback only
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
              nullptr, onStatusChanged);

// Both
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
              onCommandReceived, onStatusChanged);
```

**After construction** — use these methods any time, including after `begin()`:

```cpp
wcb.onCommand(onCommandReceived);
wcb.onStatusChange(onStatusChanged);
```

This is useful when your callback logic depends on objects that are not yet initialised at global-scope construction time.

### Lambda callbacks

Both callbacks also accept lambdas for compact inline definitions:

```cpp
WCB_Client wcb(MAC_OCT2, MAC_OCT3, PASSWORD, WCB_QUANTITY, DEVICE_ID,
    [](uint8_t sender, const char* cmd) {
        Serial.printf("From WCB%d: %s\n", sender, cmd);
    },
    [](uint8_t id, bool online) {
        Serial.printf("WCB%d %s\n", id, online ? "online" : "offline");
    }
);
```

---

## WCBStream Modes

### Broadcast mode — recommended for servo passthrough

```cpp
WCBStream stream(broadcast);
```

Sends raw bytes to all WCBs simultaneously using a single ESP-NOW packet (`sendKyber()`). Each WCB that has `?KYBER,REMOTE` configured on a serial port will forward the bytes to that port. WCBs without Kyber Remote configured ignore the packet.

**Broadcast is the preferred approach for continuous servo control.** Because only one packet is sent regardless of how many Maestros are in the system, the network stays uncongested and latency stays low even at high update rates. Use this when:

- You need continuous position updates or high-rate servo streaming
- You have Maestros on multiple WCBs and want to address all of them at once
- You don't know (or don't care) which specific WCB has the Maestro
- You want one command to ripple out to every servo controller simultaneously

Requires `?KYBER,REMOTE,S<port>` configured on each WCB that has a Maestro wired to it.

### Unicast mode

```cpp
WCBStream stream(2, 1);   // → WCB2, serial port 1
```

Sends raw bytes directly to one specific WCB's serial port using `sendRaw()`. The receiving WCB writes the bytes straight to the requested port — **`?KYBER,REMOTE` is not required**. The Kyber Remote flag is only checked on the broadcast path (target 98); the unicast path (target 97) ignores it entirely and writes unconditionally to whatever port number is in the packet.

Unicast is perfectly acceptable for low-frequency operations like triggering a `RestartScript` on a known target, but for continuous servo streaming broadcast will give noticeably better performance. Use unicast when:

- You need to target one specific WCB:port and broadcast is not an option
- You are sending infrequent commands (script triggers, one-shot moves) rather than a continuous stream

---

## Maestro Device Number

The third argument to `MiniMaestro` corresponds to the **Device Number** in Maestro Control Center → **Serial Settings**. It enables the Pololu protocol so each packet includes the `0xAA` start byte and device address.

```cpp
// Compact protocol — one Maestro, no device addressing
MiniMaestro maestro(stream, Maestro::noResetPin, Maestro::deviceNumberDefault);

// Pololu protocol — device number 1
MiniMaestro maestro(stream, Maestro::noResetPin, 1);
```

Quarter-microsecond values for common servo positions:

| µs | Quarter-µs | Position |
|----|-----------|----------|
| 1000 | 4000 | Full left / min |
| 1500 | 6000 | Centre |
| 2000 | 8000 | Full right / max |

---

## Checksum Setting

The library appends a CRC32 checksum to every outgoing command and verifies checksums on incoming commands. This must match the `?ETM,CHKSM` setting on all WCBs. The default on both the library and WCB firmware is **ON**.

```cpp
wcb.setChecksum(true);    // Default — matches WCB factory default
wcb.setChecksum(false);   // Only if ?ETM,CHKSM,OFF on all WCBs
```

---

## Examples

| Example | Description |
|---------|-------------|
| `BasicUsage` | Send and receive text commands |
| `MaestroUnicast` | Forward Maestro commands to a specific WCB:port (unicast) |
| `MaestroBroadcast` | Broadcast Maestro commands to all WCBs with Maestros configured |
| `CombinedUsage` | Text commands and Maestro forwarding running simultaneously |

---

## License

MIT — see [LICENSE](LICENSE)
