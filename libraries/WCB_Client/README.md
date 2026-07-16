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

## Limitations & Coexistence (WiFi / other ESP-NOW)

This library takes over the ESP32's radio to join the WCB ESP-NOW network. Read
this before combining it with WiFi or another ESP-NOW stack.

### It takes over WiFi (STA) — but a SoftAP can coexist

`begin()` puts WiFi into station mode and calls `WiFi.disconnect()`. **It will
drop any existing STA connection to a WiFi router**, and ESP-NOW timing assumes
no STA association. You generally **cannot associate this ESP32 with an
external WiFi router** while using this library — connecting to someone else's
AP hands channel control to that AP, and ESP-NOW only works if every WCB
happens to already be on that channel too (see below).

**Hosting your own SoftAP is a different story and is supported.** If you call
`WiFi.softAP(ssid, password, channel)` *before* `begin()`, the library detects
it and switches to `WIFI_AP_STA` instead of `WIFI_STA` — your AP (and any web
server built on it) stays up, and ESP-NOW simply rides the AP's radio channel
(AP and STA share a single channel on the ESP32, so there's no separate
negotiation). Calling `softAP()` *after* `begin()` also works, since
`WiFi.mode()` ORs in `WIFI_AP` without dropping STA. Either way, all WCBs and
clients still need to end up on the AP's channel — pick it deliberately rather
than leaving it to default to channel 1.

If you need to *associate* with an external router (not host your own AP)
alongside WCB traffic, that's still unsupported without keeping everything on
a **single fixed WiFi channel** (see below) and accepting the MAC override —
in practice it's far simpler to dedicate this ESP32 to the WCB network and use
a second board for that WiFi workload.

### Shared radio channel

ESP-NOW operates on whatever channel the WiFi radio is currently on. **Every WCB
and every client must be on the same channel.** If something in your sketch
associates with an access point, the channel is locked to that AP's channel and
ESP-NOW only works if all peers happen to be on it too. Mismatched channels =
packets silently dropped, no error.

### The WiFi MAC is overwritten

`begin()` calls `esp_wifi_set_mac()` to force the station MAC to the WCB scheme
`02:oct2:oct3:00:00:<deviceID>` so the WCBs recognize this device as a peer.
Consequences:

- Anything that depends on the factory/default MAC (licensing, MAC-based
  device identity, router MAC filtering, DHCP reservations) **will not see the
  real MAC**.
- **Two devices with the same `deviceID` + `oct2`/`oct3` get an identical MAC.**
  Never reuse a device ID on the same network — use the special slot (`20`) for
  an extra out-of-band device.

### Only one ESP-NOW stack, one instance

- ESP-NOW is global ESP-IDF state. `begin()` calls `esp_now_init()` and
  registers **the** single receive callback via `esp_now_register_recv_cb()`.
  **You cannot run a second, independent ESP-NOW network/library alongside this
  one** — whichever registers its receive callback last wins, and the other
  stack stops receiving.
- **Only one `WCB_Client` instance is supported per sketch** (it's a singleton).
  Declare it once at global scope.

### WiFi power save

If you do bring up WiFi for any reason, modem sleep can cause this device to
miss ESP-NOW packets. Disable it with `esp_wifi_set_ps(WIFI_PS_NONE)` (or
`WiFi.setSleep(false)`) after `begin()`.

### Quick reference

| If you need… | Result with this library |
|---|---|
| WCB ESP-NOW only | ✅ Intended use — works out of the box |
| SoftAP (hosted web UI) + WCB on the same board | ✅ Supported — `begin()` preserves an existing/later SoftAP as `WIFI_AP_STA`. The AP owns the single radio's channel, so bring it up on the mesh channel (`WiFi.softAP(ssid, pass, 1)`); the library **warns** (never force-moves it) if the radio ends up elsewhere. Match a non-default fleet channel with `setMeshChannel()`. |
| WiFi STA (associating with an external router) + WCB on the same board | ⚠️ Only if all peers share the router's fixed channel; `begin()` still disconnects STA |
| A second, separate ESP-NOW network | ❌ Not supported — single global receive callback |
| Multiple `WCB_Client` objects | ❌ Not supported — singleton |
| Factory MAC preserved | ❌ MAC is overwritten by design |

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
| `send(wcbID, command, [ensured=true])` | Unicast a text command to one WCB. Ensured by default (retransmits until ACK'd, like WCB-to-WCB ETM); pass `false` for fire-and-forget. |
| `broadcast(command, [ensured=true])` | Broadcast a text command to all WCBs simultaneously. Ensured by default (retransmits per-board until every online board ACKs); pass `false` for fire-and-forget telemetry. |
| `sendRaw(wcbID, port, data, len)` | Unicast raw bytes to a specific WCB's serial port. |
| `sendKyber(data, len)` | Broadcast raw bytes to all WCBs — any with `?KYBER,REMOTE` will forward to their Maestro. |
| `monitorRaw(serial, target, port, [gap_ms])` | Watch a UART; flush buffered bytes via `sendRaw()` or `sendKyber()` (pass `broadcast` as target) when silent for `gap_ms`. |
| `monitorSerial(serial, target, [terminator])` | Watch a UART for newline-terminated text commands; forward each line via `send()` or `broadcast()`. |
| `isOnline(wcbID)` | Returns `true` if the specified WCB has sent a heartbeat recently. |
| `onCommand(callback)` | Register or replace the received-command callback. |
| `onStatusChange(callback)` | Register or replace the online/offline status callback. |
| `setChecksum(bool)` | Enable/disable CRC32 checksums. Must match `?ETM,CHKSM` setting on WCBs (default: on). |
| `setMeshChannel(uint8_t)` | Set the ESP-NOW mesh channel (1–11) this device expects. Best called before `begin()` if your fleet runs on a non-default channel (set on the WCBs via `?WCBCH` / the Wizard); calling it after `begin()` also re-pins the radio live when no SoftAP is active. One radio = one channel (default: 1). |

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

### Device Identity (WDP)

```cpp
void setIdentity(const char* type, const char* fw,
                 const char* hwRev = nullptr, const char* caps = nullptr);
```

Advertises this device on the WCB mesh so every board discovers it automatically
— it shows up in `?WDP,LIST` and the config tool by name and firmware version,
with no manual port labeling. This is the mesh counterpart of the serial
**WDP-DA** device-announce protocol: a device describes itself the same way
whether it's wired to a WCB serial port or joined over ESP-NOW.

- `type` — canonical device name (e.g. `"NaviCore"`, `"Flthy HP Controller"`);
  use a name from the shared WCB device vocabulary. Required — a null/empty
  `type` leaves advertising off.
- `fw` — this device's firmware version string.
- `hwRev` — optional hardware revision (`"revB"`); omit with `nullptr`.
- `caps` — optional space-separated capability tags (`"hp.servo hp.led"`);
  omit with `nullptr`.

Call it from `setup()` (before or after `begin()`); the advert goes out as a
short boot burst and then periodically (~60 s, staggered) from `update()`.
Requires ETM active on the WCBs (the factory default).

```cpp
wcb.setIdentity("NaviCore", "v0.2.0");
```

### Discovering neighbors (WDP consumer)

The same WDP adverts that `setIdentity()` broadcasts are also *decoded* by this
library, so your device can learn every other board on the mesh — each WCB and
each `setIdentity()`-enabled client — without any manual configuration.

```cpp
void onNeighbor(WCBNeighborCallback callback);   // fires when an advert is learned or expires
const WCBNeighbor* getNeighbor(uint8_t wcbNumber);// latest record for a board, or nullptr
uint8_t neighborCount();                          // how many neighbors are currently known
```

A neighbor is described by a `WCBNeighbor`:

```cpp
struct WCBNeighbor {
    bool          valid;          // false when the slot has aged out
    bool          isClient;       // true = a WCB_Client device, false = a WCB
    uint8_t       wcbNumber;      // 1..20
    char          name[25];       // WCB alias, or the client's device type
    char          fw[28];         // firmware version string
    uint8_t       hwVer;          // WCB numeric hardware version (0 for clients)
    char          hwRev[16];      // client hardware revision ("" for WCBs)
    uint16_t      capFlags;       // WCB capability bitmap — test with WCB_CAP_*
    char          capTags[49];    // client capability tags, space-separated
    uint8_t       ctrlId;         // controller (special-peer) id this board links to; 0 = none
    uint8_t       maestroIds[9];  // this board's local Maestro IDs
    uint8_t       maestroCount;
    char          portLabels[5][25]; // advertised serial-port labels ("" = unlabeled)
    unsigned long lastSeenMs;     // millis() of the last advert heard
};
```

`onNeighbor` fires on the ESP-NOW (WiFi) task — keep it minimal and poll
`getNeighbor()` from `loop()` for heavier work. Neighbors age out after
`WCB_WDP_NEIGHBOR_TTL_MS` (180 s, ~6 missed cycles) of silence; the callback
fires once more with `valid == false` on expiry so you can drop stale entries.

```cpp
wcb.onNeighbor([](const WCBNeighbor& nb) {
    if (!nb.valid) { Serial.printf("WCB%u went quiet\n", nb.wcbNumber); return; }
    Serial.printf("WCB%u = %s (%s)%s\n", nb.wcbNumber, nb.name, nb.fw,
                  nb.capFlags & WCB_CAP_MAESTRO_HOST ? "  [has Maestro]" : "");
});
```

#### Auto-join

By default the same adverts also drive **auto-join**: when this device hears a
regular WCB it isn't already peered with (after two adverts), it registers that
board as an ESP-NOW peer live. So you can leave `wcb_quantity` at whatever covers
the boards you address directly and still reach boards discovered later — without
pre-registering all 20 slots (the ESP-NOW peer table is small). Client devices
and the special peer are never auto-joined.

A learned peer is **permanent**: it is saved to NVS, restored on every `begin()`,
and from then on always expected to be on and ready. Heartbeats drive its
online/offline state, but membership never expires on its own — a board that's
powered down stays a member and is simply reported offline until it returns.
Removing one is deliberately a user action:

```cpp
void setAutoJoin(bool enabled);   // default true
bool autoJoinEnabled() const;
void forgetPeer(uint8_t id);      // drop one learned peer (deregisters + clears NVS)
void clearLearnedPeers();         // drop all learned peers
```

Turn auto-join off to pin the peer set to exactly `1..wcb_quantity` (+ special peer):

```cpp
wcb.setAutoJoin(false);
```

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
| `SpecialPeer` | Talk to the out-of-band controller (e.g. NaviCore) at id 20 |
| `NeighborDiscovery` | Learn the mesh over WDP — who's out there and what they can do |
| `MgmtRelay` | Turn any ESP32 into a USB-serial ↔ ESP-NOW management relay — drive the Config Tool (Via WCB) and manage every WCB + the NaviCore over one USB port |
| `AllFeatures` | Interactive "kitchen sink" — every public method in one sketch |

---

## Changelog

### 1.9.7

- **Configurable ESP-NOW mesh channel.** The mesh channel (1–11) is now an explicit,
  operator-selectable setting instead of an accident of "an unassociated STA lands on
  channel 1." On the WCBs it's set via `?WCBCH,<n>` or the Wizard's **Advanced → Mesh
  Channel** field, persisted in NVS, and applied on the next reboot. On this library the default is
  `1`; call `setMeshChannel(ch)` before `begin()` to match a fleet running elsewhere.
- **Channel awareness in `begin()` / `update()`.** With no hosted SoftAP, `begin()`
  now pins the radio to the mesh channel deterministically. With a SoftAP active (which
  owns the single radio's channel), the library instead **warns** — rate-limited and
  re-checked on every heartbeat — when the radio isn't on the mesh channel, rather than
  silently failing to reach the mesh. It never force-moves a deliberately chosen SoftAP
  channel.

### 1.9.6

- **`device_id` may now sit above `wcb_quantity`.** `begin()` no longer hard-fails
  when `device_id > wcb_quantity` (for ids 1–19) — it logs a non-fatal warning and
  joins the mesh anyway. `wcb_quantity` means "how many WCBs I pre-register," not a
  cap on this device's own id. A device above the floor is reachable **inbound**
  once the other boards auto-join it from its WDP advert, so call `setIdentity()`
  and keep auto-join on (the default). The normal case (`device_id <= wcb_quantity`)
  is byte-for-byte unchanged. This also un-breaks the `AllFeatures` example, whose
  `device_id 19 / quantity 4` combo previously failed `begin()`.
- **Reply to any authenticated sender.** A board can now unicast an ACK/reply to
  an above-floor client that commands it *before* auto-join has registered it: the
  sender is flagged on the ESP-NOW receive callback and added as a **transient**
  ESP-NOW peer on the loop task (no NVS write, no ≥2-advert wait), matching the
  proven auto-join discipline. So e.g. the NaviCore answers a `MgmtRelay` at id 19
  on the config tool's very next ping instead of never — which is what makes the
  above-floor `device_id` genuinely usable for a bridge/relay/monitor.
- **New example: `MgmtRelay`** — turn any ESP32 into a USB-serial ↔ ESP-NOW
  management relay so a host tool (e.g. the NaviCore Config Tool in "Via WCB" mode)
  can drive every WCB *and* the NaviCore over one USB port.

### 1.9.5

- **`forgetPeer()` now also clears the peer's WDP neighbor record**, so a forgotten
  peer drops off `getNeighbor()` immediately instead of lingering until its advert
  ages out (up to the neighbor TTL).

### 1.9.4

- **`isLearnedPeer(uint8_t id)` accessor.** Query whether an id is currently an
  auto-joined (learned) peer — a node above the `wcb_quantity` floor that was heard
  over WDP and made a permanent peer. Membership persists across reboots and is
  independent of online/offline state, so a consumer can keep a learned peer listed
  (e.g. on a status panel) even while it's powered off — its WDP advert ages out and,
  for a client device, it never heartbeats.
- **New example: `AllFeatures`** — an interactive, serial-menu "kitchen sink" sketch
  exercising every public method in one place.

### 1.9.3

- **Auto-join now learns CLIENT devices too** (mesh monitors, other controllers,
  command-accepting clients), not just regular WCBs — anything heard on the mesh
  becomes a persistent, NVS-remembered ESP-NOW peer you can `send()` to.
- **Ensured `send()` to a never-heartbeating learned client no longer silently
  degrades to fire-and-forget.** A learned client that only advertises (WDP) but
  never sends heartbeats is now treated as still-outstanding and retried up to
  `ETM_MAX_RETRIES`, while a board that *was* online and then dropped still gives up.
- **Cross-core safety on the pending table.** `_pending[]` is mutated from both the
  loop task (`_sendPacket` claim, `update()` retry service) and the WiFi/RX task
  (ACK handler); all three now run under a `portMUX` critical section (with the
  blocking `_transmit()` done on snapshots *after* releasing the lock), closing a
  torn-slot / dropped-ensured-command race.
- **`_findFreePending()` no longer evicts an outstanding ensured delivery.** When the
  table is saturated it returns "no slot" and the send reports non-guaranteed
  delivery (best-effort once) instead of silently dropping an in-flight guaranteed
  command.
- **CRC strip/verify is gated on `_checksumEnabled` and anchored to the tail.** A
  legitimate payload that merely *contains* the literal `|CRC` (e.g. a command-library
  command) is no longer truncated/rejected; a real checksum must be `|CRC` + exactly
  8 hex digits at the end.
- **Special-peer slot hardening.** A client advertising at `WCB_SPECIAL_ID` (20) can
  no longer be learned/persisted and then have its ESP-NOW MAC deleted by
  `clearLearnedPeers()`/`forgetPeer()` (which would silently break
  `sendToSpecialPeer()`); `forgetPeer()` also resets the advert counter so a
  forgotten peer must be heard twice again before it can re-join.
- **`WCBStream` no longer truncates a large burst mid-frame.** Added
  `flushNow()`/`bytesFree()`, and `write()` now returns 0 when the buffer is full, so
  a producer (e.g. a Maestro burst programming many channels) splits on frame
  boundaries into multiple packets instead of forwarding a corrupt byte-stream.

### 1.9.2

- **Boot-loop crash fix (`begin()` failure was unhandled).** If `begin()` returned
  `false` — most commonly because `esp_now_init()` failed (WiFi driver not up) —
  the sketch would keep calling `update()`, whose first heartbeat called
  `esp_now_send()` on an **uninitialised ESP-NOW driver**, dereferencing a NULL
  driver global and panicking (`Guru Meditation LoadProhibited`, `EXCVADDR 0x4c`).
  On an ESP32-S3 this presented as a hard boot loop straight out of the ROM
  bootloader. Now:
  - `update()` and every send method (`send`/`broadcast`/`sendRaw`/`sendKyber`/
    `sendToSpecialPeer`/`sendRawPacket`/`_sendHeartbeat`) **no-op when ESP-NOW is
    not initialised** (`_started == false`) instead of crashing.
  - `begin()` prints the **exact `esp_now_init()` error name/code** and warns if
    `WiFi.mode(WIFI_STA)` reports failure, so a failed init is obvious in the boot
    log rather than a bare panic.
  - The **NeighborDiscovery** example now checks `begin()`'s return value and halts
    with a clear message on failure (all examples should — ignoring it is what let
    the crash happen).

  *Note:* the 1.9.1 deferred-join change below is a genuine robustness fix but was
  **not** the cause of the S3 boot loop — this is.

### 1.9.1

- **Robustness (auto-join off the WiFi task).** The auto-join added in 1.9.0 called
  `esp_now_add_peer()` and wrote NVS from inside the ESP-NOW receive callback
  (WiFi task), whose small stack a flash write can overflow. The receive path now
  only *flags* a join; the actual `esp_now_add_peer` + NVS write happen in
  `update()` on the loop task, matching how the WCB firmware defers all WDP flash
  work. (This hardens the callback path; the boot loop some users saw was the
  `begin()`-failure crash fixed in 1.9.2.)

### 1.9.0

- **Auto-join (on by default)** — the device now registers a regular WCB it hears
  over WDP as an ESP-NOW peer *live*, once it has seen the board advertise at
  least twice. This means it discovers the fleet on its own: you no longer have
  to set `wcb_quantity` high enough to cover every board, and it doesn't burn
  peer-table slots (the ESP-NOW cap is ~20) on boards that may not exist. Client
  devices (other `setIdentity()` peers) and the special peer are never
  auto-joined. Learned peers are **permanent**: persisted to NVS, restored on
  every `begin()`, always expected on the mesh (offline detection covers them
  too) — membership never expires on its own. Manage it with `setAutoJoin(bool)`
  / `autoJoinEnabled()` / `forgetPeer(id)` / `clearLearnedPeers()`.
- **Sender-MAC binding** — inbound packets are now dropped when the claimed
  `structSenderID` disagrees with the source MAC's last octet (every board forces
  its derived MAC `02:oct2:oct3:00:00:id`). Closes id-spoofing — a rogue in-group
  node can no longer claim another board's id to satisfy an ACK or trigger
  auto-join. Mirrors the same hardening added to the WCB firmware.

### 1.8.0

- **WDP consumer** — the library now decodes the WDP adverts it already
  broadcasts, so a client can discover the whole mesh. `onNeighbor()` fires as
  boards are learned or age out, `getNeighbor(n)` returns the latest record for a
  board, and `neighborCount()` reports how many are currently known. Each
  `WCBNeighbor` carries alias/device-type, firmware, hardware version/revision,
  capability flags (`WCB_CAP_*`) and tags, the controller link id, local Maestro
  IDs and serial-port labels. Neighbors expire after 180 s of silence. Consuming
  is passive — no extra traffic, and it works whether or not you called
  `setIdentity()`.

### 1.7.0

- **Device identity over WDP** — `setIdentity(type, fw, hwRev, caps)` broadcasts
  this device's identity on the mesh via the Wireless Discovery Protocol, so
  every WCB discovers it by name and firmware version automatically (it appears
  in `?WDP,LIST` and the config tool as a named device, not a bare ID). This is
  the mesh twin of the serial `WDP-DA` device-announce model. Advertising is a
  short boot burst plus a staggered ~60 s backstop driven from `update()`, and
  stays off until you call `setIdentity()`. Requires ETM active on the WCBs
  (the factory default).

### 1.3.1

Hardening release for the 1.3.0 fragmentation feature — fixes four review
findings before wide adoption.

- **Fragmented sends are now NON-BLOCKING.** 1.3.0 ran `delay(10)` loops inside
  `send()` (up to ~0.5 s), freezing animation loops and — if called from the
  command callback — stalling the WiFi task. Chunks are now queued and
  transmitted from `update()` (~10 ms pacing); `send()` returns immediately
  (true = queued) and the result is logged on completion. One fragmented send
  in flight at a time; a second call while busy returns false.
- **Unicast stays unicast.** Fragmented commands are now sent with a dedicated
  packet type; on firmware 6.1.1+ the reassembled command keeps target-local
  semantics — previously, broadcastable tokens in a fragmented unicast would
  re-broadcast to every board on the network once the command crossed the
  length threshold. (Older firmware still executes the command; it just keeps
  the old broadcast behavior.)
- **Oversize guard fixed.** The chunk count was bounds-checked after a uint8
  cast, so absurdly long commands (~45 KB+) could wrap the counter and bypass
  the rejection — silently truncating. The length is now checked first.
- **Serial monitor lines are clamped, not dropped.** A monitored text line
  longer than the single-packet limit (187 chars with checksum) is truncated
  and delivered; 1.3.0 either dropped it entirely (broadcast target) or pushed
  it through the fragmentation path inside `update()` (unicast target).
- Success is now judged **per chunk across passes** (a chunk accepted in any
  pass counts), and the single-packet limit is derived from the packet struct
  instead of magic numbers.

### 1.3.0

**Automatic fragmentation for long unicast commands** — `send()` now transparently
handles commands longer than a single ESP-NOW packet (199 chars, or 187 with
checksum enabled). Previously they were **silently truncated**: a long
`?SEQ,SAVE,...` arrived cut short and a corrupt sequence got stored on the board
(or, with checksum on, was rejected entirely after burning the retries).

- **`send()` auto-fragments** oversized commands using the WCB firmware's MGMT
  fragmentation protocol (the same one the web config tool uses): the command is
  split into 179-char chunks, the target board reassembles them and executes the
  whole command through its normal parser. Works up to ~2.8 KB — comfortably
  covering the firmware's 1800-char stored-sequence limit.
- **Reliability:** the MGMT layer has no per-chunk ACK, so the full chunk set is
  transmitted in multiple passes (3 when `ensured`, 2 otherwise). Duplicates are
  harmless — the board stores each chunk once and discards retransmits of
  completed sessions. A fragmented send blocks ~10 ms per chunk per pass
  (a maximal ensured send ≈ 0.5 s).
- **`broadcast()` does NOT fragment** — fragmentation is unicast-only (the
  reassembly session on the board is per-target). An oversized broadcast now
  **fails loudly** (error log + `return false`) instead of silently truncating;
  `send()` the long command to each board individually.
- No API changes — existing sketches gain this automatically.

> Requires WCB firmware with the MGMT fragmentation handler (present in all
> 6.x firmware this library targets).

### 1.2.0

Adds **ensured delivery**, and makes it the default for text commands so the
library is **reliable by default** — matching the WCB firmware, which transmits
with ETM ON by default. (Earlier 1.x releases never retransmitted, so
client→WCB text was always best-effort regardless of the WCBs' ETM setting.)

- **`send()` and `broadcast()` now default to `ensured=true`.** A lost packet or
  ACK is retransmitted instead of silently missed, just like WCB-to-WCB ETM.
  Mirrors the firmware's retry engine: after the initial send, the packet is
  retried as a **per-board unicast** (reusing the original sequence number, so
  receivers dedup it) to each expected board that hasn't ACK'd — up to
  `ETM_MAX_RETRIES` per board. For a unicast the expected recipient is the
  target; for a broadcast it's every board that was online at send time. A board
  that drops offline mid-flight is dropped from the expected set rather than
  retried forever.
- **Opt out with `ensured=false`** for high-rate telemetry / status spam where
  the next update supersedes a lost one — it avoids spending a pending slot and
  airtime per send. (For binary streaming use the raw helpers `sendRaw()` /
  `sendKyber()` / `WCBStream`, which are best-effort and unaffected by this
  default — exactly as the firmware keeps PWM/servo streaming out of ETM.)
- Tuning via `ETM_RETRY_INTERVAL_MS` (default 500 ms) and `ETM_MAX_RETRIES`
  (default 3) — matched to the firmware's `etmTimeoutMs` and retry count.
- The in-flight tracking table grew from 3 to **10 slots** (`WCB_PENDING_MAX`),
  matching the firmware's `ETM_PENDING_MAX`. When full it now evicts the oldest
  entry rather than dropping the new send, so ensured commands always get tracked.

**Upgrade note:** if you have a sketch that calls `broadcast()` or `send()` at a
high rate for telemetry, add `, false` to those calls to keep them best-effort.
Discrete commands need no change — they just became reliable.

### 1.1.0

Reliability fixes for the ETM acknowledgement layer. No API changes — existing
sketches compile and run unchanged.

- **Fixed: duplicate sequence numbers under concurrent sends.** The per-command
  sequence counter is now atomic. It is incremented from both the main loop
  (`send()` / `broadcast()`) and the ESP-NOW receive callback (when a command
  callback replies with `send()`). A non-atomic increment could hand the same
  sequence number to two packets, causing the receiving WCB's duplicate
  detection to silently drop one of them.
- **Fixed: heavy broadcasting could starve unicast ACK tracking.** Broadcasts
  are fire-and-forget and are no longer recorded in the small in-flight ACK
  table (`WCB_PENDING_MAX` = 3 slots). Previously, broadcasting filled all
  tracking slots within seconds and left no room to match unicast ACKs.
- **Improved: stale ACK-pending slots are reclaimed automatically.** A unicast
  whose ACK never arrives no longer holds its slot indefinitely; slots older
  than 1 second are freed for reuse. (Reclaim only — at 1.1.0 the library does
  not retransmit at all; a lost packet is simply missed. Application-layer ETM
  retransmission arrives in 1.2.0.)
- **Fixed: pending-table command copy is now always null-terminated.**

### 1.0.0

Initial release.

- Join a WCB ESP-NOW network as a first-class peer
- Unicast and broadcast text commands (`send()` / `broadcast()`)
- Raw byte forwarding to Pololu Maestro controllers via `WCBStream`
  (unicast and Kyber broadcast)
- UART monitors (`monitorRaw()` / `monitorSerial()`) for transparent passthrough
- ETM heartbeat tracking with online/offline status callbacks
- Optional CRC32 checksums matching WCB firmware

---

## License

MIT — see [LICENSE](LICENSE)
