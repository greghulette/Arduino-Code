# WcbCmd — shared WCB command translators

One command vocabulary for the whole **WCB** (Wireless Communication Board) droid
ecosystem. `WcbCmd` turns a WCB `;`-command verb into a device's **native serial wire
protocol**, on an injected `Stream` — nothing more. It is compiled by **both** the WCB
firmware and a WCB_Client host (e.g. [NaviCore](https://github.com/greghulette/NaviCore)),
so the **same command string produces the same device bytes** whether it is reached
across the mesh over ESP-NOW (a WCB parses it) or out a host-local serial port (the host
parses it).

You can also use it **standalone** — with no WCB anywhere — as a compact way to drive a
Pololu Maestro, SparkFun MP3 Trigger, DFPlayer Mini, WLED node, or Human-Cyborg-Relations
(HCR) vocalizer from short text commands over a serial port. See [Quick start](#quick-start).

> Related projects: [WCB firmware](https://github.com/greghulette/Wireless_Communication_Board-WCB) ·
> [WCB_Client](https://github.com/greghulette/WCBClient) ·
> [NaviCore](https://github.com/greghulette/NaviCore)

## Installation

- **Arduino IDE Library Manager** (recommended): *Tools → Manage Libraries…*, search
  **WcbCmd**, click Install.
- **arduino-cli:** `arduino-cli lib install WcbCmd`
- **Manual .ZIP:** download a release from the
  [Releases page](https://github.com/greghulette/WcbCmd/releases), then in the IDE
  *Sketch → Include Library → Add .ZIP Library…*
- **From Git:** clone into your `Arduino/libraries/` folder.

Targets **ESP32** (`architectures=esp32`) — install the ESP32 board package first. No other
library dependencies.

## Quick start

Each device is wired to its own UART; you open that port at the device's baud, then hand the
`Stream` to `WcbCmd`. Include the umbrella header `WcbCmd.h` (or a single module header).

```cpp
#include <WcbCmd.h>

// Three modules are stateful objects; create them once, globally:
Mp3Codec      mp3;   // SparkFun MP3 Trigger
DfPlayerCodec dfp;   // DFPlayer Mini
HcrCodec      hcr;   // Human-Cyborg-Relations vocalizer

void setup() {
  Serial1.begin(9600);     // MP3 Trigger  (9600 or 38400)
  Serial2.begin(9600);     // Pololu Maestro (use the baud set in the Maestro Control Center)
  // (WLED wants 115200; HCR wants 9600; a DFPlayer is 9600 ONLY — wire each to its own UART)

  mp3.begin(Serial1);      // bind the MP3 Trigger's port
  // mp3.onFinished = [](const char* key){ /* track ended */ };   // optional callback

  // Maestro — trigger script subroutine 1 on device id 1 (";M11"):
  WcbMaestro::emit(Serial2, "M11");

  // MP3 — set volume then play track 5 (";A,PLAY,5"):
  mp3.handle("PLAY,5");

  // DFPlayer — play /01/002.mp3 (";D,FOLDER,1,2"). Volume is NOT re-sent per play,
  // and its scale is 0=silent..30=loudest — the inverse of the MP3 Trigger's.
  dfp.begin(Serial1);      // (its own 9600 UART in a real build)
  dfp.handle("FOLDER,1,2");

  // WLED — recall preset 2 (";L,PS,2"):
  WcbWled::emit(Serial1, "PS,2");     // (use the WLED node's own 115200 UART)

  // HCR — Stimulate the HAPPY emotion at intensity 50 (";H,FN,4,0,50"):
  hcr.emit(Serial1, HcrCodec::Stimulate, HcrCodec::Happy, 50);
}

void loop() {
  mp3.poll();              // pump MP3 Trigger responses (fires onFinished / onError)
  dfp.poll();              // same for the DFPlayer's 10-byte reply frames
}
```

See **File → Examples → WcbCmd → BasicUsage** for the runnable version, and **GoldenVectors**
for the byte-exact contract test.

## Serial settings (per device)

The library never opens a port — you do, at the baud the target device expects:

| Device | Verb | Typical baud | Notes |
|---|---|---|---|
| Pololu Maestro | `;M` | as configured in Maestro Control Center | Pololu protocol; subroutine-trigger only |
| SparkFun MP3 Trigger | `;A` | 9600 or 38400 | wire TX↔RX both ways to use responses/ONFIN |
| DFPlayer Mini | `;D` | **9600 only** (module-fixed) | volume is `0`=silent…`30`=loudest — the INVERSE of the MP3 Trigger's. Needs ~1.5–3 s after power-on |
| WLED (serial) | `;L` | 115200 | enable serial in the WLED config; TX→WLED RX |
| HCR vocalizer | `;H` | 9600 | HCR owns its RX for status; here we only write |

## Modules (one per device)

| Module | Verb | API surface | Notes |
|---|---|---|---|
| `WcbMaestro` | `;M<id><seq>` | **namespace** — `emit()` / `parse()` / `buildSubroutineFrame()` | stateless; subroutine-trigger only |
| MP3 (`WcbMp3.h`) | `;A,…` | **`Mp3Codec`** class — `begin()`, `handle()`, `poll()` | stateful (volume shadow, ONFIN callback, RX pump) |
| DFPlayer (`WcbDfPlayer.h`) | `;D,…` | **`DfPlayerCodec`** class — `begin()`, `handle()`, `poll()` | stateful; builds 10-byte frames. Does **not** re-send volume before each play (the module remembers it), and `VOLUP`/`VOLDN` emit an absolute frame so the shadow can't drift |
| WLED (`WcbWled.h`) | `;L,…` | **namespace** `WcbWled` — `emit()` / `build()` | stateless JSON builder (WLED `/json/state`) |
| HCR (`WcbHcr.h`) | `;H` (fn/chan/track) | **`HcrCodec`** class — `emit()`, `format()`, named `Fn`/`Emotion`/`Audio` constants | device-wire formatter; tiny volume shadow; no `HCRVocalizer` dependency |

Maestro, MP3 and DFPlayer **tolerate an optional leading verb letter**, so the exact same token
can be handed to the local translator *and* relayed to a WCB; `WcbWled::build()` instead expects the
id-stripped body (matching the WCB, which strips the `L<id>,` prefix before dispatch). The three stateful classes (`Mp3Codec`,
`DfPlayerCodec`, `HcrCodec`) hold per-device state (volume shadow, ONFIN key); the two namespaces are stateless.

The DFPlayer module deliberately does **not** depend on `DFRobotDFPlayerMini`: that library's
`sendStack()` calls `delay(10)` after every ACK-off frame and blocks outright with ACK on,
neither of which survives on a board whose `loop()` also re-emits SBUS every ~9 ms. Frames
here are written and forgotten; at 9600 baud the UART paces them by itself.

## What's in scope (and what isn't)

`WcbCmd` is **pure translation** — verb parse → native bytes. Everything else stays in each
firmware:

| In the library | In the firmware |
|---|---|
| Verb parse + argument clamp | Local-vs-remote routing decision |
| Native byte / JSON generation | Config slots, aliases, `?`-config commands |
| Per-device runtime state (MP3 volume, HCR shadow) | NVS / `Preferences` |
| | ESP-NOW relay, broadcast fan-out |
| | Port reservation / conflict guards |

WCB-only globals become **injected params**: a `Stream&` for output, an optional `Print* diag`
for debug, and a couple of hooks (e.g. MP3's `onVolumeChanged`).

## ⚠ Two-copy hazard (maintainers)

`WcbCmd` lives as **two physical copies**: the standalone repo (canonical) and the
`Arduino-Code/libraries/WcbCmd` copy each in-tree sketch compiles. **If they drift, a WCB and
the NaviCore emit *different* bytes for the same command — the exact opposite of the goal,
invisible until hardware misbehaves.** Keep them byte-identical, and let `WCBCMD_VERSION` catch
a stale copy loudly.

Three guardrails:
1. **Golden vectors** (`examples/GoldenVectors`) are the contract — the exact bytes each command
   must produce. Any copy that changes the bytes fails the test.
2. **The WCB is the reference board** — adopt/verify a module on the WCB first.
3. **One canonical copy**, mirrored — never a hand-maintained fork.

## Integrating into a firmware

- **NaviCore / WCB_Client hosts:** `#include <WcbCmd.h>`; call the translator on the device's
  local `Stream`.
- **WCB firmware:** see [`docs/WCB-INTEGRATION.md`](docs/WCB-INTEGRATION.md) — the WCB keeps its
  handlers and swaps only the byte-emitting core for a `WcbCmd` call (byte-identical), so no
  field behavior changes.

## License

MIT — see [LICENSE](LICENSE). © 2026 Greg Hulette.
