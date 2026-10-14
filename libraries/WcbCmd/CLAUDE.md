# WcbCmd — Working Notes

One command vocabulary for the whole WCB droid ecosystem. Turns a WCB `;`-command verb
(`;M` Maestro, `;A` MP3, `;L` WLED, `;H` HCR) into that device's **native serial wire
protocol**, on an injected `Stream`. Nothing more — routing, config, NVS, and ESP-NOW relay
all stay in the firmware that calls this.

Arduino library, `architectures=esp32`, no dependencies. Also usable standalone with no WCB
present. Integration detail is in [`docs/WCB-INTEGRATION.md`](docs/WCB-INTEGRATION.md).

---

## Rules that are easy to break

1. **This library is compiled by two different firmwares** — the
   [WCB firmware](https://github.com/greghulette/Wireless_Communication_Board-WCB) *and*
   every `WCB_Client` host, currently [NaviCore](https://github.com/greghulette/NaviCore).
   That is the entire point: the same command string must produce the same device bytes
   whether it arrived across the ESP-NOW mesh or out a host-local serial port. **A change to
   a translator changes the wire bytes for both, at once.** Never reason about it as
   belonging to one project.
2. **Changing emitted bytes is a breaking change to physical hardware**, not just to an API.
   Servos move, sounds play, lights change. If a byte sequence is currently correct for a
   device, the burden of proof for altering it is high.
3. **`examples/GoldenVectors/GoldenVectors.ino` is the contract.** It asserts each module
   reproduces the exact bytes WCB and NaviCore emit today, byte-for-byte, against a capture
   `Stream`. If a change makes it fail, the change is what's wrong — unless the change *is*
   a deliberate protocol fix, in which case update the vectors in the same commit and say so.
4. **CI clones this repo directly** rather than using `arduino-cli lib install`, so consumers
   pick up a push immediately with no Library Manager indexing lag. A change ships the moment
   it lands on master — there is no staging step. See
   `~/.claude/docs/ECOSYSTEM.md` § Shared libraries.
5. **Bump `version=` in `library.properties`** when behaviour changes. The version is how the
   consuming firmwares' users tell copies apart.

## Verifying

There is no host-side test runner. The check is:

```
Flash examples/GoldenVectors/GoldenVectors.ino to any ESP32, open Serial @115200.
Every line should read OK and the last line PASSED.
```

Compiling is necessary but not sufficient — this library's whole job is *which bytes*, and
only the golden vectors check that. A change that compiles and was not vector-tested should
be reported as untested.

## Layout

| File | Device | Verb |
|---|---|---|
| `src/WcbMaestro.*` | Pololu Maestro servo controller | `;M` |
| `src/WcbMp3.*` | SparkFun MP3 Trigger | `;A` |
| `src/WcbWled.*` | WLED node | `;L` |
| `src/WcbHcr.*`, `src/WcbHcrFade.*` | Human-Cyborg-Relations vocalizer | `;H` |
| `src/WcbCmd.h` | umbrella header | — |

`Mp3Codec` and `HcrCodec` are **stateful** — created once, globally. The others are stateless
translation.

## Conventions

- **The code is the source of truth.** The emitted bytes are defined by `src/`, and the
  authoritative statement of what they should be is `GoldenVectors.ino`. Never quote a byte
  sequence from the README or `docs/` without checking it — cite `file:line`.
- **Pure translation only.** If a change needs config storage, routing, NVS, or knowledge of
  the mesh, it belongs in the calling firmware, not here. That boundary is what lets the same
  code compile into both.
- **Keep the docs current — same commit as the code.** **Any change to emitted bytes, a new
  verb or action, or a change to a module's stateful behaviour** updates
  `docs/WCB-INTEGRATION.md`, the README command table, and the golden vectors in the same
  commit, with a dated row in that page's **Revision log**. The bar is lower here than
  elsewhere — this library's entire contract is *which bytes*, so a "small" fix that changes
  one byte is a documented change. Internal refactors that emit identical bytes need nothing.
  Full test in `~/.claude/docs/CONVENTIONS.md` § What earns a doc update.
  *That page predates this rule and has no log yet — add one the first time you change it.*
- Keep `keywords.txt` and the README's command tables in step with any new verb.
