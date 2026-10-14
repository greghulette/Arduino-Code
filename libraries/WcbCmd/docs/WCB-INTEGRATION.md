# WCB firmware → WcbCmd integration (handoff to the WCB session)

This is the WCB-firmware side of the "one command source" refactor. The NaviCore
side and the `WcbCmd` library are built in the NaviCore session; **this doc is what
the WCB session applies.** Each phase is byte-identical — no WCB field behavior
changes, so it's safe to ship after a compile + a quick bench check.

The principle for every module: **the WCB keeps its handler and its routing; only
the inline byte-generation is replaced by a `WcbCmd` call that produces the exact
same bytes.**

---

## Step 0 — make `WcbCmd` reachable by the WCB build (one-time)

`WcbCmd` is distributed as a standalone repo, mirrored into each build's libraries
folder (same convention as `WCB_Client`). Put a copy where the WCB's `arduino-cli`
resolves libraries (the folder that already holds your other shared libs), e.g.:

```
<your WCB libraries path>/WcbCmd/
  library.properties
  src/WcbCmd.h, WcbMaestro.h, WcbMaestro.cpp   (+ later: WcbMp3, WcbWled, WcbHcr)
```

Keep it **byte-identical** to the canonical copy. `WcbCmd.h` defines
`WCBCMD_VERSION`; optionally assert it in `setup()` so a stale copy fails loudly:

```cpp
#include <WcbCmd.h>
static_assert(sizeof(WCBCMD_VERSION) >= 1, "");   // compile-time presence check
// or at boot:  Serial.printf("WcbCmd %s\n", WCBCMD_VERSION);
```

---

## Phase 1 — Maestro (`;M<id><seq>`)  ← apply now

**File:** `Code/WCB/WCB_Maestro.cpp`

**1. Add the include** at the top:
```cpp
#include <WcbCmd.h>
```

**2. Replace each inline frame with the shared builder.** There are **6** identical
frames (lines ~106, 142, 149, 173, 183, 194), all of the form:

```cpp
// BEFORE
uint8_t command[] = {0xAA, <ID_EXPR>, 0x27, scriptNumber};
```
```cpp
// AFTER  (byte-for-byte identical: {0xAA, <ID_EXPR>, 0x27, scriptNumber})
uint8_t command[4];
WcbMaestro::buildSubroutineFrame(<ID_EXPR>, scriptNumber, command);
```

`<ID_EXPR>` is whatever that line already uses (`maestroID`,
`maestroConfigs[i].maestroID`, or `WCB_Number`). Everything after — the
`stream.write(command, 4)` / port selection / broadcast fan-out — stays exactly as
is. `buildSubroutineFrame` is `{0xAA, id, 0x27, seq}` and returns 4.

> The `;M<id><seq>` **parser** (`processMaestroCommand`, `WCB.ino:5547-5552`) does not
> need to change — it already yields `(id, seq)`, and the WCB routing already owns the
> local-vs-remote and slot decisions. Only the frame bytes move into the lib.
> (Optionally, `processMaestroCommand` could call `WcbMaestro::parse(...)` to share the
> parse too, but the existing `substring` parse is already identical — low priority.)

**3. Verify.** Flash the `WcbCmd` `GoldenVectors` example (or eyeball): the frames
must be unchanged —
```
;M11   -> AA 01 27 01
;M25   -> AA 02 27 05
;M2100 -> AA 02 27 64
;M0127 -> AA 00 27 7F
```
(From 0.9.2 a subroutine of 128 or more is refused, `;M0255` included: its byte would read as a command byte.)
Then bench-test one real subroutine trigger on the WCB. Because the bytes are
identical by construction, "compiles + one trigger works" is sufficient.

**Scope note:** `;M` is subroutine-trigger only. Set-Target / Speed / Accel are NOT
part of `;M` and are out of scope for this phase on either board.

---

## Phase 2 — MP3 (`;A,…`)  ← apply now (needs WcbCmd ≥ 0.2.0)

**File:** `Code/WCB/WCB_MP3.cpp`. The `;A` verbs, native bytes, volume-shadow clamps,
the ONFIN callback and the RX pump all move into a `Mp3Codec` instance. Routing,
`?MP3` config, NVS and the "not configured" guard stay put.

**1. Include + one module-static codec:**
```cpp
#include <WcbCmd.h>
static Mp3Codec mp3Codec;
```

**2. Bind it to the resolved port** (in `beginMP3`/`loadMP3Settings`, and whenever the
port changes), and wire the hooks that used to be inline side-effects:
```cpp
mp3Codec.begin(getSerialStream(mp3Config.serialPort), &Serial);
mp3Codec.setVolume(mp3Volume);                                    // sync shadow from NVS (no emit)
mp3Codec.onVolumeChanged = [](uint8_t v){ mp3Volume = v; saveMP3Settings(); };
mp3Codec.onFinished      = [](const char* k){ recallCommandSlot(String(k), 0); };
mp3Codec.onError         = []{ if (strlen(mp3Config.onErrCmd)) recallCommandSlot(String(mp3Config.onErrCmd), 0); };
```

**3. Delegate the runtime paths** (keep the top-of-function
`if (!mp3Config.configured)` guard exactly as is):
```cpp
// processMP3AudioCommand(message):  message starts with 'A'. Strip it and one comma
// first, so ;APLAY,5 and ;A,PLAY,5 both reach handle() as "PLAY,5".
String rest = message.substring(1);
rest.trim();
if (rest.startsWith(",")) rest = rest.substring(1);
mp3Codec.handle(rest.c_str());         // replaces the whole PLAY/STOP/VOL/... body

// processMP3Responses():
mp3Codec.poll();                       // replaces the whole RX switch
```
`handle()` strips a leading `A` too, but **only as a standalone prefix**: followed by a comma
or the end of the body (`A,STOP`, `A`). A letter glued to the verb is read as part of the
verb, so `ASTOP` is rejected. No MP3 verb starts with `A`, and the WCB strips its own prefix
before calling, so the rule changes nothing on this path; it exists for `;D,DEVICE` (Phase 5)
and is kept the same in both codecs. Keep `mp3Volume` as the source of truth for backup/`?MP3` display — `onVolumeChanged`
mirrors it, and `setVolume()` pushes it back into the codec on load.

**4. Verify** (golden vectors — volume is `0=loudest..64=inaudible`):
```
;A,PLAY,5 -> 76 14 74 05   ;A,STOP -> 4F   ;A,NEXT -> 46   ;A,PREV -> 52
;A,VOL,25 -> 76 19         ;A,VOLUP/VOLDN -> 76 <vol>
;A,COUNT  -> 53 31         ;A,VER  -> 53 30
```
NaviCore already emits these exact bytes (`NaviCore.ino:1056-1085`), so this is a
true no-op refactor on the WCB — compile + one real play is sufficient.

## Phase 3 — WLED (`;L,…`)  ← apply now (needs WcbCmd ≥ 0.3.0)

**File:** `Code/WCB/WCB_WLED.cpp`. WLED is stateless, so this is the smallest phase:
the entire verb→JSON core (`wledSend` + every `ON/OFF/BRI/PS/COL/FX/PAL/JSON` branch
inside `wledDispatchLocal`) becomes one `WcbWled::build()` call. **Routing stays put**
— `processWLEDRuntimeCommand` (the `;L<id>` parse, slot lookup, remote ESP-NOW
forward), `?WLED` config, NVS, port reservation and WDP auto-add do **not** change.

**1. Include:**
```cpp
#include <WcbCmd.h>
```

**2. Collapse `wledDispatchLocal` to a thin wrapper** — delete the `wledSend` helper
and all ~75 lines of verb branches; keep only:
```cpp
static void wledDispatchLocal(int port, const String &cmd) {
  String json = WcbWled::build(cmd, &Serial);   // usage/unknown msgs -> Serial, exactly as today
  if (json.length() == 0) return;               // error already printed; nothing to send
  Stream &out = getSerialStream(port);
  out.print(json);
  out.print('\n');
  if (debugEnabled) Serial.printf("[WLED-DBG] TX -> %s\n", json.c_str());
}
```
`build()` reproduces the identical JSON and prints the same usage/unknown-verb
messages to the `diag` you pass (`&Serial`), so console output is unchanged too. The
`&Serial` diag keeps those messages unconditional; the `debugEnabled` TX log stays
here in the firmware. (`build()` expects the id-stripped body — it does **not** strip a
leading `L`/`L<id>,` — which matches the WCB, whose dispatcher hands over the stripped `rest`.)

**3. Verify** (each is the WLED `/json/state` doc, `\n`-terminated):
```
;L,ON  -> {"on":true}      ;L,OFF -> {"on":false}     ;L,TOGGLE -> {"on":"t"}
;L,BRI,128 -> {"bri":128}  ;L,PS,2 -> {"ps":2}         ;L,PAL,3 -> {"seg":[{"pal":3}]}
;L,COL,FF0000  -> {"seg":[{"col":[[255,0,0]]}]}
;L,FX,9,200,128 -> {"seg":[{"fx":9,"sx":200,"ix":128}]}
;L,JSON,{...}  -> {...} verbatim
```
Compile + one real preset recall (`;L,PS,<n>`) on a wired WLED is sufficient — the
bytes are identical by construction.

## Phase 4 — HCR (`;H`)  ← needs a decision before applying (WcbCmd ≥ 0.4.0)

HCR is different from Maestro/MP3/WLED: the module is `HcrCodec`, and it formats the
**numeric `(fn,chan,track)` action** into the HCR device wire string (`<SH50,QEH,QT>\n`,
`<CA0003,QPA>\n`, `<PVV50>\n`, …) — *not* the readable `;H,STIM/PLAY/VOL` verbs. That's
deliberate: NaviCore's local-serial path and a WCB's `;H,FN,<fn>,<chan>,<track>` handler
both reduce to `(fn,chan,track)`, so that triplet is the true common denominator. The
readable verbs, the WCB's synthesized **fades**, and all **status RX parsing** stay in the
WCB firmware on the `HCRVocalizer` library — this phase touches only the WRITE bytes.

### `fn 8` (Stop) — RESOLVED: 4-frame (= `HCRVocalizer::Stop()`)

`HcrCodec::format()` is verified byte-identical to the WCB's `HCRVocalizer` for **every**
fn. Stop was the only one that had disagreed, and it's now settled on the WCB's form:

| | Stop (`;H,FN,8` / `;H,STOP`) |
|---|---|
| **WCB `HCRVocalizer::Stop()`** (canonical) | `<PSV,QT>\n<PSV,QPV>\n<PSA,QPA>\n<PSB,QPB>\n`  (4 frames: StopEmote + StopWAV V/A/B) |
| **`HcrCodec` fn 8** (now matches) | `<PSV,QT>\n<PSV,QPV>\n<PSA,QPA>\n<PSB,QPB>\n` |

`HcrCodec` and NaviCore's local path were both updated to the 4-frame form, so
**the WCB needs no Stop change** — its `;H,FN,8` already calls `Stop()` and stays 4-frame.
Routing the FN handler through `HcrCodec` (below) is therefore a **clean no-op** for the WCB:
all fns, Stop included, produce identical bytes.

### Applying it

**File:** `Code/WCB/WCB_HCR.cpp`. Minimal, low-risk scope — route only the numeric FN
handler (the path NaviCore actually uses over the mesh) through the shared formatter:

**1. Include + one codec instance:**
```cpp
#include <WcbCmd.h>
static HcrCodec hcrCodec;
```

**2. In `processHCRRuntimeCommand`, replace the `FN` switch body** (the
`case 2: _hcr->SetEmotion(...)` … block) with the shared formatter, writing straight to
the reserved port:
```cpp
if (vU == "FN") {
  int fn = hcrField(body,1).toInt(), chan = hcrField(body,2).toInt(), track = hcrField(body,3).toInt();
  if (_hcrPort && hcrCodec.emit(*_hcrPort, (uint8_t)fn, chan, track)) {
    if (debugHCR || debugEnabled) Serial.printf("[HCR-DBG] FN %d,%d,%d\n", fn, chan, track);
  } else {
    Serial.printf("[HCR] FN %d,%d,%d rejected\n", fn, chan, track);
  }
  return;
}
```
`HcrCodec::normalize()` owns the same per-fn ranges the old switch did — and, matching the
WCB, it **rejects chan 4** (no emotion-4→Overload shortcut) and caps track at 0–99 — so
behaviour is unchanged (except fn 8 per above, and fn 17 per below).

**fn 17 takes 0–100, not 0–99** (since 0.9.0). The 0–99 cap was inherited wholesale from the
old `;H,FN` switch, where it belongs to emotion levels; the HCR's own API documents volume as
0–100 (`hcr.h` `SetVolume`) and clamps nothing. The old cap made `;H,FN,17,<ch>,100` fail
outright while `;H,VOL,<ch>,100` quietly clamped to 99 — the same request answered two ways.
The codec's shadow and `HcrFade` were raised to match, so a fade can no longer step a channel
sitting at 100 down to 99. **This is a wire change at maximum volume only**: `<PVV100>` now
goes out where `<PVV99>` did.

**fn 20 / fn 21 are graceful stops** (since 0.9.0), the only way to reach the vocalizer's
`<PSG>`:

| fn | Emits | Notes |
|---|---|---|
| 20 | `<PSG>\n` | StopEmoteGraceful. Bare — no chained query. Unlike PSV, the vocalizer is still speaking when it acknowledges, so a `QT` would report a duration about to change. |
| 21 | `<PSG>\n<PSA,QPA>\n<PSB,QPB>\n` | StopAllGraceful. **Deliberately omits fn 8's `<PSV,QPV>` frame** — channel V *is* the vocalizer, so `StopWAV(V)` would cut it off right after PSG asked it to wind down, making the graceful path a no-op. |

They start at 20 rather than filling the gaps at 1/12/15, which are unexplained in the
numbering inherited from NaviCore and may mean something upstream. Keep `hcrCodec.setVol()`
in sync wherever the WCB sets an absolute volume (the readable `;H,VOL` handler and load),
so the relative `;H,VOLUP/VOLDN` step math agrees across the two write paths.

**3. Optional (consistency):** to make the readable `;H,STOP` verb match `;H,FN,8`, change
its `_hcr->Stop();` to `hcrCodec.emit(*_hcrPort, 8, 0, 0);`. Leave the other readable verbs
(STIM/PLAY/VOL/FADE) and the whole fade machine + status RX on `HCRVocalizer` — they map to
the same fns and this keeps the change small.

**4. Verify** (golden vectors, `\n` between frames):
```
;H,FN,2,0,50 -> <OH50,QEH>       ;H,FN,4,2,30 -> <SM30,QEM,QT>   ;H,FN,5,0,0 -> <SE,QT>
;H,FN,14,1,3 -> <CA0003,QPA>     ;H,FN,16,2,0 -> <PSB,QPB>       ;H,FN,17,0,50 -> <PVV50>
;H,FN,17,3,40 -> <PVV40><PVA40><PVB40>      ;H,FN,8,0,0 -> <PSV,QT><PSV,QPV><PSA,QPA><PSB,QPB>  (4 frames)
;H,FN,17,0,100 -> <PVV100>       ;H,FN,20,0,0 -> <PSG>           ;H,FN,21,0,0 -> <PSG><PSA,QPA><PSB,QPB>
```

## Phase 5 — DFPlayer (`;D,…`)  ← applied (WcbCmd ≥ 0.8.0; the prefix and RX rules need ≥ 0.9.1)

**File:** `Code/WCB/WCB_DFP.cpp`. Same shape as MP3: one module-static `DfPlayerCodec`,
bound to the resolved port with the volume shadow pushed in (`dfpBindCodec()`), and hooks
for `onVolumeChanged`, `onFinished` (ONFIN → `recallCommandSlot`) and `onError`. Routing,
`?DFP` config, NVS and the "not configured" guard stay in the firmware.

```cpp
// processDFPCommand(message):  message starts with 'D'. Strip it and one comma first.
String rest = message.substring(1);
rest.trim();
if (rest.startsWith(",")) rest = rest.substring(1);
dfpCodec.handle(rest.c_str());

// processDFPResponses(), once per loop():
dfpCodec.poll();
```

**`handle()` strips a leading `D` only as a standalone prefix**, followed by a comma or the
end of the body. `DEVICE` is the reason: the WCB hands over `DEVICE,2` with its own `D,`
already gone, and an unconditional strip would read that as `EVICE,2`. So the WCB's
`DEVICE,2` and NaviCore's local `D,DEVICE,2` (`g_dfp.handle(";D,DEVICE,2" + 1)`) both select
the device, while a `D` glued to a verb (`DSTOP`, `DDEVICE,2`) is not a prefix and is
rejected. A new verb that starts with `D` needs a golden vector in both forms.

**`poll()` accepts only reply frames that begin `7E FF 06`.** It checks the header as bytes
arrive: a byte that breaks it drops the partial frame, and if that byte is `7E` it starts
the next one. A partial frame is also dropped once more than 200 ms passes between two bytes
`poll()` reads. Together these keep one stray or lost byte from leaving the buffer part-full,
where the next track-finished frame would fail the `EF` tail check and its ONFIN would never
fire. What that costs:
- A clone module whose frames carry another version byte (not `FF`) or length (not `06`) is
  ignored on the RX side. TX is unaffected.
- The 200 ms is measured between reads, not arrivals. A `loop()` that stalls for more than
  200 ms while a frame is half-read drops that frame, so call `poll()` at least that often.
- A partial frame whose header is valid, followed within 200 ms by the next frame, still
  costs that frame. The checksum is never verified, so there is nothing else to re-anchor on.

NaviCore never calls `poll()` on its local DFPlayer path (`NaviCore.ino` ≈1878), so the RX
rules reach the WCB only.

**Verify** (golden vectors; every frame is `7E FF 06 <CMD> 00 <PH> <PL> <CKH> <CKL> EF`):
```
;D,PLAY,5   -> 7E FF 06 03 00 00 05 FE F3 EF    ;D,STOP -> 7E FF 06 16 00 00 00 FE E5 EF
;D,DEVICE,2 -> 7E FF 06 09 00 00 02 FE F0 EF    (as "DEVICE,2" from a WCB and "D,DEVICE,2" from NaviCore)
;D,VOLUP from 25 -> 7E FF 06 06 00 00 1B FE DA EF    (an absolute 0x06 frame, never the relative 0x04)
RX 7E 01 02, then 7E FF 06 3D 00 00 05 FE B9 EF -> onFinished(<ONFIN key>)
```

The config/`?`-command paths (NVS, port reservation, WDP, backup, poll, status) stay
untouched in all phases.

---

## Revision log

| Date | Version | Change |
|---|---|---|
| 2026-10-07 | 0.9.2 | **`;M` numbers are range-checked before the cast to their wire width, and refused out of range.** `build()` cast the channel, target and speed first, so `setTarget,261,6000` sent channel 5, a target of 70000 sent 4464 and a speed of 65537 sent 1 (NaviCore HIL plan D-NC76, D-NC56). A subroutine is 0-127 in `parse()` and `build()`, not 0-255: 128-255 put a command-range byte inside the frame (D-NC23). Wire change only for values that used to wrap or carry a high bit. GoldenVectors: `M0255` is now `M0127`, plus `M0128` refused and a verb-form section (`build()`). |
| 2026-09-25 | 0.9.1 | **`;D` / `;A` prefix:** `DfPlayerCodec::handle()` and `Mp3Codec::handle()` strip the leading `D` / `A` only when a comma or the end of the body follows it. The unconditional strip ate the first letter of `DEVICE` once the WCB had already removed `D,`, so `;D,DEVICE,<n>` failed over the mesh while the same NaviCore action worked on a local port; `;D,DDEVICE,<n>`, which only worked because of that, is now rejected. No MP3 verb starts with `A`, so the MP3 half was latent. **DFPlayer RX:** `poll()` checks the `7E FF 06` header as bytes arrive and drops a partial frame after 200 ms (`_lastRxMs`). It had resynced only on an empty buffer, so one stray byte silently cost the next track-finished frame and its ONFIN. No TX byte changes. Golden vectors added for both; `CaptureStream` gained an RX side (`feed()`) so `poll()` can be driven. The `DEVICE,2` vector added in 0.8.0 fails against the 0.8.0 and 0.9.0 code, so the vectors were not run on those releases. `WCBCMD_VERSION` said `0.8.0` in the 0.9.0 commit and now matches `version=`. WCB `docs/HIL_FIX_TRACKER.md` #21 and #2 (branch `WIFI`), both bench-verified through a WCB on 2026-09-22. The golden vectors pass on a host build of the sketch (g++, with signed and with unsigned `char`); they have not been flashed to an ESP32 yet. |
| 2026-09-20 | 0.9.0 | `fn 17` range 0–99 → **0–100**, matching the HCR's documented volume range; the codec shadow and `HcrFade` clamps raised with it. Wire change at max volume only (`<PVV100>` where `<PVV99>` went before). New **`fn 20`** (`<PSG>`) and **`fn 21`** (`<PSG>` + A/B stops) reach the vocalizer's graceful stop, which nothing in the `;H` layer could emit. WCB issues #15 and #16, reported by Travis Cook. |
