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
;M0255 -> AA 00 27 FF
```
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
// processMP3AudioCommand(message):  message still starts with 'A'
mp3Codec.handle(message.c_str());     // replaces the whole PLAY/STOP/VOL/... body

// processMP3Responses():
mp3Codec.poll();                       // replaces the whole RX switch
```
`handle()` tolerates the leading `A`, so pass `message.c_str()` directly. Keep
`mp3Volume` as the source of truth for backup/`?MP3` display — `onVolumeChanged`
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
behaviour is unchanged (except fn 8 per above). Keep `hcrCodec.setVol()`
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
```

The config/`?`-command paths (NVS, port reservation, WDP, backup, poll, status) stay
untouched in all phases.
