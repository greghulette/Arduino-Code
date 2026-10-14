# WCB_Client delivery statistics

**Status: implemented in 1.13.0.** Compiled, not yet bench-verified — see §8.

This is step 1 of the fleet-wide mesh-statistics feature designed in
`Astromech-Control/docs/MESH_STATS.md`. Nothing downstream can start until this lands, and this
step alone is purely additive — adding counters cannot change delivery behaviour.

Where this page and `MESH_STATS.md` disagree about this library, this page is right; where this
page and `src/WCB_Client.h` disagree, the header is right.

---

## 1. The problem, stated precisely

Every ESP-NOW delivery counter in the droid fleet reads **zero**, and has since the
`ETM_Droid` → `WCB_Client` migration.

The counters were not disconnected — their **producer was deleted**. `ETM_Droid`'s `OnDataSent`
callback was what incremented them, and it went with the migration. Verified in
`Arduino-Code`:

| Symbol | State |
|---|---|
| `WCB_Client` counters | **none existed** before 1.13.0 |
| Gateway `DGSuccessCounter` / `DGFailureCounter` | declared `Droid_Gateway.ino:369-370`, shipped at `:1509-1510`, **never incremented** |
| Gateway `struct_etmSent[]` … | hardcoded `0` at `Droid_Gateway.ino:1530-1533` (four lines — `struct_etmFailed` was omitted from the original count) |
| BC per-board arrays | written as literal `0` in `setupSendStatusStruct()` |

So all 12 diagnostic counters that reach the operator's webpage are constant zero.

**The raw material already existed inside this library.** `_pending[WCB_PENDING_MAX]` tracks
in-flight commands awaiting ACK, with a per-board `retryCount[WCB_MAX_BOARDS]`. The library
therefore observes every send, every ACK, every retry and every give-up — it simply discarded
that information when a pending slot freed. This work is about *not throwing it away*.

> **Downstream, not done here:** `Droid_Gateway.ino:591-604` carries a comment stating that
> "WCB_Client owns TX but accumulates and exposes NO delivery statistics", and explicitly
> forbids restoring its predecessor's wording. That comment is now wrong and **must** be
> updated when the Gateway work lands.

---

## 2. Scope

**In scope, and delivered:** per-peer delivery counters inside `WCB_Client`, a small read-only
API, zero cost when unused.

**Out of scope** (later steps in `MESH_STATS.md`): the per-board report to the Gateway, the
`Diag_LoRa_Struct` reshape, the `#L11` drill-down, the Remote mirror, the webpage rendering.

---

## 3. What is counted

Five counters per peer, plus a broadcast frame counter.

| Counter | Increments when |
|---|---|
| `sent` | a command is first aimed at that peer — **the initial attempt only**, not retries |
| `ackd` | that peer's ACK arrives for a command we were still waiting on — **once per command** |
| `retries` | a resend fires to that peer — **every** resend, so N retries on one command counts N |
| `failed` | we stopped waiting for that peer's ACK without ever getting one |
| `unguaranteed` | a send asked for a pending slot and was denied one (table saturated) |

**The distinction that matters:** `sent` counts *commands*, `retries` counts *extra attempts*, so
`sent + retries` is the *delivery attempts* aimed at a peer. Keeping them separate is what makes
the ratio meaningful — if `sent` also counted retries, a link that retries constantly would show
a healthy-looking `ackd/sent` while flooding the mesh.

Attempts are **not** frames on the air. One ensured broadcast puts a single frame up but credits
`sent` to every board in `expected[]`, so summing `sent + retries` across peers over-counts that
frame by (N−1); per-board retries are separate unicast frames and do count once each.
`getBroadcastSent()` is the frame counter — use it rather than this sum if what you want is
airtime.

**Invariant, per peer and in the aggregate: `ackd + failed + unguaranteed <= sent`**, the difference
being commands still in flight. If a test ever shows the left side exceeding `sent`, a slot is
being settled twice. It holds in the aggregate as well as per peer because `ackd` is gated on
the same `_statExpects()` predicate that granted `sent` — see "Broadcasts" below for what that
deliberately discards.

`resetStats()` re-credits `sent` for every delivery still in flight at the moment of the reset.
Without that, active slots would stay armed to land `ackd`/`failed` against a zeroed `sent` and
permanently invert the invariant — and since the counters never wrap back, that inversion would
outlive the traffic that caused it by far. Slots are deliberately not settled or freed by a
reset: it is a diagnostic action and must never drop a guaranteed command.

### `failed` means "we stopped waiting", not "retries ran out"

This is the correction that matters most, and the one the original spec got wrong.

Defining `failed` as "abandoned after the final retry" and hooking it at the retry-exhausted
branch would miss the **common** case. `_checkOfflineBoards()` runs at the top of `update()`,
and the offline window is `_heartbeatIntervalSec × _missedBeforeOffline` = 10 s × 5 = **50 s**
(`src/WCB_Client.h`), against a retry budget of `ETM_RETRY_INTERVAL_MS × ETM_MAX_RETRIES` =
500 ms × 3 = **~2 s**. So:

- target powered off **under ~50 s** — still believed online, retries fire and exhaust. The
  naive hook works.
- target powered off **beyond ~50 s**, i.e. essentially always — the board is already `!online`,
  `_ensuredComplete()` reports the slot done ("stop waiting on a board that's simply gone"), and
  the slot frees with `retryCount == 0`. The naive hook fires **never**.

So `failed` is credited at **every** site that stops waiting on a peer: the two per-board
give-ups (dropped-offline, retries-exhausted) credit directly, because clearing `expected[b]`
destroys the evidence; every slot teardown then sweeps whatever is still expected-but-unacked
via `_settleSlot()`. Clearing `expected[b]` *is* the "already counted" marker, which is what
makes the sweep safe to run at every teardown without double counting.

### `ackd` counts the transition, not the packet

A peer re-ACKs every retransmit it hears — `_sendAck()` is the first thing the COMMAND case
does, ahead of the CRC reject and the de-dup check — and an ensured **broadcast** slot stays
open to collect ACKs from several boards. One board can therefore ACK the same slot up to
`1 + ETM_MAX_RETRIES` = **4** times. Incrementing beside the `ackReceived[] = true` assignment
would over-count 4×; the increment is gated on the flag not already being set.

It is additionally gated on `_statExpects()`, so a late ACK arriving after we already gave up
(which cleared `expected[b]`) cannot be counted on top of that `failed`.

### Counters are `uint32_t`

`uint16_t` wraps at 65,535. At a few commands per second a busy board reaches that in hours, and
a wrapped counter reads as a catastrophic failure. `MESH_STATS.md` §3 confirms there is room in
the eventual LoRa struct (52-byte telemetry struct, ~203 bytes free — **not** the "252 of 255"
the old Arduino-Code comments claimed; `Droid_Gateway.ino:390-393` is corrected, but `:713`
still asserts the old figure and remains a downstream fix).

### Broadcasts

An **ensured** broadcast is *not* fire-and-forget. It is tracked, it snapshots every online
board into `expected[]`, it retries **per-board unicast**, and it collects per-board ACKs. It
therefore credits `sent` to every board it expects to ACK — it is a per-board guaranteed
delivery, and crediting it to none of them would leave those boards' `ackd`/`retries`/`failed`
with no matching `sent`, breaking the invariant.

`getBroadcastSent()` is a **frame** counter — broadcast COMMAND frames put on the air, ensured
or not — and is *not* a subset of any peer's `sent`. One ensured broadcast increments both, by
design. A **best-effort** broadcast is untracked and increments only the frame counter; that is
the one case the original "fire-and-forget" wording actually described.

One consequence worth knowing: a board that was offline at send time is not in `expected[]`, but
it still physically hears the broadcast and ACKs it. That ACK is **discarded** — the
`_statExpects()` gate rejects it, because it has no matching `sent` and counting it would push
that peer's `ackd` past its `sent`. It is real evidence the board is reachable, and that evidence
is simply not this feature's job: `isOnline()` and `getNeighbor()` answer "who is out there",
these counters answer "did what I sent arrive".

### What is NOT counted, deliberately

These counters measure the **ETM COMMAND layer only** — `send()`, `broadcast()`,
`sendToSpecialPeer()`, and the bulk back-channel frames that ride `send()`. There are eight
`esp_now_send()` call sites in the library and only one is downstream of `_sendPacket()`.

Uncounted: `sendRaw()` and `sendKyber()` — and therefore **every `WCBStream` / Maestro byte**,
usually the highest-volume class on the mesh — plus `sendRawPacket()` (OTA), the MGMT chunks an
oversized `send()` fragments into, heartbeats, WDP adverts, and outbound ACKs.

None of these is acknowledged. Counting them as `sent` would leave `ackd` pinned at zero against
a climbing `sent`, which reads as a dead link rather than as "not measured" — precisely the
failure the broadcast rule above avoids. **Seeing Maestro traffic move none of these counters is
correct behaviour**, and should be checked for on the bench (§8 step 8) so it is not later
reported as a bug.

If an airtime metric is ever wanted, it needs its own counter and an `_espNowSend()` wrapper
across all eight sites. That would stop being additive, so it is deliberately not done here.
Note such a wrapper would need a class tag: the MGMT fragment path sends to the broadcast MAC
with unicast semantics.

---

## 4. API

```cpp
struct WCBPeerStats {
    uint32_t sent;       // initial send attempts (excludes retries)
    uint32_t ackd;       // ACKs received, once per command
    uint32_t retries;    // resend attempts
    uint32_t failed;     // stopped waiting without an ACK
    uint32_t unguaranteed;     // asked for a pending slot, denied one (table saturated)
};

// Per-peer, 1..WCB_MAX_BOARDS. Out-of-range returns a zeroed struct.
WCBPeerStats getPeerStats(uint8_t wcbID) const;

// Totals across all peers. Computed on demand.
WCBPeerStats getAggregateStats() const;

// Broadcast COMMAND frames on the air. NOT a subset of any peer's sent.
uint32_t getBroadcastSent() const;

// Zero everything. Takes _pendingMux — call from loop(), not a receive callback.
void resetStats();
```

### Design notes

- **Return by value**, not by const reference as originally specified. 20 bytes is five words,
  trivial next to a radio send, and a reference would hand the caller a live view of state the
  ESP-NOW receive task mutates underneath it — and would force a static zeroed instance for the
  out-of-range case.
- **Compute the aggregate on demand.** A second running total is a second thing to get wrong,
  and summing 20 peers is trivial next to a radio send.
- **No floats, no ratios in the library.** Let the consumer divide. A "success rate" field would
  need a policy for the 0/0 case that only the display layer can sensibly choose.
- **Readers take no lock.** Each counter is an aligned 32-bit word, so a single load cannot tear
  on this MCU. Do not "fix" this by adding a lock — it would put the loop task in contention
  with the RX callback for the pending-table spinlock. The honest caveat is one level up: the
  five fields are read one at a time, so a returned struct is a near-instant sample rather than
  a mutually consistent snapshot. Harmless for telemetry.
- **`resetStats()` does take the lock** — it is a write racing the RX task's `ackd` increment.

---

## 5. Where it is hooked

| Counter | Hook |
|---|---|
| `sent`, `unguaranteed`, broadcast frames | `_sendPacket()`, inside the existing slot-claim critical section |
| `retries` | `update()`'s retry pass, beside the existing `retryCount[b]++` |
| `ackd` | the ACK handler, gated on the first transition and on `_statExpects()` |
| `failed` | both per-board give-ups directly; every slot teardown via `_settleSlot()` |

Teardown sites covered by `_settleSlot()`: the early `_ensuredComplete()` free, the post-retry
free, the best-effort 1-second timeout, the unicast free in the ACK handler, and the victim
eviction in `_findFreePending()`.

**Concurrency.** ESP-NOW receive callbacks run on a different task from `update()`, and
`_pending[]` already carries a `portMUX` for exactly this reason. `ackd` is incremented in
receive-callback context while `sent`/`retries` are incremented in `update()` context, so every
increment happens **inside the existing critical section**. No second lock is introduced — a
nested or second portMUX around the same data is how deadlocks get built.

Three rules the helpers document and that are easy to violate while debugging:

1. **Never `Serial.printf` inside a critical section.** `portENTER_CRITICAL` disables interrupts
   on that core; a blocking UART write there hangs or crashes the chip. It will look like the
   stats feature broke ESP-NOW. Print after the unlock.
2. **`_settleSlot()` / `_statFail()` / `_statExpects()` must never take `_pendingMux`.** They are
   called with it already held; a recursive acquire on the same core is an instant deadlock.
   Same contract `_findFreePending()` already carries.
3. **`retries` must be credited beside `retryCount[b]++`, not at the `_transmit()` call.** The
   transmit runs after the unlock and has no per-board attribution left.

### Known accounting gaps

Both are `<=`-preserving — they show as a small permanent "in flight" residue or a pessimistic
`failed`, never as an inverted invariant.

**A re-`begin()` on a running device** wipes `_pending[]` without settling it, so in-flight
expectations are dropped without being credited to `failed` (the counters themselves survive).
That memset is already unlocked and racy today; fixing it is a delivery-behaviour change and was
deliberately left out of an additive commit.

**The final retry to a never-online peer is pessimistic.** For an advert-only client that never
heartbeats (`lastSeenMs == 0`), the retry pass can queue that board's last attempt and push
`retryCount[b]` to `ETM_MAX_RETRIES` in one go — which makes `_ensuredComplete()` true
immediately, so the slot is settled (crediting `failed`) in the same pass whose `_transmit()`
runs just after the unlock. That attempt gets no ACK window, and a peer that does answer finds
no active slot. The slot was **already** freed at that point before counters existed, so this
reports pre-existing retry behaviour rather than causing it. Giving that attempt a real window
means holding the slot one more pass — a delivery change, so it belongs in its own commit if it
is judged worth making. It cannot affect an online peer: `_ensuredComplete()` keeps a slot
outstanding while the board is online, regardless of `retryCount`.

---

## 6. Reset semantics

`resetStats()` ships. Without it, a board that had one bad hour looks bad forever and the ratio
stops meaning anything.

**Who calls it is still open**, and is deliberately deferred to the Gateway work, since that is
where a fan-out would live. The options remain: a mesh command (e.g. a `#L` verb that fans out),
a local-only call on some condition, or reboot only. The library imposes nothing.

`examples/AllFeatures` wires it to the `z` key — an untriggerable reset is how the most useful
call in the API goes unnoticed.

---

## 7. Memory cost

**Measured: 400 bytes** (`examples/AllFeatures`, esp32s3 — globals 54,944 → 55,344 B). That is
20 peers × 20 B, with the 4-byte broadcast scalar absorbed into existing class padding, so it
comes in slightly under the 404 B the arithmetic predicts.

It lands in `.bss` on the seven consumers that declare the client at global scope, and on the
internal-SRAM heap on **NaviCore**, which allocates it (`NaviCore.ino:195` declares
`WCB_Client* wcb = nullptr;`) — only a 4-byte pointer is in NaviCore's `.bss`.

The tightest consumer is the **Body Controller**, the only WROOM32 in the set: it overflowed
`.dram0.bss` by 8,344 bytes when `WCB_Client` (11,508 B) was first added, and has ~265 KB free
after the RC migration. 400 bytes is comfortable there and everywhere else, but that history is
why the number belongs on the record rather than being waved through.

Note the asymmetry worth knowing: `WCB_PENDING_MAX` is **10** against `WCB_MAX_BOARDS` = 20.
This is *not* ten slots for twenty peers — a slot is **per command, not per peer**, so one
ensured broadcast to 20 boards occupies one slot and tracks all 20 ACKs inside it. The reason
counters must be incremented at the hook points rather than derived from `_pending[]` at read
time is that a slot is freed the instant its delivery resolves, discarding the history — not the
10/20 ratio.

---

## 8. Testing

Nothing below has been done yet. What a clean compile proves is only that this builds, that the
signatures are right, and the measured RAM delta in §7. **A wrong counter compiles perfectly.**

**`examples/DeliveryStats` automates the mechanics of this.** It prints a per-peer table with
deltas between samples, self-checks the invariant on every dump (so a violation announces itself
rather than waiting to be spotted in five columns of running totals), and its `g` key prints the
procedure below with the expected deltas at each step. Keys `1`–`6` fire the traffic each step
needs, including a raw/Maestro burst for step 8 and a rapid-send burst to provoke `unguaranteed`.

Real verification needs two boards:

1. **Baseline** — note `getPeerStats(B)` on board A.
2. **B on**, send N ensured unicasts A→B. Expect `sent += N`, `ackd += N`, `failed` unchanged.
   Non-zero `retries` on a marginal link is real signal, not a bug.
3. **B off, under ~50 s.** Send M. B is still believed online, so expect `retries` ≈ `M × 3` and
   `failed += M` via the retries-exhausted path.
4. **B off, beyond ~50 s.** Send M more. Expect `retries` **unchanged** and `failed += M` via the
   early free, with `retryCount == 0`. **This is the path the original spec missed, and it is
   the common one.** If `failed` does not move here, the counter is broken in exactly the way
   nobody would notice.
5. **Ensured broadcast with 2+ boards online.** Expect `sent` to increment on *each* expected
   peer and `getBroadcastSent()` by 1. Watch that no peer's `ackd` exceeds its `sent` — that is
   the duplicate-ACK over-count showing up.
6. **Assert `ackd + failed + unguaranteed <= sent` per peer throughout.**
7. **Cross-core** — sustained bidirectional traffic (A→B while B→A) for several minutes, so
   `ackd` increments on Core 0 concurrently with `sent`/`retries` on Core 1. A misplaced
   increment relative to the critical section shows up as a stuck or wildly wrong counter, and
   only under real concurrency.
8. **Boundary check** — run a `WCBStream` / Maestro sweep and confirm the counters do **not**
   move. That is intended behaviour (§3) and someone will otherwise report it as a bug.

Steps 3 and 4 are the ones worth doing carefully — they exercise the give-up paths, and those
are the counters nobody will notice are wrong. Step 4 needs a real 50-second wait per iteration.

---

## 9. Shipping

- This repo (`greghulette/WCBClient`) is the **shipping** copy. Consumer CI clones it directly.
- `Arduino-Code/libraries/WCB_Client` is a **mirror** and ships nothing — but it *shadows* the
  real one in any local compile, so it must be synced or local builds will disagree with CI.
  Pin with `arduino-cli compile --library <this repo>` and confirm the "Using library … in
  folder" line before trusting a build.
- Mirror commits must say so: `WCB_Client <ver>: … (mirror of greghulette/WCBClient)`.
- Version: **1.13.0** (additive API). The seq-0 fix that preceded it went out as **1.12.1**,
  deliberately as its own commit — this one's whole claim is that it cannot alter delivery, and
  keeping a behaviour change out of it is what makes that claim checkable.
- Consumers: **eight** — six Arduino-Code droid sketches, `Leia_Projector`, and NaviCore. The
  WCB firmware does **not** compile this library (zero `#include <WCB_Client.h>` in
  `Wireless_Communication_Board-WCB/Code/`), nor does SBUSController; publishing does not
  rebuild the WCB firmware.

---

## 10. Decisions on the record

- **Name collisions: none.** `WCBPeerStats`, `getPeerStats`, `getAggregateStats` and
  `getBroadcastSent` appear nowhere else in any repo. `resetStats` has one unrelated hit —
  `fl::SpectralEqualizer::resetStats()` in vendored FastLED — a class member in namespace `fl`,
  so no conflict despite FastLED being on every local compile's include path.
- **`WCB_MAC_OCT2 / OCT3 / QUANTITY` are irrelevant here.** This is internal accounting.
- **Reset trigger** — deferred to the Gateway work (§6).
- **Does NaviCore (W20) report too?** Does not affect this library; the counters exist per-peer
  either way. Deferred to `MESH_STATS.md` open question 5.
- **The Sled (W10) does not report statistics.** It joins as a temporary mesh peer
  (`setTemporary(true)` + `setIdentity(...)`, already applied in
  `Arduino-Code/Sled_Arduino_ESP32S3/Sled_Arduino_ESP32S3.ino`), so it is an occasional peer
  whose stats row would be stale more often than not. No reporting path, no row.
- **`MESH_STATS.md` line numbers have drifted** 10–16 lines throughout against Arduino-Code
  (`:588`→`:598`, `:1515-1519`→`:1530-1533`, `:1168`→`:1184`). Navigate by this page, not that
  one, and take a correction row in its revision log when the Gateway work lands.
- **Downstream, worth resolving before the Gateway work:** there are **two**
  `setupSendStatusStruct()` functions. The GUI one zeros the ETM arrays
  (`Body_Controller_ESP32_GUI.ino:525-528`) but the non-GUI one still writes real data from
  `etmBoardTable[]` (`Body_Controller_ESP32.ino:876`). Confirm which sketch ships — the
  downstream problem may be narrower than both documents assume.

---

## Revision log

| Date | Commit | Change | Why |
|---|---|---|---|
| 2026-08-12 | — | Spec written | Step 1 of `Astromech-Control/docs/MESH_STATS.md`, split out to be implemented in a session on this repo. Records that the counters were deleted rather than disconnected, that `_pending[]`/`retryCount[]` already observe every outcome, and that the Sled is excluded by decision |
| 2026-08-12 | _(this commit)_ | Implemented as 1.13.0; spec corrected to as-built | Four counter definitions were wrong against the code. `failed` hooked only at retry-exhaustion would never fire for a powered-off target (50 s offline window vs ~2 s retry budget) — the common case; now credited at every stop-waiting site via `_settleSlot()`. Ensured broadcasts are not fire-and-forget and now credit per-peer `sent`, with `getBroadcastSent()` redefined as a frame counter. `ackd` gated on the ACK transition (a peer re-ACKs every retransmit — up to 4× per slot). Added `noSlot` (renamed `unguaranteed` in 1.13.1) for any tracked send denied a pending slot, which `sent`/`ackd`/`failed` alone could never account for. Return by value, not const reference. Also corrected: measured RAM is 400 B not 324 B, NaviCore heap-allocates the client, the WCB firmware does not compile this library, and the 10-vs-20 "asymmetry" is per-command vs per-peer, not a shortfall |
| 2026-08-12 | _(same commit)_ | Three defects found by adversarial review of the first implementation, fixed before landing | (1) `noSlot` was gated on `ensured`, but a best-effort **unicast** is tracked too and can equally be denied a slot — it credited `sent` and could then never be ackd or failed, accruing a permanent phantom "in flight" residue. Gate is now `track`, and the counter is redefined as "asked for a slot, denied one". (2) `resetStats()` zeroed the counters while pending slots stayed armed, so in-flight commands landed `ackd`/`failed` against a zeroed `sent` and permanently **inverted** the invariant; it now re-credits `sent` for outstanding expectations. (3) The header and this page both claimed an aggregate `ackd > sent` surplus from offline boards ACKing a broadcast — impossible, because the `_statExpects()` gate discards that ACK outright; the invariant holds in the aggregate too. Also documented, not fixed: the final retry to a never-online peer is settled in the same pass that transmits it (see "Known accounting gaps") |
| 2026-08-13 | _(this commit)_ | Renamed `noSlot` → `unguaranteed`; released as 1.13.1 | Every other counter names what it *is*; `noSlot` named what the implementation ran out of, and nothing got a reader from `NoSlot: 4` to "four commands went out without a delivery guarantee". The concept stays — it is genuinely distinct from `failed` (local backpressure vs a link problem) — only the name changed. `unguaranteed` was chosen over `bestEffort` (would read as counting every `ensured=false` send, which it does not) and `noETM` (actively wrong: the packet IS a full 252-byte ETM COMMAND, is ACKed by the receiver, and is deduped — only the sender's tracking is absent). Source-breaking for consumers of `WCBPeerStats`; done immediately after 1.13.0 while NaviCore was the only one |
| 2026-08-14 | _(this commit)_ | Corrected the airtime claim; released as 1.13.2 | `sent + retries` was described as "total airtime for a peer". True for unicast, wrong once broadcasts are involved: one ensured broadcast puts a SINGLE frame on the air but credits `sent` to every board in `expected[]`, so summing across peers over-counts the initial frame by (N-1). Per-board retries are separate unicast frames and do count once each. Reworded to "delivery attempts", with `getBroadcastSent()` named as the frame counter — which is what the header already said it was, so the two no longer contradict each other. Docs only, no code change. Found by code review alongside two `MgmtRelay` defects (oversized-line tail reaching `broadcast()`, and silent OTA send failures) fixed in the same commit |
