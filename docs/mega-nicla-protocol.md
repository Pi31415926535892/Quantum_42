# Mega ⇄ Nicla Host-Link Protocol v1.0

Status: ratification draft — single source of truth for the UART link between the
Arduino Mega 2560 Pro (host / real-time base) and the Arduino Nicla Vision (perception
daughter) of the Quantum 42 rescue robot.

> **Implementation status (2026-10-22):** the physical link is **Serial3** on the Mega —
> **RX3 = D15** ← Nicla TX, **TX3 = D14** → Nicla RX, through the BSS138 shifter. The main
> sketch now services RX3 every loop tick through `NiclaToF.h`, but it reads the Nicla's
> **ASCII** ToF fields (`{dist}` padded to four characters), not this binary protocol. The
> wire format below remains a design contract; D14/D15 are in use, and the binary message
> set is not implemented on either side of the main sketch yet.

Every numeric value, field, scale, and constant in this document is **normative**. Two
independent implementations (Arduino C++ side and MicroPython side) are expected to
interoperate from this text alone; where a hex example appears, it is directly assertable
in unit tests.

## 1. Conformance and global assumptions

- **Byte order:** all multi-byte fields are big-endian (network order).
- **UART framing:** 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control,
  no auto-baud, full duplex. See §2.4.
- **LEN is authoritative** for payload size. Receivers must never infer payload size from
  the message id alone.
- **Forward compatibility:** any message id not listed here is reserved. Receivers ignore
  reserved ids without counting a frame error.
- **CRC scope:** the CRC covers LEN + message-id + payload. The two sync bytes are
  **not** included (§3.1).
- The link is byte-stream based; there is no framing signal other than the fixed sync
  preamble + CRC.

## 2. Physical layer

### 2.1 Why a level shifter

Mega UART drives 5 V logic; the Nicla I/O is 3.3 V logic. Direct connection risks damage
and undefined receive levels, so a **BSS138 4-channel bidirectional level shifter**
carries both directions. Two channels are used; the other two are spare.

### 2.2 Wiring

| Signal | 5 V side (Mega) | 3.3 V side (Nicla) | Shifter channel |
|--------|-----------------|--------------------|-----------------|
| Mega → Nicla | Serial3 **TX3 = D14** | Nicla RX | ch1 |
| Nicla → Mega | Serial3 **RX3 = D15** | Nicla TX | ch2 |
| — | — | — | ch3, ch4 spare |

The ASCII ToF path uses the Nicla → Mega direction only (RX3 = D15); the Mega → Nicla
direction (TX3 = D14) is wired per this table for the binary protocol.

Per-channel BSS138 wiring (identical for both active channels):

- **Source (low side):** the 3.3 V Nicla-side signal; 10 kΩ pull-up to 3.3 V.
- **Drain (high side):** the 5 V Mega-side signal; 10 kΩ pull-up to 5 V.
- **Gate:** tied to the 3.3 V rail (never 5 V).

Direction is auto-sensing per channel; the source/drain labels above match the voltage
sides. The exact Nicla-side pin names are recorded by the hardware crew in `HARDWARE.md`;
this section is otherwise pin-agnostic.

### 2.3 Ground discipline

- Common ground: Mega, Nicla, shifter, L298N, and the battery negative are tied at **one
  star point** near the battery.
- The link must never be the return path for another circuit.
- Route the two UART lines as a twisted pair, ≤ 15 cm long, kept away from motor leads and
  the motor supply. EMI under motor switching at 115200 baud is the reason the protocol
  carries CRC + re-sync (§5.4).

### 2.4 UART configuration

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 (both sides, no auto-baud) |
| Data bits | 8 |
| Parity | none |
| Stop bits | 1 |
| Flow control | none |
| Line idle | UART idles HIGH; a low start bit begins each byte |
| Duplex | full, independent per direction |

## 3. Frame format

```
Byte index       0      1      2        3             4 .. 3+PL            4+PL
                 +------+------+--------+-------------+--------------------+------+
Field            | 0xAA | 0x55 | LEN     | MESSAGE_ID  | payload (PL bytes) | CRC8 |
                 +------+------+--------+-------------+--------------------+------+
```

- **SYNC0** = `0xAA`, **SYNC1** = `0x55`, both fixed.
- **LEN** = number of bytes that *follow* byte 2: message-id + payload + CRC = `PL + 2`.
  Valid range `2..255`; `LEN` 0 or 1 is a framing error.
- **PL** = payload length = `LEN − 2`.
- **CRC8** = CRC over `LEN, MESSAGE_ID, payload` — sync bytes excluded.
- Total on-wire length = `PL + 5` bytes.
- Minimum frame (empty payload): `AA 55 02 <ID> <CRC>`.

### 3.1 CRC-8 definition (normative)

| Parameter | Value |
|-----------|-------|
| Polynomial | `0x07` = x⁸ + x² + x¹ + 1 |
| Initial value | `0x00` |
| RefIn | false (bits processed MSB-first) |
| RefOut | false |
| Final XOR | `0x00` (none) |

Reference routine (port the C below 1:1 to MicroPython — 8 inner shifts, MSB-first):

```c
uint8_t crc8(const uint8_t *p, uint16_t n) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
      else            crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}
```

### 3.2 Worked example — HEARTBEAT

Fields: sync `AA 55`, LEN `0x02`, message-id `0x00`, no payload.
`crc8([0x02, 0x00])` → **`0x2A`**.

Test vector (assert byte-for-byte):

```
AA 55 02 00 2A
```

### 3.3 Worked example — RUN_START with flags = 0x0003

Fields: sync `AA 55`, LEN `0x04`, message-id `0x11`, payload `00 03`.
`crc8([0x04, 0x11, 0x00, 0x03])` → **`0x98`**.

Test vector (assert byte-for-byte):

```
AA 55 04 11 00 03 98
```

## 4. Message tables

Field types: `u8/u16/u32` unsigned big-endian, `i8/i16/i32` two's-complement big-endian.
The "LEN" column is the derived value (`PL + 2`) — use it to assert frames in tests.

### 4.1 Shared (both directions)

| ID | Name | PL | LEN | Payload layout |
|----|------|----|-----|----------------|
| 0x00 | HEARTBEAT | 0 | 2 | — |

### 4.2 Mega → Nicla (host → daughter)

| ID | Name | PL | LEN | Payload layout |
|----|------|----|-----|----------------|
| 0x10 | MODE_SET | 2 | 4 | `mode:u8, arg:u8`. mode: 0=IDLE, 1=RUN, 2=CAL, 3=DIAG; `arg` reserved (send 0, ignore 0) |
| 0x11 | RUN_START | 2 | 4 | `flags:u16`. bit0 = RESUME_LOP (Mega expects radar assist; radar secondary to QTR re-acquisition); other bits reserved (0) |
| 0x12 | DIAG_GET | 1 | 3 | `target:u8`. 0=link counters, 1=sensor_status, 2=config echo. Answered by DIAG_REPORT |
| 0x13 | CAL_START | 1 | 3 | `channel:u8`. 0 = arena color calibration (goal red, triangle red/green, silver/black tapes). Others reserved |
| 0x14 | CAL_ABORT | 0 | 2 | — (cancels any running calibration) |
| 0x15 | REQUEST_BLOB | 2 | 4 | `mask:u16`. bit0=goal_tape, bit1=triangle_red, bit2=triangle_green, bit3=silver_tape, bit4=black_tape; other bits reserved. Response: matching *_BLOB frames |
| 0x16 | REQUEST_RANGE | 0 | 2 | — (one-shot → TOF_RANGE) |
| 0x17 | REQUEST_RADAR | 2 | 4 | `step_deg:u8` (0 = default 20), `sectors:u8` (0 = default 18). Response: one RADAR_RESULT per sweep |
| 0x18 | CONFIG_PUSH | 7 | 9 | `tag:u8, key:u16, value:u32`. tag: 0=float32 (value = IEEE-754 bit pattern), 1=int32, 2=uint32. Keys per §4.4; applied immediately, not persisted |
| 0x19 | SENSOR_GATE | 2 | 4 | `mask:u16`. bit0=yaw stream, bit1=pitch stream, bit2=tof stream, bit3=vision blob stream; other bits reserved. Independent of one-shot REQUEST_* |
| 0x1A | VERSION | 4 | 6 | `proto_major:u8, proto_minor:u8, impl_major:u8, impl_minor:u8` — either side may send; authoritative version comes in READY |

### 4.3 Nicla → Mega (daughter → host)

| ID | Name | PL | LEN | Payload layout |
|----|------|----|-----|----------------|
| 0x01 | READY | 6 | 8 | `proto_major:u8, proto_minor:u8, cal_cached:u8, sensor_ok:u8, impl_version:u16`. cal_cached: 1 = QSPI color cal loaded. sensor_ok bit0=camera, bit1=tof, bit2=imu, bit3=qspi |
| 0x02 | MODE_ACK | 2 | 4 | `mode:u8, accepted:u8`. accepted: 1=yes, 0=no (echoes current mode when rejected) |
| 0x03 | GOAL_BLOB | 9 | 11 | `cx:u16, cy:u16, width:u16, height:u16, confidence:u8` |
| 0x04 | TRIANGLE_BLOB | 6 | 8 | `color:u8, cx:u16, cy:u16, size:u8`. color: 0=red, 1=green |
| 0x05 | TOF_RANGE | 3 | 5 | `mm:u16, status:u8`. status: 0=valid, 1=out_of_range, 2=error |
| 0x06 | RADAR_RESULT | 2 | 4 | `zone:u8, class:u8`. See scale notes below |
| 0x07 | IMU_FRAME | 5 | 7 | `yaw:i16, pitch:i16, flags:u8`. flags bit0=valid; other bits reserved |
| 0x08 | TAPE_BLOB | 6 | 8 | `type:u8, cx:u16, cy:u16, size:u8`. type: 0=silver (entrance), 1=black (exit) |
| 0x09 | DIAG_REPORT | 5 | 7 | `target:u8, data:u32`. target echoes the DIAG_GET target. For target 0, data packs: bits 7..0 = bad-frames counter, bits 15..8 = TX-overflow counter, bits 23..16 = frames decoded |
| 0x0A | CAL_COMPLETE | 1 | 3 | `status:u8`. 0=ok, 1=canceled, 2=rejected (insufficient contrast), 3=error |
| 0x0B | ERROR | 3 | 5 | `code:u8, info:u16`. code: 0=tof_unusable, 1=radar_incomplete, 2=qspi_write_failed, 3=cal_flash_failed, 4=internal. Control info value semantic |

**Scales (normative):**

- `cx`, `cy`: normalized image coordinates, `0..1000`; 0 = left/top, 1000 = right/bottom.
- `width`, `height`: normalized, `0..1000`.
- `size`: normalized approximate blob diagonal, `0..1000`.
- `confidence`: percent, `0..100`.
- `mm`: millimeters, `0..4000`.
- `zone`: sector index `0..sectors−1` toward the best (most open) re-acquisition direction; `0xFF` = no result.
- `class`: **1 = enclosed (evacuation chamber / walled region), 0 = open (sparse line region), 2 = ambiguous (re-sweep needed)**. (cORRECTION 2026-09-22: both codec implementations — the Mega-side codec (formerly `src/rescue_robot/link.h`, removed 2026-10-20) and `nicla/link.py` — emit `1`=enclosed / `0`=open on the wire; the pre-signing draft had this inverted. Both ends and `tof.py`'s conservative-`open` ambiguity handling agree with the corrected mapping. The `zone` worked example above reads a close-run; the RULE is "most open contiguous sector" — the example is illustrative only.)
- `yaw`/`pitch`: `0.01 °/LSB`; yaw in `[-18000, +18000]` (±180.00°), pitch likewise.
- `flags` (IMU): bit0 valid; others reserved zero.

### 4.4 CONFIG_PUSH key registry

Keys apply immediately and are **not** persisted across power cycles (QSPI caches only the
arena color calibration). Unknown keys are ignored by the receiver — no frame error.

| key | name | tag | units | meaning |
|-----|------|-----|-------|---------|
| 0x0001 | TOF_VALID_MAX_MM | 2 (u32) | mm | Ranges beyond this → TOF_RANGE status 1 (out_of_range). Default 3500 |
| 0x0002 | BLOB_MIN_SIZE | 2 (u32) | 0..1000 | Minimum normalized blob size to report. Default 20 |
| 0x0003 | GOAL_CONF_MIN | 2 (u32) | % | Minimum GOAL_BLOB confidence to emit. Default 50 |
| 0x0004 | YAW_OFFSET_DEG100 | 1 (i32) | 0.01° | Additive yaw bias applied to IMU_FRAME in the emitter. Default 0 |

## 5. Runtime contract

### 5.1 Heartbeat

- Each side **must** send HEARTBEAT (`0x00`) every `250 ms ± 50 ms` while powered with the
  link enabled; never faster than 250 ms. Heartbeat is independent — other traffic does
  not substitute for it.
- Liveness, measured from the last received frame of any type:

| State | Last-frame age |
|-------|----------------|
| FRESH | < 1000 ms |
| STALE | ≥ 1000 ms and < 2500 ms |
| LOST | ≥ 2500 ms |

### 5.2 Last-good cache and freshness flag

- Every receiver keeps, per message type, the last decoded value plus its receipt timestamp.
- Per-type freshness flag: FRESH while `(now − ts) < TTL`, else STALE.
- TTL defaults: GOAL/TRIANGLE/TAPE blobs `500 ms`; TOF_RANGE `1000 ms`; IMU_FRAME `1000 ms`;
  RADAR_RESULT `2500 ms`; READY and MODE_ACK are **sticky** (valid until re-READY/restart).
- A cached value may be consumed only while its type is FRESH. A STALE cache is *no*
  decision source.
- Cross-validation: the mission rule that no scored behavior rides a single sensor picks up
  the two fresh sources; if either is stale, that behavior defers rather than proceeding.

### 5.3 Triggers

Link-level (evaluated on the liveness state machine):

- **LINK_STALE** — first transition to STALE. Mega: stop using link-sourced values for
  fresh decisions; hold last-good values for a bounded hold (time-boxed per reflex), still
  waiting.
- **LINK_LOST** — first transition to LOST. Mega: light the error LED, degrade to
  line-only reflexes (QTR-based), keep the no-capture lane open, never block on the link.
- **LINK_REVIVED** — first valid frame after LOST. Mega: clear the degrade and refresh
  gated streams (re-send SENSOR_GATE and any needed one-shots).

Per-type STALE/REVIVED transitions use the same names scoped to that type.

### 5.4 Noise and re-sync rule

Receiver state is `HUNT` (scanning for the preamble) or `FRAME` (consuming a frame).

- In `FRAME`, each of the following increments a `BAD_FRAMES` counter:
  - CRC mismatch (LEN + message-id + payload did not produce the received CRC byte);
  - LEN outside `2..255`, or the frame ends before `LEN` bytes of body arrive;
  - SYNC1 expected-but-mismatch while mid-frame.
- Unknown message-id is **not** a violation (§1 forward compatibility).
- A fully decoded frame (preamble + valid LEN + CRC match) resets `BAD_FRAMES` to 0.
- **When `BAD_FRAMES == 3`:** discard the partial frame, drop to `HUNT`, discard any
  buffered bytes, and re-synchronize on the preamble.
- `HUNT` scan: find the consecutive pair `0xAA` then `0x55`. A lone `0xAA` whose next byte
  is not `0x55` consumes both bytes (the `0xAA` is dropped and the mismatch byte is
  re-examined as a potential `0xAA`). After a valid pair, parse LEN, then payload, then CRC.
- The bad-frame counter resets to 0 on the first fully decoded frame after a re-sync.

### 5.5 Non-blocking send/receive (Mega is host; applies to the link path)

- **RX:** drain Serial3 each main-loop tick with a byte budget `≤ 64 bytes/tick`, and decode
  **at most one frame per tick**. Never wait for a byte to arrive.
- **TX:** build frames into a TX FIFO. Each tick, transmit while the TX register is empty
  and the FIFO is non-empty. If the FIFO is full, drop the **oldest** frame and increment
  the TX-overflow counter (visible through DIAG_GET target 0 / DIAG_REPORT).
- Nothing in the link path may block; all waits are millis-scheduled, and the hardware
  watchdog is kicked every main-loop tick.
- Sizing: 10 bytes ≈ 0.87 ms at 115200 baud; a 250 ms heartbeat cadence needs ~1 byte per
  ms. The TX FIFO must hold at least 8 frames (e.g., 64 bytes) so a sensor-quiesce burst
  never spills.
- **Nicla mirror:** same byte-budget framing; if no frame arrives from the Mega for
  `> 1000 ms`, the Nicla suspends gated streams (stops emitting vision/tof/imu frames) but
  keeps heartbeating and holds its READY state; the first Mega frame resumes the streams.

### 5.6 Time-boxed reads

The link slice per main-loop tick is bounded by the RX byte budget and the one-frame-decode
cap (§5.5). If the slice exceeds the budget the remaining bytes are deferred to the next
tick — the loop never spins. A wedged clock/FSM is bounded by the watchdog (§5.5).

## 6. LoP / restart semantics

- On **any** power-up (cold boot or an LoP power cut through the start button), the Nicla
  runs its own self-check: camera present, ToF present, IMU present, QSPI readable. When
  all pass it sends **READY** once, carrying the protocol version plus `sensor_ok` and
  `cal_cached`. READY is gated **only** by the Nicla self-check — never by Mega state and
  never by whether any Mega frame has been received. The Nicla re-issues READY after every
  reset.
- **Mega boot sequence:** enter WAIT_READY armed with a CONFIG value
  `LINK_READY_TIMEOUT` (default `5000 ms`). Any received frame proves liveness, but the go
  condition is a decoded READY. If READY arrives in budget → handshake complete. If the
  timeout expires → error LED, degrade-to-line-only; the link remains LOST until READY
  eventually arrives, then LINK_REVIVED.
- **QSPI cal-cache:** the Nicla persists the arena color calibration (goal red, triangle
  red/green, silver/black tapes) to QSPI flash with a stamp, so a torn power-cut write is
  detectable. On boot it loads the stamp-valid cache and reports `cal_cached = 1`. Cold
  boot + QSPI load must complete comfortably inside `LINK_READY_TIMEOUT` (target ≤ 2 s).
- With `cal_cached = 0`, the Mega may still start, but color-vision behaviors stay bypassed
  (GOAL/TRIANGLE/TAPE blobs suppressed) until a CAL_START + `CAL_COMPLETE{0}` round-trip
  re-caches. The QSPI write happens once, after the full capture, and is power-failure-safe
  in the sense that a partial write is caught by the stamp on the next boot.

## 7. Versioning and compatibility

- Protocol version is **1.0**; the `<proto_major 1, proto_minor 0>` fields in READY are
  authoritative, and VERSION (4.2/4.3) may be exchanged at link-up.
- Compatibility rules:
  - Receivers must ignore unknown message ids (no frame error).
  - Receivers must ignore unknown CONFIG_PUSH keys.
  - Payload layout never changes for a fixed id within a major version; new fields appear
    only via new message ids (minor roll). Semantic changes that affect mission behavior
    require a major roll.
  - If a peer reports `proto_major ≠ 1`, both sides fall back to Link-Dead discipline
    (degrade-to-line-only + error LED); no partial-hybrid modes.
  - Both implementations must log and compare versions at link-up for the boot diagnostic
    dump.

## 8. Changelog

| Version | Date | Change |
|---------|------|--------|
| 1.0 | 2026-09-22 | Initial ratification: frame format, CRC-8 (poly 0x07, init 0x00, no final XOR), message set both directions, heartbeat/staleness/re-sync contract, LoP handshake + QSPI cal-cache semantics, CONFIG_PUSH key registry, versioning guarantees. |
| 1.0 (physical rev A) | 2026-10-22 | Physical link moved from Serial1 (D18/D19) to **Serial3 (TX3 = D14, RX3 = D15)**. Wire format, message ids, CRC and protocol version 1.0 are unchanged. |