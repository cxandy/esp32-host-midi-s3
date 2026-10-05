# Testing BLE on the AciduinoV2Box

`ProbeE` runs `BLEClientConnection`, which is a BLE **central**: the board scans
and connects *out* to a BLE MIDI peripheral. So testing it needs something that
**advertises as a BLE MIDI peripheral**.

## The filter it matches against

From `src/BLEClientConnection.cpp:283-322`:

- Service UUID `03B80E5A-EDE8-4B33-A751-6CE34EC4C700` (standard MIDI service)
- Characteristic `7772E5DB-3868-4112-A1A9-F2669D106BF3`

With no target filter set — which is how `ProbeE` calls `bleClient.begin()` —
an advertisement is accepted only if it carries that service UUID. An app that
merely advertises a *name* will be ignored.

## Option 1: the library's own sender (best — no extra hardware)

The repo ships a ready-made peripheral. `examples/T-Display-S3-BLE-Sender`
advertises the MIDI service, so it is exactly what `ProbeE` looks for.

Needs a second ESP32 to run it on. Any ESP32 board with BLE works — it does not
need to be an S3 or have an OLED. The T-Display examples are named for their
display, but the advertising and MIDI-send parts are plain BLE.

Build it on whatever spare board you have and power it up. Then `ProbeE`'s
page 2 should go from `no` to `YES` and show the peer address on the OLED. The
serial log prints `[BLE] connected to <addr>`.

If you have only one board, this option is out and use option 2.

## Option 2: a phone as the peripheral

Cheapest option if you have no spare board.

1. Install a BLE MIDI app — on iOS, **MIDI Wrench** or **SysEx MIDI**; on
   Android, **MIDI BLE** (or connect a Bluetooth-LE MIDI keyboard to the app).
2. Open the app so it starts advertising as a BLE MIDI peripheral.
3. `ProbeE` should connect.

Caveat worth knowing: whether it works depends on the app. Some only start
advertising once a "connect" or "enable" button is pressed, and some expose a
hardware MIDI port rather than a BLE one. If `ProbeE` stays on `scanning`, the
usual cause is the app advertising a name but not the MIDI service UUID — the
filter at line 284-293 rejects it silently, so nothing appears on the OLED.

## Option 3: a hardware BLE MIDI keyboard

Plug in a BLE-capable controller (e.g. a Bluetooth-LE MIDI keyboard) and power
it on. It advertises the MIDI service, so `ProbeE` connects directly and you get
a real end-to-end MIDI path with no software in the loop.

## Reading the result

| Where | What |
|---|---|
| Serial | `[BLE] connected to aa:bb:cc:dd:ee:ff`, then `[MIDI] NoteOn 60 ch1` per note |
| OLED page 2 | `conn YES`, `peer aa:bb:...`, `ever connected` |
| OLED page 3 | `rx` counter and the last event received |

Note the direction. Because `BLEClientConnection` is the central, MIDI **flows
in** from the peripheral — play notes on the sender and they appear in
`rx`. Sending is `midiHandler.sendNoteOn()` going out over the same link, which
`ProbeE` does not exercise.

## What has already been proven

BLE is not untested. `ProbeE` ran 245 s on the board with
`ble=scanning` throughout, heap sawtoothing 217k–219k with no downward trend
and PSRAM untouched at 8382776. That establishes the parts that do not need a
peer: `BLEClientConnection.cpp` compiles on 3.3.12 (it was the one file that
failed on 2.0.17), the BLE stack comes up, the shared `BLEScan` window cycles
without stalling, and it coexists with a live UART transport.

What a peer adds is the parts that genuinely cannot be exercised alone:
`connectToDevice()`, the notify path into `rxQueue`, and `sendMidiMessage()`
over a live GATT link.
