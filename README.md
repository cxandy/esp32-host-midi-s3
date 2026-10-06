# ESP32_Host_MIDI — ESP32-S3 (AciduinoV2Box) feasibility

CI-only build harness for [ESP32_Host_MIDI](https://github.com/sauloverissimo/ESP32_Host_MIDI)
against a pinned arduino-esp32 core, plus the on-hardware bring-up notes that
took the most work to establish.

**Status: verified.** The library compiles unmodified on arduino-esp32 3.3.12
and runs on the real board, including OPI PSRAM, the SH1106 OLED, and the DIN-5
MIDI TX path.

Nothing in the library is modified — it is consumed as a versioned GitHub
source dependency. Every fix lives in the environment (the core version).

## Verified results

UART path, from a `ProbeD` run, 264 s uptime:

```
[PROBE-D] ESP32_Host_MIDI DIN-5 probe, OLED + buttons
[PROBE-D] OLED at 0x3C: responding
[PROBE-D] DIN-5 RX=IO4 TX=IO15, reset reason=0xb
[ALIVE] up=2s   heap=331364 psram=8382776 queue=0 rx=0
[ALIVE] up=264s heap=331364 psram=8382776 queue=0 rx=0
[SEND] NoteOn C4 / [SEND] NoteOff C4   (alternating every 2 s)
```

BLE path, from a `ProbeE` run, 245 s uptime:

```
[PROBE-E] BLE central scanning: yes
[PROBE-E] reset reason=0xb, transports: UART + BLE
[ALIVE] up=2s   heap=218620 psram=8382776 queue=0 rx=0 ble=scanning
[ALIVE] up=245s heap=217396 psram=8382776 queue=0 rx=0 ble=scanning
```

| Claim | Evidence |
|---|---|
| Library compiles unmodified on 3.3.12 | CI green on all probes; the four 2.0.17 errors in `BLEClientConnection.cpp` are gone |
| Runs on hardware without crashing | 264 s (UART) and 245 s (UART+BLE) uptime, no watchdog reset |
| **`PSRAM=opi` really brings up the octal PSRAM** | `psram=8382776` (~8 MB) in every run, matching the N16R8 spec |
| **`BLEClientConnection` works** — the file that broke on 2.0.17 | BLE central scans for 245 s, `ble=scanning` throughout |
| Two transports coexist | UART + BLE registered together; `midiHandler.task()` dispatches to both |
| No heap leak under BLE scanning | sawtooth 217.4k–219.8k with no downward trend; BLE costs ~113 KB vs 331 kB baseline |
| OLED usable | SH1106 ACKs at 0x3C on SCL=IO11 / SDA=IO21 |
| MIDI TX path works | `sendNoteOn/Off` → UART 31250 → IO15, self-test note on C4 |
| MIDI RX path (BLE) | **verified by `MidiMonitor`**: 1298 events from an iPhone in one 695 s run, `rx=1298 tx=1298 fail=0` |
| MIDI RX path (DIN-5) | **verified end to end**, both directions — see below |

### The DIN-5 IN fault that was not software

For most of this project's life the DIN-5 IN path produced nothing: `rx=0`, and
a 6-byte burst sent out of IO15 never came back. It was diagnosed as a hardware
fault, and every measurement agreed with that. It was not one — the wiring was
wrong, and the only thing that could have proved it was a probe or a hand.

What the software could establish, all of it on a board whose loopback did not
close:

- IO15 drive reached the pad (`io15hi=0%` / `io15lo=100%`), and the UART really
  shifted the six bytes out (`txf` moved with the burst).
- The IN pad never followed it: `io4hi=100%` **and** `io4lo=100%` means IO4 read
  LOW for 100% of both windows, with IO15 high and with IO15 low.
- IO4 sat at 0 mV, and the internal ~45 kΩ pull-up only lifted it to ~780 mV, so
  something on the board was holding it down rather than leaving it floating.
- The UART therefore saw a permanent break: `pk=00`, one `0x00` and nothing else,
  every one of which the parser correctly discarded (data byte, no running
  status).

After the wiring was corrected, every one of those numbers inverted:

| Reading | Broken wiring | Fixed |
|---|---|---|
| `mv` (IO4 at rest) | 0 mV | **3198 mV** (idle = mark) |
| `mvpu` (IO4 vs 45 kΩ pull-up) | 782 mV | **3198 mV** (nothing holds it down) |
| `io4hi` (IO4 low while IO15 driven high) | 100% | **0%** |
| `io4lo` (IO4 low while IO15 driven low) | 100% | **100%** — IO4 follows IO15 |
| `pk` (last byte the parser saw) | `00` | **`90` / `80` / `64`** |
| `rx` / `fifo` | 0 / 1 | climbing / 3 |
| loopback burst | no events | `[MIDI] D ON C4 v100 c1` |

`D` on that last line is the DIN-5 source, which means the whole chain closed in
one direction: the sketch's own `90 3C 64 / 80 3C 00` → IO15 → OUT driver → jack →
cable → IN optocoupler → IO4 → UART → parser → display.

### Two instruments that lied, and how they were caught

Both were caught by the firmware disagreeing with itself, which is the reason the
line-by-line capture is worth more than the OLED.

1. **`digitalRead()` on a UART pad does not read the pad.** With `func_sel=UART`
   the read comes from the GPIO side rather than the peripheral, so the idle-high
   TX pad reported LOW on every sample and the IN pad reported HIGH regardless
   of the wire. Both looked exactly like hardware faults. The giveaway was
   `lo15 == lo4 == loopCount` on a freshly booted board — a UART cannot be low on
   every iteration — and `io2` (a plain pull-up GPIO) disagreeing with them. The
   `lo15`/`lo4` fields are gone; pad-level truth now comes only from the
   self-test, which reads under an explicit `pinMode()`.
2. **The library's queue depth is not output latency.** The sketch drains every
   loop iteration but never pops what it has read — there is no per-event pop in
   the library API, and `clearQueue()` resets the very event indices the display
   de-duplicates on. So the queue saturates at `maxEvents` and stays there.
   Harmless (everything evicted was already on screen) but the STATUS page was
   showing a number that could never mean anything; it now shows `din rx`.

## Board

AciduinoV2Box — ESP32-S3-WROOM **N16R8** (16 MB flash, 8 MB **OPI** PSRAM),
bare WROOM module with **no USB-UART bridge chip**.

Known-good FQBN for the probes:

```
esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=default_8MB,
         USBMode=hwcdc,CDCOnBoot=cdc,LoopCore=1,EventsCore=1
```

`PSRAM=opi` is mandatory. The module is octal PSRAM, so the generic
`PSRAM=enabled` (quad) setting leaves the PSRAM interface mismatched and the
board reset-loops at boot (`rst:0x3 RTC_SW_SYS_RST`). Confirmed working here:
`psram=8382776`.

Avoid GPIOs 26–32 (flash), 33–37 (octal PSRAM), 22–25 (not bonded).
IO19/20 = native USB. IO43/44 = UART0.

### Hardware findings that shaped this harness

**IO0 is the SHIFT button.** `硬件接线规划.md` lists "0 已被 SHIFT 使用" and
`使用说明.md` maps `SHIFT=IO0`. Since IO0 is a boot strapping pin, the board's
boot mode is decided by a *button*: SHIFT held = ROM downloader, SHIFT released
= boot the sketch.

**USB-initiated resets can never boot the sketch.** The S3 ROM enters its
downloader unconditionally on any reset that arrives over USB
(`rst:0x15 USB_UART_CHIP_RESET`, `boot:0x0 DOWNLOAD`, `waiting for download`).
Verified dead ends, all with the chip left in the downloader:

- DTR asserted *or* deasserted, then an RTS pulse — the reset source decides, not
  DTR. Measured `GPIO_IN` (0x60004004) with DTR forced each way: IO0 unchanged.
- `esptool run` — sends `ESP_FLASH_END` with param 1 ("run user code"); confirmed
  on the wire via `--trace` (`op=0x04 data=01000000`), still no boot.
- The 1200 bps touch — needs the sketch already running to be observed, so it
  cannot bootstrap itself.

Only a reset from the **EN pin** boots the sketch. So after every flash:

> release SHIFT, then press RESET once.

One consequence worth knowing: `arduino-cli`/esptool cannot automate that step,
which is why `ProbeD` exposes it as a button action (see below).

**USB Host MIDI is not physically possible on this board.** The S3 has a single
internal USB PHY on IO19/20, and the only connector is a USB-C device port
(`硬件接线规划.md` 方案 B). The board's own plan (方案 B+) is USB-MIDI in
**device** role — the board appears to the PC as a MIDI device. The library's
`USBConnection` is a USB **host** implementation (`<usb/usb_host.h>`), so it
cannot be exercised here without adding an external host controller or a
different board. See "Next steps".

## Probes

| Sketch   | What it proves |
|----------|----------------|
| `ProbeA` | Library core (`MIDITransport` / `MIDIHandler` / `MIDIHandlerConfig`) compiles. Lowest layer. |
| `ProbeB` | `UARTConnection` DIN-5 transport on the board's real MIDI pins (RX IO4, TX IO15). |
| `ProbeC` | No library, no PSRAM, output on USB-Serial-JTAG. Isolates "can any sketch boot" from "is the probe or the PSRAM config wrong". |
| `ProbeD` | DIN-5 MIDI **plus** the board's SH1106 OLED and buttons. The instrumented one. |
| `ProbeE` | DIN-5 UART **and** `BLEClientConnection` as BLE central, two transports at once. Proves the most version-fragile file in the library. |
| `ProbeF` | DIN-5 out and BLE in **together**, 305 s, real notes from an iPhone — both transports on one board. |
| `ProbeG` | Measures the OLED stall instead of assuming a cause. |
| `ProbeH` | Separates OLED RAM cost from bus cost at three bus speeds: `sendBuffer()` is 31 ms regardless, so the bus is already at its ceiling. |
| `MidiMonitor` | **The application**, not a probe: monitor + BLE→DIN-5 bridge + the DIAG page described below. |

### ProbeD: OLED and buttons

The OLED is the point, not decoration. Serial capture here is unreliable — it
only works when the host happens to leave COM8 closed across the EN reset
(polling the port open/closed in a loop is what finally caught a boot). The
display is I2C and does not care about the host at all: a screen reading
`RUNNING` settles whether the sketch booted, with no serial archaeology.

Pages, cycled with NAV-1 (IO2) / NAV-2 (IO1):

| Page | Shows |
|---|---|
| 0 | big `RUNNING`, uptime, heap |
| 1 | heap, free PSRAM, largest block, `psram OPI ok` / `ABSENT` |
| 2 | DIN-5 queue depth, last event, RX/TX counters |
| 3 | `ESP32_HOST_MIDI_HAS_{USB,BLE,PSRAM,ETH}` as compiled |
| 4 | key map |
| 5 | reset reason, so a silent reboot loop is identifiable |

Button actions:

- **long-press DIR-DOWN (IO41)** — `ESP.restart()`, relaunch without touching EN
- **long-press SHIFT (IO0)** — drive IO0 low and restart, the in-firmware route
  back into the ROM downloader so the next flash needs no BOOT press

### Pin map used by ProbeB / ProbeD

```
MIDI IN   = IO4  (RX, via optocoupler)
MIDI OUT  = IO15 (TX)
OLED SCL  = IO11     OLED SDA = IO21   (SH1106, U8g2 SH1106_128X64_NONAME_F_HW_I2C)
SHIFT     = IO0   NAV-2 = IO1   NAV-1 = IO2
DIR       = RIGHT IO39 / UP IO40 / DOWN IO41 / LEFT IO42
LED       = IO48
```

The OLED constructor matches the board's own port header
(`src/ports/esp32/esp32s3.h`) rather than inventing a second display stack.

## MidiMonitor — the application

`MidiMonitor/` is what all of the above was for: a MIDI monitor that runs on the
board itself — OLED, buttons, both transports — with a BLE → DIN-5 bridge
underneath. The AciduinoV2Box firmware is out of scope here; only its pin map
was borrowed.

DIN-5 on IO4/IO15, plus a BLE peripheral advertising as `ESP32-S3 MIDI` that a
phone connects to as central (the direction iOS allows — an iOS app cannot
advertise as a peripheral). Forwarding is **BLE → DIN-5 only**, on by default,
toggled with DIR LEFT: one-way on purpose, because a two-way bridge echoes the
phone's own notes straight back at it.

Pages, cycled with NAV-1 (IO2) / NAV-2 (IO1):

| Page | Shows |
|---|---|
| 0 | monitor — last four events newest-at-bottom, `rx tx FWD/MUTE` header; UP/DOWN scrolls |
| 1 | STATUS — BLE connection age, DIN-5 rx/fifo + forward failures, heap + uptime + `DIAG` flag |
| 2 | KEYS — the key map |
| 3 | DIAG — the instrumentation below |

Every button press prints `[UI] <name> -> page N`, so a serial capture can tell
"the input path is dead" from "the screen simply never redrew".

Long-press DOWN restarts the sketch; long-press SHIFT drives IO0 low and
restarts, which is the in-firmware route back into the ROM downloader.

Long-press UP pins IO15 at a steady level so a multimeter can be put on the OUT
jack without racing a UART burst — `HIGH` (mark), `LOW` (space), `hiZ`
(released), then back to the UART.

### Serial command channel

The buttons need a hand, and a hand is not always at the board — the whole
DIN-5 fault above was only closable because the OUT levels could be driven from
the host. Open COM8 at 115200, send single characters, no line protocol:

| Key | Does |
|---|---|
| `1` / `0` / `z` / `u` | hold IO15 `HIGH` / `LOW` / `hiZ` / give it back to the UART |
| `n` | step through those four |
| `a` | toggle the hand-free sweep: `HIGH → LOW → hiZ`, 3 s each, reporting the IN pad at every step |
| `t` | toggle the DIN-5 self-test (every 5 s) |
| `b` | toggle the six-byte loopback burst, which implies `t` |
| `?` | list them |

`tools\serial.ps1 -Send 'a' -Seconds 50` does this from PowerShell.

**Both diagnostics are off by default**, and that is a behaviour change rather
than a default value: the self-test ends the UART every 5 s for ~15 ms, which
throws away whatever MIDI arrived in that window, and the burst plays a phantom
C4 at a real synth every 5 s. They earned their keep while the DIN-5 IN path
was dead; with the wiring fixed they would be the thing causing glitches. `t`
and `b` bring them back.

### DIAG page — how to read it

| Field | Normal | When it is not normal |
|---|---|---|
| `loop` | advancing every second (~70 k/s) | **frozen = `loop()` itself is stuck.** The buttons and the display are both polled there, so this one number separates "the firmware hung" from "only the picture stopped". Readable with no COM port open. |
| `gapmax` | ≈32 ms | worst gap ever seen between two loop iterations. 32 ms is exactly one `sendBuffer()` (ProbeH), i.e. a redraw. Hundreds of ms or seconds = a blocking call, and the usual suspects are serial and I2C. |
| `print` | 0 ms | worst single `Serial` write. See the freeze below. |
| `x<n>` beside it | grows only when nobody is draining the port | prints deliberately dropped instead of blocking — counted, not hidden. |
| `boot` | reason for the last reset | `usb` = raised from the USB side, `panic` / `taskWdt` = the firmware died, `poweron` / `ext` = real reset. |

### The freeze, and why serial was the cause

Symptom: once a BLE central connected and MIDI arrived, the buttons stopped
responding, with no crash and no log line to explain it.

The cause came from reading `cores/esp32/HWCDC.cpp` in arduino-esp32 3.3.12 and
then measuring it on the board:

- `HWCDC::write` pushes into a **256-byte** ring. When the host is not draining
  it, each write waits `tx_timeout_ms` and retries, up to **20 attempts**
  (`HWCDC.cpp:581`, `max_consec_timeouts = 20`).
- `tx_timeout_ms` defaults to **100 ms** (`HWCDC.cpp:105`), so a single
  `Serial.print` could block `loop()` for **2 s** — and `loop()` is where the
  buttons are polled and the screen is drawn.
- The sketch printed one line per MIDI event, so the first burst after a BLE
  connection stalled the loop long enough to read as a hang.

Measured, same board, before and after:

| | before | after |
|---|---|---|
| `print` | 100 ms — the retry cap already hit at a 5 ms timeout; 2000 ms at the stock 100 ms | 0 ms |
| `gapmax` | 100 ms, spent inside `Serial.write` | 32 ms — one redraw |
| 695 s with 1298 notes | reset `rst:0x15` mid-burst | no reset, `rx=1298 tx=1298 fail=0` |

Four changes, all inside `MidiMonitor.ino` — the library is untouched:

1. `Serial.setTxTimeoutMs(5)` — bounds a blocking write to 100 ms, not 2 s.
2. `monPrint()` checks `availableForWrite()` against the line length first and
   **skips instead of blocking**; skipped lines land in `skip`.
3. Each line is built in one buffer and written once. Two writes meant the host
   could stop draining between them, drop the tail, and leave a bare `[MIDI] `
   prefix with the next line glued on — which reads exactly like memory
   corruption, and nearly sent this investigation down the wrong path.
4. Event lines are rate-limited to 25/s.

A healthy capture now reads:

```
[ALIVE] up=455s heap=211948 rx=1298 tx=1298 fail=0 queue=64 ble=connected
        fwd=on log=24/24 loop=30917474 gap=34ms print=0ms skip=940 boot=usb
```

`rx == tx` with `fail=0` is the bridge working: every note the phone sent left
the DIN-5 output.

## Build

Push to `main`, or run it from the Actions tab. The run publishes
`.bin` / `.elf` / `.map` artifacts named `sketch-build-<ProbeName>`.

The core install is ~1.85 GB of downloads; the first run takes a while, later
runs hit the cache. `actions/cache` is keyed on the core version only, so
editing a probe does not re-download the toolchain.

### Why arduino-cli is driven directly

Arduino's own Actions have been restructured out from under this:

- `arduino/compile-action` **no longer exists** (folded into `compile-sketches`,
  which only emits a JSON report and never uploads build output).
- `arduino/setup-task@v1` accepts only `version` and `repo-token`. It is a
  version resolver now — it installs nothing, so `arduino-cli` ends up missing
  from `PATH`.

So the workflow installs a pinned `arduino-cli` from `downloads.arduino.cc` and
does `core install` / `compile --export-binaries` / `upload-artifact` itself.
The library is fetched by cloning the pinned tag into the sketchbook, because
arduino-cli 1.5 refuses `lib install --git-url` unless
`sketch.always_allow_unsafe_lib_url` is enabled.

## Flash

```powershell
gh run download <run-id> --repo cxandy/esp32-host-midi-s3 -n sketch-build-ProbeD -D .\artifacts
.\tools\flash.ps1 -Port COM8 -ArtifactDir .\artifacts
```

`flash.ps1` uses the `esptool.exe` that ships inside the locally installed
arduino-esp32 2.0.17 core, so flashing needs no extra download. It writes four
images at their own offsets (bootloader, partition table, boot_app0, app) rather
than the 16 MB `merged.bin`, which is both quicker and less destructive.

To read the board afterwards:

- `tools\poll.ps1 -Port COM8` — polls open/closed. **Use this right after a
  flash**, because it leaves the port closed for half of every cycle and that is
  what lets the EN reset actually boot the sketch.
- `tools\listen.ps1` — holds the port open. Fine once the sketch is already up.

## Why CI and not a local build

Local `arduino-cli` was pinned to arduino-esp32 2.0.17, which cannot compile the
library at all: `src/BLEClientConnection.cpp` has four type errors because its
`ESP_ARDUINO_VERSION` 2.x branch assumes a `BLEAddress` constructor and a
`String toString()` that 2.0.17 does not provide. Upgrading to 3.3.12 fixes it
without touching the library.

The upgrade also does not fit locally: 3.3.12 needs ~1.85 GB of downloads
expanding to ~3 GB, and the C: drive had under 5 GB free behind an existing
4.9 GB of `Arduino15` packages.

## Transport summary

| Transport | Fits this board? | State |
|---|---|---|
| DIN-5 UART | yes | TX verified; RX needs a device |
| BLE central | yes | scanning verified; connect needs a peripheral |
| USB Host | **no** | single PHY + USB-C device port |
| USB-MIDI device | hardware allows | library has no device-side MIDI class |

Two BLE roles exist and they are opposites — easy to mix up:

- `BLEClientConnection` — **central**: this board scans and connects *out* to a
  BLE MIDI peripheral. Used by `ProbeE`.
- `BLEConnection` — **peripheral**: this board advertises and others connect
  *in*. Used by `examples/T-Display-S3-BLE-Receiver`.

## Next steps

1. **Exercise MIDI RX.** Needs either a MIDI device on DIN-5, a DIN-5 loopback
   (board OUT back to IN) so the self-test note on C4 returns as an RX event, or
   a BLE MIDI peripheral for the `ProbeE` page 2 path. TX and RX share one queue
   path through `MIDIHandler`, so TX passing is encouraging but not proof of the
   receive direction.
2. **USB-MIDI device mode**, not host — see the hardware finding above. The
   library does not implement a USB device MIDI class, so this would be separate
   work rather than a configuration change.
