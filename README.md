# ESP32_Host_MIDI — ESP32-S3 (AciduinoV2Box) feasibility build

CI-only build harness. Compiles the [ESP32_Host_MIDI](https://github.com/sauloverissimo/ESP32_Host_MIDI)
library against a pinned arduino-esp32 core and emits a flashable `.bin`.

Nothing in the library itself is modified — it is consumed as a versioned
GitHub source dependency. All fixes live in the environment (the core version).

## Board

AciduinoV2Box — ESP32-S3-WROOM **N16R8** (16 MB flash, 8 MB **OPI** PSRAM).

```
fqbn: esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=default_8MB,
             USBMode=hwcdc,CDCOnBoot=cdc,LoopCore=1,EventsCore=1
```

`PSRAM=opi` is mandatory. The module is octal PSRAM, so the generic
`PSRAM=enabled` (quad) setting leaves the flash/PSRAM interface mismatched and
the board reset-loops at boot (`rst:0x3 RTC_SW_SYS_RST`).

Avoid GPIOs 26–32 (flash), 33–37 (octal PSRAM), 22–25 (not bonded).
IO19/20 = native USB. IO43/44 = UART0.

## Probes

| Sketch   | What it proves |
|----------|----------------|
| `ProbeA` | Library core (`MIDITransport` / `MIDIHandler` / `MIDIHandlerConfig`) compiles and runs. Lowest layer. |
| `ProbeB` | `UARTConnection` DIN-5 transport on the board's real MIDI pins (RX IO4, TX IO15), echoing NoteOn/Off back out. Needs no USB mode change and no BLE. |

ProbeB logs to the USB CDC serial port at 115200 baud and blinks IO48 as a
heartbeat.

## Pin map used by ProbeB

```
MIDI IN  = IO4  (RX, via optocoupler)
MIDI OUT = IO15 (TX)
LED      = IO48
```

## Build

Push to `main`, or run it manually from the Actions tab. The run publishes
`.bin` / `.elf` / `.map` artifacts named `sketch-build-<ProbeName>`.

The core install is ~1.85 GB of downloads; the first run takes a while, later
runs hit the cache.

## Why CI and not a local build

Local `arduino-cli` was pinned to arduino-esp32 2.0.17, which cannot compile the
library at all: `src/BLEClientConnection.cpp` has four type errors because its
`ESP_ARDUINO_VERSION` 2.x branch assumes a `BLEAddress` constructor and a
`String toString()` that 2.0.17 does not provide. Upgrading to 3.3.12 fixes it
without touching the library.

The upgrade also does not fit locally: 3.3.12 needs ~1.85 GB of downloads
expanding to ~3 GB, and the C: drive had under 5 GB free behind an existing
4.9 GB of `Arduino15` packages.