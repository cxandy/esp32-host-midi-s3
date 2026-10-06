// ESP32_Host_MIDI / MidiMonitor -- the application, not a probe.
//
// Scope, as settled: ESP32_Host_MIDI + OLED + buttons on this board, with a
// BLE -> DIN-5 bridge underneath the monitor. The AciduinoV2Box firmware is
// not part of it; it only supplied the pin map.
//
// What the probes already established, so this does not re-test it:
//   - both transports run together (ProbeF: DIN-5 out, BLE in with rx=118 real
//     phrases from an iPhone, connection stable 305 s, heap flat)
//   - sendBuffer() costs ~31 ms regardless of bus speed, because 1024 bytes at
//     400 kHz is what it costs (ProbeH: clear=2ms send=31ms wireErr=0 at 100
//     and 400 kHz alike). The bus is already at its ceiling; that number is
//     physics, not a bug to fix.
//   - a render is therefore expensive enough to matter: redrawing on every
//     incoming event would spend the whole loop on I2C during a chord.
//
// So this sketch is built around one rule: the screen is redrawn only when
// something actually changed, and never more often than every 150 ms. That is
// the difference between this and ProbeF, which redrew every 200 ms unconditionally
// and so paid 31 ms five times a second whether or not the picture had moved.
//
// Layout: 4 pages, NAV1/NAV2 to move between them, UP/DOWN to scroll the log,
// LEFT toggles BLE -> DIN-5 forwarding. The last page is DIAG: loop counter,
// worst loop gap, worst single serial print, boot reset reason -- evidence that
// is visible on the panel even when nobody is reading the COM port. Every log
// line carries its source: B for BLE, D for DIN-5. Long-press DOWN restarts;
// long-press SHIFT forces download mode (IO0 low).

#include <Arduino.h>
#include <ESP32_Host_MIDI.h>
#include <UARTConnection.h>
#include <BLEConnection.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_system.h>
// gpio_set_pull_mode(), for the electrical self-test: the sketch has to take
// its own pull-up on and off around IO4, and pinMode() has no "no pull"
// variant that does not also change the direction.
#include <driver/gpio.h>

// ---- Board pin map ---------------------------------------------------------
#define PIN_OLED_SCL   11
#define PIN_OLED_SDA   21

#define PIN_SHIFT      0
#define PIN_NAV2       1
#define PIN_NAV1       2
#define PIN_DIR_LEFT   42
#define PIN_DIR_UP     40
#define PIN_DIR_DOWN   41
#define PIN_LED        48

#define MIDI_RX_PIN    4
#define MIDI_TX_PIN    15

#define LONG_PRESS_MS  1500

// BLE -> DIN-5 forwarding: the point of the box is a phone driving whatever is
// plugged into the 5-pin out. DIR LEFT toggles it at runtime, since a bridge
// that can only be turned on by reflashing is not much of a bridge.
static bool fwdEnabled = true;

// 150 ms floor between redraws. A render costs 31 ms of I2C (ProbeH), so this
// caps the screen at ~6.6 redraws/second even during a chord, and leaves the
// loop free for MIDI in between.
#define MIN_RENDER_MS  150

static const char* BLE_NAME = "ESP32-S3 MIDI";

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE,
                                                 PIN_OLED_SCL, PIN_OLED_SDA);

static UARTConnection dinMIDI;
static BLEConnection bleServer;

static bool oledOk = false;
static uint8_t page = 0;
static const uint8_t pageCount = 4;

// ---- Event log -------------------------------------------------------------
// A ring of the most recent messages. 24 entries x 30 bytes is ~720 bytes of
// BSS, nothing next to the 8 MB of PSRAM, and it gives the monitor something
// to show after a burst has already finished -- which is when a musician looks
// at the screen.
#define LOG_N 24
static char logText[LOG_N][30];
static uint8_t logHead = 0;    // slot the next entry goes into
static uint16_t logCount = 0;  // total ever written, to tell "empty" from "wrapped"
static uint16_t scroll = 0;    // 0 = showing the newest entry

static uint64_t lastEventIndex = 0;
static uint32_t rxEvents = 0;
static uint32_t txEvents = 0;
static uint32_t fwdFailures = 0;

static bool bleEverConnected = false;
static unsigned long bleConnectedAtMs = 0;
static unsigned long bleConnectedForS = 0;

// Screen state. dirty means the picture no longer matches the data; the render
// path checks it, so an idle screen costs nothing at all.
static bool dirty = true;
static unsigned long lastRender = 0;
static unsigned long lastBlink = 0, lastBeat = 0, lastTick = 0;

// ---- Loop/print telemetry ---------------------------------------------------
// There to answer one question: when the screen and the buttons go dead, is
// the loop stuck or is only the picture not being drawn? A stuck loop shows up
// as a large gap between iterations, a stalled serial write as a large single
// print -- and both are readable on the DIAG page even when nobody is reading
// the COM port, which is exactly when the fault was last reported.
static uint32_t loopCount = 0;
static uint32_t loopGapMaxUs = 0;
static uint32_t printWorstUs = 0;
static uint32_t printSkipped = 0;
static unsigned long lastLoopUs = 0;
static unsigned long lastMidiPrintMs = 0;
static esp_reset_reason_t bootReason = ESP_RST_UNKNOWN;

// DIN-5 receive is the one path no run has ever exercised (every capture so
// far is `rx=0` on the wire, 1298/1298 events from BLE). If a test shows
// nothing on screen, this says which half to blame: `fifo` is the deepest the
// UART RX FIFO was ever seen holding, sampled before midiHandler.task() eats
// the bytes. fifo > 0 with no event = the parser dropped them; fifo = 0 with a
// device plugged in = nothing is arriving at IO4 at all.
static uint16_t dinRxFifoPeak = 0;

// Where the DIN-5 chain actually breaks, split at the wire.
//
// IO15 is the UART TX pad, IO4 the pad behind the MIDI IN optocoupler; a UART
// idles high, so a pad that reads LOW at rest is not carrying a usable idle
// state. Four counters, because the first pair alone lied: lo15/lo4 both came
// back equal to the loop count -- literally LOW on every sample -- which is
// indistinguishable from a real stuck-low line and from a pad whose input
// buffer is off. So each reading now carries its own referee:
//
//   lo2    -- IO2 is a button with INPUT_PULLUP, so it *must* idle high. If it
//             also counts every loop, digitalRead is the broken part and the
//             other two numbers say nothing.
//   rxav   -- loops in which the UART RX FIFO actually held a byte. A line
//             stuck low is a permanent break: the UART emits 0x00 at 3125 B/s,
//             so rxav climbs ~15k per 5 s window even though the parser
//             discards every one of them (data byte, no running status).
//   txf    -- loops in which the UART TX FIFO was non-empty, i.e. the library's
//             write really reached the peripheral rather than only returning
//             true.
static uint32_t dinTxLowSamples = 0;
static uint32_t dinRxLowSamples = 0;
static uint32_t nav1LowSamples = 0;
static uint32_t rxAvailHits = 0;
static uint32_t txFifoBusyHits = 0;
// The last byte that showed up in the DIN-5 RX FIFO (see the peek() comment
// in loop) and how many times something showed up since the last [DIN] line.
static uint8_t lastPeekByte = 0;
static uint32_t peekHits = 0;
static uint32_t dinSelfTests = 0;
// Result of the last electrical self-test, as percent of samples that read
// LOW: io4 while IO15 is driven high / low, and IO15 read back while driven
// high. They are only refreshed by a test run, so 0/0/0 before `st` moves.
static uint8_t testPctIo4AtHigh = 0;
static uint8_t testPctIo4AtLow = 0;
static uint8_t testPctIo15Read = 0;
// The other two layers of the same electrical test: IO4 while the sketch puts
// its own pull-up on the pad (100% = something on the board pulls it down
// harder than 45 kOhm; 0% = it was only floating), and IO4 read back right
// after the UART claims the pins again (a value that differs from
// testPctIo4AtHigh means the UART's pin setup, not the circuit, sets that
// level).
static uint8_t testPctIo4Pullup = 0;
static uint8_t testPctIo4AtUart = 0;
// IO15 read back while driven low: together with testPctIo15Read (driven high)
// this is the proof that our own drive reaches the pad. Anything other than
// 0% / 100% means the electrical test measured the pad's idle state instead.
static uint8_t testPctIo15ReadLow = 0;
// ADC millivolts on IO4, floating and against the internal pull-up: the
// quantitative answer to "who pulls this down" that digitalRead cannot give.
static uint16_t testMvIo4Float = 0;
static uint16_t testMvIo4Pullup = 0;

// ---- DIN-5 OUT test levels -------------------------------------------------
// Long-press UP pins IO15 (the DIN-5 OUT driver's input) at one steady level
// so a multimeter can be put on the jack without racing a UART burst. The
// periodic self-test is suspended while this is active: that test takes IO15
// over too, and two owners fighting for one pad is how you end up measuring
// neither.
//   0 = off, the UART owns IO15 again
//   1 = held HIGH (mark -- the level that should carry no loop current)
//   2 = held LOW  (space -- the level that should switch loop current on)
//   3 = released (input, nothing driving, for seeing what the board does alone)
static uint8_t txTestMode = 0;

// Hand-held sweep: serviceSerial()'s 'a' walks the three levels by itself.
static uint8_t sweepOn = 0;
static uint8_t sweepPhase = 0;      // 0 = HIGH, 1 = LOW, 2 = hiZ
static uint32_t sweepAtMs = 0;

// The periodic DIN-5 self-test is off by default, and that is a behaviour
// change, not a default value: it ends the UART every 5 s for ~15 ms, which
// throws away whatever arrived in that window, and (with the burst below) it
// plays a phantom C4 at a real synth every 5 s. It earned its keep while the
// DIN-5 IN path was dead; now that the wiring is fixed it would be the thing
// causing glitches. Serial 't' turns it back on when the line needs proving
// again, 'b' adds the six-byte loopback burst.
static uint8_t diagEnabled = 0;
static uint8_t burstEnabled = 0;

// ---- Buttons ---------------------------------------------------------------
struct Button {
  uint8_t pin;
  const char* name;
  bool held = false;
  bool longFired = false;
  unsigned long downAt = 0;
};

static Button btnShift = { PIN_SHIFT,    "SHIFT" };
static Button btnNav1  = { PIN_NAV1,     "NAV1" };
static Button btnNav2  = { PIN_NAV2,     "NAV2" };
static Button btnUp    = { PIN_DIR_UP,   "UP" };
static Button btnDown  = { PIN_DIR_DOWN, "DOWN" };
static Button btnLeft  = { PIN_DIR_LEFT, "LEFT" };

// Returns true on press and again on the long-press threshold, which is what
// makes "hold to restart" work without blocking in delay().
static bool pollButton(Button& b) {
  bool now = (digitalRead(b.pin) == LOW);
  bool fired = false;
  if (now && !b.held) {
    b.held = true; b.downAt = millis(); b.longFired = false; fired = true;
  } else if (now && b.held && !b.longFired &&
             millis() - b.downAt >= LONG_PRESS_MS) {
    b.longFired = true; fired = true;
  } else if (!now && b.held) {
    b.held = false;
  }
  return fired;
}

static const char* txTestName(uint8_t m) {
  switch (m) {
    case 1:  return "HIGH";
    case 2:  return "LOW";
    case 3:  return "hiZ";
    default: return "off";
  }
}

// Changing the OUT level has to be reported the moment it changes, not on the
// next 5 s beat: whoever is holding a meter on the jack is watching the board,
// not a log file. `how` says what asked for it (button, serial command, sweep)
// so the line reads as a cause, not just a state.
static void setTxTestMode(uint8_t m, const char* how) {
  txTestMode = m;
  applyTxTest();
  char tt[80];
  if (m) {
    // analogReadMilliVolts() owns the pad until pinMode() hands it back.
    uint32_t mv4 = analogReadMilliVolts(MIDI_RX_PIN);
    pinMode(MIDI_RX_PIN, INPUT);
    snprintf(tt, sizeof(tt), "%s -> %s io15=%s io4=%umV", how, txTestName(m),
             m == 3 ? "float"
                    : (digitalRead(MIDI_TX_PIN) == HIGH ? "HIGH" : "low"),
             (unsigned)mv4);
  } else {
    snprintf(tt, sizeof(tt), "%s -> off, UART owns IO15", how);
  }
  logAdd(txTestMode ? txTestName(txTestMode) : "OUT: uart");
  monPrint("[TXTEST] ", tt);
  dirty = true;
}

// ---- serial command channel -------------------------------------------------
// The buttons need a hand, and a hand is not always at the board. Single
// characters, no line protocol: whatever arrives is acted on immediately, so a
// host script over USB can drive the same states the buttons do.
static void serviceSerial() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    switch (c) {
      case '1': setTxTestMode(1, "cmd"); break;
      case '0': setTxTestMode(2, "cmd"); break;
      case 'z': case 'Z': setTxTestMode(3, "cmd"); break;
      case 'u': case 'U': sweepOn = 0; setTxTestMode(0, "cmd"); break;
      case 'n': case 'N':
        setTxTestMode((uint8_t)((txTestMode + 1) % 4), "cmd");
        break;
      case 'a': case 'A':
        // Hands-free: walk HIGH -> LOW -> hiZ on its own every 3 s and report
        // the IN pad at each step, so a full sweep is one capture instead of
        // three timed button presses.
        sweepOn = (uint8_t)!sweepOn;
        sweepAtMs = millis();
        if (sweepOn) {
          sweepPhase = 0;
          setTxTestMode(1, "sweep start");
        } else {
          setTxTestMode(0, "sweep stop");
        }
        break;
      case 't': case 'T':
        diagEnabled = (uint8_t)!diagEnabled;
        monPrint("[DIAG] self-test ", diagEnabled ? "ON (5 s)" : "off");
        logAdd(diagEnabled ? "diag: on" : "diag: off");
        dirty = true;
        break;
      case 'b': case 'B':
        burstEnabled = (uint8_t)!burstEnabled;
        // The burst lives inside the self-test, so asking for it implies
        // asking for the test.
        if (burstEnabled) diagEnabled = 1;
        monPrint("[DIAG] loopback burst ",
                 burstEnabled ? "ON (C4 every 5 s)" : "off");
        logAdd(burstEnabled ? "burst: on" : "burst: off");
        dirty = true;
        break;
      case '?': case 'h': case 'H':
        monPrint("[CMD] OUT 1=HIGH 0=LOW z=hiZ u=off n=step a=sweep | "
                 "diag t=test b=burst | ? this", nullptr);
        break;
      default:
        break;   // CR, LF and anything else are ignored on purpose
    }
  }
}

static void serviceSweep() {
  if (!sweepOn) return;
  if ((int32_t)(millis() - sweepAtMs) < 3000) return;
  sweepAtMs = millis();
  sweepPhase = (uint8_t)((sweepPhase + 1) % 3);
  setTxTestMode((uint8_t)(sweepPhase + 1), "sweep");
}

// Put IO15 into (or back out of) the state named by txTestMode. Going back to
// 0 re-opens the UART rather than just re-attaching it: begin() on a running
// port can return early on an unchanged configuration and leave the pad routed
// to GPIO, which would mean "off" never comes back.
static void applyTxTest() {
  if (txTestMode == 0) {
    Serial1.end();
    Serial1.begin(31250, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN);
    return;
  }
  if (txTestMode == 3) {
    pinMode(MIDI_TX_PIN, INPUT);
    gpio_set_pull_mode((gpio_num_t)MIDI_TX_PIN, GPIO_FLOATING);
    return;
  }
  pinMode(MIDI_TX_PIN, OUTPUT);
  digitalWrite(MIDI_TX_PIN, txTestMode == 1 ? HIGH : LOW);
}

// ---- Print path -------------------------------------------------------------
// These two live after pollButton on purpose: the Arduino preprocessor puts
// its generated prototypes before the first function in the file, and that
// point has to stay below struct Button or pollButton's prototype is emitted
// with no known argument types and the build fails.

static const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "ext";
    case ESP_RST_SW:        return "sw";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "intWdt";
    case ESP_RST_TASK_WDT:  return "taskWdt";
    case ESP_RST_WDT:       return "wdt";
    case ESP_RST_DEEPSLEEP: return "sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    case ESP_RST_USB:       return "usb";
    case ESP_RST_JTAG:      return "jtag";
    default:                return "other";
  }
}

// Every print in this sketch goes through here, and it is timed.
//
// HWCDC::write (arduino-esp32 3.3.12, cores/esp32/HWCDC.cpp:540) is not
// free-running: when the CDC link counts as connected it pushes into a
// 256-byte ring and, if the host is not draining it, waits tx_timeout_ms per
// attempt for up to 20 attempts -- 2 s per call at the 100 ms default, during
// which nothing else in loop() runs. The buttons are polled there, so a burst
// of output with a slow reader looks exactly like "buttons stopped
// responding". setup() lowers the timeout to 5 ms; this records what the worst
// single call actually cost.
//
// The timeout alone is not enough. HWCDC::write's wait is 20 x tx_timeout_ms
// *per call*, and this build measured exactly that (100 ms) as soon as MIDI
// started arriving with nothing draining the port -- so the fix is to not
// write at all when the ring has no room for the line, and count it instead.
static void monPrint(const char* prefix, const char* body) {
  // One buffer, one write. prefix and body used to be two separate write()
  // calls, and the host can stop draining between them: the prefix lands, the
  // body is short-written away, and the capture shows "[MIDI] " followed by
  // the next line glued to it -- which reads exactly like memory corruption
  // and nearly cost this investigation a wrong turn.
  char buf[240];
  int n = snprintf(buf, sizeof(buf), "%s%s\n", prefix, body ? body : "");
  if (n <= 0 || (size_t)n >= sizeof(buf)) {
    printSkipped++;
    return;
  }
  size_t need = (size_t)n;
  if ((size_t)Serial.availableForWrite() < need) {
    printSkipped++;
    return;
  }
  unsigned long t0 = micros();
  size_t wrote = Serial.write((const uint8_t*)buf, need);
  unsigned long took = micros() - t0;
  if (took > printWorstUs) printWorstUs = took;
  if (wrote < need) printSkipped++;  // backpressure ate the tail; counted, not hidden
}

static void logAdd(const char* text) {
  snprintf(logText[logHead], sizeof(logText[0]), "%s", text);
  logHead = (logHead + 1) % LOG_N;
  if (logCount < LOG_N * 4) logCount++;  // saturate; only 0 vs >0 is meaningful
  scroll = 0;                            // a new event pulls the view back to now
  dirty = true;
}

// ---- Setup -----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // See monPrint(): the HWCDC default makes one undrained print cost up to 2 s
  // of loop time, which is a frozen UI with a perfectly healthy firmware.
  Serial.setTxTimeoutMs(5);
  delay(300);
  monPrint("[MIDI-MONITOR] ESP32_Host_MIDI + OLED + buttons", nullptr);
  // Say how to drive the OUT levels without a hand, so a host script (or a
  // person at a terminal) does not have to read the source to find that out.
  monPrint("[MIDI-MONITOR] type ? for the serial command list", nullptr);
  // The DIN-5 self-test is off by default, so say how to get it back rather
  // than leaving its absence to be discovered as a missing [TEST] line.
  monPrint("[DIAG] self-test off -- type t (t=repeat, b=+loopback burst)",
           nullptr);
  bootReason = esp_reset_reason();
  char rst[48];
  snprintf(rst, sizeof(rst), "reset=%s (0x%x)", resetName(bootReason),
           (unsigned)bootReason);
  monPrint("[MIDI-MONITOR] ", rst);

  const uint8_t pins[] = { PIN_SHIFT, PIN_NAV1, PIN_NAV2,
                           PIN_DIR_UP, PIN_DIR_DOWN, PIN_DIR_LEFT };
  for (uint8_t p : pins) pinMode(p, INPUT_PULLUP);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "starting...");
  u8g2.sendBuffer();
  delay(50);
  // beginTransmission() returns void on arduino-esp32; the ACK is
  // endTransmission()==0, since Wire.beginTransmission() itself tells you
  // nothing about whether the device answered.
  Wire.beginTransmission(0x3C);
  oledOk = (Wire.endTransmission() == 0);
  monPrint("[MIDI-MONITOR] OLED at 0x3C: ",
           oledOk ? "responding" : "no ACK");

  // Transport 1: DIN-5 wire MIDI.
  dinMIDI.begin(Serial1, MIDI_RX_PIN, MIDI_TX_PIN);
  midiHandler.addTransport(&dinMIDI);

  // Transport 2: BLE as peripheral. The board advertises the MIDI service and
  // the phone connects in as central -- the direction iOS actually supports,
  // since iOS forbids apps from advertising as a BLE MIDI peripheral.
  bleServer.begin(BLE_NAME);
  midiHandler.addTransport(&bleServer);

  MIDIHandlerConfig cfg;
  // 64 rather than the default 20: while forwarding, an event sitting in the
  // queue is a note that has not reached the synth yet, so queue depth would
  // be output latency -- except this sketch never removes what it has read.
  // There is no per-event pop in the library API, and clearQueue() resets the
  // event indices the display de-duplicates on (MIDIHandler.cpp:321), so the
  // queue saturates at maxEvents and then evicts the oldest. That is harmless
  // here: drainQueue() runs every loop iteration, so anything evicted was
  // already on screen. What it means is that the number is queue pressure, not
  // latency -- so it is not shown on the OLED, and `rx` is the field to read.
  cfg.maxEvents = 64;
  cfg.bleName = BLE_NAME;
  midiHandler.begin(cfg);

  pinMode(PIN_LED, OUTPUT);

  logAdd(oledOk ? "monitor ready" : "OLED MISSING");
  monPrint("[MIDI-MONITOR] advertising as ", BLE_NAME);
}

// ---- Incoming events -------------------------------------------------------
static void drainQueue() {
  const auto& queue = midiHandler.getQueue();
  for (const auto& ev : queue) {
    if (ev.index <= lastEventIndex) continue;
    lastEventIndex = ev.index;
    rxEvents++;

    char noteBuf[8];
    char line[30];
    switch (ev.statusCode) {
      case MIDI_NOTE_ON:
      case MIDI_NOTE_OFF:
        MIDIHandler::noteWithOctave(ev.noteNumber, noteBuf, sizeof(noteBuf));
        // NoteOff arrives with velocity 0 on most gear, so show it as a
        // release rather than "NoteOff vel 0" -- this is a monitor, and the
        // musician reading it cares which key went up, not the release speed.
        if (ev.statusCode == MIDI_NOTE_ON && ev.velocity7 > 0)
          snprintf(line, sizeof(line), "ON  %s v%d c%d", noteBuf,
                   ev.velocity7, ev.channel0 + 1);
        else
          snprintf(line, sizeof(line), "OFF %s   c%d", noteBuf,
                   ev.channel0 + 1);
        break;
      case MIDI_CONTROL_CHANGE:
        snprintf(line, sizeof(line), "CC %d =%3d c%d", ev.noteNumber,
                 ev.velocity7, ev.channel0 + 1);
        break;
      case MIDI_PROGRAM_CHANGE:
        snprintf(line, sizeof(line), "PC %d   c%d", ev.noteNumber,
                 ev.channel0 + 1);
        break;
      case MIDI_PITCH_BEND:
        snprintf(line, sizeof(line), "PB %d c%d", ev.pitchBend14,
                 ev.channel0 + 1);
        break;
      default: {
        const char* what = MIDIHandler::statusName(ev.statusCode);
        snprintf(line, sizeof(line), "%s", what ? what : "?");
        break;
      }
    }
    // Source marker. Nothing else on screen distinguishes a note that arrived
    // over DIN-5 from one the phone sent, and knowing which input moved is the
    // whole point of a monitor. B = BLE peripheral, D = DIN-5 wire.
    char tagged[40];
    snprintf(tagged, sizeof(tagged), "%c %s",
             ev.source == &dinMIDI ? 'D' : 'B', line);
    logAdd(tagged);
    // Serial output is a measured cost here, not a free one: 25 lines a second
    // is more than anyone can read, and every line beyond that is time taken
    // away from the loop (see monPrint). The ones not written are counted as
    // `skip` on the DIAG page instead of silently happening.
    if (millis() - lastMidiPrintMs >= 40) {
      lastMidiPrintMs = millis();
      monPrint("[MIDI] ", tagged);
    } else {
      printSkipped++;
    }

    // ---- BLE -> DIN-5 bridge ------------------------------------------------
    // Every transport the handler is holding except the one this came in on.
    // In practice that means "BLE in -> DIN out", which is the direction that
    // matters: the phone is the sequencer, the DIN-5 jack drives the synth.
    //
    // Two things this depends on, both verified in the library source:
    //   - channel numbering. The queue stores channel0 as 0..15 (MIDI spec
    //     convention); send* takes 1..16 and rejects 0 outright
    //     (MIDIHandler.cpp:688). Passing channel0 through unchanged would make
    //     every MIDI channel 1 note return false and never leave the board --
    //     silently, since it just looks like a send that did not happen.
    //   - target is passed explicitly rather than left to default. With no
    //     target sendNoteOn walks every transport and stops at the first that
    //     accepts (MIDIHandler.cpp:692) -- DIN today only because dinMIDI was
    //     registered first. Explicit means reordering addTransport() later
    //     cannot turn this into a loop that echoes the phone's notes back.
    //
    // The library also normalizes NoteOn@vel0 to MIDI_NOTE_OFF on the way in
    // (MIDIHandler.cpp:664), so statusCode forwards as-is and the synth never
    // sees an ambiguous release.
    if (fwdEnabled && ev.source != &dinMIDI) {
      const uint8_t ch = ev.channel0 + 1;   // queue 0..15 -> send* 1..16
      bool ok = false;
      bool supported = true;
      switch (ev.statusCode) {
        case MIDI_NOTE_ON:
          ok = midiHandler.sendNoteOn(ch, ev.noteNumber, ev.velocity7, &dinMIDI);
          break;
        case MIDI_NOTE_OFF:
          ok = midiHandler.sendNoteOff(ch, ev.noteNumber, ev.velocity7, &dinMIDI);
          break;
        case MIDI_CONTROL_CHANGE:
          ok = midiHandler.sendControlChange(ch, ev.noteNumber, ev.velocity7,
                                             &dinMIDI);
          break;
        case MIDI_PROGRAM_CHANGE:
          ok = midiHandler.sendProgramChange(ch, ev.noteNumber, &dinMIDI);
          break;
        case MIDI_PITCH_BEND:
          // Queue carries the MIDI 1.0 14-bit form, 0..16383 with centre at
          // 8192; sendPitchBend wants -8192..8191.
          ok = midiHandler.sendPitchBend(ch, (int)ev.pitchBend14 - 8192,
                                         &dinMIDI);
          break;
        default:
          // Channel/poly pressure have no send helper in the library, and
          // SysEx never reaches this function -- it lives in its own queue
          // (MIDISysExEvent) with its own callback. Both are left alone rather
          // than hand-rolled: nothing here needs aftertouch or SysEx, and a
          // half-forwarded message is worse than none.
          supported = false;
          break;
      }
      if (ok) txEvents++;
      else if (supported) fwdFailures++;
    }
  }
}

// ---- OLED ------------------------------------------------------------------
static void row(uint8_t y, const char* fmt, ...) {
  char buf[24];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  u8g2.drawStr(0, y, buf);
}

// Index of the log entry i rows above the scroll position; 0 is the newest.
static uint8_t logIndex(uint16_t i) {
  uint16_t back = scroll + i;
  if (back >= logCount) back = logCount ? logCount - 1 : 0;
  return (logHead + LOG_N - 1 - (back % LOG_N)) % LOG_N;
}

static void render() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  switch (page) {
    case 0: {
      // The monitor itself: four most recent events, newest at the bottom,
      // the way a console reads. Header carries the running total so a screen
      // full of nothing is distinguishable from one that stopped updating,
      // plus the forwarding state -- because a bridge that silently mutes is
      // worse than one that was never wired up.
      row(11, "rx=%lu tx=%lu %s", (unsigned long)rxEvents,
          (unsigned long)txEvents, fwdEnabled ? "FWD" : "MUTE");
      u8g2.drawHLine(0, 14, 128);
      if (logCount == 0) {
        row(30, "no MIDI yet");
        row(44, "pair a phone or");
        row(56, "patch in DIN-5");
      } else {
        const uint8_t lines = 4;
        for (uint8_t i = 0; i < lines; i++) {
          uint16_t back = scroll + (lines - 1 - i);
          if (back >= logCount) break;
          row((uint8_t)(26 + i * 12), "%s", logText[logIndex(back)]);
        }
        if (scroll) row(11, "rx=%lu tx=%lu %s ^", (unsigned long)rxEvents,
                        (unsigned long)txEvents, fwdEnabled ? "FWD" : "MUTE");
      }
      break;
    }
    case 1: {
      bool conn = bleServer.isConnected();
      row(11, "STATUS");
      u8g2.drawHLine(0, 14, 128);
      if (conn) row(26, "ble    conn %lus", bleConnectedForS);
      else      row(26, "ble    advertising");
      // fifo = deepest the DIN-5 RX FIFO ever got before the library drained
      // it. Forwarding state lives on page 0's header and in [ALIVE].
      if (txTestMode) {
        // Multimeter mode: where IO15 is pinned and what the IN pad says at
        // that moment, so the meter and the screen can be read together.
        row(38, "txtest %s io15=%s", txTestName(txTestMode),
            txTestMode == 3 ? "f"
                            : (digitalRead(MIDI_TX_PIN) == HIGH ? "H" : "L"));
        row(50, "io4=%umV", (unsigned)analogReadMilliVolts(MIDI_RX_PIN));
        pinMode(MIDI_RX_PIN, INPUT);
      } else {
        row(38, "din5 fifo=%u tx=%lu", (unsigned)dinRxFifoPeak,
            (unsigned long)txEvents);
        // Events shown, and forwarding failures. Not the library's queue
        // depth: that saturates at maxEvents because this sketch never pops
        // what it has read (see cfg.maxEvents), so it would sit at 64 forever
        // and mean nothing to a person reading the screen.
        row(50, "din rx %lu f%u", (unsigned long)rxEvents,
            (unsigned)fwdFailures);
      }
      row(62, "heap %u %lus %s", (unsigned)ESP.getFreeHeap(),
          millis() / 1000, diagEnabled ? "DIAG" : "");
      break;
    }
    case 2:
      row(11, "KEYS");
      u8g2.drawHLine(0, 14, 128);
      row(26, "NAV1/2 page  L=fwd");
      row(38, "UP/DN scroll B/D=src");
      row(50, "hold UP or type: OUT");
      row(62, "D:restart S:download");
      break;
    case 3:
      row(11, "DIAG");
      u8g2.drawHLine(0, 14, 128);
      // loop advances every iteration, gapmax is the worst gap ever seen
      // between two of them: this page says on its own whether the loop is
      // alive, with nobody reading the COM port required.
      row(26, "loop %lu", (unsigned long)loopCount);
      row(38, "gapmax %lums", (unsigned long)(loopGapMaxUs / 1000));
      row(50, "print %lums x%lu", (unsigned long)(printWorstUs / 1000),
          (unsigned long)printSkipped);
      row(62, "boot %s", resetName(bootReason));
      break;
  }

  // 31 ms on the wire, every call -- the cost measured by ProbeH and not
  // reducible from software. Which is exactly why the caller only calls this
  // when something changed.
  u8g2.sendBuffer();
}

// ---- Loop ------------------------------------------------------------------
void loop() {
  // Gap between iterations, measured before anything else can delay it. This
  // is the number that answers "is the loop stuck?" for any cause at all --
  // serial, I2C, BLE -- not only the ones we happened to think of.
  uint32_t nowUs = micros();
  if (lastLoopUs) {
    uint32_t gap = nowUs - (uint32_t)lastLoopUs;
    if (gap > loopGapMaxUs) loopGapMaxUs = gap;
  }
  lastLoopUs = nowUs;
  loopCount++;

  // Sample before the library drains it: at 31250 baud a byte sits in the FIFO
  // for ~320 us while the loop runs every ~14 us, so this cannot miss.
  int pending = Serial1.available();
  if (pending > (int)dinRxFifoPeak) dinRxFifoPeak = (uint16_t)pending;

  if (digitalRead(MIDI_TX_PIN) == LOW) dinTxLowSamples++;
  if (digitalRead(MIDI_RX_PIN) == LOW) dinRxLowSamples++;
  if (digitalRead(PIN_NAV1) == LOW) nav1LowSamples++;
  if (pending > 0) {
    rxAvailHits++;
    // What exactly arrived? The parser discards a data byte that has no status
    // byte in front of it (UARTConnection.cpp:136), so a byte can enter the
    // FIFO and leave no trace in the monitor. peek() leaves it for the library
    // to consume; a lone 0x00 means "break", 0x90 means one of our own six
    // bytes actually came back.
    int v = Serial1.peek();
    if (v >= 0) {
      lastPeekByte = (uint8_t)v;
      peekHits++;
    }
  }
  // 128 is the hardware FIFO depth: anything less means bytes are sitting in
  // the transmitter right now. It also reads 0 before the UART is begun, so
  // txf pinned at the loop count is itself a finding, not a measurement.
  if (Serial1.availableForWrite() < 128) txFifoBusyHits++;

  midiHandler.task();
  drainQueue();

  // Host commands and the hand-free sweep run before anything else claims
  // IO15, so a command that arrives mid-iteration still wins that iteration.
  serviceSerial();
  serviceSweep();

  // ---- DIN-5 self-test: the firmware tests its own loopback ---------------
  // Nothing here needs a phone, a synth or a second device, and it runs in two
  // layers because the serial one alone cannot separate "no signal on the
  // wire" from "UART not wired to that pad":
  //
  //   1. Electrical. Serial1.end() releases IO15, the sketch drives it high
  //      for 1.5 ms and low for 1.5 ms while sampling IO4. This bypasses UART,
  //      parser and cable protocol entirely -- if the loopback (driver -> jack
  //      -> cable -> opto -> IO4) works, io4 must follow. io15read is the same
  //      read-back on IO15, so a pad whose input buffer cannot be read is
  //      visible rather than silently poisoning the number.
  //   2. Serial. The UART is re-opened on 4/15 and six bytes go out. A working
  //      path brings them back as `D` lines with rx climbing while tx stands
  //      still (the test never enters tx), and rxav/fifo moving with them.
  //
  // The 400 ms quiet gate keeps the burst from interleaving with a forwarded
  // note, and drainQueue() only forwards events whose source is not dinMIDI,
  // so what comes back is displayed, not echoed.
  {
    static uint32_t lastRxCount = 0;
    static unsigned long quietSinceMs = 0;
    static unsigned long lastSelfTestMs = 0;
    if (rxEvents != lastRxCount) {
      lastRxCount = rxEvents;
      quietSinceMs = millis();
    } else if (quietSinceMs == 0) {
      quietSinceMs = millis();
    }
    if (diagEnabled && txTestMode == 0 && !sweepOn &&
        millis() - quietSinceMs > 400 && millis() - lastSelfTestMs >= 5000) {
      lastSelfTestMs = millis();

      // --- layer 1: electrical ------------------------------------------------
      // Seven readings, each 1.5 ms of samples, chosen so that every failure
      // mode has its own number instead of sharing one ambiguous "it reads
      // low":
      //   io4hi   IO4 while IO15 is driven high (mark) -- who holds it?
      //   io4lo   IO4 while IO15 is driven low  (space) -- does it follow?
      //   io4pu   IO4 with a pull-up on top     -- driven low, or just floating?
      //   io15hi  IO15 read back while driven high -- must be 0%
      //   io15lo  IO15 read back while driven low  -- must be 100%
      //   io4uart IO4 immediately after the UART is re-opened -- if this
      //           differs from io4hi, the UART's own pin setup is what changes
      //           the pad, not the circuit.
      // io15hi/io15lo are the check that the other five mean anything: driving
      // through gpio_set_direction() without pinMode() leaves the peripheral
      // manager unregistered and digitalWrite() then does nothing at all
      // (esp32-hal-gpio.c:182), which reports a perfectly plausible-looking
      // "I drove it high" as a pad that never moved.
      //
      // pinMode() is used for exactly that reason -- it registers the pin as a
      // GPIO bus, and its mode mask turns OUTPUT into GPIO_MODE_INPUT_OUTPUT,
      // so the pad keeps its input buffer while we drive it.
      Serial1.end();
      // pinMode() does everything this needs: it registers the pin with the
      // peripheral manager (which is what makes digitalWrite() do anything at
      // all, esp32-hal-gpio.c:182) and gpio_config() re-selects the pad as
      // GPIO (idf gpio.c:427). Its mode mask turns OUTPUT into
      // GPIO_MODE_INPUT_OUTPUT, so the pad keeps its input buffer while we
      // drive it -- which is what lets us read our own drive back.
      pinMode(MIDI_TX_PIN, OUTPUT);
      pinMode(MIDI_RX_PIN, INPUT);
      gpio_set_pull_mode((gpio_num_t)MIDI_RX_PIN, GPIO_FLOATING);   // no help from us

      auto samplePct = [](uint8_t pin) {
        uint32_t lows = 0, n = 0;
        uint32_t t0 = micros();
        while (micros() - t0 < 1500) {
          n++;
          if (digitalRead(pin) == LOW) lows++;
        }
        return (uint8_t)(lows * 100 / (n ? n : 1));
      };

      digitalWrite(MIDI_TX_PIN, HIGH);
      testPctIo4AtHigh = samplePct(MIDI_RX_PIN);
      testPctIo15Read  = samplePct(MIDI_TX_PIN);

      digitalWrite(MIDI_TX_PIN, LOW);
      testPctIo4AtLow = samplePct(MIDI_RX_PIN);
      testPctIo15ReadLow = samplePct(MIDI_TX_PIN);

      digitalWrite(MIDI_TX_PIN, HIGH);
      gpio_set_pull_mode((gpio_num_t)MIDI_RX_PIN, GPIO_PULLUP_ONLY);
      testPctIo4Pullup = samplePct(MIDI_RX_PIN);
      gpio_set_pull_mode((gpio_num_t)MIDI_RX_PIN, GPIO_FLOATING);

      // How hard is IO4 being held down? digitalRead only says "< Vih"; the
      // ADC says how much. Against the ~45 kOhm internal pull-up a board
      // pull-down lands around 0.5 V, a saturated opto collector near 0 V,
      // and a healthy idle line at 3.3 V. Last measurements of the block,
      // because analogReadMilliVolts() takes the pad over until pinMode()
      // runs again.
      testMvIo4Float = analogReadMilliVolts(MIDI_RX_PIN);
      gpio_set_pull_mode((gpio_num_t)MIDI_RX_PIN, GPIO_PULLUP_ONLY);
      testMvIo4Pullup = analogReadMilliVolts(MIDI_RX_PIN);
      gpio_set_pull_mode((gpio_num_t)MIDI_RX_PIN, GPIO_FLOATING);
      pinMode(MIDI_RX_PIN, INPUT);

      // --- layer 2: serial ----------------------------------------------------
      // Re-open rather than begin(): end() released the pad routing, and the
      // library's own begin() would return early on its _initialized guard.
      Serial1.begin(31250, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN);
      testPctIo4AtUart = samplePct(MIDI_RX_PIN);

      static const uint8_t msg[6] = { 0x90, 60, 100, 0x80, 60, 0 };
      if (burstEnabled) Serial1.write(msg, sizeof(msg));
      dinSelfTests++;
    }
  }

  if (bleServer.isConnected()) {
    if (!bleEverConnected) {
      bleEverConnected = true;
      bleConnectedAtMs = millis();
      logAdd("BLE connected");
      monPrint("[BLE] ", "central connected");
    }
    bleConnectedForS = (millis() - bleConnectedAtMs) / 1000;
  } else if (bleEverConnected) {
    bleEverConnected = false;
    bleConnectedForS = 0;
    logAdd("BLE dropped");
    monPrint("[BLE] ", "central disconnected");
  }

  // Button presses are logged, not just acted on: a capture that shows no
  // [UI] line while a page is not changing is the difference between "input
  // path is dead" and "the screen never redrew".
  if (pollButton(btnNav1)) {
    page = (page + 1) % pageCount;
    char what[12];
    snprintf(what, sizeof(what), "page %u", (unsigned)page);
    monPrint("[UI] NAV1 -> ", what);
    dirty = true;
  }
  if (pollButton(btnNav2)) {
    page = (page + pageCount - 1) % pageCount;
    char what[12];
    snprintf(what, sizeof(what), "page %u", (unsigned)page);
    monPrint("[UI] NAV2 -> ", what);
    dirty = true;
  }
  if (pollButton(btnLeft)) {
    fwdEnabled = !fwdEnabled;
    logAdd(fwdEnabled ? "fwd ON  -> DIN" : "fwd OFF");
    monPrint("[FWD] BLE -> DIN-5 ", fwdEnabled ? "enabled" : "muted");
    dirty = true;
  }
  if (pollButton(btnUp)) {
    if (btnUp.longFired) {
      // Hold UP to walk IO15 through the OUT test levels -- see txTestMode.
      // The short press it also fires on the way down still scrolls, which is
      // what makes this usable one-handed at a bench. Serial 'n' does the same
      // thing without a hand.
      setTxTestMode((uint8_t)((txTestMode + 1) % 4), "btn UP");
    } else {
      if (scroll + 4 < logCount) scroll++;
      monPrint("[UI] ", "UP");
      dirty = true;
    }
  }
  bool downNow = pollButton(btnDown);
  if (downNow && !btnDown.longFired) monPrint("[UI] ", "DOWN");
  if (downNow && btnDown.longFired) {
    monPrint("[UI] ", "restarting sketch");
    delay(50);
    ESP.restart();
  }
  bool shiftNow = pollButton(btnShift);
  if (shiftNow && !btnShift.longFired) monPrint("[UI] ", "SHIFT");
  if (shiftNow && btnShift.longFired) {
    monPrint("[UI] ", "forcing download mode: IO0 low + restart");
    pinMode(PIN_SHIFT, OUTPUT);
    digitalWrite(PIN_SHIFT, LOW);
    delay(50);
    ESP.restart();
  }

  if (millis() - lastBlink > 500) {
    lastBlink = millis();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
  // The status and DIAG pages show counters, so they need a heartbeat even
  // when nothing else happens -- DIAG in particular must keep ticking for it
  // to mean anything; the log page does not, and stays untouched to save the
  // bus.
  if ((page == 1 || page == 3) && millis() - lastTick > 1000) {
    lastTick = millis();
    dirty = true;
  }

  if (dirty && millis() - lastRender >= MIN_RENDER_MS) {
    lastRender = millis();
    dirty = false;
    if (oledOk) render();
  }

  if (millis() - lastBeat > 5000) {
    lastBeat = millis();
    // One line, built first and printed once, so the timing monPrint records
    // covers the whole thing. gap/print/skip are the diagnostic fields: gap is
    // the worst loop stall ever seen, print the worst single serial call, skip
    // how many event lines were dropped by the rate limiter.
    // lo15/lo4 are the physical-layer probe: LOW samples seen on the UART TX
    // pad and on the pad behind the MIDI IN optocoupler during this 5 s
    // window, with lo2 as the referee (IO2 has a pull-up, so it must stay 0)
    // and rxav/txf as the UART-side truth. The line is separate from [ALIVE]
    // only for length; it prints on the same beat.
    static uint32_t lastLo15 = 0, lastLo4 = 0, lastLo2 = 0;
    static uint32_t lastRxAv = 0, lastTxF = 0;
    char din[200];
    snprintf(din, sizeof(din),
             "[DIN] lo15=%lu lo4=%lu lo2=%lu rxav=%lu txf=%lu pk=%02X:%lu",
             (unsigned long)(dinTxLowSamples - lastLo15),
             (unsigned long)(dinRxLowSamples - lastLo4),
             (unsigned long)(nav1LowSamples - lastLo2),
             (unsigned long)(rxAvailHits - lastRxAv),
             (unsigned long)(txFifoBusyHits - lastTxF),
             (unsigned)lastPeekByte, (unsigned long)peekHits);
    lastLo15 = dinTxLowSamples;
    lastLo4 = dinRxLowSamples;
    lastLo2 = nav1LowSamples;
    lastRxAv = rxAvailHits;
    lastTxF = txFifoBusyHits;
    peekHits = 0;
    monPrint(din, nullptr);

    // Only worth printing while the test is the thing running: with diag off
    // these numbers are frozen history, and a frozen number on a live log is
    // worse than no number at all.
    if (diagEnabled) {
      char tst[200];
      snprintf(tst, sizeof(tst),
               "[TEST] st=%lu io4hi=%u%% io4lo=%u%% io4pu=%u%% io15hi=%u%% "
               "io15lo=%u%% io4uart=%u%% mv=%u mvpu=%u",
               (unsigned long)dinSelfTests, (unsigned)testPctIo4AtHigh,
               (unsigned)testPctIo4AtLow, (unsigned)testPctIo4Pullup,
               (unsigned)testPctIo15Read, (unsigned)testPctIo15ReadLow,
               (unsigned)testPctIo4AtUart, (unsigned)testMvIo4Float,
               (unsigned)testMvIo4Pullup);
      monPrint(tst, nullptr);
    }

    // While IO15 is held for the multimeter: report what the pad is actually
    // driving and what the IN side sees at that moment, so the meter reading
    // and this line can be read against each other. analogReadMilliVolts()
    // takes IO4 over until pinMode() puts it back as a GPIO.
    if (txTestMode) {
      uint32_t mv4 = analogReadMilliVolts(MIDI_RX_PIN);
      pinMode(MIDI_RX_PIN, INPUT);
      char ttx[200];
      snprintf(ttx, sizeof(ttx), "[TXTEST] mode=%s io15=%s io4=%umV",
               txTestName(txTestMode),
               txTestMode == 3 ? "float"
                               : (digitalRead(MIDI_TX_PIN) == HIGH ? "HIGH"
                                                                   : "low"),
               (unsigned)mv4);
      monPrint(ttx, nullptr);
    }

    char al[200];
    snprintf(al, sizeof(al),
             "[ALIVE] up=%lus heap=%u rx=%lu tx=%lu fail=%u queue=%u "
             "ble=%s fwd=%s log=%u/%u loop=%lu gap=%lums print=%lums skip=%lu "
             "fifo=%u boot=%s",
             millis() / 1000, (unsigned)ESP.getFreeHeap(),
             (unsigned long)rxEvents, (unsigned long)txEvents,
             (unsigned)fwdFailures,
             (unsigned)midiHandler.getQueue().size(),
             bleServer.isConnected() ? "connected" : "advertising",
             fwdEnabled ? "on" : "off",
             (unsigned)(logCount < LOG_N ? logCount : LOG_N), LOG_N,
             (unsigned long)loopCount, (unsigned long)(loopGapMaxUs / 1000),
             (unsigned long)(printWorstUs / 1000), (unsigned long)printSkipped,
             (unsigned)dinRxFifoPeak, resetName(bootReason));
    monPrint(al, nullptr);
  }
}
