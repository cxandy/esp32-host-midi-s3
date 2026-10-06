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
// Layout: 3 pages, NAV1/NAV2 to move between them, UP/DOWN to scroll the log,
// LEFT toggles BLE -> DIN-5 forwarding. Long-press DOWN restarts; long-press
// SHIFT forces download mode (IO0 low).

#include <Arduino.h>
#include <ESP32_Host_MIDI.h>
#include <UARTConnection.h>
#include <BLEConnection.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_system.h>

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
static const uint8_t pageCount = 3;

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
  delay(300);
  Serial.println();
  Serial.println("[MIDI-MONITOR] ESP32_Host_MIDI + OLED + buttons");

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
  Serial.print("[MIDI-MONITOR] OLED at 0x3C: ");
  Serial.println(oledOk ? "responding" : "no ACK");

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
  // queue is a note that has not reached the synth yet, so queue depth is
  // literally output latency. A 16th-note run at 120 bpm is ~8 notes a second;
  // 64 gives headroom for a chord-heavy burst without dropping the oldest.
  cfg.maxEvents = 64;
  cfg.bleName = BLE_NAME;
  midiHandler.begin(cfg);

  pinMode(PIN_LED, OUTPUT);

  logAdd(oledOk ? "monitor ready" : "OLED MISSING");
  Serial.printf("[MIDI-MONITOR] advertising as \"%s\"\n", BLE_NAME);
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
    logAdd(line);
    Serial.print("[MIDI] ");
    Serial.println(line);

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
      row(38, "din5   tx=%lu%s", (unsigned long)txEvents,
          fwdEnabled ? "  fwd ON" : "  MUTE");
      row(50, "queue  %u  fail %u",
          (unsigned)midiHandler.getQueue().size(), (unsigned)fwdFailures);
      row(62, "heap   %u  up %lus", (unsigned)ESP.getFreeHeap(),
          millis() / 1000);
      break;
    }
    case 2:
      row(11, "KEYS");
      u8g2.drawHLine(0, 14, 128);
      row(26, "NAV1/2 page  L=fwd");
      row(38, "UP/DOWN scroll log");
      row(50, "hold DOWN  restart");
      row(62, "hold SHIFT download");
      break;
  }

  // 31 ms on the wire, every call -- the cost measured by ProbeH and not
  // reducible from software. Which is exactly why the caller only calls this
  // when something changed.
  u8g2.sendBuffer();
}

// ---- Loop ------------------------------------------------------------------
void loop() {
  midiHandler.task();
  drainQueue();

  if (bleServer.isConnected()) {
    if (!bleEverConnected) {
      bleEverConnected = true;
      bleConnectedAtMs = millis();
      logAdd("BLE connected");
      Serial.println("[BLE] central connected");
    }
    bleConnectedForS = (millis() - bleConnectedAtMs) / 1000;
  } else if (bleEverConnected) {
    bleEverConnected = false;
    bleConnectedForS = 0;
    logAdd("BLE dropped");
    Serial.println("[BLE] central disconnected");
  }

  if (pollButton(btnNav1)) { page = (page + 1) % pageCount; dirty = true; }
  if (pollButton(btnNav2)) { page = (page + pageCount - 1) % pageCount; dirty = true; }
  if (pollButton(btnLeft)) {
    fwdEnabled = !fwdEnabled;
    logAdd(fwdEnabled ? "fwd ON  -> DIN" : "fwd OFF");
    Serial.printf("[FWD] BLE -> DIN-5 %s\n", fwdEnabled ? "enabled" : "muted");
    dirty = true;
  }
  if (pollButton(btnUp)) {
    if (scroll + 4 < logCount) scroll++;
    dirty = true;
  }
  if (pollButton(btnDown) && btnDown.longFired) {
    Serial.println("[UI] restarting sketch");
    delay(50);
    ESP.restart();
  }
  if (pollButton(btnShift) && btnShift.longFired) {
    Serial.println("[UI] forcing download mode: IO0 low + restart");
    pinMode(PIN_SHIFT, OUTPUT);
    digitalWrite(PIN_SHIFT, LOW);
    delay(50);
    ESP.restart();
  }

  if (millis() - lastBlink > 500) {
    lastBlink = millis();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
  // The status page counts seconds, so it needs a heartbeat even when nothing
  // else happens; the log page does not, and stays untouched to save the bus.
  if (page == 1 && millis() - lastTick > 1000) {
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
    Serial.printf("[ALIVE] up=%lus heap=%u rx=%lu tx=%lu fail=%u queue=%u "
                  "ble=%s fwd=%s log=%u/%u\n",
                  millis() / 1000, (unsigned)ESP.getFreeHeap(),
                  (unsigned long)rxEvents, (unsigned long)txEvents,
                  (unsigned)fwdFailures,
                  (unsigned)midiHandler.getQueue().size(),
                  bleServer.isConnected() ? "connected" : "advertising",
                  fwdEnabled ? "on" : "off",
                  (unsigned)(logCount < LOG_N ? logCount : LOG_N), LOG_N);
  }
}
