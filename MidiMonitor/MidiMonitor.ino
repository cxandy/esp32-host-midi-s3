// ESP32_Host_MIDI / MidiMonitor -- the application, not a probe.
//
// Scope, as settled: ESP32_Host_MIDI + OLED + buttons on this board. The
// AciduinoV2Box firmware is not part of it; it only supplied the pin map.
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
// Layout: 3 pages, NAV1/NAV2 to move between them, UP/DOWN to scroll the log.
// Long-press DOWN restarts; long-press SHIFT forces download mode (IO0 low).

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
#define PIN_DIR_UP     40
#define PIN_DIR_DOWN   41
#define PIN_LED        48

#define MIDI_RX_PIN    4
#define MIDI_TX_PIN    15

#define LONG_PRESS_MS  1500

// 150 ms: a shade under the 31 ms render cost times nothing -- it exists so a
// run of NoteOns cannot queue up redraws faster than the bus can drain them.
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
                           PIN_DIR_UP, PIN_DIR_DOWN };
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
  cfg.maxEvents = 20;
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
      // full of nothing is distinguishable from one that stopped updating.
      row(11, "MONITOR   rx=%lu", (unsigned long)rxEvents);
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
        if (scroll) row(11, "MONITOR   rx=%lu ^", (unsigned long)rxEvents);
      }
      break;
    }
    case 1: {
      bool conn = bleServer.isConnected();
      row(11, "STATUS");
      u8g2.drawHLine(0, 14, 128);
      row(26, "ble    %s", conn ? "connected" : "advertising");
      row(38, "din5   rx4 tx15");
      row(50, "queue  %u", (unsigned)midiHandler.getQueue().size());
      row(62, "heap   %u  up %lus", (unsigned)ESP.getFreeHeap(),
          millis() / 1000);
      break;
    }
    case 2:
      row(11, "KEYS");
      u8g2.drawHLine(0, 14, 128);
      row(26, "NAV1/2  page %u/%u", page + 1, pageCount);
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
    Serial.printf("[ALIVE] up=%lus heap=%u rx=%lu tx=%lu queue=%u ble=%s "
                  "log=%u/%u\n",
                  millis() / 1000, (unsigned)ESP.getFreeHeap(),
                  (unsigned long)rxEvents, (unsigned long)txEvents,
                  (unsigned)midiHandler.getQueue().size(),
                  bleServer.isConnected() ? "connected" : "advertising",
                  (unsigned)(logCount < LOG_N ? logCount : LOG_N), LOG_N);
  }
}
