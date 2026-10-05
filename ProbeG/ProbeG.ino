// ESP32_Host_MIDI / ProbeG -- diagnose the OLED stall while BLE is connected
//
// Symptom being chased: with ProbeF the OLED freezes the moment a BLE central
// connects and recovers when it disconnects. The sketch itself keeps running --
// the serial heartbeat stays regular and the heap returns to baseline on
// disconnect -- so this is starvation, not a crash.
//
// Suspected mechanism, from reading the library:
//   BLEConnection::onWrite()   (BLEConnection.cpp:100-128) runs in the BLE
//     stack's task context and enqueues notify packets.
//   BLEConnection::task()      (BLEConnection.cpp:151) only calls processQueue().
//   processQueue()             (BLEConnection.cpp:194) drains with
//     `while (dequeueMidiMessage(msg))` -- no vTaskDelay(), no yield, no bound
//     on iterations per call.
//
// midiHandler.task() reaches that loop from loop(), so while a central keeps
// notifying, the drain never returns and everything after it in loop() -- the
// UART queue and the OLED render -- gets no time at all. That matches the
// symptom exactly: queue empty when disconnected (loop returns at once),
// queue refilled by notify when connected (loop never returns).
//
// This probe measures rather than assumes. It reports the wall-clock time spent
// inside midiHandler.task() and inside the OLED transfer, per call, so the
// claim either shows up as a large midiHandler cost or it does not. It also
// counts loop iterations, which is the direct symptom: a starved loop iterates
// far less often.
//
// OLED page 2 shows the same numbers, so this can be read off the board without
// a serial terminal -- which matters, because the terminal is exactly the thing
// that used to look frozen.

#include <Arduino.h>
#include <ESP32_Host_MIDI.h>
#include <UARTConnection.h>
#include <BLEConnection.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_system.h>

// ---- AciduinoV2Box pin map --------------------------------------------------
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
static const char* BLE_NAME = "ESP32-S3 MIDI";

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE,
                                                 PIN_OLED_SCL, PIN_OLED_SDA);
static UARTConnection dinMIDI;
static BLEConnection bleServer;

static bool oledOk = false;
static uint8_t page = 0;
static const uint8_t pageCount = 6;

// ---- Measurements ----------------------------------------------------------
// Everything is a max over a sliding window so a single slow call is visible
// even though the display only refreshes a few times a second.
static uint32_t taskMaxUs = 0;      // worst midiHandler.task() call
static uint32_t oledMaxUs = 0;       // worst u8g2.sendBuffer() call
static uint32_t loopMaxUs = 0;       // worst full loop() iteration
static uint32_t loopIters = 0;       // iterations since last report
static uint32_t slowLoops = 0;       // iterations over 50 ms
static uint32_t lastReportMs = 0;

static uint64_t lastEventIndex = 0;
static uint32_t rxEvents = 0;
static char lastEvent[40] = "(none)";

// ---- Buttons ---------------------------------------------------------------
struct Button {
  uint8_t pin;
  bool held = false;
  bool longFired = false;
  unsigned long downAt = 0;
};
static Button btnShift = { PIN_SHIFT,   "SHIFT" };
static Button btnNav1  = { PIN_NAV1,    "NAV1" };
static Button btnNav2  = { PIN_NAV2,    "NAV2" };
static Button btnUp    = { PIN_DIR_UP,  "UP" };
static Button btnDown  = { PIN_DIR_DOWN,"DOWN" };

static bool pollButton(Button& b) {
  bool now = (digitalRead(b.pin) == LOW);
  bool pressed = false;
  if (now && !b.held) {
    b.held = true; b.downAt = millis(); b.longFired = false; pressed = true;
  } else if (now && b.held && !b.longFired &&
             millis() - b.downAt >= LONG_PRESS_MS) {
    b.longFired = true; pressed = true;
  } else if (!now && b.held) {
    b.held = false;
  }
  return pressed;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[PROBE-G] diagnosing OLED stall under BLE load");

  const uint8_t pins[] = { PIN_SHIFT, PIN_NAV1, PIN_NAV2, PIN_DIR_UP, PIN_DIR_DOWN };
  for (uint8_t p : pins) pinMode(p, INPUT_PULLUP);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "probing OLED...");
  u8g2.sendBuffer();
  delay(50);
  Wire.beginTransmission(0x3C);
  oledOk = (Wire.endTransmission() == 0);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, oledOk ? "OLED ok" : "OLED MISSING");
  u8g2.sendBuffer();
  Serial.print("[PROBE-G] OLED at 0x3C: ");
  Serial.println(oledOk ? "responding" : "no ACK");

  // Both transports, so the same conditions as ProbeF apply.
  dinMIDI.begin(Serial1, MIDI_RX_PIN, MIDI_TX_PIN);
  midiHandler.addTransport(&dinMIDI);
  bleServer.begin(BLE_NAME);
  midiHandler.addTransport(&bleServer);

  MIDIHandlerConfig cfg;
  cfg.maxEvents = 20;
  cfg.bleName = BLE_NAME;
  midiHandler.begin(cfg);

  pinMode(PIN_LED, OUTPUT);
  lastReportMs = millis();
  Serial.printf("[PROBE-G] advertising as \"%s\"\n", BLE_NAME);
  Serial.println("[PROBE-G] watch: taskMax / loopIters per 2 s window");
}

static void drainQueue() {
  const auto& queue = midiHandler.getQueue();
  for (const auto& ev : queue) {
    if (ev.index <= lastEventIndex) continue;
    lastEventIndex = ev.index;
    rxEvents++;
    char noteBuf[8];
    const char* what;
    switch (ev.statusCode) {
      case MIDI_NOTE_ON:
      case MIDI_NOTE_OFF:
        MIDIHandler::noteWithOctave(ev.noteNumber, noteBuf, sizeof(noteBuf));
        snprintf(lastEvent, sizeof(lastEvent), "%s %s ch%d",
                 ev.statusCode == MIDI_NOTE_ON ? "NoteOn" : "NoteOff",
                 noteBuf, ev.channel0 + 1);
        break;
      case MIDI_CONTROL_CHANGE:
        snprintf(lastEvent, sizeof(lastEvent), "CC %d = %d",
                 ev.noteNumber, ev.velocity7);
        break;
      case MIDI_PROGRAM_CHANGE:
        snprintf(lastEvent, sizeof(lastEvent), "ProgChg %d", ev.noteNumber);
        break;
      case MIDI_PITCH_BEND:
        snprintf(lastEvent, sizeof(lastEvent), "PitchBend %d", ev.pitchBend14);
        break;
      default:
        what = MIDIHandler::statusName(ev.statusCode);
        snprintf(lastEvent, sizeof(lastEvent), "%s", what ? what : "?");
        break;
    }
  }
}

static void row(uint8_t y, const char* fmt, ...) {
  char buf[24];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  u8g2.drawStr(0, y, buf);
}

static void render() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  switch (page) {
    case 0:
      u8g2.setFont(u8g2_font_t0_12_tf);
      u8g2.drawStr(4, 14, "PROBE-G");
      u8g2.setFont(u8g2_font_6x12_tf);
      row(28, "up %lus  page %u/%u", millis() / 1000, page + 1, pageCount);
      row(42, "ble %s", bleServer.isConnected() ? "connected" : "advertising");
      row(56, "NAV1/NAV2 pages");
      break;
    case 1:
      // The measurement page. This is the one to read off the board.
      row(10, "TIMING (max)");
      row(22, "task  %lu ms", (unsigned long)(taskMaxUs / 1000));
      row(34, "oled  %lu ms", (unsigned long)(oledMaxUs / 1000));
      row(46, "loop  %lu ms", (unsigned long)(loopMaxUs / 1000));
      row(58, "iters %lu", (unsigned long)loopIters);
      break;
    case 2:
      row(10, "BLE peripheral");
      row(22, "conn  %s", bleServer.isConnected() ? "YES" : "waiting");
      row(34, "rx %lu", (unsigned long)rxEvents);
      row(46, "q %u", (unsigned)midiHandler.getQueue().size());
      row(58, "last: %s", lastEvent);
      break;
    case 3:
      row(10, "MEMORY");
      row(24, "heap  %u", (unsigned)ESP.getFreeHeap());
      row(36, "psram %u", (unsigned)ESP.getFreePsram());
      row(48, "lblk  %u", (unsigned)ESP.getMaxAllocHeap());
      row(60, "psram %s", ESP.getPsramSize() ? "OPI ok" : "ABSENT");
      break;
    case 4:
      row(10, "KEYS");
      row(22, "NAV1 next  NAV2 prev");
      row(34, "hold DOWN  restart");
      row(46, "hold SHIFT -> download");
      break;
    case 5:
      row(10, "VERDICT");
      // Thresholds chosen so the verdict is readable on a 128x64 panel: a
      // starved loop shows task/loop in the hundreds of ms and a low iters
      // count, a healthy one shows single-digit ms and thousands of iterations.
      row(22, "task%lu oled%lu", (unsigned long)(taskMaxUs / 1000),
          (unsigned long)(oledMaxUs / 1000));
      row(34, "loop%lu slow%lu", (unsigned long)(loopMaxUs / 1000),
          (unsigned long)slowLoops);
      row(46, "iters%lu", (unsigned long)loopIters);
      row(58, loopIters < 50 ? "STARVED?" : "ok");
      break;
  }
  u8g2.sendBuffer();
}

void loop() {
  const uint32_t t0 = micros();

  // The call under suspicion. Time it directly: if processQueue() spins, this
  // single number goes large.
  midiHandler.task();
  const uint32_t t1 = micros();

  drainQueue();

  if (millis() - lastReportMs >= 2000) {
    Serial.printf("[TIMING] taskMax=%lums oledMax=%lums loopMax=%lums "
                  "iters=%lu slow=%lu rx=%lu q=%u conn=%d\n",
                  (unsigned long)(taskMaxUs / 1000),
                  (unsigned long)(oledMaxUs / 1000),
                  (unsigned long)(loopMaxUs / 1000),
                  (unsigned long)loopIters, (unsigned long)slowLoops,
                  (unsigned long)rxEvents,
                  (unsigned)midiHandler.getQueue().size(),
                  (int)bleServer.isConnected());
    lastReportMs = millis();
    taskMaxUs = oledMaxUs = loopMaxUs = 0;
    loopIters = 0;
    slowLoops = 0;
  }

  if (pollButton(btnNav1)) page = (page + 1) % pageCount;
  if (pollButton(btnNav2)) page = (page + pageCount - 1) % pageCount;
  if (pollButton(btnUp))   page = (page + 1) % pageCount;
  if (pollButton(btnDown) && btnDown.longFired) {
    pinMode(PIN_SHIFT, OUTPUT);
    digitalWrite(PIN_SHIFT, LOW);   // reboot into downloader rather than
    delay(50);                      // restart: we want a clean re-run anyway
    ESP.restart();
  }
  if (pollButton(btnShift) && btnShift.longFired) {
    pinMode(PIN_SHIFT, OUTPUT);
    digitalWrite(PIN_SHIFT, LOW);
    delay(50);
    ESP.restart();
  }

  digitalWrite(PIN_LED, !digitalRead(PIN_LED));

  if (oledOk) {
    uint32_t t2 = micros();
    render();
    uint32_t t3 = micros();
    uint32_t d = t3 - t2;
    if (d > oledMaxUs) oledMaxUs = d;
  }

  const uint32_t tEnd = micros();
  uint32_t taskD = t1 - t0;
  if (taskD > taskMaxUs) taskMaxUs = taskD;
  uint32_t loopD = tEnd - t0;
  if (loopD > loopMaxUs) loopMaxUs = loopD;
  if (loopD > 50000) slowLoops++;
  loopIters++;
}
