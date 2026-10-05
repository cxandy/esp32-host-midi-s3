// ESP32_Host_MIDI / ProbeE -- dual transport (DIN-5 UART + BLE central) probe
//
// Why this one exists: BLEClientConnection.cpp is the single most fragile file
// in the library. Its ESP_ARDUINO_VERSION 2.x branch is the only thing that
// failed to compile under arduino-esp32 2.0.17, and its own header explains why
// -- the default BLEAddress constructor and toString() only exist from core
// 3.3.0 on. So proving it runs is the most valuable thing left to check on
// hardware.
//
// It also registers two transports at once. That is deliberate: addTransport()
// being called twice and midiHandler.task() dispatching to both is a separate
// thing to be wrong about from either transport working alone.
//
// Roles, which are easy to mix up:
//   UARTConnection        -- wire level, DIN-5
//   BLEClientConnection   -- BLE *central*: this board scans and connects out to
//                             a BLE MIDI peripheral
//   BLEConnection         -- BLE *peripheral*: this board advertises instead
//                             (see examples/T-Display-S3-BLE-Receiver)
//
// With no peripheral nearby the BLE side legitimately stays on "scanning", so
// the bar this clears is: the BLE stack comes up, scanning runs indefinitely
// without corrupting the heap, and the UART transport keeps working alongside
// it. A real connection additionally needs a BLE MIDI peripheral to be on.

#include <Arduino.h>
#include <ESP32_Host_MIDI.h>
#include <UARTConnection.h>
#include <BLEClientConnection.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_system.h>

// ---- AciduinoV2Box pin map --------------------------------------------------
#define PIN_OLED_SCL   11
#define PIN_OLED_SDA   21

#define PIN_SHIFT      0
#define PIN_NAV2       1
#define PIN_NAV1       2
#define PIN_DIR_RIGHT  39
#define PIN_DIR_UP     40
#define PIN_DIR_DOWN   41
#define PIN_DIR_LEFT   42
#define PIN_LED        48

#define MIDI_RX_PIN    4
#define MIDI_TX_PIN    15

#define LONG_PRESS_MS  1500
#define BLE_SCAN_S     30   // refresh the on-screen scan result every 30 s

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE,
                                                 PIN_OLED_SCL, PIN_OLED_SDA);

static UARTConnection dinMIDI;
static BLEClientConnection bleClient;

static bool oledOk = false;
static uint8_t page = 0;
static const uint8_t pageCount = 6;

static uint64_t lastEventIndex = 0;
static uint32_t rxEvents = 0;
static uint32_t txEvents = 0;
static char lastEvent[40] = "(none)";

// BLE observations
static bool bleStarted = false;
static bool everConnected = false;
static char peerAddr[20] = "-";
static unsigned long bleConnectedSinceMs = 0;
static unsigned long bleLastProbe = 0;

static unsigned long lastBlink = 0, lastBeat = 0, lastRender = 0, lastSend = 0;
static bool noteOn = false;

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

static void initButtons() {
  const uint8_t pins[] = { PIN_SHIFT, PIN_NAV1, PIN_NAV2, PIN_DIR_UP, PIN_DIR_DOWN };
  for (uint8_t p : pins) pinMode(p, INPUT_PULLUP);
}

// ---- Setup -----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[PROBE-E] dual transport: DIN-5 UART + BLE central");

  initButtons();

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "probing OLED...");
  u8g2.sendBuffer();
  delay(50);
  // beginTransmission() returns void on arduino-esp32; the ACK comes from
  // endTransmission(): 0 means a device acknowledged the address.
  Wire.beginTransmission(0x3C);
  oledOk = (Wire.endTransmission() == 0);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, oledOk ? "OLED ok" : "OLED MISSING");
  u8g2.sendBuffer();
  Serial.print("[PROBE-E] OLED at 0x3C: ");
  Serial.println(oledOk ? "responding" : "no ACK");

  // Transport 1: DIN-5 wire MIDI
  dinMIDI.begin(Serial1, MIDI_RX_PIN, MIDI_TX_PIN);
  midiHandler.addTransport(&dinMIDI);

  // Transport 2: BLE central. begin() starts scanning for BLE MIDI peripherals.
  // A false return means the client registry (max 3) is full, not that BLE is
  // unavailable -- only one instance is registered here, so treat it as fatal
  // for the BLE half of the probe and say so rather than staying quiet.
  bleStarted = bleClient.begin();
  if (bleStarted) {
    midiHandler.addTransport(&bleClient);
  }
  Serial.print("[PROBE-E] BLE central scanning: ");
  Serial.println(bleStarted ? "yes" : "NO (registry full)");

  MIDIHandlerConfig cfg;
  cfg.maxEvents = 20;
  midiHandler.begin(cfg);

  pinMode(PIN_LED, OUTPUT);

  Serial.printf("[PROBE-E] reset reason=0x%x, transports: UART + %s\n",
                (unsigned)esp_reset_reason(), bleStarted ? "BLE" : "no-BLE");
}

// ---- Events ----------------------------------------------------------------
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
    Serial.print("[MIDI] ");
    Serial.println(lastEvent);
  }
}

static void testMidiOut() {
  if (millis() - lastSend < 2000) return;
  lastSend = millis();
  if (!noteOn) midiHandler.sendNoteOn(1, 60, 100);
  else         midiHandler.sendNoteOff(1, 60, 0);
  noteOn = !noteOn;
  txEvents++;
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

static const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT (EN pin)";
    case ESP_RST_SW:        return "SW (restart)";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_USB:       return "USB";
    case ESP_RST_JTAG:      return "JTAG";
    default:                return "OTHER";
  }
}

static void render() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  switch (page) {
    case 0:
      u8g2.setFont(u8g2_font_t0_12_tf);
      u8g2.drawStr(4, 14, "RUNNING");
      u8g2.setFont(u8g2_font_6x12_tf);
      row(28, "up %lus  page %u/%u", millis() / 1000, page + 1, pageCount);
      row(42, "heap %u psram %u", (unsigned)ESP.getFreeHeap(),
          (unsigned)ESP.getFreePsram());
      row(56, "UART + %s", bleStarted ? "BLE" : "noBLE");
      break;
    case 1:
      row(10, "MEMORY");
      row(24, "heap  %u", (unsigned)ESP.getFreeHeap());
      row(36, "psram %u", (unsigned)ESP.getFreePsram());
      row(48, "lblk  %u", (unsigned)ESP.getMaxAllocHeap());
      row(60, "psram %s", ESP.getPsramSize() ? "OPI ok" : "ABSENT");
      break;
    case 2: {
      bool conn = bleStarted && bleClient.isConnected();
      row(10, "BLE central");
      row(22, "scan  %s", bleStarted ? "started" : "FAILED");
      row(34, "conn  %s", conn ? "YES" : "no");
      row(46, "peer  %s", peerAddr);
      row(58, "ever  %s", everConnected ? "connected" : "not yet");
      break;
    }
    case 3:
      row(10, "DIN-5  RX=4 TX=15");
      row(22, "queue %u  rx %lu", (unsigned)midiHandler.getQueue().size(),
          (unsigned long)rxEvents);
      row(34, "tx %lu", (unsigned long)txEvents);
      row(46, "last: %s", lastEvent);
      break;
    case 4:
      row(10, "KEYS");
      row(22, "NAV1 next  NAV2 prev");
      row(34, "hold DOWN  restart");
      row(46, "hold SHIFT -> download");
      row(58, "SHIFT=IO0 is boot pin");
      break;
    case 5:
      row(10, "RESET");
      row(22, "reason %s", resetName(esp_reset_reason()));
      row(34, "raw    0x%x", (unsigned)esp_reset_reason());
      row(46, "rev    %d", (int)ESP.getChipRevision());
      row(58, "cores  %d", (int)ESP.getChipCores());
      break;
  }

  u8g2.sendBuffer();
}

// ---- Loop ------------------------------------------------------------------
void loop() {
  midiHandler.task();
  drainQueue();
  testMidiOut();

  // Sample the BLE link state. Reading isConnected() is cheap; the address is
  // only meaningful once connected, so only refresh it on a rising edge to keep
  // it from being churned on every frame.
  if (bleStarted) {
    if (bleClient.isConnected()) {
      if (!everConnected) {
        everConnected = true;
        bleConnectedSinceMs = millis();
        snprintf(peerAddr, sizeof(peerAddr), "%s",
                 bleClient.getConnectedAddress().c_str());
        Serial.print("[BLE] connected to ");
        Serial.println(peerAddr);
      }
    } else if (everConnected) {
      everConnected = false;
      Serial.println("[BLE] disconnected");
      bleConnectedSinceMs = 0;
      snprintf(peerAddr, sizeof(peerAddr), "-");
    }
  }

  if (pollButton(btnNav1)) { page = (page + 1) % pageCount; }
  if (pollButton(btnNav2)) { page = (page + pageCount - 1) % pageCount; }
  if (pollButton(btnUp))   { page = (page + 1) % pageCount; }
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
  if (millis() - lastRender > 200) {
    lastRender = millis();
    if (oledOk) render();
  }
  if (millis() - lastBeat > 2000) {
    lastBeat = millis();
    Serial.printf("[ALIVE] up=%lus heap=%u psram=%u queue=%u rx=%lu ble=%s\n",
                  millis() / 1000, (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getFreePsram(),
                  (unsigned)midiHandler.getQueue().size(),
                  (unsigned long)rxEvents,
                  bleStarted ? (bleClient.isConnected() ? "connected" : "scanning")
                             : "off");
  }
}
