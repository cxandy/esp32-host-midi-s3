// ESP32_Host_MIDI / ProbeD -- instrumented DIN-5 MIDI probe for AciduinoV2Box
//
// Purpose: prove on real hardware that ESP32_Host_MIDI initialises, runs and
// moves MIDI, and make that verdict readable without a serial terminal.
//
// Why the OLED matters here: this board's boot mode is decided by IO0, which is
// the SHIFT button, so the chip parks in the ROM downloader after every flash
// and the only way out is a physical EN press. Serial capture is therefore
// unreliable -- it works only when the host happens to leave COM8 closed across
// that reset. The OLED does not depend on the host at all: if the screen says
// RUNNING, the sketch booted, and no amount of serial archaeology is needed to
// know it.
//
// Pages (NAV-1 / NAV-2 to move):
//   0 STATUS   big RUNNING + uptime, so "did it boot" is answerable at a glance
//   1 MEMORY   heap and free PSRAM -- also the check that PSRAM=opi really
//              brought up the N16R8's octal PSRAM rather than silently failing
//   2 MIDI     DIN-5 queue depth, last event, RX/TX event counters
//   3 CAPS     ESP32_HOST_MIDI_HAS_* capability macros as compiled
//   4 KEYS     the button map
//   5 RESET    reset reason, so a silent reboot loop is identifiable
//
// Long-press DIR-DOWN  -> ESP.restart(), relaunch the sketch with no EN button
// Long-press SHIFT     -> hold IO0 low and restart, which is how you get back
//                         into the ROM downloader for the next flash
//
// Everything on the serial side is mirrored, so a capture still works when it
// can be got, but nothing here depends on it.

#include <Arduino.h>
#include <ESP32_Host_MIDI.h>
#include <UARTConnection.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_system.h>

// ---- AciduinoV2Box pin map (硬件接线规划.md / 使用说明.md) -------------------
// OLED SH1106 128x64 on I2C; the project's own port header uses this exact
// constructor: U8G2_SH1106_128X64_NONAME_F_HW_I2C(rotation, reset, clock, data)
#define PIN_OLED_SCL   11
#define PIN_OLED_SDA   21

// Buttons, all active low with internal pull-up
#define PIN_SHIFT      0   // also the boot strapping pin -- see long-press below
#define PIN_NAV2       1
#define PIN_NAV1       2
#define PIN_DIR_RIGHT  39
#define PIN_DIR_UP     40
#define PIN_DIR_DOWN   41
#define PIN_DIR_LEFT   42
#define PIN_LED        48

// DIN-5 MIDI: RX from the optocoupler output, TX to the MIDI OUT circuit
#define MIDI_RX_PIN    4
#define MIDI_TX_PIN    15

#define LONG_PRESS_MS  1500

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE,
                                                 PIN_OLED_SCL, PIN_OLED_SDA);

static UARTConnection dinMIDI;

static bool oledOk = false;
static uint8_t page = 0;
static uint8_t pageCount = 6;

// MIDI observation
static uint64_t lastEventIndex = 0;
static uint32_t rxEvents = 0;
static uint32_t txEvents = 0;
static char lastEvent[40] = "(none)";

// Timing
static unsigned long lastBlink = 0;
static unsigned long lastBeat = 0;
static unsigned long lastRender = 0;
static unsigned long lastSend = 0;
static bool noteOn = false;

// ---- Buttons ---------------------------------------------------------------
struct Button {
  uint8_t pin;
  const char* name;
  bool held = false;
  bool longFired = false;
  unsigned long downAt = 0;
};

static Button btnShift  = { PIN_SHIFT,    "SHIFT" };
static Button btnNav1   = { PIN_NAV1,     "NAV1" };
static Button btnNav2   = { PIN_NAV2,     "NAV2" };
static Button btnUp     = { PIN_DIR_UP,   "UP" };
static Button btnDown   = { PIN_DIR_DOWN, "DOWN" };

// Debounced per-button edge + long-press detection. Returns true once per press.
static bool pollButton(Button& b) {
  bool now = (digitalRead(b.pin) == LOW);
  bool pressed = false;
  if (now && !b.held) {
    b.held = true;
    b.downAt = millis();
    b.longFired = false;
    pressed = true;
  } else if (now && b.held && !b.longFired &&
             millis() - b.downAt >= LONG_PRESS_MS) {
    b.longFired = true;
    pressed = true;
  } else if (!now && b.held) {
    b.held = false;
  }
  return pressed;
}

static void initButtons() {
  const uint8_t pins[] = { PIN_SHIFT, PIN_NAV1, PIN_NAV2,
                            PIN_DIR_UP, PIN_DIR_DOWN };
  for (uint8_t p : pins) {
    pinMode(p, INPUT_PULLUP);
  }
}

// ---- Setup -----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[PROBE-D] ESP32_Host_MIDI DIN-5 probe, OLED + buttons");

  initButtons();

  // The OLED is the primary output; serial is the mirror. If I2C is missing the
  // sketch still runs and says so rather than hanging here.
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "probing OLED...");
  u8g2.sendBuffer();
  delay(50);
  // A SH1106 answers ACKNOWLEDGE; probing Wire directly keeps the verdict honest
  // instead of assuming success from a silent begin().
  oledOk = (Wire.beginTransmission(0x3C) != 0);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, oledOk ? "OLED ok" : "OLED MISSING");
  u8g2.sendBuffer();
  Serial.print("[PROBE-D] OLED at 0x3C: ");
  Serial.println(oledOk ? "responding" : "no ACK (continuing without it)");

  // ESP32_Host_MIDI, following examples/UART-MIDI-Basic.
  dinMIDI.begin(Serial1, MIDI_RX_PIN, MIDI_TX_PIN);
  midiHandler.addTransport(&dinMIDI);

  MIDIHandlerConfig cfg;
  cfg.maxEvents = 20;
  midiHandler.begin(cfg);

  pinMode(PIN_LED, OUTPUT);

  Serial.printf("[PROBE-D] DIN-5 RX=IO%d TX=IO%d, reset reason=0x%x\n",
                MIDI_RX_PIN, MIDI_TX_PIN, (unsigned)esp_reset_reason());
}

// ---- MIDI ------------------------------------------------------------------
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
  if (!noteOn) {
    midiHandler.sendNoteOn(1, 60, 100);
    Serial.println("[SEND] NoteOn C4");
  } else {
    midiHandler.sendNoteOff(1, 60, 0);
    Serial.println("[SEND] NoteOff C4");
  }
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
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT (EN pin)";
    case ESP_RST_SW:       return "SW (esp_restart)";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_USB:      return "USB";
    case ESP_RST_JTAG:     return "JTAG";
    default:               return "OTHER";
  }
}

static void render() {
  u8g2.clearBuffer();

  switch (page) {
    case 0: {
      // The single most important thing on the board: proof it booted.
      u8g2.setFont(u8g2_font_10x12_tf);
      u8g2.drawStr(4, 14, "RUNNING");
      u8g2.setFont(u8g2_font_6x12_tf);
      row(28, "up %lus  page %u/%u", millis() / 1000, page + 1, pageCount);
      row(42, "heap %u", (unsigned)ESP.getFreeHeap());
      row(56, "NAV1/NAV2: pages");
      break;
    }
    case 1: {
      u8g2.setFont(u8g2_font_6x12_tf);
      row(10, "MEMORY");
      row(24, "heap  %u", (unsigned)ESP.getFreeHeap());
      row(36, "psram %u", (unsigned)ESP.getFreePsram());
      row(48, "lblk  %u", (unsigned)ESP.getMaxAllocHeap());
      row(60, "psram %s", ESP.getPsramSize() ? "OPI ok" : "ABSENT");
      break;
    }
    case 2: {
      u8g2.setFont(u8g2_font_6x12_tf);
      row(10, "MIDI DIN-5  RX=4 TX=15");
      row(22, "queue %u  rx %lu",
          (unsigned)midiHandler.getQueue().size(), (unsigned long)rxEvents);
      row(34, "tx %lu", (unsigned long)txEvents);
      row(46, "last: %s", lastEvent);
      break;
    }
    case 3: {
      u8g2.setFont(u8g2_font_6x12_tf);
      row(10, "CAPS (compiled in)");
#ifdef ESP32_HOST_MIDI_HAS_USB
      row(22, "USB  yes");
#else
      row(22, "USB  no");
#endif
#ifdef ESP32_HOST_MIDI_HAS_BLE
      row(32, "BLE  yes");
#else
      row(32, "BLE  no");
#endif
#ifdef ESP32_HOST_MIDI_HAS_PSRAM
      row(42, "PSRAM yes");
#else
      row(42, "PSRAM no");
#endif
#ifdef ESP32_HOST_MIDI_HAS_ETH_MAC
      row(52, "ETH  yes");
#else
      row(52, "ETH  no");
#endif
      break;
    }
    case 4: {
      u8g2.setFont(u8g2_font_6x12_tf);
      row(10, "KEYS");
      row(22, "NAV1 next  NAV2 prev");
      row(34, "hold DOWN  restart");
      row(46, "hold SHIFT -> download");
      row(58, "SHIFT=IO0 is boot pin");
      break;
    }
    case 5: {
      u8g2.setFont(u8g2_font_6x12_tf);
      row(10, "RESET");
      row(22, "reason %s", resetName(esp_reset_reason()));
      row(34, "raw    0x%x", (unsigned)esp_reset_reason());
      row(46, "rev    %d", (int)ESP.getChipRevision());
      row(58, "cores  %d", (int)ESP.getChipCores());
      break;
    }
  }

  u8g2.sendBuffer();
}

// ---- Loop ------------------------------------------------------------------
void loop() {
  midiHandler.task();
  drainQueue();
  testMidiOut();

  // Buttons. Note SHIFT is handled separately: a long press must not be
  // consumed as navigation, because it is the way back into the bootloader.
  if (pollButton(btnNav1)) {
    page = (page + 1) % pageCount;
    Serial.printf("[UI] page %u\n", page + 1);
  }
  if (pollButton(btnNav2)) {
    page = (page + pageCount - 1) % pageCount;
    Serial.printf("[UI] page %u\n", page + 1);
  }
  if (pollButton(btnUp)) {
    page = (page + 1) % pageCount;
    Serial.printf("[UI] page %u\n", page + 1);
  }
  if (pollButton(btnDown)) {
    if (btnDown.longFired) {
      Serial.println("[UI] restarting sketch (ESP.restart)");
      delay(50);
      ESP.restart();
    }
  }

  if (pollButton(btnShift) && btnShift.longFired) {
    // Driving IO0 low and restarting is the in-firmware route back to the ROM
    // downloader, so the next flash does not need the BOOT button at all.
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
    Serial.printf("[ALIVE] up=%lus heap=%u psram=%u queue=%u rx=%lu\n",
                  millis() / 1000, (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getFreePsram(),
                  (unsigned)midiHandler.getQueue().size(),
                  (unsigned long)rxEvents);
  }
}
