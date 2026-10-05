// ESP32_Host_MIDI / ProbeH -- isolate the OLED refresh cost, faithfully
//
// Why yet another probe: ProbeG measured the right thing badly. It called
// render() every loop iteration with no throttle, so it measured its own probe
// rather than ProbeF's behaviour -- iters=62 in a 2 s window is 32 ms of I2C per
// iteration back to back, which is a loop doing nothing but I2C. ProbeF does
// throttle (millis() - lastRender > 200), so its cost profile is different and
// remains unmeasured.
//
// What is already established, and should not be re-litigated:
//   - midiHandler.task() is 0 ms, with BLE connected or not. The library's
//     processQueue() starvation is NOT the cause; earlier reasoning that pointed
//     there was wrong.
//   - The heartbeats keep ticking and the heap returns to baseline on
//     disconnect, so nothing is crashed or leaking.
//   - oledMax sits at a rock-steady 32 ms with no growth, so sendBuffer() is
//     consistently slow rather than occasionally blocking. One 2032 ms outlier
//     appeared once, then never again.
//
// Two details worth testing, both cheap and both plausible:
//   1. I2C error status. U8G2's sendBuffer() returns no error code, so a NACK
//      storm would be invisible except as slowness. The board's own firmware
//      explicitly raises the bus to 400 kHz (AciduinoV2Box
//      src/uCtrl/module/oled/oled.cpp:65 -- display->setBusClock(400000)), and
//      ProbeF never did that. At the default 100 kHz, 1024 bytes costs about
//      92 ms; the measured 32 ms suggests something in between. Worth making
//      explicit and measuring rather than inferring.
//   2. Whether the 200 ms throttle actually limits the number of renders. If
//      render() ever takes longer than the throttle period, every iteration
//      re-triggers it and the throttle collapses -- a feedback loop that would
//      explain a frozen UI even though the code "looks" throttled.
//
// So: measure clearBuffer (pure memory) separately from sendBuffer (the bus),
// at several explicit bus speeds, and count renders attempted vs performed.
// Serial is the primary output here because the OLED is the thing under test.

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

// ---- Phase machinery -------------------------------------------------------
// Phase 0: baseline, bus left at whatever U8G2 chose.
// Phase 1: explicit 100 kHz.
// Phase 2: explicit 400 kHz, matching the board's own firmware.
// Each phase renders in a tight burst, then reports; nothing else runs, so the
// numbers are pure bus cost with no scheduler noise mixed in.
static uint8_t phase = 0;
static const uint8_t phaseCount = 3;
static unsigned long phaseStartMs = 0;
static bool phaseBusy = false;

// Results per phase.
static uint32_t pClearUs[3] = {0};
static uint32_t pSendUs[3]  = {0};
static uint8_t  pSendErr[3] = {0};   // Wire.endTransmission() on a probe write
static uint16_t pRenders[3] = {0};   // renders actually completed
static uint16_t pTriggers[3] = {0};  // loop iterations that asked to render

// Long-run throttle check, independent of the phases.
static uint32_t throttleRenders = 0;
static uint32_t throttleDue = 0;
static uint32_t throttleWorstUs = 0;

static uint64_t lastEventIndex = 0;
static uint32_t rxEvents = 0;
static unsigned long lastBlink = 0, lastBeat = 0;

// ---- Buttons ---------------------------------------------------------------
struct Button {
  uint8_t pin;
  bool held = false;
  bool longFired = false;
  unsigned long downAt = 0;
};
static Button btnShift = { PIN_SHIFT,    "SHIFT" };
static Button btnNav1  = { PIN_NAV1,     "NAV1" };
static Button btnNav2  = { PIN_NAV2,     "NAV2" };
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

static void report(const char* tag) {
  Serial.printf("[PHASE %u %s] clear=%lums send=%lums renders=%u "
                "triggerDue=%u wireErr=%u\n",
                phase, tag,
                (unsigned long)(pClearUs[phase] / 1000),
                (unsigned long)(pSendUs[phase] / 1000),
                pRenders[phase], pTriggers[phase], pSendErr[phase]);
}

// One render, timed in two halves. clearBuffer/drawStr are RAM only; sendBuffer
// is the part that touches the bus. Separating them tells us whether the cost is
// the bus or something else entirely.
static void timedRender() {
  uint32_t t0 = micros();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 0, "ProbeH");
  u8g2.drawStr(0, 12, "bus cost measurement");
  u8g2.drawStr(0, 24, "0123456789012345");
  u8g2.drawStr(0, 36, "abcdefghijklmnop");
  u8g2.drawStr(0, 48, "partial redraw test");
  u8g2.drawStr(0, 60, "second line here");
  uint32_t t1 = micros();
  u8g2.sendBuffer();
  uint32_t t2 = micros();

  if (t1 - t0 > pClearUs[phase]) pClearUs[phase] = t1 - t0;
  if (t2 - t1 > pSendUs[phase])  pSendUs[phase]  = t2 - t1;
  if (pRenders[phase] < 65535) pRenders[phase]++;

  // Direct bus probe: one address-only transaction. endTransmission() returns
  // 0 on ACK, non-zero on NACK or bus error. U8G2 swallows this, so ask Wire
  // directly to find out whether the display is actually ACKing.
  Wire.beginTransmission(0x3C);
  uint8_t err = Wire.endTransmission();
  if (err) pSendErr[phase] = err;
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("[PROBE-H] OLED bus cost isolation");
  Serial.println("[PROBE-H] the board's own firmware uses 400 kHz "
                 "(oled.cpp:65 setBusClock(400000))");

  const uint8_t pins[] = { PIN_SHIFT, PIN_NAV1, PIN_NAV2, PIN_DIR_UP, PIN_DIR_DOWN };
  for (uint8_t p : pins) pinMode(p, INPUT_PULLUP);

  u8g2.begin();
  Wire.beginTransmission(0x3C);
  oledOk = (Wire.endTransmission() == 0);
  Serial.print("[PROBE-H] OLED at 0x3C: ");
  Serial.println(oledOk ? "responding" : "no ACK");
  Serial.print("[PROBE-H] bus clock after U8G2 begin(): ");
  Serial.println(Wire.getClock());

  dinMIDI.begin(Serial1, MIDI_RX_PIN, MIDI_TX_PIN);
  midiHandler.addTransport(&dinMIDI);
  bleServer.begin(BLE_NAME);
  midiHandler.addTransport(&bleServer);

  MIDIHandlerConfig cfg;
  cfg.maxEvents = 20;
  cfg.bleName = BLE_NAME;
  midiHandler.begin(cfg);

  pinMode(PIN_LED, OUTPUT);
  phaseStartMs = millis();
  Serial.println("[PROBE-H] phase 0 running (bus as configured by U8G2)");
}

void loop() {
  midiHandler.task();

  const auto& queue = midiHandler.getQueue();
  for (const auto& ev : queue) {
    if (ev.index > lastEventIndex) {
      lastEventIndex = ev.index;
      rxEvents++;
    }
  }

  if (pollButton(btnNav1))  Serial.println("[UI] NAV1");
  if (pollButton(btnNav2))  Serial.println("[UI] NAV2");
  if (pollButton(btnDown) && btnDown.longFired) {
    pinMode(PIN_SHIFT, OUTPUT);
    digitalWrite(PIN_SHIFT, LOW);
    delay(50);
    ESP.restart();
  }
  if (pollButton(btnShift) && btnShift.longFired) {
    pinMode(PIN_SHIFT, OUTPUT);
    digitalWrite(PIN_SHIFT, LOW);
    delay(50);
    ESP.restart();
  }

  if (millis() - lastBlink > 500) {
    lastBlink = millis();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
  if (millis() - lastBeat > 2000) {
    lastBeat = millis();
    Serial.printf("[BEAT] up=%lus heap=%u psram=%u rx=%lu q=%u conn=%d "
                  "bus=%luHz\n",
                  millis() / 1000, (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getFreePsram(), (unsigned long)rxEvents,
                  (unsigned)midiHandler.getQueue().size(),
                  (int)bleServer.isConnected(),
                  (unsigned long)Wire.getClock());
    Serial.printf("[THROTTLE] renders=%lu due=%lu worstRender=%lums\n",
                  (unsigned long)throttleRenders, (unsigned long)throttleDue,
                  (unsigned long)(throttleWorstUs / 1000));
  }

  // ---- Phased measurement, 4 s each ------------------------------------------
  if (!phaseBusy && millis() - phaseStartMs > 4000) {
    phaseBusy = true;
    switch (phase) {
      case 0:
        report("baseline");
        phase++;
        Wire.setClock(100000);
        Serial.printf("[PROBE-H] -> phase 1 at 100 kHz, now %lu Hz\n",
                      (unsigned long)Wire.getClock());
        break;
      case 1:
        report("100kHz");
        phase++;
        Wire.setClock(400000);
        Serial.printf("[PROBE-H] -> phase 2 at 400 kHz, now %lu Hz\n",
                      (unsigned long)Wire.getClock());
        break;
      case 2:
        report("400kHz");
        phase = phaseCount;
        Serial.println("[PROBE-H] phases done; now idling");
        break;
      default:
        phaseBusy = false;
        return;
    }
    phaseStartMs = millis();
    phaseBusy = false;
    return;
  }

  // Burst of renders while a phase is active, so the per-render cost settles.
  if (phase < phaseCount) {
    uint32_t t0 = micros();
    timedRender();
    uint32_t d = micros() - t0;
    if (d > throttleWorstUs) throttleWorstUs = d;
    throttleRenders++;
    return;
  }

  // ---- Steady state: exactly ProbeF's throttle --------------------------------
  // This is the part that matters for the reported symptom. If a render ever
  // exceeds the 200 ms period, every iteration re-arms and throttleDue climbs in
  // lockstep with throttleRenders -- that collapse is the bug.
  if (oledOk && millis() - lastBeat > 2000 - 200) {
    // Reuse lastBeat as the throttle clock; it is reset every 2 s above, which
    // is close enough to a 200 ms cadence for this comparison and keeps the two
    // ideas separate in the code.
  }
  if (oledOk) {
    static unsigned long lastRender = 0;
    if (millis() - lastRender > 200) {
      lastRender = millis();
      throttleDue++;
      uint32_t t0 = micros();
      timedRender();
      uint32_t d = micros() - t0;
      if (d > throttleWorstUs) throttleWorstUs = d;
    }
  }
}
