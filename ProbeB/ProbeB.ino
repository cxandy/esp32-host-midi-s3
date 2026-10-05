// ESP32_Host_MIDI — feasibility probe B
// Target: ESP32-S3-WROOM N16R8 (AciduinoV2Box board)
//
// Stage: library core + UART (DIN-5) transport on the board's real MIDI pins.
//   MIDI IN  = IO4  (RX, via optocoupler)
//   MIDI OUT = IO15 (TX)
//
// This is the transport that needs no USB mode change and no core upgrade,
// so it is the lowest-risk proof that the library works on this board.
//
// A DIN MIDI controller plugged into the IN jack should appear on Serial,
// and the firmware echoes every NoteOn back out the OUT jack.

#include <Arduino.h>
#include <ESP32_Host_MIDI.h>
#include <UARTConnection.h>

// ---- Board pin map (AciduinoV2Box / ESP32-S3-WROOM N16R8) ----
static const int PIN_MIDI_RX = 4;
static const int PIN_MIDI_TX = 15;

UARTConnection dinMIDI;
static uint64_t lastEventIndex = 0;
static unsigned long lastBlink = 0;
static unsigned long lastBeat = 0;
static unsigned long beatCount = 0;

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("[PROBE-B] ESP32_Host_MIDI / UART DIN-5");

    if (!dinMIDI.begin(Serial2, PIN_MIDI_RX, PIN_MIDI_TX)) {
        Serial.println("[PROBE-B] FATAL: UART begin failed");
        return;
    }
    midiHandler.addTransport(&dinMIDI);

    MIDIHandlerConfig cfg;
    cfg.maxEvents = 20;
    cfg.bleName = "AcidBoxProbe";
    midiHandler.begin(cfg);

    Serial.printf("[PROBE-B] MIDI RX=IO%d TX=IO%d @31250\n", PIN_MIDI_RX, PIN_MIDI_TX);
    Serial.printf("[PROBE-B] caps: USB=%d BLE=%d PSRAM=%d ETH=%d  PSRAM=%u bytes\n",
                  ESP32_HOST_MIDI_HAS_USB, ESP32_HOST_MIDI_HAS_BLE,
                  ESP32_HOST_MIDI_HAS_PSRAM, ESP32_HOST_MIDI_HAS_ETH_MAC,
                  (unsigned)ESP.getPsramSize());
    Serial.printf("[PROBE-B] free heap=%u\n", (unsigned)ESP.getFreeHeap());
    Serial.println("[PROBE-B] ready - plug a controller into MIDI IN");
}

void loop() {
    midiHandler.task();

    const auto& q = midiHandler.getQueue();
    for (const auto& ev : q) {
        if (ev.index <= lastEventIndex) continue;
        lastEventIndex = ev.index;

        if (ev.statusCode == MIDI_NOTE_ON || ev.statusCode == MIDI_NOTE_OFF) {
            char noteBuf[8];
            MIDIHandler::noteWithOctave(ev.noteNumber, noteBuf, sizeof(noteBuf));
            Serial.printf("[IN] %-7s ch=%2u %-4s vel=%3u\n",
                          ev.statusCode == MIDI_NOTE_ON ? "NoteOn" : "NoteOff",
                          ev.channel0 + 1, noteBuf, ev.velocity7);
            // Loop back to the DIN OUT so a controller/LED can confirm the path.
            if (ev.statusCode == MIDI_NOTE_ON) {
                midiHandler.sendNoteOn(ev.channel0 + 1, ev.noteNumber, ev.velocity7);
            } else {
                midiHandler.sendNoteOff(ev.channel0 + 1, ev.noteNumber, 0);
            }
        } else if (ev.statusCode == MIDI_CONTROL_CHANGE) {
            Serial.printf("[IN] CC       ch=%2u cc=%3u val=%3u\n",
                          ev.channel0 + 1, ev.noteNumber, ev.velocity7);
        } else {
            Serial.printf("[IN] %s ch=%u\n", MIDIHandler::statusName(ev.statusCode),
                          ev.channel0 + 1);
        }
    }

    // Heartbeat on the board LED (IO48) so a bare board shows it is alive.
    if (millis() - lastBlink > 500) {
        lastBlink = millis();
        digitalWrite(48, !digitalRead(48));
    }

    // Periodic heartbeat over serial. The setup() banner is printed ~300 ms
    // after boot, which is easy to miss when the terminal attaches late, so keep
    // saying something -- this is what makes the probe self-verifying without
    // needing to win a race against the reset.
    if (millis() - lastBeat > 2000) {
        lastBeat = millis();
        beatCount++;
        Serial.printf("[ALIVE] #%lu  up=%lus  heap=%u  queue=%u  freePSRAM=%u\n",
                      beatCount, millis() / 1000, (unsigned)ESP.getFreeHeap(),
                      (unsigned)midiHandler.getQueue().size(),
                      (unsigned)ESP.getFreePsram());
    }
}