// Feasibility probe A: always-on pieces only (MIDITransport + MIDIHandler + MIDIHandlerConfig)
// Purpose: does the library core compile on arduino-esp32 2.0.17 for ESP32-S3?
#include <Arduino.h>
#include <ESP32_Host_MIDI.h>

static uint64_t lastEventIndex = 0;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("[PROBE-A] core compile start");

    MIDIHandlerConfig cfg;
    cfg.maxEvents = 20;
    cfg.bleName = "AcidProbeA";
    midiHandler.begin(cfg);

    Serial.printf("[PROBE-A] USB cap=%d BLE cap=%d PSRAM cap=%d ETH cap=%d\n",
                  ESP32_HOST_MIDI_HAS_USB, ESP32_HOST_MIDI_HAS_BLE,
                  ESP32_HOST_MIDI_HAS_PSRAM, ESP32_HOST_MIDI_HAS_ETH_MAC);
    Serial.println("[PROBE-A] core OK");
}

void loop() {
    midiHandler.task();
    const auto& q = midiHandler.getQueue();
    for (const auto& ev : q) {
        if (ev.index <= lastEventIndex) continue;
        lastEventIndex = ev.index;
        Serial.print("[PROBE-A] ");
        Serial.print(MIDIHandler::statusName(ev.statusCode));
        Serial.print(" ch=");
        Serial.print(ev.channel0 + 1);
        Serial.print(" a=");
        Serial.print(ev.noteNumber);
        Serial.print(" b=");
        Serial.println(ev.velocity7);
    }
}