// Minimal boot probe: no ESP32_Host_MIDI, no PSRAM, no TinyUSB.
// Its only job is to answer one question: can this board run *any* sketch
// when reset over the native USB port?
//
// Serial goes to the USB-Serial-JTAG peripheral (USBMode=hwcdc), which is
// the same COM port the ROM downloader uses. That matters: Windows already
// has that port enumerated, so output is capturable without waiting for a
// TinyUSB re-enumeration, and unlike TinyUSB CDC it is not gated on DTR.
//
// Prints every 500 ms on purpose -- even if the sketch immediately faults and
// the chip resets, the lines that were already sent stay visible.

#include <Arduino.h>

static unsigned long beat = 0;

void setup() {
  Serial.begin(115200);
  pinMode(48, OUTPUT);
  Serial.println("\n[PROBE-C] minimal boot probe, no library, no PSRAM");
}

void loop() {
  beat++;
  digitalWrite(48, (beat / 2) & 1);
  Serial.printf("[PROBE-C] beat=%lu up=%lus heap=%u\n",
                beat, millis() / 1000, (unsigned)ESP.getFreeHeap());
  delay(500);
}
