// VfConverter: V/F converter (digital output)
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty.
// All the demos of all the examples are in the Modes example.

#include <Wire.h>
#include <AnalogMimic.h>
#include <MimicTestSignal.h>

Mimic0x mimic(MIMIC_VDD_5V);
MimicTestSignal testSignal;
const float SIGNAL_HZ = 100.0f;  // test signal frequency: every setting below scales with it

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!mimic.begin(&Wire)) {
    Serial.println("Analog Mimic not found");
    while (1);
  }
  vf();
}

void loop() {}

// @demo Slow triangle -> V/F: a square wave from 0 to 20 x signal
void vf() {
  testSignal.begin(MimicTestSignal::TRIANGLE, SIGNAL_HZ / 10);
  // 10 x SIGNAL_HZ at Vref, 8 x SIGNAL_HZ per volt: 0 Hz (HIGH) at the bottom of the input
  mimic.setVfConverter(SIGNAL_HZ * 8, SIGNAL_HZ * 10);
}
