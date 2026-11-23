// Differentiator: differentiator
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
  differentiator();
}

void loop() {}

// @demo Triangle -> differentiator: a square wave
void differentiator() {
  testSignal.begin(MimicTestSignal::TRIANGLE, SIGNAL_HZ);
  // The triangle slope (2.5 V per half period) gives about +-1 V
  mimic.setDifferentiator(2e5f / SIGNAL_HZ);
}
