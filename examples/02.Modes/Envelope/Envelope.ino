// Envelope: envelope follower and peak hold
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty. Pick another demo in setup().
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
  envelope();
  // or: peakHold()
}

void loop() {}

// @demo Sine -> envelope follower (decay 10 periods): follows the peaks
void envelope() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setEnvelopeFollower(1e7f / SIGNAL_HZ);
}

// @demo Sine -> peak hold: keeps the highest peak
void peakHold() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setPeakHold();
}
