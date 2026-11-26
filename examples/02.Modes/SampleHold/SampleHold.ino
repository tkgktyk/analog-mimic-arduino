// SampleHold: sample & hold and track & hold
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
  sampleHold();
  // or: trackHold(), aliasing()
}

void loop() {}

// @demo Sine -> sample & hold at 10 x signal: a staircase
void sampleHold() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setSampleAndHold(SIGNAL_HZ * 10);
}

// @demo Sine -> track & hold at 10 x signal, tracking half of each period
void trackHold() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setTrackAndHold(SIGNAL_HZ * 10, 0.5f);
}

// @demo Sine -> sample & hold at 1.1 x signal: a slow sine at 0.1 x signal (aliasing)
void aliasing() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setSampleAndHold(SIGNAL_HZ * 1.1f);
}
