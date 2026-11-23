// Rectifier: precision rectifiers
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
  fullWave();
  // or: halfWave()
}

void loop() {}

// @demo Sine -> full-wave rectifier: the negative half folds up (double frequency)
void fullWave() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setRectifierFull();
}

// @demo Sine -> half-wave rectifier: the negative half stays at Vref
void halfWave() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setRectifierHalf();
}
