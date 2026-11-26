// Multiplier: squarer and square rooter
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty. Pick another demo in setup().
// All the demos of all the examples are in the Modes example.
//
// The outputs are magnitudes from 0 V; the lowest 0.1 V cannot be output
// (lift it with setOutputOffset() if needed).

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
  squarer();
  // or: squareRooter()
}

void loop() {}

// @demo Sine -> squarer: twice the frequency, from 0 V
void squarer() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  // (in - Vref)^2 / unit, unit VDD / 4 = 1.25 V: 0 - 1.25 V
  mimic.setSquarer();
}

// @demo Triangle -> square rooter: 0 below Vref, a square-root curve above
void squareRooter() {
  testSignal.begin(MimicTestSignal::TRIANGLE, SIGNAL_HZ);
  // sqrt((in - Vref) x 1.25 V): 0 - 1.25 V
  mimic.setSquareRooter();
}
