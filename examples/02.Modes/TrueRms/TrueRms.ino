// TrueRms: true RMS-to-DC converter
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty. Pick another demo in setup().
// All the demos of all the examples are in the Modes example.
//
// The output is a magnitude from 0 V. Use it without decimation.

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
  rmsSine();
  // or: rmsSquare()
}

void loop() {}

// @demo Sine -> true RMS: about 0.88 V DC (1.25 V / sqrt 2)
void rmsSine() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  // Averaging time constant: 10 periods of the signal, in us
  mimic.setTrueRms(1e7f / SIGNAL_HZ);
}

// @demo Square -> true RMS: 1.25 V DC (an average-responding meter reads 11 % more)
void rmsSquare() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setTrueRms(1e7f / SIGNAL_HZ);
}
