// SlewRate: slew rate limiter, like the slew rate of an op-amp
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
  slewSquare();
  // or: slewSine()
}

void loop() {}

// @demo Square -> slew rate limiter: the edges take a quarter period
void slewSquare() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  // The 2.5 V step in a quarter period: 10 x SIGNAL_HZ V/s, in V/us
  mimic.setSlewRateLimiter(SIGNAL_HZ * 1e-5f);
}

// @demo Sine -> slew rate below the steepest slope of the sine: a smaller triangle
void slewSine() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  // The sine rises at up to 2 pi f x 1.25 V = 7.9 x SIGNAL_HZ V/s; the limit is about half of it
  mimic.setSlewRateLimiter(SIGNAL_HZ * 4e-6f);
}
