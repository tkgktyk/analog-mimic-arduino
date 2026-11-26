// Noise: white and pink noise
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
  white();
  // or: pink()
}

void loop() {}

// @demo No input -> white noise, +/-1 V
void white() {
  testSignal.begin(MimicTestSignal::OFF, SIGNAL_HZ);
  mimic.setNoise(1.0f);
}

// @demo No input -> pink noise (-3 dB/octave), rms 1/3 V
void pink() {
  testSignal.begin(MimicTestSignal::OFF, SIGNAL_HZ);
  mimic.setNoise(1.0f, MIMIC_NOISE_PINK);
}
