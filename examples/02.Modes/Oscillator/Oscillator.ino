// Oscillator: oscillator (function generator)
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty. Pick another demo in setup().
// All the demos of all the examples are in the Modes example.
//
// setOscillator(0, ...) or setConstantVoltage() gives a constant voltage.

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
  sine();
  // or: sawtooth()
}

void loop() {}

// @demo No input -> sine at the signal frequency, 1 V peak
void sine() {
  testSignal.begin(MimicTestSignal::OFF, SIGNAL_HZ);
  mimic.setOscillator(SIGNAL_HZ, 1.0f);
}

// @demo No input -> sawtooth at the signal frequency, 1 V peak
void sawtooth() {
  testSignal.begin(MimicTestSignal::OFF, SIGNAL_HZ);
  // Also MIMIC_OSC_WAVE_TRIANGLE and MIMIC_OSC_WAVE_SQUARE
  mimic.setOscillator(SIGNAL_HZ, 1.0f, MIMIC_OSC_WAVE_SAWTOOTH);
}
