// Modulator: AM and ring (balanced) modulators
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
  am();
  // or: ring()
}

void loop() {}

// @demo Sine -> AM on a carrier at 20 x signal, 50 % modulation
void am() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  // Carrier 1 V; depth 0.4 per volt: the input peak (1.25 V) gives 50 %
  mimic.setAmModulator(SIGNAL_HZ * 20, 1.0f, 0.4f);
}

// @demo Sine -> ring modulator, carrier at 20 x signal: the envelope crosses zero
void ring() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setRingModulator(SIGNAL_HZ * 20);
}
