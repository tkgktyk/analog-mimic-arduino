// Amplifier: bypass and programmable gain
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
  bypassSquare();
  // or: bypassSine(), gain1_5(), gain0_5()
}

void loop() {}

// @demo Square -> bypass: the input as it is (about 20 us later)
void bypassSquare() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setBypassDSP();
}

// @demo Sine -> bypass
void bypassSine() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setBypassDSP();
}

// @demo Sine -> PGA x1.5 around Vref
void gain1_5() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setPGA(1.5f);
}

// @demo Sine -> PGA x0.5 around Vref
void gain0_5() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setPGA(0.5f);
}
