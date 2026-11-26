// Logarithm: logarithmic and antilogarithmic amplifiers
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty. Pick another demo in setup().
// All the demos of all the examples are in the Modes example.
//
// The antilog output is a magnitude from 0 V; the lowest 0.1 V cannot be output
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
  logAmp();
  // or: antilogAmp()
}

void loop() {}

// @demo Triangle -> log amplifier (Vref 1.0 V, 1 V/decade): the ramps bend
void logAmp() {
  testSignal.begin(MimicTestSignal::TRIANGLE, SIGNAL_HZ);
  // The input (1.25 - 3.75 V) must stay above Vref: 0.25 - 2.75 V above 1.0 V gives 0.7 - 1.74 V
  mimic.setVref(1.0f);
  mimic.setLog(1.0f, 0.5f);
}

// @demo Triangle -> antilog amplifier (1 V/oct): the output doubles per volt
void antilogAmp() {
  testSignal.begin(MimicTestSignal::TRIANGLE, SIGNAL_HZ);
  // 1 / log10(2) V per decade; 0.5 V at Vref, 0.21 - 1.19 V for the input (from 0 V)
  mimic.setAntilog(1.0f / log10f(2.0f), 0.5f);
}
