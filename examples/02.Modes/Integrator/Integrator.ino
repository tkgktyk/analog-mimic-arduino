// Integrator: integrator
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty.
// All the demos of all the examples are in the Modes example.
//
// Without the leak (leakTimeUs = 0) the output drifts to a rail, because the
// input is never exactly centred on Vref.
// With the leak, the DC gain is leakTimeUs / integrationTimeUs (33 here): an offset of a
// few mV between the input centre and Vref moves the centre of the output triangle by
// 0.1 - 0.2 V, as in an analog integrator.

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
  integrator();
}

void loop() {}

// @demo Square -> integrator with leak: a triangle wave
void integrator() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  // 1.25 V for half a period ramps the output by about 2 V; the leak (10 periods) stops the drift
  // DC gain 33: a few mV of input offset move the output centre by 0.1 - 0.2 V
  mimic.setIntegrator(3e5f / SIGNAL_HZ, 1e7f / SIGNAL_HZ);
}
