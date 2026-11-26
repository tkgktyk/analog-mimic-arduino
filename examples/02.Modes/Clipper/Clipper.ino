// Clipper: hard and soft clippers, dead zone
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
  hardClip();
  // or: softClip(), deadZone()
}

void loop() {}

// @demo Sine -> hard clipper (3.0 V / 2.0 V): flat tops
void hardClip() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setClipper(3.0f, 2.0f);
}

// @demo Sine -> soft clipper (3.0 V / 2.0 V): rounded tops
void softClip() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setSoftClipper(3.0f, 2.0f);
}

// @demo Sine -> dead zone (3.0 V / 2.0 V): only the parts beyond the limits pass
void deadZone() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setDeadZone(3.0f, 2.0f);
}
