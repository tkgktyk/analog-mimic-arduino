// ConstantVoltage: a fixed output voltage (no input needed)
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty.
// All the demos of all the examples are in the Modes example.
//
// The clipper with the upper limit equal to the lower limit. Vref is not changed,
// and the calibration of the module applies as usual.

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
  constantVoltage();
}

void loop() {}

// @demo No input -> 1.2 V at the output
void constantVoltage() {
  testSignal.begin(MimicTestSignal::OFF, SIGNAL_HZ);
  mimic.setConstantVoltage(1.2f);
}
