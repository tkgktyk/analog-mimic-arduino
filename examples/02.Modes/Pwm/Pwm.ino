// Pwm: PWM modulator (digital output)
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty.
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
  pwm();
}

void loop() {}

// @demo Sine -> PWM at 20 x signal: the duty cycle follows the input
void pwm() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  // Full scale 1.25 V: 0 % at the bottom of the input, 100 % at the top
  mimic.setPwmModulator(SIGNAL_HZ * 20, 1.25f);
}
