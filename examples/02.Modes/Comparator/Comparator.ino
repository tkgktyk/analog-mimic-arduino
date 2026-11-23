// Comparator: comparators with a digital output
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
  schmitt();
  // or: window()
}

void loop() {}

// @demo Sine -> Schmitt trigger (3.0 V / 2.0 V): clean edges with hysteresis
void schmitt() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setComparatorSchmitt(3.0f, 2.0f);
}

// @demo Sine -> window comparator (3.0 V / 2.0 V): high outside the window
void window() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setComparatorWindow(3.0f, 2.0f);
}
