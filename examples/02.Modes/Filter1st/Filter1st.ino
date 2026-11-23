// Filter1st: first-order filters (one RC)
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
  lowPass();
  // or: lowPassHeavy(), highPass(), allPass()
}

void loop() {}

// @demo Square -> 1st-order LPF (fc = 10 x signal): rounds the edges
void lowPass() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilter1stLPF(SIGNAL_HZ * 10);
}

// @demo Square -> 1st-order LPF (fc = signal): heavily smooths the wave
void lowPassHeavy() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilter1stLPF(SIGNAL_HZ);
}

// @demo Square -> 1st-order HPF (fc = 10 x signal): spikes at the edges
void highPass() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilter1stHPF(SIGNAL_HZ * 10);
}

// @demo Sine -> 1st-order APF (fc = signal): 90 degrees behind, the same amplitude
void allPass() {
  // A square wave would jump by twice its step at each edge (the gain is 1 at high
  // frequencies and -1 at DC) and go past the supply
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setFilter1stAPF(SIGNAL_HZ);
}
