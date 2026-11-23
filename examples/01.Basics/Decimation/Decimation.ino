// Decimation: running the processing at a lower rate
//
// setDecimation(N) averages 2^N input samples and runs the mode at 200 kHz / 2^N.
// Every time and frequency of the mode is scaled by 2^N: the settings keep their
// numbers, the circuit becomes 2^N times slower. Use it for long time constants
// and low cutoff frequencies (and to average the input; see getAdcValue()).
//
// Wiring: Arduino UNO R4 A0 -> module IN, SDA / SCL / 5V / GND -> module.
// Look at the module OUT (and at A0 for the input) with an oscilloscope.
// The demo runs from setup(); loop() is empty. Pick another demo in setup().

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
  lowPassN0();
  // or: lowPassN3(), bypassN6()
}

void loop() {}

// @demo Square -> 1st-order LPF (fc = 10 x signal), N = 0: rounds the edges
void lowPassN0() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setDecimation(0);
  mimic.setFilter1stLPF(SIGNAL_HZ * 10);
}

// @demo The same LPF setting with N = 3: 8 times slower (fc = 1.25 x signal)
void lowPassN3() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setDecimation(3);
  mimic.setFilter1stLPF(SIGNAL_HZ * 10);  // same number: the decimation scales it
}

// @demo Sine -> bypass with N = 6 (3.125 kHz): steps of 320 us show the lower rate
void bypassN6() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setDecimation(6);
  mimic.setBypassDSP();
}
