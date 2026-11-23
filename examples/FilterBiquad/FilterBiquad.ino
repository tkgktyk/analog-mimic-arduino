// FilterBiquad: second-order filters (state variable filter)
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
  lowPassBessel();
  // or: lowPassButterworth(), lowPassChebyshev(), highPassBessel(), highPassButterworth(), highPassChebyshev(), bandPass(), notch(), allPass(), unity()
}

void loop() {}

// @demo Square -> biquad LPF (fc = 10 x signal, Bessel): smooth, almost no ringing
void lowPassBessel() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadLPF(SIGNAL_HZ * 10, MIMIC_FILTER_Q_BESSEL);
}

// @demo Square -> biquad LPF (Butterworth): maximally flat passband
void lowPassButterworth() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadLPF(SIGNAL_HZ * 10, MIMIC_FILTER_Q_BUTTERWORTH);
}

// @demo Square -> biquad LPF (Chebyshev): sharp cutoff, ringing
void lowPassChebyshev() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadLPF(SIGNAL_HZ * 10, MIMIC_FILTER_Q_CHEBYSHEV);
}

// @demo Square -> biquad HPF (fc = 10 x signal, Bessel)
void highPassBessel() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadHPF(SIGNAL_HZ * 10, MIMIC_FILTER_Q_BESSEL);
}

// @demo Square -> biquad HPF (Butterworth)
void highPassButterworth() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadHPF(SIGNAL_HZ * 10, MIMIC_FILTER_Q_BUTTERWORTH);
}

// @demo Square -> biquad HPF (Chebyshev)
void highPassChebyshev() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadHPF(SIGNAL_HZ * 10, MIMIC_FILTER_Q_CHEBYSHEV);
}

// @demo Square -> biquad BPF (3 x signal, Q = 5): picks the 3rd harmonic (a sine)
void bandPass() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadBPF(SIGNAL_HZ * 3, 5.0f);
}

// @demo Square -> biquad notch (3 x signal, Q = 1): removes the 3rd harmonic
void notch() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadNotch(SIGNAL_HZ * 3, 1.0f);
}

// @demo Square -> biquad APF (3 x signal): shape changes, amplitude stays
void allPass() {
  testSignal.begin(MimicTestSignal::SQUARE, SIGNAL_HZ);
  mimic.setFilterBiquadAPF(SIGNAL_HZ * 3);
}

// @demo Sine -> unity-gain biquad: flat, the same as bypass
void unity() {
  testSignal.begin(MimicTestSignal::SINE, SIGNAL_HZ);
  mimic.setFilterBiquadUnity();
}
