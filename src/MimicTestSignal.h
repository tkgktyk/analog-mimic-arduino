/**
 * @file MimicTestSignal.h
 * @brief Test signal for the examples: a square, sine or triangle wave on the DAC pin (A0)
 *        of an Arduino UNO R4 (Minima / WiFi).
 *
 * The wave is centred at half the DAC full scale (2.5 V on a 5 V board), which is the
 * module's default Vref, with a peak of a quarter of the full scale (+-1.25 V on a 5 V board).
 * A 200 kHz timer interrupt updates the DAC, so the frequency is exact when 200 kHz divided
 * by it is a power of two; otherwise a period is off by one sample (5 us) now and then.
 *
 * The edges of this signal move by a few microseconds from one period to the next (timer
 * interrupt latency of the Arduino). Trigger the oscilloscope on the edge you are looking at.
 *
 * Only for the Renesas boards (UNO R4). Other boards do not have this class.
 */
#ifndef MIMIC_TEST_SIGNAL_H
#define MIMIC_TEST_SIGNAL_H

#include <Arduino.h>

#if defined(ARDUINO_ARCH_RENESAS)

class MimicTestSignal {
public:
  enum Wave : uint8_t {
    OFF = 0,  ///< 0 V (for modes that do not use the input)
    SQUARE,   ///< starts high
    SINE,     ///< starts at the centre, rising
    TRIANGLE  ///< starts at the centre, rising
  };

  /**
   * @brief Starts the signal, or changes it if it is already running.
   * @param wave   MimicTestSignal::SQUARE, SINE, TRIANGLE or OFF.
   * @param freqHz Frequency in Hz (up to about 10 kHz is useful).
   * @return false if no timer is available.
   */
  bool begin(Wave wave = SQUARE, float freqHz = 100.0f);

  /** @brief Changes the wave and keeps the frequency. */
  void setWave(Wave wave);

  /** @brief Changes the frequency and keeps the wave. */
  void setFrequency(float freqHz);
};

#endif // ARDUINO_ARCH_RENESAS
#endif // MIMIC_TEST_SIGNAL_H
