/**
 * @file MimicTestSignal.cpp
 * @brief Test signal on the DAC of an Arduino UNO R4 (see MimicTestSignal.h).
 */
#include "MimicTestSignal.h"

#if defined(ARDUINO_ARCH_RENESAS)

#include <FspTimer.h>

namespace {

constexpr float UPDATE_HZ = 200000.0f;  // DAC updates per second
constexpr uint16_t CENTER = 2048;       // 12-bit DAC codes
constexpr uint16_t PEAK = 1024;
constexpr int SINE_POINTS = 256;

volatile uint16_t *const DAC_DATA = (uint16_t *)0x4005E000;  // DAC12 DADR0 (pin A0)

FspTimer dacTimer;
bool running = false;
uint16_t sineTable[SINE_POINTS];
volatile uint8_t currentWave = MimicTestSignal::OFF;
volatile uint32_t phaseIncrement = 0;
uint32_t phase = 0;

void onTimer(timer_callback_args_t *) {
  phase += phaseIncrement;
  uint16_t code;
  switch (currentWave) {
    case MimicTestSignal::SQUARE:
      code = (phase & 0x80000000u) ? CENTER - PEAK : CENTER + PEAK;
      break;
    case MimicTestSignal::SINE:
      code = sineTable[phase >> 24];
      break;
    case MimicTestSignal::TRIANGLE: {
      uint32_t q = (phase + 0x40000000u) >> 20;  // 0 - 4095, centre rising at phase 0
      uint32_t a = (q < 2048) ? q : 4095 - q;      // 0 - 2047
      code = CENTER - PEAK + a;
      break;
    }
    default:
      code = 0;
      break;
  }
  *DAC_DATA = code;
}

}  // namespace

bool MimicTestSignal::begin(Wave wave, float freqHz) {
  setFrequency(freqHz);
  setWave(wave);
  if (running) return true;

  for (int i = 0; i < SINE_POINTS; i++) {
    sineTable[i] = CENTER + (int)lroundf(PEAK * sinf(2.0f * PI * i / SINE_POINTS));
  }
  analogWriteResolution(12);
  analogWrite(A0, 0);  // enables the DAC on A0

  uint8_t type = 0;
  int8_t channel = FspTimer::get_available_timer(type);
  if (channel < 0) return false;
  if (!dacTimer.begin(TIMER_MODE_PERIODIC, type, channel, UPDATE_HZ, 50.0f, onTimer)) return false;
  // Highest priority: USB and millis() interrupts must not delay the DAC update
  dacTimer.setup_overflow_irq(0);
  dacTimer.open();
  dacTimer.start();
  running = true;
  return true;
}

void MimicTestSignal::setWave(Wave wave) {
  currentWave = wave;
}

void MimicTestSignal::setFrequency(float freqHz) {
  if (freqHz < 0.0f) freqHz = 0.0f;
  if (freqHz > UPDATE_HZ / 2) freqHz = UPDATE_HZ / 2;
  phaseIncrement = (uint32_t)((double)freqHz * 4294967296.0 / UPDATE_HZ);
}

#endif // ARDUINO_ARCH_RENESAS
