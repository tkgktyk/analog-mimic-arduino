/**
 ******************************************************************************
 * @file    AnalogMimicExperimental.h
 * @author  TAKAGI Katsuyuki
 * @brief   Experimental modes of the Analog Mimic module.
 *-----------------------------------------------------------------------------
 * Copyright (C) 2026 TAKAGI Katsuyuki
 *
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 ******************************************************************************
 *
 * These modes are implemented in the firmware, but not yet verified on hardware and in
 * application examples. Their API may change (or they may be removed) before they are
 * released in AnalogMimic.h. The functions are documented in AnalogMimic.h.
 *
 * Usage:
 *   #include <AnalogMimicExperimental.h>
 *   MimicExperimental<Mimic0x> mimic(MIMIC_VDD_5V);   // every function of Mimic0x, too
 *
 * When a mode is released, its function becomes public in MimicBase and its line here
 * is removed; sketches using MimicExperimental<> keep working unchanged.
 */

#ifndef ANALOG_MIMIC_EXPERIMENTAL_H
#define ANALOG_MIMIC_EXPERIMENTAL_H

#include "AnalogMimic.h"

template <class Base>   // Mimic0x or Mimic1x
class MimicExperimental : public Base
{
public:
  using Base::Base;

  // Time: sample & hold, slew rate, delay
  using Base::setSampleAndHold;
  using Base::setTrackAndHold;
  using Base::setSlewRateLimiter;
  using Base::setDigitalDelay;

  // Signal sources
  using Base::setOscillator;
  using Base::setVco;

  // Nonlinear: logarithm, multiplier, true RMS
  using Base::setLog;
  using Base::setAntilog;
  using Base::setSquarer;
  using Base::setSquareRooter;
  using Base::setTrueRms;
};

#endif // ANALOG_MIMIC_EXPERIMENTAL_H
